// Level 2: Fuel management + pit stops for refueling
// Builds on Level 1: same speed optimization, adds fuel tracking and pit strategy
// Scoring: base_score + fuel_bonus
#include <iostream>
#include <iomanip>
#include <cmath>
#include "parser.h"
#include "physics.h"
#include "output.h"

// Estimate fuel used per lap at a given uniform target speed
double estimate_fuel_per_lap(const RaceData &rd, double target_speed,
                             const std::vector<double> &required_speed,
                             double entry_speed_first_straight)
{
    int n = rd.segments.size();
    double fuel = 0;
    double speed = entry_speed_first_straight;

    for (int i = 0; i < n; i++)
    {
        if (rd.segments[i].type == "straight")
        {
            int next = (i + 1) % n;
            double exit_sp = required_speed[next];
            double ts = std::min(target_speed, rd.car.max_speed);
            double bd = braking_distance(ts, exit_sp, rd.car.brake_decel);
            bd = std::ceil(bd * 100.0) / 100.0;

            auto res = simulate_straight(speed, ts, bd, rd.segments[i].length,
                                         rd.car.accel, rd.car.brake_decel,
                                         rd.car.max_speed, rd.car.crawl_speed);
            fuel += res.fuel;
            speed = res.exit_speed;
        }
        else
        {
            double cs = std::min(speed, required_speed[i]);
            cs = std::max(cs, rd.car.crawl_speed);
            fuel += fuel_usage(cs, cs, rd.segments[i].length);
            speed = cs;
        }
    }
    return fuel;
}

// Estimate time per lap at a given target speed
double estimate_time_per_lap(const RaceData &rd, double target_speed,
                             const std::vector<double> &required_speed,
                             double entry_speed)
{
    int n = rd.segments.size();
    double time = 0;
    double speed = entry_speed;

    for (int i = 0; i < n; i++)
    {
        if (rd.segments[i].type == "straight")
        {
            int next = (i + 1) % n;
            double exit_sp = required_speed[next];
            double ts = std::min(target_speed, rd.car.max_speed);
            double bd = braking_distance(ts, exit_sp, rd.car.brake_decel);
            bd = std::ceil(bd * 100.0) / 100.0;

            auto res = simulate_straight(speed, ts, bd, rd.segments[i].length,
                                         rd.car.accel, rd.car.brake_decel,
                                         rd.car.max_speed, rd.car.crawl_speed);
            time += res.time;
            speed = res.exit_speed;
        }
        else
        {
            double cs = std::min(speed, required_speed[i]);
            cs = std::max(cs, rd.car.crawl_speed);
            time += rd.segments[i].length / cs;
            speed = cs;
        }
    }
    return time;
}

int main(int argc, char *argv[])
{
    std::string inputPath = "inputFiles/level2input.txt";
    if (argc > 1)
        inputPath = argv[1];

    RaceData rd = load_race_data(inputPath);
    int n = rd.segments.size();

    // Select best tyre (dry weather in L2)
    auto best = select_best_tyre(rd, rd.starting_weather);
    double tyre_friction = best.friction;
    std::cerr << "Selected tyre: " << best.compound << " (id=" << best.id
              << "), friction=" << tyre_friction << "\n";

    // Resolve corner chains
    auto required_speed = resolve_corner_chains(
        rd.segments, tyre_friction, rd.car.crawl_speed, rd.car.max_speed);

    // Binary search for optimal target speed that maximizes total score
    // Score = base_score(time) + fuel_bonus(fuel_used)
    // Higher speed → faster time (better base_score) but more fuel (worse fuel_bonus if over cap)
    // Lower speed → slower time but less fuel

    double best_score = -1e18;
    double best_target = rd.car.max_speed;

    // Normal lap entry speed (from last corner of previous lap)
    double normal_entry = required_speed[(n - 1) % n];
    if (rd.segments[n - 1].type == "straight")
        normal_entry = rd.car.crawl_speed;
    // Pit exit entry speed
    double pit_entry = rd.race.pit_exit_speed;

    // Try speeds from 30 to 90 m/s in increments of 0.5
    for (double ts = 30.0; ts <= rd.car.max_speed; ts += 0.5)
    {
        double fuel_per_lap_normal = estimate_fuel_per_lap(rd, ts, required_speed, normal_entry);
        double fuel_per_lap_pit = estimate_fuel_per_lap(rd, ts, required_speed, pit_entry);
        double time_per_lap_normal = estimate_time_per_lap(rd, ts, required_speed, normal_entry);
        double time_per_lap_pit = estimate_time_per_lap(rd, ts, required_speed, pit_entry);

        // First lap starts at 0 m/s
        double fuel_first = estimate_fuel_per_lap(rd, ts, required_speed, 0.0);
        double time_first = estimate_time_per_lap(rd, ts, required_speed, 0.0);

        // Determine how many laps we can do per tank
        double tank = rd.car.initial_fuel;
        double total_time = time_first;
        double total_fuel = fuel_first;
        double remaining_fuel = tank - fuel_first;
        int laps_done = 1;
        int pit_stops = 0;

        while (laps_done < rd.race.laps)
        {
            double next_fuel = fuel_per_lap_normal;

            if (remaining_fuel < next_fuel)
            {
                // Need pit stop — refuel to full tank
                double refuel_amount = std::min(rd.car.fuel_capacity - remaining_fuel, rd.car.fuel_capacity);
                double pt = pit_stop_time(refuel_amount, rd.race.pit_refuel_rate,
                                          rd.race.pit_tyre_swap_time, rd.race.base_pit_time, false);
                total_time += pt;
                remaining_fuel += refuel_amount;
                total_fuel += refuel_amount; // refueled this amount (not counted as "used" from cap perspective)
                pit_stops++;
                // After pit, entry speed is pit_exit_speed
                next_fuel = fuel_per_lap_pit;
                total_time += time_per_lap_pit;
            }
            else
            {
                total_time += time_per_lap_normal;
            }

            remaining_fuel -= next_fuel;
            total_fuel = (tank + pit_stops * rd.car.fuel_capacity) - remaining_fuel; // total drawn from fuel system
            laps_done++;
        }

        // Calculate total fuel actually consumed (burned)
        // fuel_used = initial_fuel + all refuels - remaining_fuel
        double fuel_burned = rd.car.initial_fuel - remaining_fuel;
        // Add fuel from pit refuels
        // Actually: total fuel used = initial - remaining + sum(refuels)...
        // Let me recalculate properly
        // We track remaining_fuel which starts at initial_fuel and we add refuel amounts
        // fuel_used = (initial_fuel + total_refueled) - remaining_fuel
        // But for scoring, fuel_used = total fuel consumed from the tank(s)

        // Simpler: just sum up all the per-lap fuel consumption
        double total_fuel_used = fuel_first;
        double rem = tank - fuel_first;
        int pits = 0;
        double t_time = time_first;
        for (int lap = 1; lap < rd.race.laps; lap++)
        {
            double nf = (pits > 0 && lap == 1) ? fuel_per_lap_pit : fuel_per_lap_normal;
            bool pitted_this_lap = false;
            if (rem < fuel_per_lap_normal)
            {
                double refuel = rd.car.fuel_capacity - rem;
                rem += refuel;
                pits++;
                pitted_this_lap = true;
                nf = fuel_per_lap_pit;
            }
            total_fuel_used += nf;
            rem -= nf;
        }

        double base_s = calc_base_score(rd.race.time_reference, total_time);
        double fuel_b = calc_fuel_bonus(total_fuel_used, rd.race.fuel_soft_cap);
        double score = base_s + fuel_b;

        if (score > best_score)
        {
            best_score = score;
            best_target = ts;
        }
    }

    std::cerr << "Optimal target speed: " << best_target << " m/s, score: " << best_score << "\n";

    // Now do the actual simulation with the best target speed
    double target_speed = best_target;
    auto final_plans = compute_straight_plans(rd.segments, required_speed, rd.car.max_speed, rd.car.brake_decel);
    // Override target speeds
    for (int i = 0; i < n; i++)
    {
        if (rd.segments[i].type == "straight")
        {
            int next = (i + 1) % n;
            double exit_sp = required_speed[next];
            double bd = braking_distance(target_speed, exit_sp, rd.car.brake_decel);
            bd = std::ceil(bd * 100.0) / 100.0;
            final_plans[i] = {target_speed, bd};
        }
    }

    // Forward simulate to determine pit stops
    std::vector<LapPlan> lap_plans;
    double current_speed = 0.0;
    double fuel_remaining = rd.car.initial_fuel;
    double total_time = 0.0;
    double total_fuel_used = 0.0;

    for (int lap = 0; lap < rd.race.laps; lap++)
    {
        LapPlan lp;
        lp.straight_plans = final_plans;
        lp.pit = {false, 0, 0};

        // Simulate this lap
        double lap_fuel = 0;
        double lap_time = 0;
        double speed = current_speed;

        for (int i = 0; i < n; i++)
        {
            if (rd.segments[i].type == "straight")
            {
                auto res = simulate_straight(speed, final_plans[i].target_speed,
                                             final_plans[i].brake_start, rd.segments[i].length,
                                             rd.car.accel, rd.car.brake_decel,
                                             rd.car.max_speed, rd.car.crawl_speed);
                lap_time += res.time;
                lap_fuel += res.fuel;
                speed = res.exit_speed;
            }
            else
            {
                double cs = std::min(speed, required_speed[i]);
                cs = std::max(cs, rd.car.crawl_speed);
                lap_time += rd.segments[i].length / cs;
                lap_fuel += fuel_usage(cs, cs, rd.segments[i].length);
                speed = cs;
            }
        }

        total_time += lap_time;
        total_fuel_used += lap_fuel;
        fuel_remaining -= lap_fuel;
        current_speed = speed;

        // Check if we need to pit (not enough fuel for next lap)
        if (lap < rd.race.laps - 1)
        {
            double next_lap_fuel = estimate_fuel_per_lap(rd, target_speed, required_speed, speed);
            if (fuel_remaining < next_lap_fuel)
            {
                double refuel_amount = std::min(rd.car.fuel_capacity - fuel_remaining, rd.car.fuel_capacity);
                // Only refuel what we need for remaining laps (optimize fuel usage)
                int laps_left = rd.race.laps - lap - 1;
                double fuel_needed = next_lap_fuel * laps_left;
                refuel_amount = std::min(refuel_amount, std::max(0.0, fuel_needed - fuel_remaining));
                refuel_amount = std::min(refuel_amount, rd.car.fuel_capacity - fuel_remaining);
                refuel_amount = std::ceil(refuel_amount * 100.0) / 100.0;

                double pt = pit_stop_time(refuel_amount, rd.race.pit_refuel_rate,
                                          rd.race.pit_tyre_swap_time, rd.race.base_pit_time, false);
                total_time += pt;
                fuel_remaining += refuel_amount;
                total_fuel_used += refuel_amount;
                current_speed = rd.race.pit_exit_speed;
                lp.pit = {true, 0, refuel_amount};
                std::cerr << "Pit stop after lap " << lap + 1 << ": refuel=" << refuel_amount
                          << "L, pit_time=" << pt << "s\n";
            }
        }

        lap_plans.push_back(lp);
    }

    double base_score = calc_base_score(rd.race.time_reference, total_time);
    // For fuel_bonus, we need total fuel consumed (burned) not including refuel amounts
    // fuel burned = sum of all lap_fuel values
    double fuel_burned = total_fuel_used;
    // Wait, total_fuel_used includes refuel amounts added. Let me recalc.
    // Actually: fuel_burned = initial_fuel + total_refueled - fuel_remaining
    double total_refueled = 0;
    for (auto &lp : lap_plans)
        if (lp.pit.enter)
            total_refueled += lp.pit.fuel_refuel_amount;
    fuel_burned = rd.car.initial_fuel + total_refueled - fuel_remaining;

    double fuel_bonus = calc_fuel_bonus(fuel_burned, rd.race.fuel_soft_cap);
    double final_score = base_score + fuel_bonus;

    std::cerr << std::fixed << std::setprecision(4);
    std::cerr << "Total time: " << total_time << " s\n";
    std::cerr << "Fuel burned: " << fuel_burned << " L (cap=" << rd.race.fuel_soft_cap << ")\n";
    std::cerr << "Base score: " << base_score << "\n";
    std::cerr << "Fuel bonus: " << fuel_bonus << "\n";
    std::cerr << "Final score: " << final_score << "\n";

    std::cout << generate_output(best.id, rd.segments, lap_plans, rd.race.laps).dump(2) << std::endl;
    return 0;
}
