// Level 3: Weather adaptation + tyre swaps + fuel management
// Weather cycles through conditions; need to swap tyres when weather changes
// Scoring: base_score + fuel_bonus
#include <iostream>
#include <iomanip>
#include <cmath>
#include "parser.h"
#include "physics.h"
#include "output.h"

int main(int argc, char *argv[])
{
    std::string inputPath = "inputFiles/level3input.txt";
    if (argc > 1)
        inputPath = argv[1];

    RaceData rd = load_race_data(inputPath);
    int n = rd.segments.size();

    std::cerr << "Race: " << rd.race.name << " | " << rd.race.laps << " laps | "
              << n << " segments\n";
    std::cerr << "Weather cycle: ";
    for (auto &w : rd.weather)
        std::cerr << w.condition << "(" << w.duration << "s) ";
    std::cerr << "\n";

    // Forward simulate lap by lap, tracking time for weather
    std::vector<LapPlan> lap_plans;
    double current_speed = 0.0;
    double fuel_remaining = rd.car.initial_fuel;
    double elapsed_time = 0.0;
    std::string current_weather = rd.starting_weather;
    int current_tyre_id = -1;
    std::string current_compound;
    double current_friction = 0;
    bool first_tyre_selected = false;

    // Track which tyre sets have been used (for L3 each set has 1 id, so no issue)
    // We can reuse the same set if not blown

    for (int lap = 0; lap < rd.race.laps; lap++)
    {
        // Determine weather at start of this lap
        std::string weather_now = get_weather_string_at_time(rd.weather, elapsed_time);
        auto &wc = get_weather_at_time(rd.weather, elapsed_time);
        double accel_eff = rd.car.accel * wc.accel_mult;
        double brake_eff = rd.car.brake_decel * wc.decel_mult;

        // Select best tyre for current weather
        auto best_tyre = select_best_tyre(rd, weather_now);

        // Do we need a tyre swap?
        bool need_tyre_swap = false;
        if (!first_tyre_selected)
        {
            current_tyre_id = best_tyre.id;
            current_compound = best_tyre.compound;
            current_friction = best_tyre.friction;
            first_tyre_selected = true;
        }
        else if (best_tyre.compound != current_compound)
        {
            // Weather changed, different tyre optimal — check if pit is worth it
            // Cost: base_pit_time + tyre_swap_time = 23s; benefit: better friction → faster laps
            // For simplicity, always swap when best compound changes
            need_tyre_swap = true;
        }

        // If we swapped last lap, update tyre info
        if (need_tyre_swap && lap > 0)
        {
            // We'll handle the pit at the END of prev lap; but we need to set tyre now
            current_compound = best_tyre.compound;
            current_friction = best_tyre.friction;
            current_tyre_id = best_tyre.id;
        }

        // Recompute friction based on current tyre and weather
        auto &tp = rd.tyre_props[current_compound];
        double friction_mult = get_friction_multiplier(tp, weather_now);
        double friction = tp.life_span * friction_mult;

        // Resolve corner chains and straight plans for this lap
        auto required_speed = resolve_corner_chains(
            rd.segments, friction, rd.car.crawl_speed, rd.car.max_speed);

        // Use max speed as target
        double target = rd.car.max_speed;
        std::vector<StraightPlan> straight_plans(n, {0, 0});
        for (int i = 0; i < n; i++)
        {
            if (rd.segments[i].type == "straight")
            {
                int next = (i + 1) % n;
                double exit_sp = required_speed[next];
                double bd = braking_distance(target, exit_sp, brake_eff);
                bd = std::ceil(bd * 100.0) / 100.0;
                straight_plans[i] = {target, bd};
            }
        }

        // Simulate this lap
        double lap_time = 0;
        double lap_fuel = 0;
        double speed = current_speed;

        for (int i = 0; i < n; i++)
        {
            if (rd.segments[i].type == "straight")
            {
                auto res = simulate_straight(speed, straight_plans[i].target_speed,
                                             straight_plans[i].brake_start, rd.segments[i].length,
                                             accel_eff, brake_eff,
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

        elapsed_time += lap_time;
        fuel_remaining -= lap_fuel;
        current_speed = speed;

        // Build lap plan
        LapPlan lp;
        lp.straight_plans = straight_plans;
        lp.pit = {false, 0, 0};

        // Check if we need to pit: tyre swap or refuel
        if (lap < rd.race.laps - 1)
        {
            // Check weather for next lap
            std::string next_weather = get_weather_string_at_time(rd.weather, elapsed_time);
            auto next_best = select_best_tyre(rd, next_weather);
            bool swap_next = (next_best.compound != current_compound);

            // Estimate fuel for next lap
            auto &next_wc = get_weather_at_time(rd.weather, elapsed_time);
            double next_accel = rd.car.accel * next_wc.accel_mult;
            double next_brake = rd.car.brake_decel * next_wc.decel_mult;
            auto &next_tp = rd.tyre_props[swap_next ? next_best.compound : current_compound];
            double next_fmult = get_friction_multiplier(next_tp, next_weather);
            double next_friction = next_tp.life_span * next_fmult;
            auto next_req = resolve_corner_chains(rd.segments, next_friction, rd.car.crawl_speed, rd.car.max_speed);

            // Estimate next lap fuel at max speed
            double next_fuel_estimate = 0;
            double sp = swap_next ? rd.race.pit_exit_speed : speed;
            for (int i = 0; i < n; i++)
            {
                if (rd.segments[i].type == "straight")
                {
                    int nx = (i + 1) % n;
                    double exit_sp = next_req[nx];
                    double bd = braking_distance(target, exit_sp, next_brake);
                    bd = std::ceil(bd * 100.0) / 100.0;
                    auto res = simulate_straight(sp, target, bd, rd.segments[i].length,
                                                 next_accel, next_brake,
                                                 rd.car.max_speed, rd.car.crawl_speed);
                    next_fuel_estimate += res.fuel;
                    sp = res.exit_speed;
                }
                else
                {
                    double cs = std::min(sp, next_req[i]);
                    cs = std::max(cs, rd.car.crawl_speed);
                    next_fuel_estimate += fuel_usage(cs, cs, rd.segments[i].length);
                    sp = cs;
                }
            }

            bool need_refuel = fuel_remaining < next_fuel_estimate * 1.05;

            if (swap_next || need_refuel)
            {
                double refuel_amount = 0;
                if (need_refuel)
                {
                    int laps_left = rd.race.laps - lap - 1;
                    double fuel_needed = next_fuel_estimate * laps_left;
                    refuel_amount = std::min(fuel_needed - fuel_remaining, rd.car.fuel_capacity - fuel_remaining);
                    refuel_amount = std::max(0.0, refuel_amount);
                    refuel_amount = std::min(refuel_amount, rd.car.fuel_capacity - fuel_remaining);
                    refuel_amount = std::ceil(refuel_amount * 100.0) / 100.0;
                }

                double pt = pit_stop_time(refuel_amount, rd.race.pit_refuel_rate,
                                          rd.race.pit_tyre_swap_time, rd.race.base_pit_time, swap_next);
                elapsed_time += pt;
                fuel_remaining += refuel_amount;
                current_speed = rd.race.pit_exit_speed;

                int swap_id = swap_next ? next_best.id : 0;
                lp.pit = {true, swap_id, refuel_amount};

                if (swap_next)
                {
                    current_compound = next_best.compound;
                    current_tyre_id = next_best.id;
                    current_friction = next_best.friction;
                }

                std::cerr << "Pit after lap " << lap + 1 << ": swap=" << (swap_next ? next_best.compound : "no")
                          << " refuel=" << refuel_amount << "L time=" << pt << "s\n";
            }
        }

        lap_plans.push_back(lp);
    }

    // Compute fuel burned
    double total_refueled = 0;
    for (auto &lp : lap_plans)
        if (lp.pit.enter)
            total_refueled += lp.pit.fuel_refuel_amount;
    double fuel_burned = rd.car.initial_fuel + total_refueled - fuel_remaining;

    double base_score = calc_base_score(rd.race.time_reference, elapsed_time);
    double fuel_bonus = calc_fuel_bonus(fuel_burned, rd.race.fuel_soft_cap);
    double final_score = base_score + fuel_bonus;

    std::cerr << std::fixed << std::setprecision(4);
    std::cerr << "Total time: " << elapsed_time << " s\n";
    std::cerr << "Fuel burned: " << fuel_burned << " L (cap=" << rd.race.fuel_soft_cap << ")\n";
    std::cerr << "Base score: " << base_score << "\n";
    std::cerr << "Fuel bonus: " << fuel_bonus << "\n";
    std::cerr << "Final score: " << final_score << "\n";

    // Initial tyre = first tyre selected
    int init_tyre = lap_plans.empty() ? 1 : select_best_tyre(rd, rd.starting_weather).id;
    std::cout << generate_output(init_tyre, rd.segments, lap_plans, rd.race.laps).dump(2) << std::endl;
    return 0;
}
