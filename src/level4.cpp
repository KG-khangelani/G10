// Level 4: Full simulation — tyre degradation + weather + fuel + multi-set pit strategy
// Scoring: base_score + fuel_bonus + tyre_bonus
#include <iostream>
#include <iomanip>
#include <cmath>
#include <set>
#include "parser.h"
#include "physics.h"
#include "output.h"

// Per-set state tracking
struct SetState
{
    int id;
    std::string compound;
    double degradation;
    double life_span;
    bool blown;
};

int main(int argc, char *argv[])
{
    std::string inputPath = "inputFiles/level4input.txt";
    if (argc > 1)
        inputPath = argv[1];

    RaceData rd = load_race_data(inputPath);
    int n = rd.segments.size();

    std::cerr << "Race: " << rd.race.name << " | " << rd.race.laps << " laps | "
              << n << " segments\n";

    // Initialize all tyre set states
    std::map<int, SetState> all_sets;
    for (auto &ts : rd.tyre_sets)
    {
        auto &tp = rd.tyre_props[ts.compound];
        for (int id : ts.ids)
        {
            all_sets[id] = {id, ts.compound, 0.0, tp.life_span, false};
        }
    }

    // Forward simulate
    std::vector<LapPlan> lap_plans;
    double current_speed = 0.0;
    double fuel_remaining = rd.car.initial_fuel;
    double elapsed_time = 0.0;

    // Select initial tyre
    std::string start_weather = rd.starting_weather;
    auto initial_tyre = select_best_tyre(rd, start_weather);
    int current_set_id = initial_tyre.id;
    std::string current_compound = initial_tyre.compound;
    double current_degradation = 0.0;
    std::set<int> used_set_ids;
    used_set_ids.insert(current_set_id);

    std::cerr << "Initial tyre: " << current_compound << " (id=" << current_set_id << ")\n";

    for (int lap = 0; lap < rd.race.laps; lap++)
    {
        // Current weather
        auto &wc = get_weather_at_time(rd.weather, elapsed_time);
        std::string weather_now = wc.condition;
        double accel_eff = rd.car.accel * wc.accel_mult;
        double brake_eff = rd.car.brake_decel * wc.decel_mult;

        // Current tyre friction with degradation
        auto &tp = rd.tyre_props[current_compound];
        double deg_rate = get_degradation_rate(tp, weather_now);
        double friction_mult = get_friction_multiplier(tp, weather_now);
        double tyre_friction = calc_tyre_friction(tp.life_span, current_degradation, friction_mult);

        if (tyre_friction < 0.01)
            tyre_friction = 0.01; // avoid zero friction

        // Resolve corner chains for current friction
        auto required_speed = resolve_corner_chains(
            rd.segments, tyre_friction, rd.car.crawl_speed, rd.car.max_speed);

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

        // Simulate this lap segment by segment
        double lap_time = 0;
        double lap_fuel = 0;
        double speed = current_speed;
        double lap_degradation = 0;
        bool limp_mode = false;

        for (int i = 0; i < n; i++)
        {
            if (limp_mode)
            {
                // Limp mode: constant speed, no accel/decel
                double t = rd.segments[i].length / rd.car.limp_speed;
                lap_time += t;
                lap_fuel += fuel_usage(rd.car.limp_speed, rd.car.limp_speed, rd.segments[i].length);
                speed = rd.car.limp_speed;
                continue;
            }

            if (rd.segments[i].type == "straight")
            {
                auto res = simulate_straight(speed, straight_plans[i].target_speed,
                                             straight_plans[i].brake_start, rd.segments[i].length,
                                             accel_eff, brake_eff,
                                             rd.car.max_speed, rd.car.crawl_speed);
                lap_time += res.time;
                lap_fuel += res.fuel;

                // Tyre degradation for straight
                lap_degradation += tyre_deg_straight(deg_rate, rd.segments[i].length);

                // Braking degradation (if braking occurred)
                if (res.distance_brake > 0 && res.speed_at_brake > res.exit_speed)
                {
                    lap_degradation += tyre_deg_braking(deg_rate, res.speed_at_brake, res.exit_speed);
                }

                speed = res.exit_speed;
            }
            else
            {
                // Corner
                double cs = std::min(speed, required_speed[i]);
                cs = std::max(cs, rd.car.crawl_speed);
                lap_time += rd.segments[i].length / cs;
                lap_fuel += fuel_usage(cs, cs, rd.segments[i].length);

                // Corner degradation
                lap_degradation += tyre_deg_corner(deg_rate, cs, rd.segments[i].radius);

                speed = cs;
            }

            // Check fuel
            if (fuel_remaining - lap_fuel <= 0)
            {
                limp_mode = true;
                speed = rd.car.limp_speed;
            }

            // Check tyre blowout
            if (current_degradation + lap_degradation >= tp.life_span)
            {
                limp_mode = true;
                speed = rd.car.limp_speed;
                all_sets[current_set_id].blown = true;
            }
        }

        elapsed_time += lap_time;
        fuel_remaining -= lap_fuel;
        current_degradation += lap_degradation;
        all_sets[current_set_id].degradation = current_degradation;
        current_speed = speed;

        // Build lap plan
        LapPlan lp;
        lp.straight_plans = straight_plans;
        lp.pit = {false, 0, 0};

        // Pit decision for next lap
        if (lap < rd.race.laps - 1)
        {
            std::string next_weather = get_weather_string_at_time(rd.weather, elapsed_time);
            auto &next_wc = get_weather_at_time(rd.weather, elapsed_time);

            // Check if we need tyre swap: weather change or tyre nearly blown
            auto &cur_tp = rd.tyre_props[current_compound];
            double next_deg_rate = get_degradation_rate(cur_tp, next_weather);
            double next_fmult = get_friction_multiplier(cur_tp, next_weather);
            double next_friction = calc_tyre_friction(cur_tp.life_span, current_degradation, next_fmult);

            // Estimate degradation for next lap (rough: same as last lap)
            double est_next_deg = lap_degradation * 1.1; // safety margin
            bool tyre_danger = (current_degradation + est_next_deg >= cur_tp.life_span * 0.95);

            // Check if different tyre compound is better for next weather
            auto next_best = select_best_tyre(rd, next_weather);
            bool better_compound = (next_best.compound != current_compound &&
                                    next_best.friction > next_friction * 1.1);

            // Need refuel?
            // Estimate fuel for next lap
            auto next_req = resolve_corner_chains(rd.segments, std::max(next_friction, 0.1),
                                                  rd.car.crawl_speed, rd.car.max_speed);
            double next_fuel_est = 0;
            double sp = current_speed;
            for (int i = 0; i < n; i++)
            {
                if (rd.segments[i].type == "straight")
                {
                    int nx = (i + 1) % n;
                    double bd = braking_distance(target, next_req[nx], brake_eff);
                    bd = std::ceil(bd * 100.0) / 100.0;
                    auto res = simulate_straight(sp, target, bd, rd.segments[i].length,
                                                 accel_eff, brake_eff,
                                                 rd.car.max_speed, rd.car.crawl_speed);
                    next_fuel_est += res.fuel;
                    sp = res.exit_speed;
                }
                else
                {
                    double cs = std::min(sp, next_req[i]);
                    cs = std::max(cs, rd.car.crawl_speed);
                    next_fuel_est += fuel_usage(cs, cs, rd.segments[i].length);
                    sp = cs;
                }
            }

            bool need_refuel = fuel_remaining < next_fuel_est * 1.05;
            bool need_tyre_swap = tyre_danger || better_compound || limp_mode;

            if (need_tyre_swap || need_refuel)
            {
                // Select new tyre set
                int swap_id = 0;
                if (need_tyre_swap)
                {
                    // Find a non-blown set for best compound
                    // Try best compound first, then any compound
                    auto best_for_weather = select_best_tyre(rd, next_weather);
                    // Find a set with remaining life
                    int chosen_id = -1;
                    for (auto &ts : rd.tyre_sets)
                    {
                        if (ts.compound != best_for_weather.compound)
                            continue;
                        for (int id : ts.ids)
                        {
                            if (id == current_set_id)
                                continue;
                            if (all_sets[id].blown)
                                continue;
                            if (all_sets[id].degradation < all_sets[id].life_span * 0.9)
                            {
                                chosen_id = id;
                                break;
                            }
                        }
                        if (chosen_id >= 0)
                            break;
                    }
                    // Fallback: any non-blown set with remaining life
                    if (chosen_id < 0)
                    {
                        for (auto &ts : rd.tyre_sets)
                        {
                            for (int id : ts.ids)
                            {
                                if (id == current_set_id)
                                    continue;
                                if (all_sets[id].blown)
                                    continue;
                                if (all_sets[id].degradation < all_sets[id].life_span * 0.9)
                                {
                                    chosen_id = id;
                                    break;
                                }
                            }
                            if (chosen_id >= 0)
                                break;
                        }
                    }
                    // Last resort: reuse current (no swap)
                    if (chosen_id < 0)
                    {
                        need_tyre_swap = false;
                    }
                    else
                    {
                        swap_id = chosen_id;
                    }
                }

                double refuel_amount = 0;
                if (need_refuel)
                {
                    int laps_left = rd.race.laps - lap - 1;
                    double fuel_needed = next_fuel_est * laps_left;
                    refuel_amount = std::min(fuel_needed - fuel_remaining, rd.car.fuel_capacity - fuel_remaining);
                    refuel_amount = std::max(0.0, refuel_amount);
                    refuel_amount = std::min(refuel_amount, rd.car.fuel_capacity - fuel_remaining);
                    refuel_amount = std::ceil(refuel_amount * 100.0) / 100.0;
                }

                if (need_tyre_swap || refuel_amount > 0)
                {
                    double pt = pit_stop_time(refuel_amount, rd.race.pit_refuel_rate,
                                              rd.race.pit_tyre_swap_time, rd.race.base_pit_time,
                                              need_tyre_swap);
                    elapsed_time += pt;
                    fuel_remaining += refuel_amount;
                    current_speed = rd.race.pit_exit_speed;
                    lp.pit = {true, need_tyre_swap ? swap_id : 0, refuel_amount};

                    if (need_tyre_swap && swap_id > 0)
                    {
                        current_compound = all_sets[swap_id].compound;
                        current_degradation = all_sets[swap_id].degradation;
                        current_set_id = swap_id;
                        used_set_ids.insert(swap_id);
                        std::cerr << "Pit lap " << lap + 1 << ": swap to " << current_compound
                                  << " (id=" << swap_id << " deg=" << current_degradation << ")";
                    }
                    else
                    {
                        std::cerr << "Pit lap " << lap + 1 << ": refuel only";
                    }
                    std::cerr << " refuel=" << refuel_amount << "L time=" << pt << "s\n";
                }
            }
        }

        lap_plans.push_back(lp);
    }

    // Compute scoring
    double total_refueled = 0;
    for (auto &lp : lap_plans)
        if (lp.pit.enter)
            total_refueled += lp.pit.fuel_refuel_amount;
    double fuel_burned = rd.car.initial_fuel + total_refueled - fuel_remaining;

    double total_degradation = 0;
    int blowouts = 0;
    for (auto &[id, ss] : all_sets)
    {
        if (used_set_ids.count(id))
        {
            total_degradation += ss.degradation;
            if (ss.blown)
                blowouts++;
        }
    }

    double base_score = calc_base_score(rd.race.time_reference, elapsed_time);
    double fuel_bonus = calc_fuel_bonus(fuel_burned, rd.race.fuel_soft_cap);
    double tyre_bonus = calc_tyre_bonus(total_degradation, blowouts);
    double final_score = base_score + fuel_bonus + tyre_bonus;

    std::cerr << std::fixed << std::setprecision(4);
    std::cerr << "Total time: " << elapsed_time << " s\n";
    std::cerr << "Fuel burned: " << fuel_burned << " L (cap=" << rd.race.fuel_soft_cap << ")\n";
    std::cerr << "Tyre degradation sum: " << total_degradation << " | blowouts: " << blowouts << "\n";
    std::cerr << "Base score: " << base_score << "\n";
    std::cerr << "Fuel bonus: " << fuel_bonus << "\n";
    std::cerr << "Tyre bonus: " << tyre_bonus << "\n";
    std::cerr << "Final score: " << final_score << "\n";

    std::cout << generate_output(initial_tyre.id, rd.segments, lap_plans, rd.race.laps).dump(2) << std::endl;
    return 0;
}
