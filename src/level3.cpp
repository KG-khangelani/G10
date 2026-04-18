// Level 3: Weather adaptation + tyre swaps + fuel management (OPTIMIZED)
// Key optimizations:
// 1. Only swap tyres when friction gain > 10% AND enough laps in new weather
// 2. Combine refuel + swap into single pit stops
// 3. Look-ahead to combine upcoming fuel stops with tyre swaps
// 4. When pitting for fuel, add cheap swap if beneficial (+3s only)
// 5. No empty pit stops
// Scoring: base_score + fuel_bonus
#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include "parser.h"
#include "physics.h"
#include "output.h"

struct LapEstimate
{
    double time = 0.0;
    double fuel = 0.0;
    double exit_speed = 0.0;
};

// Compute time remaining in current weather condition
double time_until_weather_change(const std::vector<WeatherCondition> &conditions, double elapsed)
{
    if (conditions.empty())
        return 1e18;
    double total_cycle = 0;
    for (auto &wc : conditions)
        total_cycle += wc.duration;
    double t = std::fmod(elapsed, total_cycle);
    if (t < 0)
        t += total_cycle;
    double acc = 0;
    for (auto &wc : conditions)
    {
        acc += wc.duration;
        if (t < acc)
            return acc - t;
    }
    return 0;
}

static std::vector<int> get_straight_indices(const RaceData &rd)
{
    std::vector<int> indices;
    for (int i = 0; i < static_cast<int>(rd.segments.size()); i++)
    {
        if (rd.segments[i].type == "straight")
            indices.push_back(i);
    }
    return indices;
}

static LapEstimate simulate_lap(const RaceData &rd,
                                const std::vector<double> &required_speed,
                                const std::vector<StraightPlan> &straight_plans,
                                double entry_speed,
                                double accel_eff,
                                double brake_eff)
{
    LapEstimate estimate;
    int n = static_cast<int>(rd.segments.size());
    double speed = entry_speed;

    for (int i = 0; i < n; i++)
    {
        if (rd.segments[i].type == "straight")
        {
            auto res = simulate_straight(speed, straight_plans[i].target_speed,
                                         straight_plans[i].brake_start, rd.segments[i].length,
                                         accel_eff, brake_eff,
                                         rd.car.max_speed, rd.car.crawl_speed);
            estimate.time += res.time;
            estimate.fuel += res.fuel;
            speed = res.exit_speed;
        }
        else
        {
            double cs = std::min(speed, required_speed[i]);
            cs = std::max(cs, rd.car.crawl_speed);
            estimate.time += rd.segments[i].length / cs;
            estimate.fuel += fuel_usage(cs, cs, rd.segments[i].length);
            speed = cs;
        }
    }

    estimate.exit_speed = speed;
    return estimate;
}

static std::vector<double> optimize_targets(const RaceData &rd,
                                            const std::vector<double> &required_speed,
                                            double entry_speed,
                                            double accel_eff,
                                            double brake_eff)
{
    int n = static_cast<int>(rd.segments.size());
    std::vector<double> targets(n, 0.0);
    for (int i = 0; i < n; i++)
    {
        if (rd.segments[i].type == "straight")
            targets[i] = rd.car.max_speed;
    }

    auto straight_indices = get_straight_indices(rd);
    auto eval_time = [&](const std::vector<double> &candidate)
    {
        auto plans = compute_straight_plans(rd.segments, required_speed, candidate,
                                            rd.car.max_speed, brake_eff);
        return simulate_lap(rd, required_speed, plans, entry_speed, accel_eff, brake_eff).time;
    };

    double best_time = eval_time(targets);
    const std::vector<double> steps = {8.0, 2.0, 0.5, 0.1};

    for (double step : steps)
    {
        bool improved = true;
        while (improved)
        {
            improved = false;
            for (int idx : straight_indices)
            {
                double current = targets[idx];
                int next = (idx + 1) % n;
                double floor_target = std::max(required_speed[next], rd.car.crawl_speed);
                double local_best_target = current;
                double local_best_time = best_time;

                double local_lo = std::max(floor_target, current - step * 4.0);
                double local_hi = std::min(rd.car.max_speed, current + step * 2.0);

                for (double ts = local_lo; ts <= local_hi + 1e-9; ts += step)
                {
                    std::vector<double> candidate = targets;
                    candidate[idx] = ts;
                    double candidate_time = eval_time(candidate);
                    if (candidate_time + 1e-9 < local_best_time)
                    {
                        local_best_time = candidate_time;
                        local_best_target = ts;
                    }
                }

                if (std::abs(local_best_target - current) > 1e-9)
                {
                    targets[idx] = local_best_target;
                    best_time = local_best_time;
                    improved = true;
                }
            }
        }
    }

    return targets;
}

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

    // Forward simulate lap by lap
    std::vector<LapPlan> lap_plans;
    double current_speed = 0.0;
    double fuel_remaining = rd.car.initial_fuel;
    double elapsed_time = 0.0;

    // Select initial tyre
    auto initial = select_best_tyre(rd, rd.starting_weather);
    int current_tyre_id = initial.id;
    std::string current_compound = initial.compound;
    double last_lap_time = 250.0; // initial estimate

    std::cerr << "Initial tyre: " << current_compound << " (id=" << current_tyre_id << ")\n";

    for (int lap = 0; lap < rd.race.laps; lap++)
    {
        // Get weather for this lap
        auto &wc = get_weather_at_time(rd.weather, elapsed_time);
        std::string weather_now = wc.condition;
        double accel_eff = rd.car.accel * wc.accel_mult;
        double brake_eff = rd.car.brake_decel * wc.decel_mult;

        // Current tyre friction for this weather
        auto &tp = rd.tyre_props[current_compound];
        double friction_mult = get_friction_multiplier(tp, weather_now);
        double friction = tp.life_span * friction_mult;

        // Resolve corners and compute plans
        auto required_speed = resolve_corner_chains(
            rd.segments, friction, rd.car.crawl_speed, rd.car.max_speed);
        auto target_speeds = optimize_targets(rd, required_speed, current_speed,
                                              accel_eff, brake_eff);
        auto straight_plans = compute_straight_plans(rd.segments, required_speed, target_speeds,
                                                     rd.car.max_speed, brake_eff);
        LapEstimate lap_est = simulate_lap(rd, required_speed, straight_plans,
                                           current_speed, accel_eff, brake_eff);
        double lap_time = lap_est.time;
        double lap_fuel = lap_est.fuel;
        double speed = lap_est.exit_speed;

        elapsed_time += lap_time;
        fuel_remaining -= lap_fuel;
        current_speed = speed;
        last_lap_time = lap_time;

        // Build lap plan
        LapPlan lp;
        lp.straight_plans = straight_plans;
        lp.pit = {false, 0, 0};

        // ── Pit decisions (end of lap) ──
        if (lap < rd.race.laps - 1)
        {
            std::string next_weather = get_weather_string_at_time(rd.weather, elapsed_time);
            auto &next_wc = get_weather_at_time(rd.weather, elapsed_time);
            double next_accel = rd.car.accel * next_wc.accel_mult;
            double next_brake = rd.car.brake_decel * next_wc.decel_mult;
            auto next_best = select_best_tyre(rd, next_weather);

            // Evaluate tyre swap: friction gain must be > 10%
            bool want_swap = false;
            if (next_best.compound != current_compound)
            {
                auto &cur_tp = rd.tyre_props[current_compound];
                double cur_fmult = get_friction_multiplier(cur_tp, next_weather);
                double cur_friction = cur_tp.life_span * cur_fmult;
                double best_friction = next_best.friction;

                if (best_friction > cur_friction * 1.10)
                {
                    // Check if enough laps remain (min of weather duration and race end)
                    double weather_remaining = time_until_weather_change(rd.weather, elapsed_time);
                    int race_laps_left = rd.race.laps - lap - 1;
                    double est_laps = std::min(weather_remaining / last_lap_time,
                                               (double)race_laps_left);
                    if (est_laps >= 5.0)
                    {
                        want_swap = true;
                    }
                }
            }

            // Estimate fuel for next lap (using whichever tyre we'll have)
            auto &next_tp = rd.tyre_props[want_swap ? next_best.compound : current_compound];
            double next_fmult = get_friction_multiplier(next_tp, next_weather);
            double next_friction = next_tp.life_span * next_fmult;
            auto next_req = resolve_corner_chains(rd.segments, next_friction,
                                                  rd.car.crawl_speed, rd.car.max_speed);
            double entry_sp = want_swap ? rd.race.pit_exit_speed : speed;
            auto next_targets = optimize_targets(rd, next_req, entry_sp,
                                                 next_accel, next_brake);
            auto next_plans = compute_straight_plans(rd.segments, next_req, next_targets,
                                                     rd.car.max_speed, next_brake);
            double next_fuel_est = simulate_lap(rd, next_req, next_plans,
                                                entry_sp, next_accel, next_brake)
                                       .fuel;

            bool need_refuel = fuel_remaining < next_fuel_est * 1.05;

            // Look ahead: if fuel needed within 3 laps AND swap is wanted, combine now
            if (!need_refuel && want_swap)
            {
                double laps_until_empty = fuel_remaining / next_fuel_est;
                if (laps_until_empty <= 3.0)
                    need_refuel = true;
            }

            // If pitting for fuel anyway, check if adding a cheap swap (+3s) is beneficial
            if (need_refuel && !want_swap && next_best.compound != current_compound)
            {
                auto &cur_tp = rd.tyre_props[current_compound];
                double cur_fmult = get_friction_multiplier(cur_tp, next_weather);
                double cur_friction = cur_tp.life_span * cur_fmult;
                // Lower threshold since swap only costs +3s when already pitting
                if (next_best.friction > cur_friction * 1.05)
                    want_swap = true;
            }

            // Compute refuel amount
            double refuel_amount = 0;
            if (need_refuel)
            {
                // Refuel to capacity (minimize future pit stops)
                refuel_amount = rd.car.fuel_capacity - fuel_remaining;
                refuel_amount = std::max(0.0, refuel_amount);
                refuel_amount = std::ceil(refuel_amount * 100.0) / 100.0;
            }

            // Only pit if there's actual work to do
            if (want_swap || refuel_amount > 0)
            {
                double pt = pit_stop_time(refuel_amount, rd.race.pit_refuel_rate,
                                          rd.race.pit_tyre_swap_time, rd.race.base_pit_time,
                                          want_swap);
                elapsed_time += pt;
                fuel_remaining += refuel_amount;
                current_speed = rd.race.pit_exit_speed;

                int swap_id = want_swap ? next_best.id : 0;
                lp.pit = {true, swap_id, refuel_amount};

                if (want_swap)
                {
                    current_compound = next_best.compound;
                    current_tyre_id = next_best.id;
                }

                std::cerr << "Pit after lap " << lap + 1
                          << ": swap=" << (want_swap ? next_best.compound : "no")
                          << " refuel=" << refuel_amount << "L time=" << pt << "s\n";
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

    double base_score = calc_base_score(rd.race.time_reference, elapsed_time);
    double fuel_bonus = calc_fuel_bonus(fuel_burned, rd.race.fuel_soft_cap);
    double final_score = base_score + fuel_bonus;

    std::cerr << std::fixed << std::setprecision(4);
    std::cerr << "Total time: " << elapsed_time << " s\n";
    std::cerr << "Fuel burned: " << fuel_burned << " L (cap=" << rd.race.fuel_soft_cap << ")\n";
    std::cerr << "Base score: " << base_score << "\n";
    std::cerr << "Fuel bonus: " << fuel_bonus << "\n";
    std::cerr << "Final score: " << final_score << "\n";

    std::cout << generate_output(initial.id, rd.segments, lap_plans, rd.race.laps).dump(2) << std::endl;
    return 0;
}
