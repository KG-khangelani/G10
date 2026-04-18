// Level 3: Weather adaptation + tyre swaps + fuel management (OPTIMIZED)
// Key optimizations:
// 1. Search the initial tyre choice instead of picking the best immediate weather tyre.
// 2. Per-lap per-straight target optimization.
// 3. Combine refuel + swap into single pit stops.
// 4. Only swap when the expected stint gain beats the pit cost.
// 5. No empty pit stops.
#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <limits>
#include "parser.h"
#include "physics.h"
#include "output.h"

struct LapEstimate
{
    double time = 0.0;
    double fuel = 0.0;
    double exit_speed = 0.0;
};

struct RaceEval
{
    double total_time = 0.0;
    double fuel_burned = 0.0;
    double base_score = 0.0;
    double fuel_bonus = 0.0;
    double final_score = -1e18;
    int initial_tyre_id = 0;
    std::string initial_compound;
    std::vector<LapPlan> lap_plans;
};

static double time_until_weather_change(const std::vector<WeatherCondition> &conditions, double elapsed)
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

static std::string find_compound_for_tyre_id(const RaceData &rd, int tyre_id)
{
    for (auto &ts : rd.tyre_sets)
    {
        for (int id : ts.ids)
        {
            if (id == tyre_id)
                return ts.compound;
        }
    }
    return "";
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

static double choose_refuel_amount(const RaceData &rd,
                                   double fuel_remaining,
                                   double next_fuel_est,
                                   int laps_left)
{
    if (laps_left <= 0)
        return 0.0;

    double projected_need = next_fuel_est * laps_left * 1.05;
    double target_fuel = std::min(rd.car.fuel_capacity, projected_need);
    double amount = std::max(0.0, target_fuel - fuel_remaining);
    amount = std::min(amount, rd.car.fuel_capacity - fuel_remaining);
    return std::ceil(amount * 100.0) / 100.0;
}

static RaceEval simulate_race(const RaceData &rd, int initial_tyre_id, bool build_plan)
{
    RaceEval eval;
    eval.initial_tyre_id = initial_tyre_id;
    eval.initial_compound = find_compound_for_tyre_id(rd, initial_tyre_id);
    if (build_plan)
        eval.lap_plans.reserve(rd.race.laps);

    double current_speed = 0.0;
    double fuel_remaining = rd.car.initial_fuel;
    double elapsed_time = 0.0;
    double total_refueled = 0.0;
    int current_tyre_id = initial_tyre_id;
    std::string current_compound = eval.initial_compound;
    double last_lap_time = 250.0;

    for (int lap = 0; lap < rd.race.laps; lap++)
    {
        auto &wc = get_weather_at_time(rd.weather, elapsed_time);
        std::string weather_now = wc.condition;
        double accel_eff = rd.car.accel * wc.accel_mult;
        double brake_eff = rd.car.brake_decel * wc.decel_mult;

        auto &tp = rd.tyre_props.at(current_compound);
        double friction = tp.life_span * get_friction_multiplier(tp, weather_now);

        auto required_speed = resolve_corner_chains(rd.segments, friction,
                                                    rd.car.crawl_speed, rd.car.max_speed);
        auto target_speeds = optimize_targets(rd, required_speed, current_speed,
                                              accel_eff, brake_eff);
        auto straight_plans = compute_straight_plans(rd.segments, required_speed, target_speeds,
                                                     rd.car.max_speed, brake_eff);
        LapEstimate lap_est = simulate_lap(rd, required_speed, straight_plans,
                                           current_speed, accel_eff, brake_eff);

        elapsed_time += lap_est.time;
        fuel_remaining -= lap_est.fuel;
        current_speed = lap_est.exit_speed;
        last_lap_time = lap_est.time;

        LapPlan lp;
        lp.straight_plans = straight_plans;
        lp.pit = {false, 0, 0.0};

        if (lap < rd.race.laps - 1)
        {
            std::string next_weather = get_weather_string_at_time(rd.weather, elapsed_time);
            auto &next_wc = get_weather_at_time(rd.weather, elapsed_time);
            double next_accel = rd.car.accel * next_wc.accel_mult;
            double next_brake = rd.car.brake_decel * next_wc.decel_mult;
            auto next_best = select_best_tyre(rd, next_weather);

            bool want_swap = false;
            if (next_best.compound != current_compound)
            {
                auto &cur_tp = rd.tyre_props.at(current_compound);
                double cur_fmult = get_friction_multiplier(cur_tp, next_weather);
                double cur_friction = cur_tp.life_span * cur_fmult;
                double best_friction = next_best.friction;

                if (best_friction > cur_friction * 1.10)
                {
                    double weather_remaining = time_until_weather_change(rd.weather, elapsed_time);
                    int race_laps_left = rd.race.laps - lap - 1;
                    double est_laps = std::min(weather_remaining / last_lap_time,
                                               static_cast<double>(race_laps_left));
                    if (est_laps >= 5.0)
                        want_swap = true;
                }
            }

            auto &next_tp = rd.tyre_props.at(want_swap ? next_best.compound : current_compound);
            double next_fmult = get_friction_multiplier(next_tp, next_weather);
            double next_friction = next_tp.life_span * next_fmult;
            auto next_req = resolve_corner_chains(rd.segments, next_friction,
                                                  rd.car.crawl_speed, rd.car.max_speed);
            double entry_sp = want_swap ? rd.race.pit_exit_speed : current_speed;
            auto next_targets = optimize_targets(rd, next_req, entry_sp,
                                                 next_accel, next_brake);
            auto next_plans = compute_straight_plans(rd.segments, next_req, next_targets,
                                                     rd.car.max_speed, next_brake);
            double next_fuel_est = simulate_lap(rd, next_req, next_plans,
                                                entry_sp, next_accel, next_brake)
                                       .fuel;

            bool need_refuel = fuel_remaining < next_fuel_est * 1.05;

            if (!need_refuel && want_swap)
            {
                double laps_until_empty = fuel_remaining / next_fuel_est;
                if (laps_until_empty <= 3.0)
                    need_refuel = true;
            }

            if (need_refuel && !want_swap && next_best.compound != current_compound)
            {
                auto &cur_tp = rd.tyre_props.at(current_compound);
                double cur_fmult = get_friction_multiplier(cur_tp, next_weather);
                double cur_friction = cur_tp.life_span * cur_fmult;
                if (next_best.friction > cur_friction * 1.05)
                    want_swap = true;
            }

            double refuel_amount = 0.0;
            if (need_refuel)
            {
                int laps_left = rd.race.laps - lap - 1;
                refuel_amount = choose_refuel_amount(rd, fuel_remaining, next_fuel_est, laps_left);
            }

            if (want_swap || refuel_amount > 0.0)
            {
                double pit_time = pit_stop_time(refuel_amount, rd.race.pit_refuel_rate,
                                                rd.race.pit_tyre_swap_time, rd.race.base_pit_time,
                                                want_swap);
                elapsed_time += pit_time;
                fuel_remaining += refuel_amount;
                total_refueled += refuel_amount;
                current_speed = rd.race.pit_exit_speed;

                int swap_id = want_swap ? next_best.id : 0;
                lp.pit = {true, swap_id, refuel_amount};

                if (want_swap)
                {
                    current_compound = next_best.compound;
                    current_tyre_id = next_best.id;
                }
            }
        }

        if (build_plan)
            eval.lap_plans.push_back(lp);
    }

    eval.total_time = elapsed_time;
    eval.fuel_burned = rd.car.initial_fuel + total_refueled - fuel_remaining;
    eval.base_score = calc_base_score(rd.race.time_reference, eval.total_time);
    eval.fuel_bonus = calc_fuel_bonus(eval.fuel_burned, rd.race.fuel_soft_cap);
    eval.final_score = eval.base_score + eval.fuel_bonus;
    return eval;
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

    RaceEval best_eval;
    for (auto &ts : rd.tyre_sets)
    {
        for (int id : ts.ids)
        {
            RaceEval candidate = simulate_race(rd, id, false);
            if (candidate.final_score > best_eval.final_score)
                best_eval = candidate;
        }
    }

    RaceEval final_eval = simulate_race(rd, best_eval.initial_tyre_id, true);

    std::cerr << std::fixed << std::setprecision(4);
    std::cerr << "Initial tyre: " << final_eval.initial_compound << " (id=" << final_eval.initial_tyre_id << ")\n";
    std::cerr << "Total time: " << final_eval.total_time << " s\n";
    std::cerr << "Fuel burned: " << final_eval.fuel_burned << " L (cap=" << rd.race.fuel_soft_cap << ")\n";
    std::cerr << "Base score: " << final_eval.base_score << "\n";
    std::cerr << "Fuel bonus: " << final_eval.fuel_bonus << "\n";
    std::cerr << "Final score: " << final_eval.final_score << "\n";

    std::cout << generate_output(final_eval.initial_tyre_id, rd.segments, final_eval.lap_plans, rd.race.laps).dump(2) << std::endl;
    return 0;
}
