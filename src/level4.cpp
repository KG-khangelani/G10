// Level 4: Full simulation — tyre degradation + weather + fuel + multi-set pit strategy (OPTIMIZED)
// Key optimizations:
// 1. Search the starting tyre set instead of assuming the best current-weather tyre is globally best.
// 2. Per-lap per-straight target optimization.
// 3. Minimum stint guard to avoid tyre ping-pong.
// 4. Refuel only to projected remaining race demand instead of blindly filling the tank.
// 5. Better set selection: prefer sets with enough remaining life for the next stint.
#include <iostream>
#include <iomanip>
#include <cmath>
#include <set>
#include <algorithm>
#include <vector>
#include <limits>
#include "parser.h"
#include "physics.h"
#include "output.h"

#ifndef LEVEL4_LOOKAHEAD_HORIZON
#define LEVEL4_LOOKAHEAD_HORIZON 2
#endif

#ifndef LEVEL4_TYRE_DANGER_THRESHOLD
#define LEVEL4_TYRE_DANGER_THRESHOLD 0.93
#endif

#ifndef LEVEL4_TYRE_DANGER_SOON_LAPS
#define LEVEL4_TYRE_DANGER_SOON_LAPS 2.0
#endif

#ifndef LEVEL4_SWAP_GAIN_THRESHOLD
#define LEVEL4_SWAP_GAIN_THRESHOLD 1.05
#endif

#ifndef LEVEL4_WEATHER_MIN_LAPS
#define LEVEL4_WEATHER_MIN_LAPS 4.0
#endif

struct SetState
{
    int id;
    std::string compound;
    double degradation;
    double life_span;
    bool blown;
};

struct LapEstimate
{
    double time = 0.0;
    double fuel = 0.0;
    double exit_speed = 0.0;
    double degradation = 0.0;
    bool limp_mode = false;
    bool blown = false;
};

struct RaceEval
{
    double total_time = 0.0;
    double fuel_burned = 0.0;
    double total_degradation = 0.0;
    double base_score = 0.0;
    double fuel_bonus = 0.0;
    double tyre_bonus = 0.0;
    double final_score = -1e18;
    int blowouts = 0;
    int initial_set_id = 0;
    std::string initial_compound;
    std::size_t sets_used = 0;
    std::vector<LapPlan> lap_plans;
};

struct WindowEval
{
    bool valid = false;
    double objective = -1e18;
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

static int find_best_set(const std::map<int, SetState> &all_sets,
                         const std::vector<TyreSet> &tyre_sets,
                         const std::string &target_compound,
                         int exclude_id,
                         double min_remaining = 0.15)
{
    int best_id = -1;
    double best_remaining = -1;

    for (auto &ts : tyre_sets)
    {
        if (ts.compound != target_compound)
            continue;
        for (int id : ts.ids)
        {
            if (id == exclude_id)
                continue;
            auto &ss = all_sets.at(id);
            if (ss.blown)
                continue;
            double remaining = ss.life_span - ss.degradation;
            if (remaining < min_remaining)
                continue;
            if (remaining > best_remaining)
            {
                best_remaining = remaining;
                best_id = id;
            }
        }
    }
    return best_id;
}

static int find_any_set(const std::map<int, SetState> &all_sets,
                        const std::vector<TyreSet> &tyre_sets,
                        int exclude_id,
                        double min_remaining = 0.10)
{
    int best_id = -1;
    double best_remaining = -1;

    for (auto &ts : tyre_sets)
    {
        for (int id : ts.ids)
        {
            if (id == exclude_id)
                continue;
            auto &ss = all_sets.at(id);
            if (ss.blown)
                continue;
            double remaining = ss.life_span - ss.degradation;
            if (remaining < min_remaining)
                continue;
            if (remaining > best_remaining)
            {
                best_remaining = remaining;
                best_id = id;
            }
        }
    }
    return best_id;
}

static int find_best_set_for_weather(const std::map<int, SetState> &all_sets,
                                     const RaceData &rd,
                                     const std::string &weather,
                                     int exclude_id,
                                     double min_remaining = 0.15)
{
    struct CompScore
    {
        std::string compound;
        double eff_friction;
    };

    std::vector<CompScore> ranked;
    for (auto &[compound, tp] : rd.tyre_props)
    {
        double mult = get_friction_multiplier(tp, weather);
        ranked.push_back({compound, tp.life_span * mult});
    }
    std::sort(ranked.begin(), ranked.end(),
              [](const CompScore &left, const CompScore &right)
              { return left.eff_friction > right.eff_friction; });

    for (auto &candidate : ranked)
    {
        auto &tp = rd.tyre_props.at(candidate.compound);
        double deg_rate = get_degradation_rate(tp, weather);
        double compound_min = std::max(deg_rate * 3.0, min_remaining);
        int id = find_best_set(all_sets, rd.tyre_sets, candidate.compound, exclude_id, compound_min);
        if (id >= 0)
            return id;
    }

    return find_any_set(all_sets, rd.tyre_sets, exclude_id, 0.01);
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

static std::string find_compound_for_set_id(const RaceData &rd, int set_id)
{
    for (auto &ts : rd.tyre_sets)
    {
        for (int id : ts.ids)
        {
            if (id == set_id)
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
                                double brake_eff,
                                double deg_rate,
                                double fuel_available,
                                double current_degradation,
                                double tyre_life_span)
{
    LapEstimate estimate;
    int n = static_cast<int>(rd.segments.size());
    double speed = entry_speed;

    for (int i = 0; i < n; i++)
    {
        if (estimate.limp_mode)
        {
            double time = rd.segments[i].length / rd.car.limp_speed;
            estimate.time += time;
            estimate.fuel += fuel_usage(rd.car.limp_speed, rd.car.limp_speed, rd.segments[i].length);
            speed = rd.car.limp_speed;
            continue;
        }

        if (rd.segments[i].type == "straight")
        {
            auto res = simulate_straight(speed, straight_plans[i].target_speed,
                                         straight_plans[i].brake_start, rd.segments[i].length,
                                         accel_eff, brake_eff,
                                         rd.car.max_speed, rd.car.crawl_speed);
            estimate.time += res.time;
            estimate.fuel += res.fuel;
            estimate.degradation += tyre_deg_straight(deg_rate, rd.segments[i].length);
            if (res.distance_brake > 0 && res.speed_at_brake > res.exit_speed)
                estimate.degradation += tyre_deg_braking(deg_rate, res.speed_at_brake, res.exit_speed);
            speed = res.exit_speed;
        }
        else
        {
            double cs = std::min(speed, required_speed[i]);
            cs = std::max(cs, rd.car.crawl_speed);
            estimate.time += rd.segments[i].length / cs;
            estimate.fuel += fuel_usage(cs, cs, rd.segments[i].length);
            estimate.degradation += tyre_deg_corner(deg_rate, cs, rd.segments[i].radius);
            speed = cs;
        }

        if (fuel_available - estimate.fuel <= 0)
        {
            estimate.limp_mode = true;
            speed = rd.car.limp_speed;
        }
        if (current_degradation + estimate.degradation >= tyre_life_span)
        {
            estimate.limp_mode = true;
            estimate.blown = true;
            speed = rd.car.limp_speed;
        }
    }

    estimate.exit_speed = speed;
    return estimate;
}

static std::vector<double> optimize_targets(const RaceData &rd,
                                            const std::vector<double> &required_speed,
                                            double entry_speed,
                                            double accel_eff,
                                            double brake_eff,
                                            double deg_rate,
                                            double fuel_available,
                                            double current_degradation,
                                            double tyre_life_span)
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
        return simulate_lap(rd, required_speed, plans, entry_speed, accel_eff, brake_eff,
                            deg_rate, fuel_available, current_degradation, tyre_life_span)
            .time;
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

static LapEstimate simulate_state_lap(const RaceData &rd,
                                      double elapsed_time,
                                      double entry_speed,
                                      double fuel_available,
                                      const std::string &compound,
                                      double current_degradation)
{
    auto &wc = get_weather_at_time(rd.weather, elapsed_time);
    double accel_eff = rd.car.accel * wc.accel_mult;
    double brake_eff = rd.car.brake_decel * wc.decel_mult;

    auto &tp = rd.tyre_props.at(compound);
    double deg_rate = get_degradation_rate(tp, wc.condition);
    double friction_mult = get_friction_multiplier(tp, wc.condition);
    double tyre_friction = calc_tyre_friction(tp.life_span, current_degradation, friction_mult);
    tyre_friction = std::max(tyre_friction, 0.01);

    auto required_speed = resolve_corner_chains(rd.segments, tyre_friction,
                                                rd.car.crawl_speed, rd.car.max_speed);
    auto target_speeds = optimize_targets(rd, required_speed, entry_speed,
                                          accel_eff, brake_eff, deg_rate,
                                          fuel_available, current_degradation, tp.life_span);
    auto straight_plans = compute_straight_plans(rd.segments, required_speed, target_speeds,
                                                 rd.car.max_speed, brake_eff);
    return simulate_lap(rd, required_speed, straight_plans,
                        entry_speed, accel_eff, brake_eff, deg_rate,
                        fuel_available, current_degradation, tp.life_span);
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

static WindowEval evaluate_future_window(const RaceData &rd,
                                         int current_lap,
                                         int window_laps,
                                         double elapsed_time,
                                         double fuel_remaining,
                                         double total_refueled,
                                         double current_speed,
                                         std::map<int, SetState> all_sets,
                                         int current_set_id,
                                         std::string current_compound,
                                         double current_degradation)
{
    WindowEval eval;
    double future_fuel_burned = 0.0;
    double future_degradation = 0.0;
    int future_blowouts = 0;

    for (int step = 0; step < window_laps; step++)
    {
        LapEstimate lap_est = simulate_state_lap(rd, elapsed_time, current_speed,
                                                 fuel_remaining, current_compound,
                                                 current_degradation);

        elapsed_time += lap_est.time;
        fuel_remaining -= lap_est.fuel;
        future_fuel_burned += lap_est.fuel;
        current_degradation += lap_est.degradation;
        future_degradation += lap_est.degradation;
        current_speed = lap_est.exit_speed;
        all_sets[current_set_id].degradation = current_degradation;

        if (lap_est.blown && !all_sets[current_set_id].blown)
        {
            all_sets[current_set_id].blown = true;
            future_blowouts++;
        }

        if (current_lap + step + 1 >= rd.race.laps - 1)
            break;
    }

    double fuel_used_before_window = rd.car.initial_fuel + total_refueled - fuel_remaining - future_fuel_burned;
    double projected_fuel_used = fuel_used_before_window + future_fuel_burned;
    eval.objective = calc_base_score(rd.race.time_reference, elapsed_time) +
                     calc_fuel_bonus(projected_fuel_used, rd.race.fuel_soft_cap) +
                     100000.0 * future_degradation - 50000.0 * future_blowouts;
    eval.valid = true;
    return eval;
}

static RaceEval simulate_race(const RaceData &rd, int initial_set_id, bool build_plan)
{
    RaceEval eval;
    eval.initial_set_id = initial_set_id;
    eval.initial_compound = find_compound_for_set_id(rd, initial_set_id);
    if (eval.initial_compound.empty())
        return eval;
    if (build_plan)
        eval.lap_plans.reserve(rd.race.laps);

    std::map<int, SetState> all_sets;
    for (auto &ts : rd.tyre_sets)
    {
        auto &tp = rd.tyre_props.at(ts.compound);
        for (int id : ts.ids)
            all_sets[id] = {id, ts.compound, 0.0, tp.life_span, false};
    }

    double current_speed = 0.0;
    double fuel_remaining = rd.car.initial_fuel;
    double elapsed_time = 0.0;
    double total_refueled = 0.0;

    int current_set_id = initial_set_id;
    std::string current_compound = eval.initial_compound;
    double current_degradation = 0.0;
    std::set<int> used_set_ids;
    used_set_ids.insert(current_set_id);
    int laps_on_current_set = 0;
    double last_lap_time = 420.0;

    for (int lap = 0; lap < rd.race.laps; lap++)
    {
        auto &wc = get_weather_at_time(rd.weather, elapsed_time);
        std::string weather_now = wc.condition;
        double accel_eff = rd.car.accel * wc.accel_mult;
        double brake_eff = rd.car.brake_decel * wc.decel_mult;

        auto &tp = rd.tyre_props.at(current_compound);
        double deg_rate = get_degradation_rate(tp, weather_now);
        double friction_mult = get_friction_multiplier(tp, weather_now);
        double tyre_friction = calc_tyre_friction(tp.life_span, current_degradation, friction_mult);
        if (tyre_friction < 0.01)
            tyre_friction = 0.01;

        auto required_speed = resolve_corner_chains(rd.segments, tyre_friction,
                                                    rd.car.crawl_speed, rd.car.max_speed);
        auto target_speeds = optimize_targets(rd, required_speed, current_speed,
                                              accel_eff, brake_eff, deg_rate,
                                              fuel_remaining, current_degradation, tp.life_span);
        auto straight_plans = compute_straight_plans(rd.segments, required_speed, target_speeds,
                                                     rd.car.max_speed, brake_eff);
        LapEstimate lap_est = simulate_lap(rd, required_speed, straight_plans,
                                           current_speed, accel_eff, brake_eff, deg_rate,
                                           fuel_remaining, current_degradation, tp.life_span);

        elapsed_time += lap_est.time;
        fuel_remaining -= lap_est.fuel;
        current_degradation += lap_est.degradation;
        all_sets[current_set_id].degradation = current_degradation;
        current_speed = lap_est.exit_speed;
        laps_on_current_set++;
        last_lap_time = lap_est.time;
        if (lap_est.blown)
            all_sets[current_set_id].blown = true;

        LapPlan lp;
        lp.straight_plans = straight_plans;
        lp.pit = {false, 0, 0.0};

        if (lap < rd.race.laps - 1)
        {
            std::string next_weather = get_weather_string_at_time(rd.weather, elapsed_time);
            auto &cur_tp = rd.tyre_props.at(current_compound);
            double next_fmult = get_friction_multiplier(cur_tp, next_weather);
            double next_friction = calc_tyre_friction(cur_tp.life_span, current_degradation, next_fmult);

            double est_next_deg = lap_est.degradation * 1.15;
            bool tyre_danger = (current_degradation + est_next_deg >= cur_tp.life_span * LEVEL4_TYRE_DANGER_THRESHOLD);
            bool tyre_danger_soon = (current_degradation + est_next_deg * LEVEL4_TYRE_DANGER_SOON_LAPS >= cur_tp.life_span * LEVEL4_TYRE_DANGER_THRESHOLD);
            if (lap_est.limp_mode)
                tyre_danger = true;

            LapEstimate stay_next_lap = simulate_state_lap(rd, elapsed_time, current_speed,
                                                           fuel_remaining, current_compound,
                                                           current_degradation);
            bool need_refuel = fuel_remaining < stay_next_lap.fuel * 1.05;

            bool allow_swap_candidate = lap_est.limp_mode || laps_on_current_set >= 4 || tyre_danger || tyre_danger_soon;
            int preferred_swap_id = 0;
            LapEstimate swap_next_lap;
            bool swap_candidate_valid = false;

            if (allow_swap_candidate)
            {
                double min_life = std::max(lap_est.degradation * 2.5, 0.15);
                preferred_swap_id = find_best_set_for_weather(all_sets, rd, next_weather, current_set_id, min_life);
                if (preferred_swap_id > 0)
                {
                    std::string swap_compound = all_sets.at(preferred_swap_id).compound;
                    swap_next_lap = simulate_state_lap(rd, elapsed_time + rd.race.base_pit_time,
                                                       rd.race.pit_exit_speed, rd.car.fuel_capacity,
                                                       swap_compound, all_sets.at(preferred_swap_id).degradation);
                    swap_candidate_valid = true;
                }
            }

            bool weather_opportunity = false;
            if (swap_candidate_valid && laps_on_current_set >= 4)
            {
                auto &new_ss = all_sets.at(preferred_swap_id);
                auto &new_tp = rd.tyre_props.at(new_ss.compound);
                double new_fmult = get_friction_multiplier(new_tp, next_weather);
                double new_friction = calc_tyre_friction(new_tp.life_span, new_ss.degradation, new_fmult);
                if (new_friction > std::max(next_friction, 0.01) * LEVEL4_SWAP_GAIN_THRESHOLD)
                {
                    double weather_remaining = time_until_weather_change(rd.weather, elapsed_time);
                    int race_laps_left = rd.race.laps - lap - 1;
                    double est_laps = std::min(weather_remaining / last_lap_time,
                                               static_cast<double>(race_laps_left));
                    if (est_laps >= LEVEL4_WEATHER_MIN_LAPS)
                        weather_opportunity = true;
                }
            }

            if (need_refuel || tyre_danger || tyre_danger_soon || weather_opportunity)
            {
                struct PitChoice
                {
                    bool valid = false;
                    double objective = -1e18;
                    int swap_id = 0;
                    double refuel_amount = 0.0;
                };

                auto evaluate_choice = [&](int swap_id, double refuel_amount)
                {
                    PitChoice choice;
                    double candidate_elapsed = elapsed_time;
                    double candidate_fuel = fuel_remaining;
                    double candidate_refueled = total_refueled;
                    double candidate_speed = current_speed;
                    int candidate_set_id = current_set_id;
                    std::string candidate_compound = current_compound;
                    double candidate_degradation = current_degradation;
                    std::map<int, SetState> candidate_sets = all_sets;

                    if (swap_id > 0 || refuel_amount > 0.0)
                    {
                        double pit_time = pit_stop_time(refuel_amount, rd.race.pit_refuel_rate,
                                                        rd.race.pit_tyre_swap_time, rd.race.base_pit_time,
                                                        swap_id > 0);
                        candidate_elapsed += pit_time;
                        candidate_fuel += refuel_amount;
                        candidate_refueled += refuel_amount;
                        candidate_speed = rd.race.pit_exit_speed;

                        if (swap_id > 0)
                        {
                            candidate_set_id = swap_id;
                            candidate_compound = candidate_sets.at(swap_id).compound;
                            candidate_degradation = candidate_sets.at(swap_id).degradation;
                        }
                    }

                    WindowEval window = evaluate_future_window(rd, lap,
                                                               std::min(LEVEL4_LOOKAHEAD_HORIZON, rd.race.laps - lap - 1),
                                                               candidate_elapsed, candidate_fuel,
                                                               candidate_refueled, candidate_speed,
                                                               candidate_sets, candidate_set_id,
                                                               candidate_compound, candidate_degradation);
                    if (window.valid)
                    {
                        choice.valid = true;
                        choice.objective = window.objective;
                        choice.swap_id = swap_id;
                        choice.refuel_amount = refuel_amount;
                    }
                    return choice;
                };

                int laps_left = rd.race.laps - lap - 1;
                double fuel_only_amount = choose_refuel_amount(rd, fuel_remaining, stay_next_lap.fuel, laps_left);

                PitChoice best_choice = evaluate_choice(0, 0.0);
                if (fuel_only_amount > 0.0)
                {
                    PitChoice fuel_choice = evaluate_choice(0, fuel_only_amount);
                    if (fuel_choice.valid && fuel_choice.objective > best_choice.objective)
                        best_choice = fuel_choice;
                }
                if (swap_candidate_valid)
                {
                    PitChoice swap_choice = evaluate_choice(preferred_swap_id, 0.0);
                    if (swap_choice.valid && swap_choice.objective > best_choice.objective)
                        best_choice = swap_choice;

                    double fuel_swap_amount = choose_refuel_amount(rd, fuel_remaining, swap_next_lap.fuel, laps_left);
                    if (fuel_swap_amount > 0.0)
                    {
                        PitChoice combo_choice = evaluate_choice(preferred_swap_id, fuel_swap_amount);
                        if (combo_choice.valid && combo_choice.objective > best_choice.objective)
                            best_choice = combo_choice;
                    }
                }

                if (best_choice.swap_id > 0 || best_choice.refuel_amount > 0.0)
                {
                    double pit_time = pit_stop_time(best_choice.refuel_amount, rd.race.pit_refuel_rate,
                                                    rd.race.pit_tyre_swap_time, rd.race.base_pit_time,
                                                    best_choice.swap_id > 0);
                    elapsed_time += pit_time;
                    fuel_remaining += best_choice.refuel_amount;
                    total_refueled += best_choice.refuel_amount;
                    current_speed = rd.race.pit_exit_speed;
                    lp.pit = {true, best_choice.swap_id, best_choice.refuel_amount};

                    if (best_choice.swap_id > 0)
                    {
                        current_compound = all_sets.at(best_choice.swap_id).compound;
                        current_degradation = all_sets.at(best_choice.swap_id).degradation;
                        current_set_id = best_choice.swap_id;
                        used_set_ids.insert(best_choice.swap_id);
                        laps_on_current_set = 0;
                        if (build_plan)
                        {
                            std::cerr << "Pit lap " << lap + 1 << ": swap to " << current_compound
                                      << " (id=" << best_choice.swap_id << " deg=" << std::fixed << std::setprecision(3)
                                      << current_degradation << ")";
                        }
                    }
                    else if (build_plan)
                    {
                        std::cerr << "Pit lap " << lap + 1 << ": refuel only";
                    }

                    if (build_plan)
                        std::cerr << " refuel=" << best_choice.refuel_amount << "L time=" << pit_time << "s\n";
                }
            }
        }

        if (build_plan)
            eval.lap_plans.push_back(lp);
    }

    eval.total_time = elapsed_time;
    eval.fuel_burned = rd.car.initial_fuel + total_refueled - fuel_remaining;
    for (auto &[id, ss] : all_sets)
    {
        if (used_set_ids.count(id))
        {
            eval.total_degradation += ss.degradation;
            if (ss.blown)
                eval.blowouts++;
        }
    }
    eval.base_score = calc_base_score(rd.race.time_reference, eval.total_time);
    eval.fuel_bonus = calc_fuel_bonus(eval.fuel_burned, rd.race.fuel_soft_cap);
    eval.tyre_bonus = calc_tyre_bonus(eval.total_degradation, eval.blowouts);
    eval.final_score = eval.base_score + eval.fuel_bonus + eval.tyre_bonus;
    eval.sets_used = used_set_ids.size();
    return eval;
}

int main(int argc, char *argv[])
{
    std::string inputPath = "inputFiles/level4input.txt";
    if (argc > 1)
        inputPath = argv[1];

    RaceData rd = load_race_data(inputPath);
    int n = rd.segments.size();

    std::cerr << "Race: " << rd.race.name << " | " << rd.race.laps << " laps | "
              << n << " segments\n";

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

    RaceEval final_eval = simulate_race(rd, best_eval.initial_set_id, true);

    std::cerr << std::fixed << std::setprecision(4);
    std::cerr << "Initial tyre: " << final_eval.initial_compound << " (id=" << final_eval.initial_set_id << ")\n";
    std::cerr << "Total time: " << final_eval.total_time << " s\n";
    std::cerr << "Fuel burned: " << final_eval.fuel_burned << " L (cap=" << rd.race.fuel_soft_cap << ")\n";
    std::cerr << "Tyre degradation sum: " << final_eval.total_degradation << " | blowouts: " << final_eval.blowouts << "\n";
    std::cerr << "Sets used: " << final_eval.sets_used << "/9\n";
    std::cerr << "Base score: " << final_eval.base_score << "\n";
    std::cerr << "Fuel bonus: " << final_eval.fuel_bonus << "\n";
    std::cerr << "Tyre bonus: " << final_eval.tyre_bonus << "\n";
    std::cerr << "Final score: " << final_eval.final_score << "\n";

    std::cout << generate_output(final_eval.initial_set_id, rd.segments, final_eval.lap_plans, rd.race.laps).dump(2) << std::endl;
    return 0;
}
