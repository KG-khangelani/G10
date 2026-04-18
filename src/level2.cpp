// Level 2: Fuel management + pit stops for refueling
// Optimized for score = base_score + fuel_bonus
#include <iostream>
#include <iomanip>
#include <cmath>
#include <limits>
#include <vector>
#include "parser.h"
#include "physics.h"
#include "output.h"

struct RaceEval
{
    double total_time = 0.0;
    double fuel_burned = 0.0;
    double score = -1e18;
    std::vector<LapPlan> lap_plans;
};

struct LapEstimate
{
    double fuel = 0.0;
    double time = 0.0;
    double exit_speed = 0.0;
};

static LapEstimate estimate_lap(const RaceData &rd,
                                const std::vector<double> &required_speed,
                                const std::vector<StraightPlan> &plans,
                                double entry_speed)
{
    LapEstimate out;
    double speed = entry_speed;
    int n = static_cast<int>(rd.segments.size());

    for (int i = 0; i < n; i++)
    {
        if (rd.segments[i].type == "straight")
        {
            auto res = simulate_straight(speed, plans[i].target_speed,
                                         plans[i].brake_start, rd.segments[i].length,
                                         rd.car.accel, rd.car.brake_decel,
                                         rd.car.max_speed, rd.car.crawl_speed);
            out.time += res.time;
            out.fuel += res.fuel;
            speed = res.exit_speed;
        }
        else
        {
            double cs = std::min(speed, required_speed[i]);
            cs = std::max(cs, rd.car.crawl_speed);
            out.time += rd.segments[i].length / cs;
            out.fuel += fuel_usage(cs, cs, rd.segments[i].length);
            speed = cs;
        }
    }

    out.exit_speed = speed;
    return out;
}

static RaceEval simulate_race(const RaceData &rd,
                              const std::vector<double> &required_speed,
                              const std::vector<StraightPlan> &plans,
                              bool build_plan)
{
    RaceEval eval;
    double current_speed = 0.0;
    double fuel_remaining = rd.car.initial_fuel;
    double total_refueled = 0.0;

    int laps = rd.race.laps;

    for (int lap = 0; lap < laps; lap++)
    {
        LapEstimate lap_est = estimate_lap(rd, required_speed, plans, current_speed);

        // Must have enough fuel before lap start (except lap 0 where initial fuel is fixed)
        if (lap > 0 && fuel_remaining + 1e-9 < lap_est.fuel)
        {
            // infeasible race for this target with current pit policy
            return eval;
        }

        fuel_remaining -= lap_est.fuel;
        eval.total_time += lap_est.time;
        current_speed = lap_est.exit_speed;

        LapPlan lp;
        if (build_plan)
        {
            lp.straight_plans = plans;
            lp.pit = {false, 0, 0.0};
        }

        if (lap < laps - 1)
        {
            // Predict next lap burn at pit-exit speed (if we pit) and at carry-over speed (if we do not pit)
            LapEstimate next_no_pit = estimate_lap(rd, required_speed, plans, current_speed);
            LapEstimate next_from_pit = estimate_lap(rd, required_speed, plans, rd.race.pit_exit_speed);

            if (fuel_remaining + 1e-9 < next_no_pit.fuel)
            {
                int laps_left = laps - lap - 1;

                // Conservative required fuel: one lap from pit exit + rest from rolling start
                double required_future = next_from_pit.fuel;
                if (laps_left > 1)
                    required_future += (laps_left - 1) * next_no_pit.fuel;

                double desired_after_refuel = std::min(rd.car.fuel_capacity, required_future);
                double refuel_amount = std::max(0.0, desired_after_refuel - fuel_remaining);
                refuel_amount = std::ceil(refuel_amount * 100.0) / 100.0;

                if (refuel_amount > 0.0)
                {
                    double pit_t = pit_stop_time(refuel_amount, rd.race.pit_refuel_rate,
                                                 rd.race.pit_tyre_swap_time, rd.race.base_pit_time, false);
                    eval.total_time += pit_t;
                    fuel_remaining += refuel_amount;
                    total_refueled += refuel_amount;
                    current_speed = rd.race.pit_exit_speed;

                    if (build_plan)
                        lp.pit = {true, 0, refuel_amount};
                }
            }
        }

        if (build_plan)
            eval.lap_plans.push_back(lp);
    }

    eval.fuel_burned = rd.car.initial_fuel + total_refueled - fuel_remaining;
    double base_s = calc_base_score(rd.race.time_reference, eval.total_time);
    double fuel_b = calc_fuel_bonus(eval.fuel_burned, rd.race.fuel_soft_cap);
    eval.score = base_s + fuel_b;
    return eval;
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

static std::vector<double> optimize_targets(const RaceData &rd,
                                            const std::vector<double> &required_speed)
{
    int n = static_cast<int>(rd.segments.size());
    std::vector<double> targets(n, 0.0);
    for (int i = 0; i < n; i++)
    {
        if (rd.segments[i].type == "straight")
            targets[i] = rd.car.max_speed;
    }

    auto straight_indices = get_straight_indices(rd);
    auto eval_score = [&](const std::vector<double> &candidate)
    {
        auto plans = compute_straight_plans(rd.segments, required_speed, candidate,
                                            rd.car.max_speed, rd.car.brake_decel);
        return simulate_race(rd, required_speed, plans, false).score;
    };

    double best_score = eval_score(targets);
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
                double local_best_score = best_score;

                double local_lo = std::max(floor_target, current - step * 4.0);
                double local_hi = std::min(rd.car.max_speed, current + step * 2.0);

                for (double ts = local_lo; ts <= local_hi + 1e-9; ts += step)
                {
                    std::vector<double> candidate = targets;
                    candidate[idx] = ts;
                    double candidate_score = eval_score(candidate);
                    if (candidate_score > local_best_score + 1e-9)
                    {
                        local_best_score = candidate_score;
                        local_best_target = ts;
                    }
                }

                if (std::abs(local_best_target - current) > 1e-9)
                {
                    targets[idx] = local_best_target;
                    best_score = local_best_score;
                    improved = true;
                }
            }
        }
    }

    return targets;
}

int main(int argc, char *argv[])
{
    std::string inputPath = "inputFiles/level2input.txt";
    if (argc > 1)
        inputPath = argv[1];

    RaceData rd = load_race_data(inputPath);

    auto best_tyre = select_best_tyre(rd, rd.starting_weather);
    double tyre_friction = best_tyre.friction;

    auto required_speed = resolve_corner_chains(
        rd.segments, tyre_friction, rd.car.crawl_speed, rd.car.max_speed);

    auto target_speeds = optimize_targets(rd, required_speed);
    auto final_plans = compute_straight_plans(rd.segments, required_speed, target_speeds,
                                              rd.car.max_speed, rd.car.brake_decel);
    RaceEval final_eval = simulate_race(rd, required_speed, final_plans, true);

    double min_target = rd.car.max_speed;
    for (int i = 0; i < static_cast<int>(target_speeds.size()); i++)
    {
        if (rd.segments[i].type == "straight")
            min_target = std::min(min_target, target_speeds[i]);
    }

    std::cerr << std::fixed << std::setprecision(4);
    std::cerr << "Selected tyre: " << best_tyre.compound << " (id=" << best_tyre.id << ")\n";
    std::cerr << "Target speed range: " << min_target << " - " << rd.car.max_speed << " m/s\n";
    std::cerr << "Total time: " << final_eval.total_time << " s\n";
    std::cerr << "Fuel burned: " << final_eval.fuel_burned << " L (cap=" << rd.race.fuel_soft_cap << ")\n";
    std::cerr << "Score: " << final_eval.score << "\n";

    std::cout << generate_output(best_tyre.id, rd.segments, final_eval.lap_plans, rd.race.laps).dump(2) << std::endl;
    return 0;
}
