// Level 1: Speed-only optimization (no fuel limits, no degradation, no weather changes)
#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include "parser.h"
#include "physics.h"
#include "output.h"

struct LapEstimate
{
    double time = 0.0;
    double exit_speed = 0.0;
};

static LapEstimate simulate_lap(const RaceData &rd,
                                const std::vector<double> &required_speed,
                                const std::vector<StraightPlan> &plans,
                                double entry_speed)
{
    LapEstimate estimate;
    int n = static_cast<int>(rd.segments.size());
    double current_speed = entry_speed;

    for (int i = 0; i < n; i++)
    {
        if (rd.segments[i].type == "straight")
        {
            auto res = simulate_straight(current_speed, plans[i].target_speed,
                                         plans[i].brake_start, rd.segments[i].length,
                                         rd.car.accel, rd.car.brake_decel,
                                         rd.car.max_speed, rd.car.crawl_speed);
            estimate.time += res.time;
            current_speed = res.exit_speed;
        }
        else
        {
            double corner_speed = std::min(current_speed, required_speed[i]);
            corner_speed = std::max(corner_speed, rd.car.crawl_speed);
            estimate.time += rd.segments[i].length / corner_speed;
            current_speed = corner_speed;
        }
    }

    estimate.exit_speed = current_speed;
    return estimate;
}

static double simulate_total_time(const RaceData &rd,
                                  const std::vector<double> &required_speed,
                                  const std::vector<StraightPlan> &first_lap_plans,
                                  const std::vector<StraightPlan> &regular_plans)
{
    double total_time = 0.0;
    double current_speed = 0.0;

    for (int lap = 0; lap < rd.race.laps; lap++)
    {
        const auto &plans = (lap == 0) ? first_lap_plans : regular_plans;
        LapEstimate lap_est = simulate_lap(rd, required_speed, plans, current_speed);
        total_time += lap_est.time;
        current_speed = lap_est.exit_speed;
    }

    return total_time;
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
                                            const std::vector<double> &required_speed,
                                            double entry_speed)
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
                                            rd.car.max_speed, rd.car.brake_decel);
        return simulate_lap(rd, required_speed, plans, entry_speed).time;
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
    std::string inputPath = "inputFiles/level1input.txt";
    if (argc > 1)
        inputPath = argv[1];

    RaceData rd = load_race_data(inputPath);

    auto best_tyre = select_best_tyre(rd, rd.starting_weather);
    auto required_speed = resolve_corner_chains(
        rd.segments, best_tyre.friction, rd.car.crawl_speed, rd.car.max_speed);

    double regular_entry_speed = rd.segments.empty() ? 0.0 :
                                 (rd.segments.back().type == "corner"
                                      ? required_speed.back()
                                      : rd.car.crawl_speed);

    auto first_lap_targets = optimize_targets(rd, required_speed, 0.0);
    auto regular_targets = optimize_targets(rd, required_speed, regular_entry_speed);

    auto first_lap_plans = compute_straight_plans(rd.segments, required_speed, first_lap_targets,
                                                  rd.car.max_speed, rd.car.brake_decel);
    auto regular_plans = compute_straight_plans(rd.segments, required_speed, regular_targets,
                                                rd.car.max_speed, rd.car.brake_decel);

    double total_time = simulate_total_time(rd, required_speed, first_lap_plans, regular_plans);
    double base_score = calc_base_score(rd.race.time_reference, total_time);

    double min_target = rd.car.max_speed;
    for (int i = 0; i < static_cast<int>(regular_targets.size()); i++)
    {
        if (rd.segments[i].type == "straight")
            min_target = std::min(min_target, std::min(first_lap_targets[i], regular_targets[i]));
    }

    std::cerr << std::fixed << std::setprecision(4);
    std::cerr << "Selected tyre: " << best_tyre.compound << " (id=" << best_tyre.id << ")\n";
    std::cerr << "Target speed range: " << min_target << " - " << rd.car.max_speed << " m/s\n";
    std::cerr << "Total time: " << total_time << " s\n";
    std::cerr << "Base score: " << base_score << "\n";

    std::vector<LapPlan> lap_plans(rd.race.laps);
    for (int lap = 0; lap < rd.race.laps; lap++)
    {
        lap_plans[lap].straight_plans = (lap == 0) ? first_lap_plans : regular_plans;
        lap_plans[lap].pit = {false, 0, 0};
    }

    std::cout << generate_output(best_tyre.id, rd.segments, lap_plans, rd.race.laps).dump(2) << std::endl;
    return 0;
}
