// Level 1: Speed-only optimization (no fuel limits, no degradation, no weather changes)
#include <iostream>
#include <iomanip>
#include "parser.h"
#include "physics.h"
#include "output.h"

int main(int argc, char *argv[])
{
    std::string inputPath = "inputFiles/level1input.txt";
    if (argc > 1)
        inputPath = argv[1];

    RaceData rd = load_race_data(inputPath);
    int n = rd.segments.size();

    // Select best tyre for starting weather
    auto best = select_best_tyre(rd, rd.starting_weather);
    double tyre_friction = best.friction;
    std::cerr << "Selected tyre: " << best.compound << " (id=" << best.id
              << "), friction=" << tyre_friction << "\n";

    // Compute corner chains and straight plans
    auto required_speed = resolve_corner_chains(
        rd.segments, tyre_friction, rd.car.crawl_speed, rd.car.max_speed);
    auto straight_plans = compute_straight_plans(
        rd.segments, required_speed, rd.car.max_speed, rd.car.brake_decel);

    // Debug output
    for (int i = 0; i < n; i++)
    {
        if (rd.segments[i].type == "corner")
            std::cerr << "Seg " << rd.segments[i].id << " corner: chain=" << required_speed[i] << "\n";
        else
            std::cerr << "Seg " << rd.segments[i].id << " straight: target="
                      << straight_plans[i].target_speed << " brake=" << straight_plans[i].brake_start << "\n";
    }

    // Simulate
    double total_time = 0.0;
    double current_speed = 0.0;

    for (int lap = 0; lap < rd.race.laps; lap++)
    {
        for (int i = 0; i < n; i++)
        {
            if (rd.segments[i].type == "straight")
            {
                auto res = simulate_straight(
                    current_speed, straight_plans[i].target_speed,
                    straight_plans[i].brake_start, rd.segments[i].length,
                    rd.car.accel, rd.car.brake_decel,
                    rd.car.max_speed, rd.car.crawl_speed);
                total_time += res.time;
                current_speed = res.exit_speed;
            }
            else
            {
                double corner_speed = std::min(current_speed, required_speed[i]);
                corner_speed = std::max(corner_speed, rd.car.crawl_speed);
                total_time += rd.segments[i].length / corner_speed;
                current_speed = corner_speed;
            }
        }
    }

    double base_score = calc_base_score(rd.race.time_reference, total_time);
    std::cerr << std::fixed << std::setprecision(4);
    std::cerr << "Total time: " << total_time << " s\n";
    std::cerr << "Base score: " << base_score << "\n";

    // Build lap plans (all laps identical, no pit)
    std::vector<LapPlan> lap_plans(rd.race.laps);
    for (auto &lp : lap_plans)
    {
        lp.straight_plans = straight_plans;
        lp.pit = {false, 0, 0};
    }

    // Output
    std::cout << generate_output(best.id, rd.segments, lap_plans, rd.race.laps).dump(2) << std::endl;
    return 0;
}
