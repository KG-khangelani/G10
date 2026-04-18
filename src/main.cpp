#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <map>
#include <iomanip>
#include "json.hpp"

using json = nlohmann::json;

static constexpr double GRAVITY = 9.8;
static constexpr double K_BASE = 0.0005;
static constexpr double K_DRAG = 0.0000000015;
static constexpr double K_STRAIGHT = 0.0000166;
static constexpr double K_BRAKING = 0.0398;
static constexpr double K_CORNER = 0.000265;

struct Car
{
    double max_speed;
    double accel;
    double brake_decel;
    double limp_speed;
    double crawl_speed;
    double fuel_capacity;
    double initial_fuel;
};

struct Race
{
    std::string name;
    int laps;
    double base_pit_time;
    double pit_tyre_swap_time;
    double pit_refuel_rate;
    double crash_penalty;
    double pit_exit_speed;
    double fuel_soft_cap;
    int starting_weather_id;
    double time_reference;
};

struct Segment
{
    int id;
    std::string type;
    double length;
    double radius;
};

struct TyreProps
{
    double life_span;
    double dry_friction_mult;
    double cold_friction_mult;
    double light_rain_friction_mult;
    double heavy_rain_friction_mult;
    double dry_deg;
    double cold_deg;
    double light_rain_deg;
    double heavy_rain_deg;
};

struct TyreSet
{
    std::vector<int> ids;
    std::string compound;
};

// Braking distance from v_i to v_f at deceleration d: (v_i^2 - v_f^2) / (2*d)
double braking_distance(double vi, double vf, double decel)
{
    if (vf >= vi)
        return 0.0;
    return (vi * vi - vf * vf) / (2.0 * decel);
}

// Acceleration distance from v_i to v_f at acceleration a: (v_f^2 - v_i^2) / (2*a)
double accel_distance(double vi, double vf, double accel)
{
    if (vf <= vi)
        return 0.0;
    return (vf * vf - vi * vi) / (2.0 * accel);
}

// Time to go from vi to vf at rate r
double time_speed_change(double vi, double vf, double rate)
{
    return std::abs(vf - vi) / rate;
}

// Distance during speed change: (vf^2 - vi^2) / (2*a)  (signed accel)
double dist_speed_change(double vi, double vf, double rate)
{
    return (vf * vf - vi * vi) / (2.0 * rate);
}

// Time to cover distance d at constant speed v
double time_const(double d, double v)
{
    if (v <= 0)
        return 1e18;
    return d / v;
}

// Compute max corner speed: sqrt(friction * gravity * radius) + crawl_constant
double max_corner_speed(double friction, double radius, double crawl_speed)
{
    double val = friction * GRAVITY * radius;
    if (val < 0)
        val = 0;
    return std::sqrt(val) + crawl_speed;
}

// Simulate a straight segment, returning {exit_speed, time_taken}
// entry_speed: speed entering the straight
// target_speed: desired cruising speed
// brake_start_before_next: distance from end of straight where braking begins
// length: straight length
// accel_rate, brake_rate: car constants
// next_corner_speed: speed we need to reach by end of straight (for braking target)
std::pair<double, double> simulate_straight(
    double entry_speed, double target_speed, double brake_start_before_next,
    double length, double accel_rate, double brake_rate, double next_corner_speed,
    double max_speed, double crawl_speed)
{
    // Clamp target to max
    target_speed = std::min(target_speed, max_speed);
    // Per assumption 11: if target < entry, we just cruise at entry speed
    if (target_speed < entry_speed)
        target_speed = entry_speed;
    target_speed = std::min(target_speed, max_speed);

    double total_time = 0.0;
    double pos = 0.0;
    double speed = entry_speed;

    // Phase 1: Accelerate from entry_speed to target_speed
    double accel_dist = accel_distance(entry_speed, target_speed, accel_rate);
    double brake_point = length - brake_start_before_next;
    if (brake_point < 0)
        brake_point = 0;

    if (accel_dist > brake_point)
    {
        // Can't reach target before brake point; accelerate until brake point
        double v_at_brake = std::sqrt(entry_speed * entry_speed + 2.0 * accel_rate * brake_point);
        v_at_brake = std::min(v_at_brake, max_speed);
        if (brake_point > 0)
        {
            double t_accel = time_speed_change(entry_speed, v_at_brake, accel_rate);
            total_time += t_accel;
        }
        speed = v_at_brake;
        pos = brake_point;
    }
    else
    {
        // Accelerate to target
        if (accel_dist > 0)
        {
            double t_accel = time_speed_change(entry_speed, target_speed, accel_rate);
            total_time += t_accel;
            pos = accel_dist;
            speed = target_speed;
        }
        // Phase 2: Cruise at target_speed until brake point
        double cruise_dist = brake_point - pos;
        if (cruise_dist > 0)
        {
            total_time += time_const(cruise_dist, speed);
            pos = brake_point;
        }
    }

    // Phase 3: Brake from current speed for remainder of straight
    double remaining = length - pos;
    if (remaining > 0)
    {
        // Determine speed at end of braking
        double v_end_sq = speed * speed - 2.0 * brake_rate * remaining;
        double v_end;
        if (v_end_sq <= crawl_speed * crawl_speed)
        {
            // Would decelerate below crawl; we reach crawl then cruise remainder
            double dist_to_crawl = (speed * speed - crawl_speed * crawl_speed) / (2.0 * brake_rate);
            if (dist_to_crawl < 0)
                dist_to_crawl = 0;
            if (dist_to_crawl > remaining)
                dist_to_crawl = remaining;
            if (dist_to_crawl > 0)
            {
                total_time += time_speed_change(speed, crawl_speed, brake_rate);
            }
            double leftover = remaining - dist_to_crawl;
            if (leftover > 0)
            {
                total_time += time_const(leftover, crawl_speed);
            }
            v_end = crawl_speed;
        }
        else
        {
            v_end = std::sqrt(v_end_sq);
            double t_brake = time_speed_change(speed, v_end, brake_rate);
            total_time += t_brake;
        }
        speed = v_end;
    }

    return {speed, total_time};
}

int main(int argc, char *argv[])
{
    std::string inputPath = "1.txt";
    if (argc > 1)
        inputPath = argv[1];

    // Read and parse JSON
    std::ifstream fin(inputPath);
    if (!fin.is_open())
    {
        std::cerr << "Error: cannot open " << inputPath << "\n";
        return 1;
    }
    json data = json::parse(fin);
    fin.close();

    // Parse car
    Car car;
    car.max_speed = data["car"]["max_speed_m/s"];
    car.accel = data["car"]["accel_m/se2"];
    car.brake_decel = data["car"]["brake_m/se2"];
    car.limp_speed = data["car"]["limp_constant_m/s"];
    car.crawl_speed = data["car"]["crawl_constant_m/s"];
    car.fuel_capacity = data["car"]["fuel_tank_capacity_l"];
    car.initial_fuel = data["car"]["initial_fuel_l"];

    // Parse race
    Race race;
    race.name = data["race"]["name"];
    race.laps = data["race"]["laps"];
    race.base_pit_time = data["race"]["base_pit_stop_time_s"];
    race.pit_tyre_swap_time = data["race"]["pit_tyre_swap_time_s"];
    race.pit_refuel_rate = data["race"]["pit_refuel_rate_l/s"];
    race.crash_penalty = data["race"]["corner_crash_penalty_s"];
    race.pit_exit_speed = data["race"]["pit_exit_speed_m/s"];
    race.fuel_soft_cap = data["race"]["fuel_soft_cap_limit_l"];
    race.starting_weather_id = data["race"]["starting_weather_condition_id"];
    race.time_reference = data["race"]["time_reference_s"];

    // Parse track
    std::vector<Segment> segments;
    for (auto &seg : data["track"]["segments"])
    {
        Segment s;
        s.id = seg["id"];
        s.type = seg["type"];
        s.length = seg["length_m"];
        s.radius = seg.contains("radius_m") ? (double)seg["radius_m"] : 0.0;
        segments.push_back(s);
    }

    // Parse tyre properties
    std::map<std::string, TyreProps> tyreProps;
    for (auto &[compound, props] : data["tyres"]["properties"].items())
    {
        TyreProps tp;
        tp.life_span = props["life_span"];
        tp.dry_friction_mult = props["dry_friction_multiplier"];
        tp.cold_friction_mult = props["cold_friction_multiplier"];
        tp.light_rain_friction_mult = props["light_rain_friction_multiplier"];
        tp.heavy_rain_friction_mult = props["heavy_rain_friction_multiplier"];
        tp.dry_deg = props["dry_degradation"];
        tp.cold_deg = props["cold_degradation"];
        tp.light_rain_deg = props["light_rain_degradation"];
        tp.heavy_rain_deg = props["heavy_rain_degradation"];
        tyreProps[compound] = tp;
    }

    // Parse available tyre sets
    std::vector<TyreSet> tyreSets;
    for (auto &s : data["available_sets"])
    {
        TyreSet ts;
        for (auto &id : s["ids"])
            ts.ids.push_back(id);
        ts.compound = s["compound"];
        tyreSets.push_back(ts);
    }

    // Parse weather to determine starting condition
    std::string weather = "dry";
    if (data.contains("weather") && data["weather"].contains("conditions"))
    {
        for (auto &w : data["weather"]["conditions"])
        {
            if ((int)w["id"] == race.starting_weather_id)
            {
                weather = w["condition"];
                break;
            }
        }
    }

    // Select best tyre for current weather (L1: dry, no degradation)
    // Best = highest effective friction = life_span * weather_multiplier (where life_span = base friction coeff)
    // Actually, tyre_friction = (base_friction - total_degradation) * weather_mult
    // In L1, no degradation, so tyre_friction = life_span * weather_mult
    int best_tyre_id = 1;
    std::string best_compound;
    double best_friction = 0;
    for (auto &ts : tyreSets)
    {
        auto &tp = tyreProps[ts.compound];
        double mult = tp.dry_friction_mult;
        if (weather == "cold")
            mult = tp.cold_friction_mult;
        else if (weather == "light_rain")
            mult = tp.light_rain_friction_mult;
        else if (weather == "heavy_rain")
            mult = tp.heavy_rain_friction_mult;
        double eff = tp.life_span * mult;
        if (eff > best_friction)
        {
            best_friction = eff;
            best_tyre_id = ts.ids[0];
            best_compound = ts.compound;
        }
    }
    double tyre_friction = best_friction;

    std::cerr << "Selected tyre: " << best_compound << " (id=" << best_tyre_id << "), friction=" << tyre_friction << "\n";

    // Compute max corner speeds
    int n = segments.size();
    std::vector<double> corner_max_speed(n, car.max_speed);
    for (int i = 0; i < n; i++)
    {
        if (segments[i].type == "corner")
        {
            corner_max_speed[i] = max_corner_speed(tyre_friction, segments[i].radius, car.crawl_speed);
            corner_max_speed[i] = std::min(corner_max_speed[i], car.max_speed);
        }
    }

    // Resolve consecutive corner chains: entry speed for a chain = min of all corners in chain
    // Also need to consider the corner right after a straight (the "exit target")
    // For each straight, find the required exit speed = min corner speed of the consecutive corner chain following it
    // A "chain" is a maximal sequence of consecutive corners (possibly wrapping around for multi-lap)

    // For each segment, compute the "required entry speed" considering forward chains
    // For corners in a chain, entry speed = min of all corners in chain
    // For a straight, entry speed isn't restricted, but exit speed = required entry speed of next segment
    std::vector<double> required_speed(n);
    for (int i = 0; i < n; i++)
    {
        if (segments[i].type == "corner")
        {
            required_speed[i] = corner_max_speed[i];
        }
        else
        {
            required_speed[i] = car.max_speed;
        }
    }

    // Resolve corner chains: propagate minimum through consecutive corners
    // Since track is circular (laps), handle wrap-around
    // Simple approach: go through twice to handle wrap
    for (int pass = 0; pass < 2; pass++)
    {
        for (int i = 0; i < n; i++)
        {
            if (segments[i].type == "corner")
            {
                int next = (i + 1) % n;
                if (segments[next].type == "corner")
                {
                    double chain_min = std::min(required_speed[i], required_speed[next]);
                    required_speed[i] = chain_min;
                    required_speed[next] = chain_min;
                }
            }
        }
    }
    // Propagate backward through chains one more time
    for (int i = n - 1; i >= 0; i--)
    {
        if (segments[i].type == "corner")
        {
            int next = (i + 1) % n;
            if (segments[next].type == "corner")
            {
                double chain_min = std::min(required_speed[i], required_speed[next]);
                required_speed[i] = chain_min;
                required_speed[next] = chain_min;
            }
        }
    }

    // For each straight, determine: target speed, brake_start_m_before_next
    // The exit speed of a straight must be <= required_speed of the next segment
    struct StraightPlan
    {
        double target_speed;
        double brake_start;
    };
    std::vector<StraightPlan> straight_plans(n);
    for (int i = 0; i < n; i++)
    {
        if (segments[i].type == "straight")
        {
            int next = (i + 1) % n;
            double exit_speed = required_speed[next];
            // Target = max speed
            double target = car.max_speed;
            // Braking distance from target to exit_speed
            double bd = braking_distance(target, exit_speed, car.brake_decel);
            // Round up to be safe
            bd = std::ceil(bd * 100.0) / 100.0;
            straight_plans[i] = {target, bd};
        }
    }

    // Print corner speeds for debugging
    for (int i = 0; i < n; i++)
    {
        if (segments[i].type == "corner")
        {
            std::cerr << "Seg " << segments[i].id << " corner: max=" << corner_max_speed[i]
                      << " chain=" << required_speed[i] << "\n";
        }
        else
        {
            std::cerr << "Seg " << segments[i].id << " straight: target=" << straight_plans[i].target_speed
                      << " brake_start=" << straight_plans[i].brake_start << "\n";
        }
    }

    // Simulate the race
    double total_time = 0.0;
    double current_speed = 0.0; // start at 0 m/s

    for (int lap = 0; lap < race.laps; lap++)
    {
        for (int i = 0; i < n; i++)
        {
            if (segments[i].type == "straight")
            {
                auto [exit_speed, t] = simulate_straight(
                    current_speed, straight_plans[i].target_speed,
                    straight_plans[i].brake_start, segments[i].length,
                    car.accel, car.brake_decel, required_speed[(i + 1) % n],
                    car.max_speed, car.crawl_speed);
                total_time += t;
                current_speed = exit_speed;
            }
            else
            {
                // Corner: constant speed = min(entry_speed, required_speed)
                double corner_speed = std::min(current_speed, required_speed[i]);
                corner_speed = std::max(corner_speed, car.crawl_speed);
                double t = segments[i].length / corner_speed;
                total_time += t;
                current_speed = corner_speed;

                // Check if we crash
                if (current_speed > corner_max_speed[i] + 0.001)
                {
                    std::cerr << "WARNING: Crash at lap " << lap + 1 << " seg " << segments[i].id
                              << " speed=" << current_speed << " max=" << corner_max_speed[i] << "\n";
                    total_time += race.crash_penalty;
                    current_speed = car.crawl_speed;
                }
            }
        }
    }

    double base_score = 500000.0 * std::pow(race.time_reference / total_time, 3.0);
    std::cerr << std::fixed << std::setprecision(4);
    std::cerr << "Total time: " << total_time << " s\n";
    std::cerr << "Time reference: " << race.time_reference << " s\n";
    std::cerr << "Base score: " << base_score << "\n";

    // Generate output JSON
    json output;
    output["initial_tyre_id"] = best_tyre_id;
    output["laps"] = json::array();

    for (int lap = 0; lap < race.laps; lap++)
    {
        json lap_obj;
        lap_obj["lap"] = lap + 1;
        lap_obj["segments"] = json::array();
        for (int i = 0; i < n; i++)
        {
            json seg;
            seg["id"] = segments[i].id;
            seg["type"] = segments[i].type;
            if (segments[i].type == "straight")
            {
                seg["target_m/s"] = straight_plans[i].target_speed;
                seg["brake_start_m_before_next"] = straight_plans[i].brake_start;
            }
            lap_obj["segments"].push_back(seg);
        }
        json pit;
        pit["enter"] = false;
        lap_obj["pit"] = pit;
        output["laps"].push_back(lap_obj);
    }

    std::cout << output.dump(2) << std::endl;

    return 0;
}
