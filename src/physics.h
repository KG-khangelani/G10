#pragma once
#include "types.h"
#include <cmath>
#include <algorithm>
#include <vector>

// ══════════════════════════════════════════════
//  Kinematics
// ══════════════════════════════════════════════

inline double braking_distance(double vi, double vf, double decel)
{
    if (vf >= vi)
        return 0.0;
    return (vi * vi - vf * vf) / (2.0 * decel);
}

inline double accel_distance(double vi, double vf, double accel)
{
    if (vf <= vi)
        return 0.0;
    return (vf * vf - vi * vi) / (2.0 * accel);
}

inline double time_speed_change(double vi, double vf, double rate)
{
    return std::abs(vf - vi) / rate;
}

inline double time_const(double d, double v)
{
    if (v <= 0)
        return 1e18;
    return d / v;
}

// ══════════════════════════════════════════════
//  Corner speed
// ══════════════════════════════════════════════

inline double calc_max_corner_speed(double friction, double radius, double crawl_speed)
{
    double val = friction * GRAVITY * radius;
    if (val < 0)
        val = 0;
    return std::sqrt(val) + crawl_speed;
}

// ══════════════════════════════════════════════
//  Fuel
// ══════════════════════════════════════════════

// Fuel used over a distance with average speed = (vi+vf)/2
inline double fuel_usage(double vi, double vf, double distance)
{
    double avg = (vi + vf) / 2.0;
    return (K_BASE + K_DRAG * avg * avg) * distance;
}

inline double pit_refuel_time(double amount, double rate)
{
    if (amount <= 0)
        return 0.0;
    return amount / rate;
}

inline double pit_stop_time(double refuel_amount, double refuel_rate,
                            double tyre_swap_time, double base_time,
                            bool changing_tyres)
{
    double t = base_time;
    if (refuel_amount > 0)
        t += pit_refuel_time(refuel_amount, refuel_rate);
    if (changing_tyres)
        t += tyre_swap_time;
    return t;
}

// ══════════════════════════════════════════════
//  Tyre friction & degradation
// ══════════════════════════════════════════════

inline double get_friction_multiplier(const TyreProps &tp, const std::string &weather)
{
    if (weather == "cold")
        return tp.cold_friction_mult;
    if (weather == "light_rain")
        return tp.light_rain_friction_mult;
    if (weather == "heavy_rain")
        return tp.heavy_rain_friction_mult;
    return tp.dry_friction_mult; // default dry
}

inline double get_degradation_rate(const TyreProps &tp, const std::string &weather)
{
    if (weather == "cold")
        return tp.cold_deg;
    if (weather == "light_rain")
        return tp.light_rain_deg;
    if (weather == "heavy_rain")
        return tp.heavy_rain_deg;
    return tp.dry_deg;
}

// tyre_friction = (base_friction - total_degradation) * weather_multiplier
inline double calc_tyre_friction(double life_span, double total_degradation,
                                 double weather_multiplier)
{
    double base = life_span - total_degradation;
    if (base < 0)
        base = 0;
    return base * weather_multiplier;
}

// Straight degradation
inline double tyre_deg_straight(double deg_rate, double segment_length)
{
    return deg_rate * segment_length * K_STRAIGHT;
}

// Braking degradation
inline double tyre_deg_braking(double deg_rate, double vi, double vf)
{
    double a = vi / 100.0;
    double b = vf / 100.0;
    return (a * a - b * b) * K_BRAKING * deg_rate;
}

// Corner degradation
inline double tyre_deg_corner(double deg_rate, double speed, double radius)
{
    return K_CORNER * (speed * speed / radius) * deg_rate;
}

// ══════════════════════════════════════════════
//  Weather
// ══════════════════════════════════════════════

// Get the weather condition at a given elapsed time (cycles through conditions)
inline const WeatherCondition &get_weather_at_time(
    const std::vector<WeatherCondition> &conditions, double elapsed)
{
    if (conditions.empty())
    {
        static WeatherCondition dry_default{0, "dry", 1e18, 1.0, 1.0};
        return dry_default;
    }
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
            return wc;
    }
    return conditions.back();
}

inline std::string get_weather_string_at_time(
    const std::vector<WeatherCondition> &conditions, double elapsed)
{
    return get_weather_at_time(conditions, elapsed).condition;
}

// ══════════════════════════════════════════════
//  Tyre selection
// ══════════════════════════════════════════════

// Select the best tyre for given weather from available sets
// Returns {tyre_id, compound, effective_friction}
struct TyreChoice
{
    int id;
    std::string compound;
    double friction;
};

inline TyreChoice select_best_tyre(const RaceData &rd, const std::string &weather,
                                   const std::vector<int> &exclude_ids = {})
{
    TyreChoice best{-1, "", 0};
    for (auto &ts : rd.tyre_sets)
    {
        auto &tp = rd.tyre_props.at(ts.compound);
        double mult = get_friction_multiplier(tp, weather);
        double eff = tp.life_span * mult;
        // Find first non-excluded id in this set
        int avail_id = -1;
        for (int id : ts.ids)
        {
            bool excluded = false;
            for (int ex : exclude_ids)
                if (id == ex)
                {
                    excluded = true;
                    break;
                }
            if (!excluded)
            {
                avail_id = id;
                break;
            }
        }
        if (avail_id < 0)
            continue;
        if (eff > best.friction)
        {
            best = {avail_id, ts.compound, eff};
        }
    }
    return best;
}

// ══════════════════════════════════════════════
//  Corner chain resolution
// ══════════════════════════════════════════════

// Compute required entry speed for each segment given friction
// Consecutive corners must all enter at the min speed of the chain
inline std::vector<double> resolve_corner_chains(
    const std::vector<Segment> &segments, double friction, double crawl_speed, double max_speed)
{
    int n = segments.size();
    std::vector<double> req(n, max_speed);

    // Set corner max speeds
    for (int i = 0; i < n; i++)
    {
        if (segments[i].type == "corner")
        {
            double cs = calc_max_corner_speed(friction, segments[i].radius, crawl_speed);
            req[i] = std::min(cs, max_speed);
        }
    }

    // Propagate chain minimums (forward + backward, 2 passes for wrap-around)
    for (int pass = 0; pass < 2; pass++)
    {
        for (int i = 0; i < n; i++)
        {
            if (segments[i].type == "corner")
            {
                int next = (i + 1) % n;
                if (segments[next].type == "corner")
                {
                    double cmin = std::min(req[i], req[next]);
                    req[i] = cmin;
                    req[next] = cmin;
                }
            }
        }
    }
    for (int i = n - 1; i >= 0; i--)
    {
        if (segments[i].type == "corner")
        {
            int next = (i + 1) % n;
            if (segments[next].type == "corner")
            {
                double cmin = std::min(req[i], req[next]);
                req[i] = cmin;
                req[next] = cmin;
            }
        }
    }
    return req;
}

// Compute brake points for all straights given required speeds
inline std::vector<StraightPlan> compute_straight_plans(
    const std::vector<Segment> &segments,
    const std::vector<double> &required_speed,
    double max_speed, double brake_decel)
{
    int n = segments.size();
    std::vector<StraightPlan> plans(n, {0, 0});
    for (int i = 0; i < n; i++)
    {
        if (segments[i].type == "straight")
        {
            int next = (i + 1) % n;
            double exit_speed = required_speed[next];
            double target = max_speed;
            double bd = braking_distance(target, exit_speed, brake_decel);
            bd = std::ceil(bd * 100.0) / 100.0;
            plans[i] = {target, bd};
        }
    }
    return plans;
}

// ══════════════════════════════════════════════
//  Straight simulation (3-phase: accel → cruise → brake)
// ══════════════════════════════════════════════

struct StraightResult
{
    double exit_speed;
    double time;
    double fuel; // fuel consumed
    double distance_accel;
    double distance_cruise;
    double distance_brake;
    double speed_at_brake; // speed when braking starts
};

inline StraightResult simulate_straight(
    double entry_speed, double target_speed, double brake_start_before_next,
    double length, double accel_rate, double brake_rate,
    double max_speed, double crawl_speed)
{
    StraightResult r{};
    target_speed = std::min(target_speed, max_speed);
    if (target_speed < entry_speed)
        target_speed = entry_speed;
    target_speed = std::min(target_speed, max_speed);

    double total_time = 0.0;
    double total_fuel = 0.0;
    double pos = 0.0;
    double speed = entry_speed;

    double brake_point = length - brake_start_before_next;
    if (brake_point < 0)
        brake_point = 0;

    // Phase 1: Accelerate
    double ad = accel_distance(entry_speed, target_speed, accel_rate);
    if (ad > brake_point)
    {
        double v_at_brake = std::sqrt(entry_speed * entry_speed + 2.0 * accel_rate * brake_point);
        v_at_brake = std::min(v_at_brake, max_speed);
        if (brake_point > 0)
        {
            total_time += time_speed_change(entry_speed, v_at_brake, accel_rate);
            total_fuel += fuel_usage(entry_speed, v_at_brake, brake_point);
        }
        r.distance_accel = brake_point;
        speed = v_at_brake;
        pos = brake_point;
    }
    else
    {
        if (ad > 0)
        {
            total_time += time_speed_change(entry_speed, target_speed, accel_rate);
            total_fuel += fuel_usage(entry_speed, target_speed, ad);
            pos = ad;
            speed = target_speed;
        }
        r.distance_accel = ad;
        double cruise_dist = brake_point - pos;
        if (cruise_dist > 0)
        {
            total_time += time_const(cruise_dist, speed);
            total_fuel += fuel_usage(speed, speed, cruise_dist);
            pos = brake_point;
        }
        r.distance_cruise = brake_point - r.distance_accel;
    }

    r.speed_at_brake = speed;

    // Phase 3: Brake
    double remaining = length - pos;
    if (remaining > 0)
    {
        double v_end_sq = speed * speed - 2.0 * brake_rate * remaining;
        double v_end;
        if (v_end_sq <= crawl_speed * crawl_speed)
        {
            double dist_to_crawl = (speed * speed - crawl_speed * crawl_speed) / (2.0 * brake_rate);
            if (dist_to_crawl < 0)
                dist_to_crawl = 0;
            if (dist_to_crawl > remaining)
                dist_to_crawl = remaining;
            if (dist_to_crawl > 0)
            {
                total_time += time_speed_change(speed, crawl_speed, brake_rate);
                total_fuel += fuel_usage(speed, crawl_speed, dist_to_crawl);
            }
            double leftover = remaining - dist_to_crawl;
            if (leftover > 0)
            {
                total_time += time_const(leftover, crawl_speed);
                total_fuel += fuel_usage(crawl_speed, crawl_speed, leftover);
            }
            v_end = crawl_speed;
        }
        else
        {
            v_end = std::sqrt(v_end_sq);
            total_time += time_speed_change(speed, v_end, brake_rate);
            total_fuel += fuel_usage(speed, v_end, remaining);
        }
        r.distance_brake = remaining;
        speed = v_end;
    }

    r.exit_speed = speed;
    r.time = total_time;
    r.fuel = total_fuel;
    return r;
}

// ══════════════════════════════════════════════
//  Scoring
// ══════════════════════════════════════════════

inline double calc_base_score(double time_reference, double total_time)
{
    return 500000.0 * std::pow(time_reference / total_time, 3.0);
}

inline double calc_fuel_bonus(double fuel_used, double fuel_soft_cap)
{
    double ratio = fuel_used / fuel_soft_cap;
    return -500000.0 * (1.0 - ratio) * (1.0 - ratio) + 500000.0;
}

inline double calc_tyre_bonus(double total_degradation, int blowouts)
{
    return 100000.0 * total_degradation - 50000.0 * blowouts;
}
