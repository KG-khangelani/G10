#pragma once
#include "types.h"
#include "json.hpp"
#include <fstream>
#include <stdexcept>

using json = nlohmann::json;

inline RaceData parse_race_data(const json &data)
{
    RaceData rd;

    // Car
    rd.car.max_speed = data["car"]["max_speed_m/s"];
    rd.car.accel = data["car"]["accel_m/se2"];
    rd.car.brake_decel = data["car"]["brake_m/se2"];
    rd.car.limp_speed = data["car"]["limp_constant_m/s"];
    rd.car.crawl_speed = data["car"]["crawl_constant_m/s"];
    rd.car.fuel_capacity = data["car"]["fuel_tank_capacity_l"];
    rd.car.initial_fuel = data["car"]["initial_fuel_l"];

    // Race
    rd.race.name = data["race"]["name"];
    rd.race.laps = data["race"]["laps"];
    rd.race.base_pit_time = data["race"]["base_pit_stop_time_s"];
    rd.race.pit_tyre_swap_time = data["race"]["pit_tyre_swap_time_s"];
    rd.race.pit_refuel_rate = data["race"]["pit_refuel_rate_l/s"];
    rd.race.crash_penalty = data["race"]["corner_crash_penalty_s"];
    rd.race.pit_exit_speed = data["race"]["pit_exit_speed_m/s"];
    rd.race.fuel_soft_cap = data["race"]["fuel_soft_cap_limit_l"];
    rd.race.starting_weather_id = data["race"]["starting_weather_condition_id"];
    rd.race.time_reference = data["race"]["time_reference_s"];

    // Track segments
    for (auto &seg : data["track"]["segments"])
    {
        Segment s;
        s.id = seg["id"];
        s.type = seg["type"];
        s.length = seg["length_m"];
        s.radius = seg.contains("radius_m") ? (double)seg["radius_m"] : 0.0;
        rd.segments.push_back(s);
    }

    // Tyre properties
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
        rd.tyre_props[compound] = tp;
    }

    // Available tyre sets
    for (auto &s : data["available_sets"])
    {
        TyreSet ts;
        for (auto &id : s["ids"])
            ts.ids.push_back(id);
        ts.compound = s["compound"];
        rd.tyre_sets.push_back(ts);
    }

    // Weather conditions
    if (data.contains("weather") && data["weather"].contains("conditions"))
    {
        for (auto &w : data["weather"]["conditions"])
        {
            WeatherCondition wc;
            wc.id = w["id"];
            wc.condition = w["condition"];
            wc.duration = w["duration_s"];
            wc.accel_mult = w["acceleration_multiplier"];
            wc.decel_mult = w["deceleration_multiplier"];
            rd.weather.push_back(wc);
        }
    }

    // Resolve starting weather condition string
    rd.starting_weather = "dry";
    for (auto &wc : rd.weather)
    {
        if (wc.id == rd.race.starting_weather_id)
        {
            rd.starting_weather = wc.condition;
            break;
        }
    }

    return rd;
}

inline RaceData load_race_data(const std::string &path)
{
    std::ifstream fin(path);
    if (!fin.is_open())
    {
        throw std::runtime_error("Cannot open input file: " + path);
    }
    json data = json::parse(fin);
    fin.close();
    return parse_race_data(data);
}
