#pragma once
#include "types.h"
#include "json.hpp"
#include <vector>

using json = nlohmann::json;

inline json generate_output(
    int initial_tyre_id,
    const std::vector<Segment> &segments,
    const std::vector<LapPlan> &lap_plans,
    int total_laps)
{
    json output;
    output["initial_tyre_id"] = initial_tyre_id;
    output["laps"] = json::array();

    int n = (int)segments.size();

    for (int lap = 0; lap < total_laps; lap++)
    {
        json lap_obj;
        lap_obj["lap"] = lap + 1;
        lap_obj["segments"] = json::array();

        auto &plan = lap_plans[lap];

        for (int i = 0; i < n; i++)
        {
            json seg;
            seg["id"] = segments[i].id;
            seg["type"] = segments[i].type;
            if (segments[i].type == "straight")
            {
                seg["target_m/s"] = plan.straight_plans[i].target_speed;
                seg["brake_start_m_before_next"] = plan.straight_plans[i].brake_start;
            }
            lap_obj["segments"].push_back(seg);
        }

        json pit;
        pit["enter"] = plan.pit.enter;
        if (plan.pit.enter)
        {
            if (plan.pit.tyre_change_set_id > 0)
                pit["tyre_change_set_id"] = plan.pit.tyre_change_set_id;
            if (plan.pit.fuel_refuel_amount > 0)
                pit["fuel_refuel_amount_l"] = plan.pit.fuel_refuel_amount;
        }
        lap_obj["pit"] = pit;

        output["laps"].push_back(lap_obj);
    }
    return output;
}
