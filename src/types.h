#pragma once
#include <string>
#include <vector>
#include <map>

// ── Physical & degradation constants ──
static constexpr double GRAVITY = 9.8;
static constexpr double K_BASE = 0.0005;       // l/m  base fuel rate
static constexpr double K_DRAG = 0.0000000015; // l/m  speed-dependent fuel rate
static constexpr double K_STRAIGHT = 0.0000166;
static constexpr double K_BRAKING = 0.0398;
static constexpr double K_CORNER = 0.000265;
static constexpr double CRASH_DEGRADATION = 0.1; // flat degradation on corner crash

// ── Data structs ──

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
    std::string type; // "straight" or "corner"
    double length;
    double radius; // only for corners
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

struct WeatherCondition
{
    int id;
    std::string condition; // "dry", "cold", "light_rain", "heavy_rain"
    double duration;
    double accel_mult;
    double decel_mult;
};

// ── Aggregate parsed data ──

struct RaceData
{
    Car car;
    Race race;
    std::vector<Segment> segments;
    std::map<std::string, TyreProps> tyre_props; // compound → properties
    std::vector<TyreSet> tyre_sets;
    std::vector<WeatherCondition> weather;
    std::string starting_weather; // condition string at t=0
};

// ── Runtime state types ──

struct TyreState
{
    int set_id; // which tyre set is mounted (one of the ids)
    std::string compound;
    double total_degradation;
    double life_span; // base friction coefficient from TyreProps
};

struct StraightPlan
{
    double target_speed;
    double brake_start; // distance from end of straight where braking begins
};

struct PitDecision
{
    bool enter;
    int tyre_change_set_id;
    double fuel_refuel_amount;
};

struct LapPlan
{
    std::vector<StraightPlan> straight_plans; // indexed by segment index (only meaningful for straights)
    PitDecision pit;
};

struct SimResult
{
    double total_time;
    double fuel_used;
    int crashes;
    int blowouts;
    double final_speed;
    std::vector<double> tyre_degradations; // per tyre set id
};
