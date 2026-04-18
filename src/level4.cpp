// Level 4: Full simulation — tyre degradation + weather + fuel + multi-set pit strategy (OPTIMIZED)
// Key optimizations:
// 1. Minimum stint of 4 laps (eliminates ping-pong)
// 2. Only swap at weather transitions with significant friction gain (>10%)
// 3. Weather duration check (need >= 4 laps in new weather to justify swap)
// 4. Better set selection: prefer set with most remaining life
// 5. Combine fuel + tyre into single pit stops
// 6. Look-ahead for combining upcoming fuel needs with swaps
// 7. No empty pit stops
// Scoring: base_score + fuel_bonus + tyre_bonus
#include <iostream>
#include <iomanip>
#include <cmath>
#include <set>
#include <algorithm>
#include "parser.h"
#include "physics.h"
#include "output.h"

// Per-set state tracking
struct SetState
{
    int id;
    std::string compound;
    double degradation;
    double life_span;
    bool blown;
};

// Compute time remaining in current weather condition
double time_until_weather_change(const std::vector<WeatherCondition> &conditions, double elapsed)
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

// Find the best available set for a compound, preferring set with MOST remaining life
// min_remaining: minimum life remaining to consider a set viable
int find_best_set(const std::map<int, SetState> &all_sets,
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
                continue; // not enough life remaining
            if (remaining > best_remaining)
            {
                best_remaining = remaining;
                best_id = id;
            }
        }
    }
    return best_id;
}

// Find any non-blown set with most remaining life (fallback)
int find_any_set(const std::map<int, SetState> &all_sets,
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

// Find best set considering weather preference order
// Tries: 1) target compound, 2) 2nd-best for weather, 3) any compound
int find_best_set_for_weather(const std::map<int, SetState> &all_sets,
                               const RaceData &rd,
                               const std::string &weather,
                               int exclude_id,
                               double min_remaining = 0.15)
{
    // Build ranked list of compounds for this weather
    struct CompScore
    {
        std::string compound;
        double eff_friction;
    };
    std::vector<CompScore> ranked;
    for (auto &[comp, tp] : rd.tyre_props)
    {
        double mult = get_friction_multiplier(tp, weather);
        ranked.push_back({comp, tp.life_span * mult});
    }
    std::sort(ranked.begin(), ranked.end(),
              [](const CompScore &a, const CompScore &b)
              { return a.eff_friction > b.eff_friction; });

    // Try each compound in order of weather suitability
    for (auto &cs : ranked)
    {
        // Compute compound-specific minimum: need at least 3 laps of life
        auto &tp = rd.tyre_props.at(cs.compound);
        double deg_rate = get_degradation_rate(tp, weather);
        double compound_min = std::max(deg_rate * 3.0, min_remaining);
        int id = find_best_set(all_sets, rd.tyre_sets, cs.compound, exclude_id, compound_min);
        if (id >= 0)
            return id;
    }
    // Absolute fallback: any set with ANY remaining life
    return find_any_set(all_sets, rd.tyre_sets, exclude_id, 0.01);
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

    // Initialize all tyre set states
    std::map<int, SetState> all_sets;
    for (auto &ts : rd.tyre_sets)
    {
        auto &tp = rd.tyre_props[ts.compound];
        for (int id : ts.ids)
        {
            all_sets[id] = {id, ts.compound, 0.0, tp.life_span, false};
        }
    }

    // Forward simulate
    std::vector<LapPlan> lap_plans;
    double current_speed = 0.0;
    double fuel_remaining = rd.car.initial_fuel;
    double elapsed_time = 0.0;

    // Select initial tyre
    std::string start_weather = rd.starting_weather;
    auto initial_tyre = select_best_tyre(rd, start_weather);
    int current_set_id = initial_tyre.id;
    std::string current_compound = initial_tyre.compound;
    double current_degradation = 0.0;
    std::set<int> used_set_ids;
    used_set_ids.insert(current_set_id);
    int laps_on_current_set = 0;
    double last_lap_time = 420.0; // initial estimate

    std::cerr << "Initial tyre: " << current_compound << " (id=" << current_set_id << ")\n";

    for (int lap = 0; lap < rd.race.laps; lap++)
    {
        // Current weather
        auto &wc = get_weather_at_time(rd.weather, elapsed_time);
        std::string weather_now = wc.condition;
        double accel_eff = rd.car.accel * wc.accel_mult;
        double brake_eff = rd.car.brake_decel * wc.decel_mult;

        // Current tyre friction with degradation
        auto &tp = rd.tyre_props[current_compound];
        double deg_rate = get_degradation_rate(tp, weather_now);
        double friction_mult = get_friction_multiplier(tp, weather_now);
        double tyre_friction = calc_tyre_friction(tp.life_span, current_degradation, friction_mult);

        if (tyre_friction < 0.01)
            tyre_friction = 0.01;

        // Resolve corner chains for current friction
        auto required_speed = resolve_corner_chains(
            rd.segments, tyre_friction, rd.car.crawl_speed, rd.car.max_speed);

        double target = rd.car.max_speed;
        std::vector<StraightPlan> straight_plans(n, {0, 0});
        for (int i = 0; i < n; i++)
        {
            if (rd.segments[i].type == "straight")
            {
                int next = (i + 1) % n;
                double exit_sp = required_speed[next];
                double bd = braking_distance(target, exit_sp, brake_eff);
                bd = std::ceil(bd * 100.0) / 100.0;
                straight_plans[i] = {target, bd};
            }
        }

        // Simulate this lap segment by segment
        double lap_time = 0;
        double lap_fuel = 0;
        double speed = current_speed;
        double lap_degradation = 0;
        bool limp_mode = false;

        for (int i = 0; i < n; i++)
        {
            if (limp_mode)
            {
                double t = rd.segments[i].length / rd.car.limp_speed;
                lap_time += t;
                lap_fuel += fuel_usage(rd.car.limp_speed, rd.car.limp_speed, rd.segments[i].length);
                speed = rd.car.limp_speed;
                continue;
            }

            if (rd.segments[i].type == "straight")
            {
                auto res = simulate_straight(speed, straight_plans[i].target_speed,
                                             straight_plans[i].brake_start, rd.segments[i].length,
                                             accel_eff, brake_eff,
                                             rd.car.max_speed, rd.car.crawl_speed);
                lap_time += res.time;
                lap_fuel += res.fuel;
                lap_degradation += tyre_deg_straight(deg_rate, rd.segments[i].length);
                if (res.distance_brake > 0 && res.speed_at_brake > res.exit_speed)
                    lap_degradation += tyre_deg_braking(deg_rate, res.speed_at_brake, res.exit_speed);
                speed = res.exit_speed;
            }
            else
            {
                double cs = std::min(speed, required_speed[i]);
                cs = std::max(cs, rd.car.crawl_speed);
                lap_time += rd.segments[i].length / cs;
                lap_fuel += fuel_usage(cs, cs, rd.segments[i].length);
                lap_degradation += tyre_deg_corner(deg_rate, cs, rd.segments[i].radius);
                speed = cs;
            }

            if (fuel_remaining - lap_fuel <= 0)
            {
                limp_mode = true;
                speed = rd.car.limp_speed;
            }
            if (current_degradation + lap_degradation >= tp.life_span)
            {
                limp_mode = true;
                speed = rd.car.limp_speed;
                all_sets[current_set_id].blown = true;
            }
        }

        elapsed_time += lap_time;
        fuel_remaining -= lap_fuel;
        current_degradation += lap_degradation;
        all_sets[current_set_id].degradation = current_degradation;
        current_speed = speed;
        laps_on_current_set++;
        last_lap_time = lap_time;

        // Build lap plan
        LapPlan lp;
        lp.straight_plans = straight_plans;
        lp.pit = {false, 0, 0};

        // ── Pit decisions (end of lap) ──
        if (lap < rd.race.laps - 1)
        {
            std::string next_weather = get_weather_string_at_time(rd.weather, elapsed_time);
            auto &next_wc = get_weather_at_time(rd.weather, elapsed_time);
            double next_accel = rd.car.accel * next_wc.accel_mult;
            double next_brake = rd.car.brake_decel * next_wc.decel_mult;

            auto &cur_tp = rd.tyre_props[current_compound];
            double next_fmult = get_friction_multiplier(cur_tp, next_weather);
            double next_friction = calc_tyre_friction(cur_tp.life_span, current_degradation, next_fmult);

            // ── 1. Blowout danger check ──
            double est_next_deg = lap_degradation * 1.15;
            bool tyre_danger = (current_degradation + est_next_deg >= cur_tp.life_span * 0.93);
            // Also danger if already blown (limp mode)
            if (limp_mode)
                tyre_danger = true;

            // ── 2. Weather-driven compound swap ──
            bool want_weather_swap = false;
            auto next_best = select_best_tyre(rd, next_weather);
            if (next_best.compound != current_compound && laps_on_current_set >= 4)
            {
                // Compare current degraded friction vs best fresh tyre friction for new weather
                double best_friction = next_best.friction;
                if (best_friction > std::max(next_friction, 0.01) * 1.10)
                {
                    // Check enough laps remain (min of weather duration and race end)
                    double weather_remaining = time_until_weather_change(rd.weather, elapsed_time);
                    int race_laps_left = rd.race.laps - lap - 1;
                    double est_laps = std::min(weather_remaining / last_lap_time,
                                               (double)race_laps_left);
                    if (est_laps >= 5.0)
                        want_weather_swap = true;
                }
            }

            // Determine if we want a tyre swap (danger overrides stint minimum)
            bool want_swap = tyre_danger || want_weather_swap;

            // ── 3. Fuel check ──
            // Estimate fuel for next lap using the tyre we'll have
            double est_friction = want_swap ? next_best.friction : std::max(next_friction, 0.1);
            auto next_req = resolve_corner_chains(rd.segments, est_friction,
                                                  rd.car.crawl_speed, rd.car.max_speed);
            double next_fuel_est = 0;
            double sp = want_swap ? rd.race.pit_exit_speed : current_speed;
            for (int i = 0; i < n; i++)
            {
                if (rd.segments[i].type == "straight")
                {
                    int nx = (i + 1) % n;
                    double bd = braking_distance(target, next_req[nx], next_brake);
                    bd = std::ceil(bd * 100.0) / 100.0;
                    auto res = simulate_straight(sp, target, bd, rd.segments[i].length,
                                                 next_accel, next_brake,
                                                 rd.car.max_speed, rd.car.crawl_speed);
                    next_fuel_est += res.fuel;
                    sp = res.exit_speed;
                }
                else
                {
                    double cs = std::min(sp, next_req[i]);
                    cs = std::max(cs, rd.car.crawl_speed);
                    next_fuel_est += fuel_usage(cs, cs, rd.segments[i].length);
                    sp = cs;
                }
            }

            bool need_refuel = fuel_remaining < next_fuel_est * 1.05;

            // Look ahead: if fuel needed within 3 laps AND swap wanted, combine now
            if (!need_refuel && want_swap)
            {
                double laps_until_empty = fuel_remaining / std::max(next_fuel_est, 0.01);
                if (laps_until_empty <= 3.0)
                    need_refuel = true;
            }

            // If pitting for fuel anyway, check if adding a cheap swap (+3s) is beneficial
            if (need_refuel && !want_swap && next_best.compound != current_compound &&
                laps_on_current_set >= 4)
            {
                double best_friction = next_best.friction;
                if (best_friction > std::max(next_friction, 0.01) * 1.05)
                    want_swap = true;
            }

            // ── 4. Select tyre set ──
            int swap_id = 0;
            if (want_swap)
            {
                // Use weather-ranked compound selection with minimum remaining life
                double min_life = std::max(lap_degradation * 2.5, 0.15);
                swap_id = find_best_set_for_weather(all_sets, rd, next_weather,
                                                     current_set_id, min_life);

                // Verify the selected set actually improves friction
                // (avoid swapping to a more-degraded set of the same compound)
                if (swap_id > 0 && !tyre_danger && !limp_mode)
                {
                    auto &new_ss = all_sets[swap_id];
                    auto &new_tp = rd.tyre_props[new_ss.compound];
                    double new_fmult = get_friction_multiplier(new_tp, next_weather);
                    double new_friction = calc_tyre_friction(new_tp.life_span, new_ss.degradation, new_fmult);
                    // Only swap if new set provides at least 10% more friction
                    if (new_friction <= next_friction * 1.10)
                        swap_id = -1; // not worth it
                }

                if (swap_id < 0)
                    want_swap = false;
            }

            // ── 5. Compute refuel amount ──
            double refuel_amount = 0;
            if (need_refuel)
            {
                refuel_amount = rd.car.fuel_capacity - fuel_remaining;
                refuel_amount = std::max(0.0, refuel_amount);
                refuel_amount = std::ceil(refuel_amount * 100.0) / 100.0;
            }

            // ── 6. Execute pit stop (only if actual work) ──
            if (want_swap || refuel_amount > 0)
            {
                double pt = pit_stop_time(refuel_amount, rd.race.pit_refuel_rate,
                                          rd.race.pit_tyre_swap_time, rd.race.base_pit_time,
                                          want_swap);
                elapsed_time += pt;
                fuel_remaining += refuel_amount;
                current_speed = rd.race.pit_exit_speed;
                lp.pit = {true, want_swap ? swap_id : 0, refuel_amount};

                if (want_swap && swap_id > 0)
                {
                    current_compound = all_sets[swap_id].compound;
                    current_degradation = all_sets[swap_id].degradation;
                    current_set_id = swap_id;
                    used_set_ids.insert(swap_id);
                    laps_on_current_set = 0;
                    std::cerr << "Pit lap " << lap + 1 << ": swap to " << current_compound
                              << " (id=" << swap_id << " deg=" << std::fixed << std::setprecision(3)
                              << current_degradation << ")";
                }
                else
                {
                    std::cerr << "Pit lap " << lap + 1 << ": refuel only";
                }
                std::cerr << " refuel=" << refuel_amount << "L time=" << pt << "s\n";
            }
        }

        lap_plans.push_back(lp);
    }

    // Compute scoring
    double total_refueled = 0;
    for (auto &lp : lap_plans)
        if (lp.pit.enter)
            total_refueled += lp.pit.fuel_refuel_amount;
    double fuel_burned = rd.car.initial_fuel + total_refueled - fuel_remaining;

    double total_degradation = 0;
    int blowouts = 0;
    for (auto &[id, ss] : all_sets)
    {
        if (used_set_ids.count(id))
        {
            total_degradation += ss.degradation;
            if (ss.blown)
                blowouts++;
        }
    }

    double base_score = calc_base_score(rd.race.time_reference, elapsed_time);
    double fuel_bonus = calc_fuel_bonus(fuel_burned, rd.race.fuel_soft_cap);
    double tyre_bonus = calc_tyre_bonus(total_degradation, blowouts);
    double final_score = base_score + fuel_bonus + tyre_bonus;

    std::cerr << std::fixed << std::setprecision(4);
    std::cerr << "Total time: " << elapsed_time << " s\n";
    std::cerr << "Fuel burned: " << fuel_burned << " L (cap=" << rd.race.fuel_soft_cap << ")\n";
    std::cerr << "Tyre degradation sum: " << total_degradation << " | blowouts: " << blowouts << "\n";
    std::cerr << "Sets used: " << used_set_ids.size() << "/9\n";
    std::cerr << "Base score: " << base_score << "\n";
    std::cerr << "Fuel bonus: " << fuel_bonus << "\n";
    std::cerr << "Tyre bonus: " << tyre_bonus << "\n";
    std::cerr << "Final score: " << final_score << "\n";

    std::cout << generate_output(initial_tyre.id, rd.segments, lap_plans, rd.race.laps).dump(2) << std::endl;
    return 0;
}
