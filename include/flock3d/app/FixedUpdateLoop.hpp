#pragma once

#include <algorithm>

#include <flock3d/sim/SimulationParameters.hpp>

namespace flock3d::app {

struct FixedUpdateLoopConfig {
    static constexpr int default_max_updates_per_rendered_frame = 8;

    int max_updates_per_rendered_frame{default_max_updates_per_rendered_frame};
};

struct FixedUpdateLoopResult {
    int updates{};
    bool reached_update_cap{};
    bool stopped_for_pause{};
};

template <typename ApplyQueuedCommands, typename IsPaused, typename UpdateSimulation, typename PollInputBetweenTicks>
FixedUpdateLoopResult run_fixed_update_catch_up(
    sim::FixedTimestepAccumulator& timestep,
    double frame_time_seconds,
    FixedUpdateLoopConfig config,
    ApplyQueuedCommands&& apply_queued_commands,
    IsPaused&& is_paused,
    UpdateSimulation&& update_simulation,
    PollInputBetweenTicks&& poll_input_between_ticks)
{
    timestep.add_frame_time(frame_time_seconds);

    const int update_cap = std::max(0, config.max_updates_per_rendered_frame);
    FixedUpdateLoopResult result{};

    while (result.updates < update_cap && timestep.has_step()) {
        apply_queued_commands();
        if (is_paused()) {
            result.stopped_for_pause = true;
            return result;
        }

        (void)timestep.consume_step();
        update_simulation(timestep.fixed_dt());
        ++result.updates;

        if (result.updates < update_cap && timestep.has_step()) {
            poll_input_between_ticks();
        }
    }

    result.reached_update_cap = timestep.has_step();
    return result;
}

} // namespace flock3d::app
