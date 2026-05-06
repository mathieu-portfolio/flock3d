#include "app/SimulationControls.hpp"

#include <algorithm>

#include <raylib.h>

namespace flock3d::app {
namespace {

[[nodiscard]] int selected_parameter_index_from_keyboard() noexcept
{
    if (IsKeyPressed(KEY_ONE)) {
        return 0;
    }
    if (IsKeyPressed(KEY_TWO)) {
        return 1;
    }
    if (IsKeyPressed(KEY_THREE)) {
        return 2;
    }
    if (IsKeyPressed(KEY_FOUR)) {
        return 3;
    }
    if (IsKeyPressed(KEY_FIVE)) {
        return 4;
    }
    if (IsKeyPressed(KEY_SIX)) {
        return 5;
    }
    if (IsKeyPressed(KEY_SEVEN)) {
        return 6;
    }
    if (IsKeyPressed(KEY_EIGHT)) {
        return 7;
    }
    return -1;
}

} // namespace

SimulationControlResult SimulationControls::handle_input(sim::BoidSimulation& simulation, bool& paused)
{
    SimulationControlResult result{};

    if (IsKeyPressed(KEY_P)) {
        paused = !paused;
        result.changed = true;
    }
    if (IsKeyPressed(KEY_R)) {
        reset(simulation);
        result.changed = true;
        result.reset = true;
    }
    if (IsKeyPressed(KEY_EQUAL) || IsKeyPressed(KEY_KP_ADD)) {
        adjust_boid_count(simulation, 128);
        result.changed = true;
    }
    if (IsKeyPressed(KEY_MINUS) || IsKeyPressed(KEY_KP_SUBTRACT)) {
        adjust_boid_count(simulation, -128);
        result.changed = true;
    }

    const int selected_index = selected_parameter_index_from_keyboard();
    if (selected_index >= 0) {
        selected_parameter_ = parameter_from_index(static_cast<std::size_t>(selected_index));
        result.changed = true;
    }

    auto& parameters = simulation.parameters();
    if (IsKeyPressed(KEY_LEFT_BRACKET)) {
        adjust_parameter(parameters, selected_parameter_, -1);
        result.changed = true;
    }
    if (IsKeyPressed(KEY_RIGHT_BRACKET)) {
        adjust_parameter(parameters, selected_parameter_, 1);
        result.changed = true;
    }

    return result;
}

void SimulationControls::reset(sim::BoidSimulation& simulation) const
{
    simulation.reset(simulation.parameters().boid_count);
}

void SimulationControls::adjust_boid_count(sim::BoidSimulation& simulation, int delta) const
{
    auto& parameters = simulation.parameters();
    const auto current = static_cast<int>(simulation.size());
    const auto next = std::clamp(current + delta, 0, 100'000);
    parameters.boid_count = static_cast<unsigned int>(next);
    simulation.reset(parameters.boid_count);
}

} // namespace flock3d::app
