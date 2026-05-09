#include "app/SimulationControls.hpp"

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

bool SimulationControls::handle_input(ControlCommandQueue& commands) const
{
    bool queued = false;

    if (IsKeyPressed(KEY_P)) {
        commands.enqueue(ControlCommand::toggle_pause());
        queued = true;
    }
    if (IsKeyPressed(KEY_R)) {
        commands.enqueue(ControlCommand::reset_simulation());
        queued = true;
    }
    if (IsKeyPressed(KEY_EQUAL) || IsKeyPressed(KEY_KP_ADD)) {
        commands.enqueue(ControlCommand::adjust_boid_count(128));
        queued = true;
    }
    if (IsKeyPressed(KEY_MINUS) || IsKeyPressed(KEY_KP_SUBTRACT)) {
        commands.enqueue(ControlCommand::adjust_boid_count(-128));
        queued = true;
    }

    const int selected_index = selected_parameter_index_from_keyboard();
    if (selected_index >= 0) {
        commands.enqueue(ControlCommand::select_parameter(
            parameter_from_index(static_cast<std::size_t>(selected_index))));
        queued = true;
    }
    if (IsKeyPressed(KEY_TAB)) {
        const int direction = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT) ? -1 : 1;
        commands.enqueue(ControlCommand::offset_selected_parameter(direction));
        queued = true;
    }

    if (IsKeyPressed(KEY_LEFT) || IsKeyPressed(KEY_LEFT_BRACKET)) {
        commands.enqueue(ControlCommand::adjust_selected_parameter(-1));
        queued = true;
    }
    if (IsKeyPressed(KEY_RIGHT) || IsKeyPressed(KEY_RIGHT_BRACKET)) {
        commands.enqueue(ControlCommand::adjust_selected_parameter(1));
        queued = true;
    }

    return queued;
}

} // namespace flock3d::app
