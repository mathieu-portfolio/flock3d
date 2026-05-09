#include "app/DebugControls.hpp"

#include <raylib.h>

namespace flock3d::app {

bool DebugControls::handle_input(ControlCommandQueue& commands) const
{
    if (!IsKeyPressed(KEY_F1)) {
        return false;
    }

    commands.enqueue(ControlCommand::toggle_overlay());
    return true;
}

} // namespace flock3d::app
