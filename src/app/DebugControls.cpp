#include "app/DebugControls.hpp"

#include <raylib.h>

namespace flock3d::app {

bool DebugControls::handle_input(bool& show_overlay) const
{
    if (!IsKeyPressed(KEY_F1)) {
        return false;
    }

    show_overlay = !show_overlay;
    return true;
}

} // namespace flock3d::app
