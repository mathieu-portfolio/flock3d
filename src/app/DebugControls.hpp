#pragma once

#include <flock3d/app/ControlCommands.hpp>

namespace flock3d::app {

class DebugControls {
public:
    bool handle_input(ControlCommandQueue& commands) const;
};

} // namespace flock3d::app
