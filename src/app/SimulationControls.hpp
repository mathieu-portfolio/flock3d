#pragma once

#include <flock3d/app/ControlCommands.hpp>

namespace flock3d::app {

class SimulationControls {
public:
    bool handle_input(ControlCommandQueue& commands) const;

    [[nodiscard]] TunableParameter selected_parameter() const noexcept { return selected_parameter_; }
    void select_parameter(TunableParameter parameter) noexcept { selected_parameter_ = parameter; }
    void offset_selected_parameter(int direction) noexcept
    {
        selected_parameter_ = offset_parameter(selected_parameter_, direction);
    }

private:
    TunableParameter selected_parameter_{TunableParameter::separation_weight};
};

} // namespace flock3d::app
