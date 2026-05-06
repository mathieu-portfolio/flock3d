#pragma once

#include <flock3d/app/ControlHelpers.hpp>
#include <flock3d/sim/BoidSimulation.hpp>

namespace flock3d::app {

struct SimulationControlResult {
    bool changed{};
    bool reset{};
};

class SimulationControls {
public:
    SimulationControlResult handle_input(sim::BoidSimulation& simulation, bool& paused);
    void reset(sim::BoidSimulation& simulation) const;

    [[nodiscard]] TunableParameter selected_parameter() const noexcept { return selected_parameter_; }

private:
    void adjust_boid_count(sim::BoidSimulation& simulation, int delta) const;

    TunableParameter selected_parameter_{TunableParameter::separation_weight};
};

} // namespace flock3d::app
