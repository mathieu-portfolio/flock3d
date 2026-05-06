#pragma once

#include <raylib.h>

#include "render/BoidRenderer.hpp"
#include <flock3d/sim/BoidSimulation.hpp>
#include <flock3d/sim/SimulationParameters.hpp>

namespace flock3d::app {

class Application {
public:
    Application();
    ~Application();

    Application(const Application&) = delete;
    Application& operator=(const Application&) = delete;
    Application(Application&&) = delete;
    Application& operator=(Application&&) = delete;

    void run();

private:
    static constexpr int screen_width = 1280;
    static constexpr int screen_height = 720;

    Camera3D camera_{};
    sim::BoidSimulation simulation_{};
    sim::FixedTimestepAccumulator timestep_{1.0 / 120.0};
    render::BoidRenderer renderer_{};
};

} // namespace flock3d::app
