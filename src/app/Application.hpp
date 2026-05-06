#pragma once

#include <array>

#include <raylib.h>

#include "app/CameraController.hpp"
#include "app/DebugControls.hpp"
#include "app/SimulationControls.hpp"
#include "render/BoidRenderer.hpp"
#include <flock3d/sim/BoidSimulation.hpp>
#include <flock3d/sim/SimulationMetrics.hpp>
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
    static constexpr std::size_t overlay_line_count = 21;

    void handle_input();
    void mark_overlay_dirty() noexcept;
    void refresh_overlay_text(float frame_time_ms);
    void draw_overlay() const;

    Camera3D camera_{};
    CameraController camera_controller_{};
    SimulationControls simulation_controls_{};
    DebugControls debug_controls_{};
    sim::BoidSimulation simulation_{};
    sim::FixedTimestepAccumulator timestep_{1.0 / 120.0};
    sim::SimulationMetrics metrics_{};
    render::BoidRenderer renderer_{};
    std::array<std::array<char, 96>, overlay_line_count> overlay_lines_{};
    double overlay_refresh_accumulator_{};
    bool paused_{};
    bool show_overlay_{true};
    bool overlay_dirty_{true};
};

} // namespace flock3d::app
