#pragma once

#include <array>

#include <raylib.h>

#include "app/CameraController.hpp"
#include "app/DebugControls.hpp"
#include "app/SimulationControls.hpp"
#include <flock3d/app/ControlCommands.hpp>
#include <flock3d/app/OverlayLayout.hpp>
#include <flock3d/experiment/MetricsExport.hpp>
#include "render/BoidRenderer.hpp"
#include <flock3d/sim/BoidSimulation.hpp>
#include <flock3d/sim/Scenario.hpp>
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
    static constexpr OverlayLayout overlay_layout{};
    static constexpr std::size_t overlay_line_count = overlay_layout.line_count;

    void poll_input_events();
    void apply_queued_commands();
    [[nodiscard]] bool apply_control_command(const ControlCommand& command);
    void mark_overlay_dirty() noexcept;
    void refresh_overlay_text(float frame_time_ms);
    void apply_scenario(sim::ScenarioType type);
    void reset_current_scenario();
    void adjust_boid_count(int delta);
    void randomize_current_seed();
    void toggle_recording();
    void cycle_export_mode();
    void adjust_sample_rate(double scale) noexcept;
    void render_overlay_texture();
    void draw_overlay();
    void draw_overlay_primitives(int origin_x, int origin_y) const;
    void draw_overlay_scrollbar(const OverlayRect& panel, int visible_height, int max_scroll) const;
    [[nodiscard]] bool update_overlay_scroll();
    bool ensure_overlay_texture();
    void unload_overlay_texture() noexcept;

    Camera3D camera_{};
    CameraController camera_controller_{};
    SimulationControls simulation_controls_{};
    DebugControls debug_controls_{};
    ControlCommandQueue command_queue_{};
    sim::ScenarioDefinition active_scenario_{sim::build_scenario(sim::ScenarioType::ClassicBoids)};
    sim::BoidSimulation simulation_{active_scenario_.simulation_parameters};
    sim::FixedTimestepAccumulator timestep_{1.0 / 120.0};
    sim::SimulationMetrics metrics_{};
    experiment::MetricsRecorder recorder_{};
    render::BoidRenderer renderer_{};
    std::array<std::array<char, 96>, overlay_line_count> overlay_lines_{};
    RenderTexture2D overlay_texture_{};
    int overlay_texture_width_{};
    int overlay_texture_height_{};
    double overlay_refresh_accumulator_{};
    double simulation_time_{};
    int overlay_scroll_offset_{};
    bool paused_{};
    bool show_overlay_{true};
    bool overlay_dirty_{true};
    bool overlay_texture_ready_{};
};

} // namespace flock3d::app
