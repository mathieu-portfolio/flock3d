#include "app/Application.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <random>
#include <string>

#include <raylib.h>

namespace flock3d::app {
namespace {

using Clock = std::chrono::steady_clock;

[[nodiscard]] double elapsed_ms(Clock::time_point start, Clock::time_point end) noexcept
{
    return std::chrono::duration<double, std::milli>{end - start}.count();
}

void write_literal(std::array<char, 96>& line, const char* text)
{
    std::snprintf(line.data(), line.size(), "%s", text);
}

void write_line(std::array<char, 96>& line, const char* format, auto... args)
{
    std::snprintf(line.data(), line.size(), format, args...);
}

} // namespace

Application::Application()
{
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(screen_width, screen_height, "flock3d");
    SetTargetFPS(0);

    camera_.position = Vector3{0.0F, 35.0F, 85.0F};
    camera_.target = Vector3{0.0F, 0.0F, 0.0F};
    camera_.up = Vector3{0.0F, 1.0F, 0.0F};
    camera_.fovy = 45.0F;
    camera_.projection = CAMERA_PERSPECTIVE;

    camera_controller_.initialize_from_camera(camera_);
}

Application::~Application()
{
    recorder_.stop(simulation_time_, simulation_.size(), metrics_);
    unload_overlay_texture();
    if (IsWindowReady()) {
        CloseWindow();
    }
}

void Application::run()
{
    while (!WindowShouldClose()) {
        const float frame_time = GetFrameTime();
        const float frame_time_ms = frame_time * 1000.0F;

        handle_input();
        if (camera_controller_.update(camera_, frame_time)) {
            mark_overlay_dirty();
        }

        if (!paused_) {
            timestep_.add_frame_time(frame_time);
            while (timestep_.consume_step()) {
                const auto simulation_start = Clock::now();
                simulation_.update(static_cast<float>(timestep_.fixed_dt()), &metrics_);
                metrics_.simulation_update_ms = elapsed_ms(simulation_start, Clock::now());
                simulation_time_ += timestep_.fixed_dt();
                recorder_.record_step(simulation_time_, simulation_.size(), metrics_);
            }
        }

        const auto render_start = Clock::now();
        if (show_overlay_) {
            overlay_refresh_accumulator_ += frame_time;
            if (overlay_dirty_ || overlay_refresh_accumulator_ >= 0.10 || !overlay_texture_ready_) {
                refresh_overlay_text(frame_time_ms);
                render_overlay_texture();
            }
        }

        BeginDrawing();
        ClearBackground(Color{12, 16, 24, 255});

        BeginMode3D(camera_);
        DrawGrid(20, 4.0F);
        renderer_.draw(simulation_);
        EndMode3D();

        if (show_overlay_) {
            draw_overlay();
        } else {
            DrawText("F1: show debug overlay", 16, 16, 18, LIGHTGRAY);
        }
        metrics_.render_ms = elapsed_ms(render_start, Clock::now());

        EndDrawing();
    }
}

void Application::handle_input()
{
    bool changed = false;

    changed = debug_controls_.handle_input(show_overlay_) || changed;

    const auto simulation_input = simulation_controls_.handle_input(simulation_, paused_);
    changed = simulation_input.changed || changed;
    if (simulation_input.changed) {
        active_scenario_.simulation_parameters = simulation_.parameters();
    }
    if (simulation_input.reset) {
        metrics_ = sim::SimulationMetrics{};
        simulation_time_ = 0.0;
    }

    if (IsKeyPressed(KEY_PERIOD)) {
        apply_scenario(sim::next_scenario_type(active_scenario_.type));
        changed = true;
    }
    if (IsKeyPressed(KEY_COMMA)) {
        apply_scenario(sim::previous_scenario_type(active_scenario_.type));
        changed = true;
    }
    if (IsKeyPressed(KEY_N)) {
        randomize_current_seed();
        changed = true;
    }
    if (IsKeyPressed(KEY_O)) {
        toggle_recording();
        changed = true;
    }
    if (IsKeyPressed(KEY_M)) {
        cycle_export_mode();
        changed = true;
    }
    if (IsKeyPressed(KEY_PAGE_UP)) {
        adjust_sample_rate(2.0);
        changed = true;
    }
    if (IsKeyPressed(KEY_PAGE_DOWN)) {
        adjust_sample_rate(0.5);
        changed = true;
    }

    if (changed) {
        mark_overlay_dirty();
    }
}

void Application::mark_overlay_dirty() noexcept
{
    overlay_dirty_ = true;
}

void Application::apply_scenario(sim::ScenarioType type)
{
    active_scenario_ = sim::build_scenario(type);
    simulation_.apply_parameters(active_scenario_.simulation_parameters);
    metrics_ = sim::SimulationMetrics{};
    simulation_time_ = 0.0;
}

void Application::reset_current_scenario()
{
    simulation_.reset();
    metrics_ = sim::SimulationMetrics{};
    simulation_time_ = 0.0;
}

void Application::randomize_current_seed()
{
    std::random_device random_device;
    active_scenario_.simulation_parameters.random_seed = random_device();
    simulation_.parameters().random_seed = active_scenario_.simulation_parameters.random_seed;
    reset_current_scenario();
}


void Application::toggle_recording()
{
    if (recorder_.is_recording()) {
        recorder_.stop(simulation_time_, simulation_.size(), metrics_);
        return;
    }

    const auto output_path = experiment::default_output_path(active_scenario_.type, recorder_.export_mode());
    recorder_.start(
        output_path,
        active_scenario_.type,
        simulation_.parameters().random_seed,
        recorder_.export_mode(),
        recorder_.sample_rate_hz());
}

void Application::cycle_export_mode()
{
    if (recorder_.is_recording()) {
        return;
    }

    switch (recorder_.export_mode()) {
    case experiment::ExportMode::Summary:
        recorder_.set_export_mode(experiment::ExportMode::SampledTimeSeries);
        break;
    case experiment::ExportMode::SampledTimeSeries:
        recorder_.set_export_mode(experiment::ExportMode::FullTrajectory);
        break;
    case experiment::ExportMode::FullTrajectory:
        recorder_.set_export_mode(experiment::ExportMode::Summary);
        break;
    }
}

void Application::adjust_sample_rate(double scale) noexcept
{
    if (recorder_.is_recording()) {
        return;
    }
    recorder_.set_sample_rate_hz(std::clamp(recorder_.sample_rate_hz() * scale, 0.25, 120.0));
}

void Application::refresh_overlay_text(float frame_time_ms)
{
    const auto& parameters = simulation_.parameters();
    const auto selected = simulation_controls_.selected_parameter();
    const auto& selected_descriptor = descriptor_for(selected);
    const auto selected_value = parameter_value(parameters, selected);
    std::size_t line = 0;

    write_literal(overlay_lines_[line++], "flock3d scientific overlay");
    write_line(
        overlay_lines_[line++],
        "Scenario %.*s",
        static_cast<int>(active_scenario_.display_name.size()),
        active_scenario_.display_name.data());
    write_line(
        overlay_lines_[line++],
        "Seed %u    FPS %d    %.2f ms    %s",
        parameters.random_seed,
        GetFPS(),
        static_cast<double>(frame_time_ms),
        paused_ ? "paused" : "running");
    write_line(
        overlay_lines_[line++],
        "Boids %-6zu   avg neighbors %.2f",
        simulation_.size(),
        static_cast<double>(metrics_.average_neighbors_per_boid));
    write_line(overlay_lines_[line++], "Sim %.3f ms    Render %.3f ms", metrics_.simulation_update_ms, metrics_.render_ms);
    write_line(
        overlay_lines_[line++],
        "Cells %-6zu   Queries %llu",
        metrics_.spatial_cell_count,
        static_cast<unsigned long long>(metrics_.neighbor_queries));
    write_line(
        overlay_lines_[line++],
        "Candidates/query avg %.1f max %zu",
        metrics_.avg_candidates_per_query,
        metrics_.max_candidates_per_query);
    write_line(
        overlay_lines_[line++],
        "Neighbors/query  avg %.1f max %zu",
        metrics_.avg_effective_neighbors_per_query,
        metrics_.max_effective_neighbors_per_query);
    write_line(
        overlay_lines_[line++],
        "Occupancy cells %zu avg %.1f max %zu",
        metrics_.spatial_cell_count,
        metrics_.avg_cell_occupancy,
        metrics_.max_cell_occupancy);
    const char* warning = "ok";
    if (metrics_.avg_candidates_per_query > 128.0) {
        warning = "high candidates/query";
    } else if (metrics_.max_cell_occupancy > 64) {
        warning = "high max cell occupancy";
    } else if (metrics_.render_ms > metrics_.simulation_update_ms) {
        warning = "render bottleneck";
    } else if (metrics_.simulation_update_ms > metrics_.render_ms) {
        warning = "simulation bottleneck";
    }
    write_line(overlay_lines_[line++], "Warning %s", warning);
    write_line(overlay_lines_[line++], "Camera speed %.1f (mouse wheel)", static_cast<double>(camera_controller_.move_speed()));
    write_literal(overlay_lines_[line++], "");
    write_literal(overlay_lines_[line++], "Collective behavior");
    write_line(
        overlay_lines_[line++],
        "Polarization %.3f   Avg speed %.2f",
        static_cast<double>(metrics_.polarization),
        static_cast<double>(metrics_.average_speed));
    write_line(
        overlay_lines_[line++],
        "Cohesion %.2f       Dispersion %.2f",
        static_cast<double>(metrics_.cohesion),
        static_cast<double>(metrics_.dispersion));
    write_line(overlay_lines_[line++], "Nearest neighbor avg %.2f", static_cast<double>(metrics_.nearest_neighbor_average_distance));
    write_line(
        overlay_lines_[line++],
        "Altitude mean %.2f var %.2f",
        static_cast<double>(metrics_.mean_altitude),
        static_cast<double>(metrics_.altitude_variance));
    write_line(
        overlay_lines_[line++],
        "Stalls %-5zu   Near ground %-5zu",
        metrics_.stall_count,
        metrics_.near_ground_count);
    write_literal(overlay_lines_[line++], "");
    write_literal(overlay_lines_[line++], "Metrics export");
    write_line(
        overlay_lines_[line++],
        "Recording %-3s  Mode %.*s",
        recorder_.is_recording() ? "on" : "off",
        static_cast<int>(experiment::to_string(recorder_.export_mode()).size()),
        experiment::to_string(recorder_.export_mode()).data());
    write_line(overlay_lines_[line++], "Sample rate %.2f Hz", recorder_.sample_rate_hz());
    const auto output_filename = recorder_.output_path().empty() ? std::string{"(none)"} : recorder_.output_path().filename().string();
    write_line(overlay_lines_[line++], "Output %.70s", output_filename.c_str());
    write_literal(overlay_lines_[line++], "");
    write_literal(overlay_lines_[line++], "Controls");
    write_literal(overlay_lines_[line++], "Camera  WASD move, Space up, Ctrl/C down");
    write_literal(overlay_lines_[line++], "        RMB look, Shift boost, wheel speed");
    write_literal(overlay_lines_[line++], "Sim     P pause, R reset, +/- boids, N seed");
    write_literal(overlay_lines_[line++], "Scenario , previous, . next");
    write_literal(overlay_lines_[line++], "Export  O record, M mode, PgUp/PgDn rate");
    write_literal(overlay_lines_[line++], "Tune    Tab/Shift+Tab, Left/Right, 1-8");
    write_literal(overlay_lines_[line++], "");
    write_line(
        overlay_lines_[line++],
        "Parameters    selected: %.*s = %.2f",
        static_cast<int>(selected_descriptor.label.size()),
        selected_descriptor.label.data(),
        static_cast<double>(selected_value));
    write_line(overlay_lines_[line++], "1  Separation weight   %.2f", static_cast<double>(parameters.separation_weight));
    write_line(overlay_lines_[line++], "2  Alignment weight    %.2f", static_cast<double>(parameters.alignment_weight));
    write_line(overlay_lines_[line++], "3  Cohesion weight     %.2f", static_cast<double>(parameters.cohesion_weight));
    write_line(overlay_lines_[line++], "4  Perception radius   %.2f", static_cast<double>(parameters.neighbor_radius));
    write_line(overlay_lines_[line++], "5  Separation radius   %.2f", static_cast<double>(parameters.separation_radius));
    write_line(overlay_lines_[line++], "6  Max speed           %.2f", static_cast<double>(parameters.max_speed));
    write_line(overlay_lines_[line++], "7  Max force           %.2f", static_cast<double>(parameters.max_force));
    write_line(overlay_lines_[line++], "8  Boid scale          %.2f", static_cast<double>(parameters.boid_scale));
    write_line(overlay_lines_[line++], "   Gravity             %.2f", static_cast<double>(parameters.gravity));
    write_line(overlay_lines_[line++], "   Max turn rate       %.2f", static_cast<double>(parameters.max_turn_rate));
    write_line(overlay_lines_[line++], "   Field of view       %.2f", static_cast<double>(parameters.field_of_view_degrees));
    write_line(overlay_lines_[line++], "   Alt correction      %.2f", static_cast<double>(parameters.altitude_correction_strength));

    overlay_refresh_accumulator_ = 0.0;
    overlay_dirty_ = false;
}

void Application::render_overlay_texture()
{
    if (!ensure_overlay_texture()) {
        return;
    }

    BeginTextureMode(overlay_texture_);
    ClearBackground(BLANK);
    draw_overlay_primitives(0, 0);
    EndTextureMode();
}

bool Application::ensure_overlay_texture()
{
    const auto panel = overlay_panel_rect(overlay_layout);
    const int width = panel.width;
    const int height = panel.height;
    if (overlay_texture_ready_ && overlay_texture_width_ == width && overlay_texture_height_ == height) {
        return true;
    }

    unload_overlay_texture();
    overlay_texture_ = LoadRenderTexture(width, height);
    overlay_texture_ready_ = overlay_texture_.id > 0 && overlay_texture_.texture.id > 0;
    if (overlay_texture_ready_) {
        overlay_texture_width_ = width;
        overlay_texture_height_ = height;
    }
    return overlay_texture_ready_;
}

void Application::unload_overlay_texture() noexcept
{
    if (overlay_texture_ready_) {
        UnloadRenderTexture(overlay_texture_);
        overlay_texture_ = RenderTexture2D{};
        overlay_texture_width_ = 0;
        overlay_texture_height_ = 0;
        overlay_texture_ready_ = false;
    }
}

void Application::draw_overlay()
{
    if (!overlay_texture_ready_) {
        render_overlay_texture();
    }

    const auto panel = overlay_panel_rect(overlay_layout);
    if (!overlay_texture_ready_) {
        draw_overlay_primitives(panel.x, panel.y);
        return;
    }

    const Rectangle source{
        0.0F,
        0.0F,
        static_cast<float>(overlay_texture_width_),
        -static_cast<float>(overlay_texture_height_),
    };
    const Vector2 position{static_cast<float>(panel.x), static_cast<float>(panel.y)};
    DrawTextureRec(overlay_texture_.texture, source, position, WHITE);
}

void Application::draw_overlay_primitives(int origin_x, int origin_y) const
{
    const auto panel = overlay_panel_rect(overlay_layout);

    DrawRectangle(origin_x, origin_y, panel.width, panel.height, Fade(BLACK, 0.68F));
    DrawRectangleLines(origin_x, origin_y, panel.width, panel.height, Fade(SKYBLUE, 0.45F));

    const std::size_t selected_parameter_line = overlay_selected_parameter_line(
        overlay_layout, simulation_controls_.selected_parameter());
    for (std::size_t i = 0; i < overlay_lines_.size(); ++i) {
        const int line_y = origin_y + overlay_line_local_y(overlay_layout, i);
        if (i == selected_parameter_line) {
            const auto highlight = overlay_highlight_local_rect(overlay_layout, i);
            DrawRectangle(
                origin_x + highlight.x,
                origin_y + highlight.y,
                highlight.width,
                highlight.height,
                Fade(ORANGE, 0.25F));
        }

        const Color color = i == selected_parameter_line ? GOLD : (overlay_is_section_header(i) ? RAYWHITE : LIGHTGRAY);
        DrawText(
            overlay_lines_[i].data(),
            origin_x + overlay_text_local_x(overlay_layout),
            line_y,
            overlay_layout.font_size,
            color);
    }
}

} // namespace flock3d::app
