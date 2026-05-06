#include "app/Application.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>

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
            }
        }

        BeginDrawing();
        ClearBackground(Color{12, 16, 24, 255});

        const auto render_start = Clock::now();
        BeginMode3D(camera_);
        DrawGrid(20, 4.0F);
        renderer_.draw(simulation_);
        EndMode3D();
        metrics_.render_ms = elapsed_ms(render_start, Clock::now());

        if (show_overlay_) {
            overlay_refresh_accumulator_ += frame_time;
            if (overlay_dirty_ || overlay_refresh_accumulator_ >= 0.10) {
                refresh_overlay_text(frame_time_ms);
            }
            draw_overlay();
        } else {
            DrawText("F1: show debug overlay", 16, 16, 18, LIGHTGRAY);
        }

        EndDrawing();
    }
}

void Application::handle_input()
{
    bool changed = false;

    changed = debug_controls_.handle_input(show_overlay_) || changed;

    const auto simulation_input = simulation_controls_.handle_input(simulation_, paused_);
    changed = simulation_input.changed || changed;
    if (simulation_input.reset) {
        metrics_ = sim::SimulationMetrics{};
    }

    if (changed) {
        mark_overlay_dirty();
    }
}

void Application::mark_overlay_dirty() noexcept
{
    overlay_dirty_ = true;
}

void Application::refresh_overlay_text(float frame_time_ms)
{
    const auto& parameters = simulation_.parameters();
    const auto selected = simulation_controls_.selected_parameter();
    const auto& selected_descriptor = descriptor_for(selected);
    const auto selected_value = parameter_value(parameters, selected);
    std::size_t line = 0;

    write_literal(overlay_lines_[line++], "flock3d debug overlay");
    write_line(overlay_lines_[line++], "FPS %d | frame %.2f ms | %s", GetFPS(), static_cast<double>(frame_time_ms), paused_ ? "paused" : "running");
    write_line(overlay_lines_[line++], "Boids: %zu | avg neighbors: %.2f", simulation_.size(), static_cast<double>(metrics_.average_neighbors_per_boid));
    write_line(overlay_lines_[line++], "Sim %.3f ms | render %.3f ms", metrics_.simulation_update_ms, metrics_.render_ms);
    write_line(overlay_lines_[line++], "Cells: %zu | queries: %llu", metrics_.spatial_hash_cell_count, static_cast<unsigned long long>(metrics_.neighbor_queries));
    write_line(overlay_lines_[line++], "Camera speed: %.1f (wheel adjusts)", static_cast<double>(camera_controller_.move_speed()));
    write_literal(overlay_lines_[line++], "");
    write_literal(overlay_lines_[line++], "Camera: WASD move, Space up, Ctrl/C down");
    write_literal(overlay_lines_[line++], "        RMB look, Shift boost, wheel speed");
    write_literal(overlay_lines_[line++], "Sim: P pause, R reset, +/- boids");
    write_literal(overlay_lines_[line++], "Tune: 1-8 select, [/] adjust selected");
    write_line(overlay_lines_[line++], "Selected: %.*s = %.2f", static_cast<int>(selected_descriptor.label.size()), selected_descriptor.label.data(), static_cast<double>(selected_value));
    write_literal(overlay_lines_[line++], "");
    write_line(overlay_lines_[line++], "1 sep %.2f | 2 align %.2f", static_cast<double>(parameters.separation_weight), static_cast<double>(parameters.alignment_weight));
    write_line(overlay_lines_[line++], "3 coh %.2f | 4 percept %.2f", static_cast<double>(parameters.cohesion_weight), static_cast<double>(parameters.neighbor_radius));
    write_line(overlay_lines_[line++], "5 sep radius %.2f | 6 max speed %.2f", static_cast<double>(parameters.separation_radius), static_cast<double>(parameters.max_speed));
    write_line(overlay_lines_[line++], "7 max force %.2f | 8 scale %.2f", static_cast<double>(parameters.max_force), static_cast<double>(parameters.boid_scale));
    write_literal(overlay_lines_[line++], "");
    write_literal(overlay_lines_[line++], "Debug: F1 toggle overlay");
    write_literal(overlay_lines_[line++], "World: free-fly camera, deterministic sim");
    write_literal(overlay_lines_[line++], "");

    overlay_refresh_accumulator_ = 0.0;
    overlay_dirty_ = false;
}

void Application::draw_overlay() const
{
    constexpr int x = 16;
    constexpr int y = 16;
    constexpr int line_height = 18;
    constexpr int padding = 10;
    constexpr int font_size = 16;
    constexpr int width = 360;
    constexpr int height = static_cast<int>(overlay_line_count) * line_height + padding * 2;

    DrawRectangle(x - padding, y - padding, width, height, Fade(BLACK, 0.68F));
    DrawRectangleLines(x - padding, y - padding, width, height, Fade(SKYBLUE, 0.45F));

    for (std::size_t i = 0; i < overlay_lines_.size(); ++i) {
        const Color color = i == 0 ? RAYWHITE : LIGHTGRAY;
        DrawText(overlay_lines_[i].data(), x, y + static_cast<int>(i) * line_height, font_size, color);
    }
}

} // namespace flock3d::app
