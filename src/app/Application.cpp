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

[[nodiscard]] bool shift_down() noexcept
{
    return IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);
}

void write_literal(std::array<char, 96>& line, const char* text)
{
    std::snprintf(line.data(), line.size(), "%s", text);
}

void write_line(std::array<char, 96>& line, const char* format, auto... args)
{
    std::snprintf(line.data(), line.size(), format, args...);
}

void adjust_float(float& value, float delta, float minimum, float maximum) noexcept
{
    value = std::clamp(value + delta, minimum, maximum);
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

    DisableCursor();
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
        UpdateCamera(&camera_, CAMERA_FREE);

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
    auto& parameters = simulation_.parameters();

    if (IsKeyPressed(KEY_F1)) {
        show_overlay_ = !show_overlay_;
        changed = true;
    }
    if (IsKeyPressed(KEY_SPACE)) {
        paused_ = !paused_;
        changed = true;
    }
    if (IsKeyPressed(KEY_R)) {
        reset_simulation();
        changed = true;
    }
    if (IsKeyPressed(KEY_EQUAL) || IsKeyPressed(KEY_KP_ADD)) {
        adjust_boid_count(128);
        changed = true;
    }
    if (IsKeyPressed(KEY_MINUS) || IsKeyPressed(KEY_KP_SUBTRACT)) {
        adjust_boid_count(-128);
        changed = true;
    }

    if (IsKeyPressed(KEY_ONE)) {
        adjust_float(parameters.separation_weight, shift_down() ? -0.1F : 0.1F, 0.0F, 10.0F);
        changed = true;
    }
    if (IsKeyPressed(KEY_TWO)) {
        adjust_float(parameters.alignment_weight, shift_down() ? -0.1F : 0.1F, 0.0F, 10.0F);
        changed = true;
    }
    if (IsKeyPressed(KEY_THREE)) {
        adjust_float(parameters.cohesion_weight, shift_down() ? -0.1F : 0.1F, 0.0F, 10.0F);
        changed = true;
    }
    if (IsKeyPressed(KEY_FOUR)) {
        adjust_float(parameters.neighbor_radius, shift_down() ? -0.25F : 0.25F, 0.5F, 30.0F);
        changed = true;
    }
    if (IsKeyPressed(KEY_FIVE)) {
        adjust_float(parameters.max_speed, shift_down() ? -0.5F : 0.5F, 0.5F, 80.0F);
        changed = true;
    }

    if (changed) {
        mark_overlay_dirty();
    }
}

void Application::adjust_boid_count(int delta)
{
    auto& parameters = simulation_.parameters();
    const auto current = static_cast<int>(simulation_.size());
    const auto next = std::clamp(current + delta, 0, 100'000);
    parameters.boid_count = static_cast<unsigned int>(next);
    simulation_.reset(parameters.boid_count);
}

void Application::reset_simulation()
{
    simulation_.reset(simulation_.parameters().boid_count);
    metrics_ = sim::SimulationMetrics{};
}

void Application::mark_overlay_dirty() noexcept
{
    overlay_dirty_ = true;
}

void Application::refresh_overlay_text(float frame_time_ms)
{
    const auto& parameters = simulation_.parameters();
    std::size_t line = 0;

    write_literal(overlay_lines_[line++], "flock3d debug overlay");
    write_line(overlay_lines_[line++], "FPS: %d", GetFPS());
    write_line(overlay_lines_[line++], "Frame: %.2f ms", static_cast<double>(frame_time_ms));
    write_line(overlay_lines_[line++], "Boids: %zu", simulation_.size());
    write_line(overlay_lines_[line++], "Avg neighbors/boid: %.2f", static_cast<double>(metrics_.average_neighbors_per_boid));
    write_line(overlay_lines_[line++], "Sim update: %.3f ms", metrics_.simulation_update_ms);
    write_line(overlay_lines_[line++], "Render: %.3f ms", metrics_.render_ms);
    write_line(overlay_lines_[line++], "Spatial cells: %zu", metrics_.spatial_hash_cell_count);
    write_line(overlay_lines_[line++], "Neighbor queries: %llu", static_cast<unsigned long long>(metrics_.neighbor_queries));
    write_literal(overlay_lines_[line++], "");
    write_literal(overlay_lines_[line++], "Parameters");
    write_line(overlay_lines_[line++], "1/Shift+1 separation: %.2f", static_cast<double>(parameters.separation_weight));
    write_line(overlay_lines_[line++], "2/Shift+2 alignment: %.2f", static_cast<double>(parameters.alignment_weight));
    write_line(overlay_lines_[line++], "3/Shift+3 cohesion: %.2f", static_cast<double>(parameters.cohesion_weight));
    write_line(overlay_lines_[line++], "4/Shift+4 perception: %.2f", static_cast<double>(parameters.neighbor_radius));
    write_line(overlay_lines_[line++], "5/Shift+5 max speed: %.2f", static_cast<double>(parameters.max_speed));
    write_literal(overlay_lines_[line++], "");
    write_line(overlay_lines_[line++], "Controls: +/- boids, Space %s", paused_ ? "resume" : "pause");
    write_literal(overlay_lines_[line++], "R reset, F1 overlay, WASD/mouse camera");

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
