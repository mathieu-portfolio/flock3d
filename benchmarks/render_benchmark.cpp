#include "benchmark_common.hpp"
#include "render/BoidRenderer.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string_view>
#include <vector>

#include <raylib.h>

#include <flock3d/sim/BoidSimulation.hpp>
#include <flock3d/sim/SimulationParameters.hpp>

namespace {

using flock3d::bench::Clock;
using flock3d::bench::UpdateStats;

struct RenderBenchmarkOptions {
    std::vector<std::uint32_t> boid_counts{1'000U, 5'000U, 10'000U};
    std::optional<double> duration_seconds{};
    std::optional<double> sample_seconds{};
    std::optional<double> warmup_seconds{};
    std::uint32_t frame_count{180U};
    std::uint32_t sample_frames{60U};
    std::uint32_t warmup_frames{30U};
    std::optional<std::uint32_t> seed{};
};

void print_render_usage(std::string_view executable)
{
    std::cerr << "Usage: " << executable
              << " [--counts 1000,5000,10000] [--frames count | --duration seconds]"
              << " [--sample seconds | --sample-frames count] [--warmup seconds | --warmup-frames count]"
              << " [--seed value]\n"
              << "Renders frozen deterministic boid states with a fixed camera and no simulation updates."
              << " CSV is printed to stdout; progress/status output is printed to stderr."
              << " Defaults: counts=1000,5000,10000, frames=180, sample-frames=60, warmup-frames=30.\n";
}

[[noreturn]] void fail_render_usage(std::string_view executable, std::string_view message)
{
    std::cerr << "render benchmark option error: " << message << "\n\n";
    print_render_usage(executable);
    std::exit(EXIT_FAILURE);
}

std::string_view require_render_value(int argc, char** argv, int& index, std::string_view argument)
{
    if (index + 1 >= argc) {
        fail_render_usage(argv[0], std::string{argument} + " requires a value");
    }
    ++index;
    return argv[index];
}

RenderBenchmarkOptions parse_render_options(int argc, char** argv)
{
    RenderBenchmarkOptions options{};
    for (int index = 1; index < argc; ++index) {
        const std::string_view argument{argv[index]};
        if (argument == "--help" || argument == "-h") {
            print_render_usage(argv[0]);
            std::exit(EXIT_SUCCESS);
        }
        if (argument == "--counts" || argument == "--boid-counts") {
            options.boid_counts = flock3d::bench::parse_positive_counts(require_render_value(argc, argv, index, argument));
            if (options.boid_counts.empty()) {
                fail_render_usage(argv[0], "--counts must be a comma-separated list of positive integers");
            }
        } else if (argument == "--duration") {
            const auto value = flock3d::bench::parse_double(require_render_value(argc, argv, index, argument));
            if (!value.has_value() || *value <= 0.0) {
                fail_render_usage(argv[0], "--duration must be a finite positive number");
            }
            options.duration_seconds = *value;
        } else if (argument == "--frames") {
            const auto value = flock3d::bench::parse_u32(require_render_value(argc, argv, index, argument));
            if (!value.has_value() || *value == 0U) {
                fail_render_usage(argv[0], "--frames must be a positive integer");
            }
            options.frame_count = *value;
            options.duration_seconds.reset();
        } else if (argument == "--sample") {
            const auto value = flock3d::bench::parse_double(require_render_value(argc, argv, index, argument));
            if (!value.has_value() || *value <= 0.0) {
                fail_render_usage(argv[0], "--sample must be a finite positive number");
            }
            options.sample_seconds = *value;
        } else if (argument == "--sample-frames") {
            const auto value = flock3d::bench::parse_u32(require_render_value(argc, argv, index, argument));
            if (!value.has_value() || *value == 0U) {
                fail_render_usage(argv[0], "--sample-frames must be a positive integer");
            }
            options.sample_frames = *value;
            options.sample_seconds.reset();
        } else if (argument == "--warmup") {
            const auto value = flock3d::bench::parse_double(require_render_value(argc, argv, index, argument));
            if (!value.has_value() || *value < 0.0) {
                fail_render_usage(argv[0], "--warmup must be a finite non-negative number");
            }
            options.warmup_seconds = *value;
        } else if (argument == "--warmup-frames") {
            const auto value = flock3d::bench::parse_u32(require_render_value(argc, argv, index, argument));
            if (!value.has_value()) {
                fail_render_usage(argv[0], "--warmup-frames must be a non-negative integer");
            }
            options.warmup_frames = *value;
            options.warmup_seconds.reset();
        } else if (argument == "--seed") {
            const auto value = flock3d::bench::parse_u32(require_render_value(argc, argv, index, argument));
            if (!value.has_value()) {
                fail_render_usage(argv[0], "--seed must be a non-negative integer");
            }
            options.seed = *value;
        } else {
            fail_render_usage(argv[0], std::string{"unknown option: "} + std::string{argument});
        }
    }
    return options;
}

flock3d::sim::BoidSimulation make_frozen_simulation(std::uint32_t boid_count, std::optional<std::uint32_t> seed)
{
    auto parameters = flock3d::bench::parameters_for_model(
        flock3d::sim::SimulationModel::ClassicBoids,
        boid_count,
        seed.value_or(12'345U + boid_count));
    parameters.thread_count = 1U;
    return flock3d::sim::BoidSimulation{parameters};
}

Camera3D make_fixed_camera()
{
    Camera3D camera{};
    camera.position = Vector3{0.0F, 35.0F, 85.0F};
    camera.target = Vector3{0.0F, 0.0F, 0.0F};
    camera.up = Vector3{0.0F, 1.0F, 0.0F};
    camera.fovy = 45.0F;
    camera.projection = CAMERA_PERSPECTIVE;
    return camera;
}

void render_frame(const flock3d::render::BoidRenderer& renderer, const flock3d::sim::BoidSimulation& simulation, const Camera3D& camera)
{
    BeginDrawing();
    ClearBackground(Color{12, 16, 24, 255});
    BeginMode3D(camera);
    renderer.draw(simulation);
    EndMode3D();
    EndDrawing();
}

double elapsed_seconds(Clock::time_point start, Clock::time_point stop)
{
    return std::chrono::duration<double>{stop - start}.count();
}

bool sample_should_continue(const RenderBenchmarkOptions& options, std::uint32_t sample_frame_count, Clock::time_point sample_start)
{
    if (sample_frame_count == 0U) {
        return true;
    }
    if (options.sample_seconds.has_value()) {
        return elapsed_seconds(sample_start, Clock::now()) < *options.sample_seconds;
    }
    return sample_frame_count < options.sample_frames;
}

bool benchmark_should_continue(const RenderBenchmarkOptions& options, std::uint32_t completed_frames, Clock::time_point benchmark_start)
{
    if (options.duration_seconds.has_value()) {
        return elapsed_seconds(benchmark_start, Clock::now()) < *options.duration_seconds;
    }
    return completed_frames < options.frame_count;
}

void run_warmup(
    const RenderBenchmarkOptions& options,
    const flock3d::render::BoidRenderer& renderer,
    const flock3d::sim::BoidSimulation& simulation,
    const Camera3D& camera)
{
    if (options.warmup_seconds.has_value()) {
        if (*options.warmup_seconds <= 0.0) {
            return;
        }
        const auto start = Clock::now();
        while (elapsed_seconds(start, Clock::now()) < *options.warmup_seconds) {
            render_frame(renderer, simulation, camera);
        }
        return;
    }

    for (std::uint32_t frame = 0; frame < options.warmup_frames; ++frame) {
        render_frame(renderer, simulation, camera);
    }
}

void run_count(std::uint32_t boid_count, const RenderBenchmarkOptions& options, const flock3d::render::BoidRenderer& renderer, const Camera3D& camera)
{
    auto simulation = make_frozen_simulation(boid_count, options.seed);
    run_warmup(options, renderer, simulation, camera);

    const char* scenario = "classic_frozen_render";
    std::uint32_t completed_frames = 0U;
    std::uint32_t sample_index = 0U;
    const auto benchmark_start = Clock::now();

    while (benchmark_should_continue(options, completed_frames, benchmark_start)) {
        UpdateStats stats{};
        const auto sample_start = Clock::now();
        while (benchmark_should_continue(options, completed_frames, benchmark_start)
            && sample_should_continue(options, static_cast<std::uint32_t>(stats.count), sample_start)) {
            const double milliseconds = flock3d::bench::time_ms([&]() {
                render_frame(renderer, simulation, camera);
            });
            stats.record(milliseconds);
            ++completed_frames;
        }

        if (stats.count == 0U) {
            break;
        }

        const double fps = stats.wall_seconds() > 0.0 ? static_cast<double>(stats.count) / stats.wall_seconds() : 0.0;
        const double elapsed = elapsed_seconds(benchmark_start, Clock::now());
        std::cout << scenario << ',' << boid_count << ',' << elapsed << ',' << sample_index << ',' << stats.count << ','
                  << stats.mean_ms() << ',' << stats.min_or_zero() << ',' << stats.max_ms << ','
                  << stats.p50_ms() << ',' << stats.p95_ms() << ',' << stats.p99_ms() << ',' << fps << '\n';
        ++sample_index;
    }
}

} // namespace

int main(int argc, char** argv)
{
    const RenderBenchmarkOptions options = parse_render_options(argc, argv);

    InitWindow(1280, 720, "flock3d render benchmark");
    SetTargetFPS(0);

    const Camera3D camera = make_fixed_camera();
    const flock3d::render::BoidRenderer renderer{};

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "scenario,boid_count,elapsed_seconds,sample_index,frames_in_sample,mean_render_ms,min_render_ms,max_render_ms,"
                 "p50_render_ms,p95_render_ms,p99_render_ms,frames_per_second\n";

    for (const std::uint32_t boid_count : options.boid_counts) {
        std::cerr << "Running render benchmark for " << boid_count << " frozen boids\n";
        run_count(boid_count, options, renderer, camera);
    }

    if (IsWindowReady()) {
        CloseWindow();
    }
    return EXIT_SUCCESS;
}
