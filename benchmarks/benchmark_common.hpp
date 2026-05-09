#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

#if defined(_WIN32)
#include <io.h>
#else
#include <unistd.h>
#endif

#include <flock3d/sim/Scenario.hpp>
#include <flock3d/sim/SimulationParameters.hpp>

namespace flock3d::bench {

using Clock = std::chrono::steady_clock;

inline constexpr double default_duration_seconds = 30.0;
inline constexpr double default_sample_seconds = 5.0;
inline constexpr double default_warmup_seconds = 1.0;
inline constexpr float fixed_dt = 1.0F / 120.0F;

using flock3d::sim::SimulationParameters;
using flock3d::sim::sync_spatial_cell_size_to_query_radius;

struct BenchmarkOptions {
    double duration_seconds{default_duration_seconds};
    double sample_seconds{default_sample_seconds};
    double warmup_seconds{default_warmup_seconds};
};

struct UpdateStats {
    std::size_t count{};
    double total_ms{};
    double min_ms{std::numeric_limits<double>::max()};
    double max_ms{};

    void record(double milliseconds) noexcept
    {
        ++count;
        total_ms += milliseconds;
        min_ms = std::min(min_ms, milliseconds);
        max_ms = std::max(max_ms, milliseconds);
    }

    [[nodiscard]] double mean_ms() const noexcept
    {
        return count > 0 ? total_ms / static_cast<double>(count) : 0.0;
    }

    [[nodiscard]] double min_or_zero() const noexcept
    {
        return count > 0 ? min_ms : 0.0;
    }
};

inline void print_usage(std::string_view executable)
{
    std::cerr << "Usage: " << executable
              << " [--duration seconds] [--sample seconds] [--warmup seconds]\n"
              << "CSV is printed to stdout. Progress is printed to stderr only when stderr is a terminal.\n";
}

inline BenchmarkOptions parse_options(int argc, char** argv)
{
    BenchmarkOptions options{};
    for (int index = 1; index < argc; ++index) {
        const std::string_view argument{argv[index]};
        if (argument == "--help" || argument == "-h") {
            print_usage(argv[0]);
            std::exit(EXIT_SUCCESS);
        }
        if (index + 1 >= argc) {
            print_usage(argv[0]);
            std::exit(EXIT_FAILURE);
        }

        const double value = std::atof(argv[++index]);
        if (argument == "--duration") {
            options.duration_seconds = std::max(0.001, value);
        } else if (argument == "--sample") {
            options.sample_seconds = std::max(0.001, value);
        } else if (argument == "--warmup") {
            options.warmup_seconds = std::max(0.0, value);
        } else {
            print_usage(argv[0]);
            std::exit(EXIT_FAILURE);
        }
    }
    return options;
}

inline bool stream_is_terminal(int file_descriptor) noexcept
{
#if defined(_WIN32)
    return _isatty(file_descriptor) != 0;
#else
    return isatty(file_descriptor) != 0;
#endif
}

inline bool progress_enabled_for_streams(bool stdout_is_terminal, bool stderr_is_terminal) noexcept
{
    (void)stdout_is_terminal;
    return stderr_is_terminal;
}

inline bool progress_enabled() noexcept
{
#if defined(_WIN32)
    return progress_enabled_for_streams(stream_is_terminal(_fileno(stdout)), stream_is_terminal(_fileno(stderr)));
#else
    return progress_enabled_for_streams(stream_is_terminal(STDOUT_FILENO), stream_is_terminal(STDERR_FILENO));
#endif
}

class ProgressBar {
public:
    explicit ProgressBar(bool enabled) noexcept
        : enabled_{enabled}
    {
    }

    void update(std::string_view benchmark, std::string_view scenario, std::size_t boid_count, double elapsed, double target)
    {
        if (!enabled_) {
            return;
        }
        const double ratio = target > 0.0 ? std::clamp(elapsed / target, 0.0, 1.0) : 1.0;
        const int percent = static_cast<int>(std::round(ratio * 100.0));
        constexpr int width = 20;
        const int filled = static_cast<int>(std::round(ratio * static_cast<double>(width)));

        std::cerr << '\r' << '[';
        for (int index = 0; index < width; ++index) {
            std::cerr << (index < filled ? '#' : '-');
        }
        std::cerr << "] " << std::setw(3) << percent << "% | " << benchmark << " | " << scenario << " | "
                  << boid_count << " boids | " << std::fixed << std::setprecision(1) << elapsed << "s / " << target
                  << 's' << std::flush;
    }

    void finish()
    {
        if (enabled_) {
            std::cerr << "\033[2K\r" << std::flush;
        }
    }

private:
    bool enabled_{};
};

inline std::vector<std::uint32_t> benchmark_boid_counts()
{
    return {64U, 128U, 256U, 384U};
}

inline SimulationParameters base_parameters(std::uint32_t boid_count, std::uint32_t seed)
{
    SimulationParameters parameters{};
    parameters.boid_count = boid_count;
    parameters.random_seed = seed;
    parameters.world_half_extent = 40.0F;
    parameters.neighbor_radius = 4.0F;
    parameters.separation_radius = 2.0F;
    parameters.max_speed = 10.0F;
    parameters.max_force = 12.0F;
    sync_spatial_cell_size_to_query_radius(parameters);
    return parameters;
}

inline SimulationParameters parameters_for_model(flock3d::sim::SimulationModel model, std::uint32_t boid_count, std::uint32_t seed)
{
    using flock3d::sim::ScenarioType;
    ScenarioType scenario = ScenarioType::ClassicBoids;
    switch (model) {
    case flock3d::sim::SimulationModel::ClassicBoids:
        scenario = ScenarioType::ClassicBoids;
        break;
    case flock3d::sim::SimulationModel::BirdFlight:
        scenario = ScenarioType::BirdFlight;
        break;
    case flock3d::sim::SimulationModel::FishSchool:
        scenario = ScenarioType::FishSchool;
        break;
    case flock3d::sim::SimulationModel::NoiseExperiment:
        scenario = ScenarioType::NoiseExperiment;
        break;
    }

    SimulationParameters parameters = flock3d::sim::build_scenario(scenario).simulation_parameters;
    parameters.boid_count = boid_count;
    parameters.random_seed = seed;
    sync_spatial_cell_size_to_query_radius(parameters);
    return parameters;
}

inline std::string_view model_name(flock3d::sim::SimulationModel model) noexcept
{
    switch (model) {
    case flock3d::sim::SimulationModel::ClassicBoids:
        return "ClassicBoids";
    case flock3d::sim::SimulationModel::BirdFlight:
        return "BirdFlight";
    case flock3d::sim::SimulationModel::FishSchool:
        return "FishSchool";
    case flock3d::sim::SimulationModel::NoiseExperiment:
        return "NoiseExperiment";
    }
    return "Unknown";
}

template <typename Fn>
inline double time_ms(Fn&& function)
{
    const auto start = Clock::now();
    function();
    const auto stop = Clock::now();
    return std::chrono::duration<double, std::milli>(stop - start).count();
}

} // namespace flock3d::bench
