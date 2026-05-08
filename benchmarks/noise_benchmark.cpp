#include "benchmark_common.hpp"

#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <string_view>

#include <flock3d/sim/BoidSimulation.hpp>
#include <flock3d/sim/SimulationParameters.hpp>

namespace {

using flock3d::bench::BenchmarkOptions;
using flock3d::bench::Clock;
using flock3d::bench::ProgressBar;
using flock3d::bench::UpdateStats;

struct NoiseVariant {
    std::string_view name;
    float steering{};
    float perception{};
    float velocity{};
};

flock3d::sim::SimulationParameters noise_parameters(std::uint32_t boid_count, const NoiseVariant& variant)
{
    auto parameters = flock3d::bench::parameters_for_model(
        flock3d::sim::SimulationModel::NoiseExperiment,
        boid_count,
        33'000U + boid_count);
    parameters.noise_enabled = true;
    parameters.noise_seed_offset = 10'000U;
    parameters.steering_noise_strength = variant.steering;
    parameters.perception_noise_strength = variant.perception;
    parameters.velocity_noise_strength = variant.velocity;
    flock3d::sim::sync_spatial_cell_size_to_query_radius(parameters);
    return parameters;
}

void run_scenario(const NoiseVariant& variant, std::uint32_t boid_count, const BenchmarkOptions& options, ProgressBar& progress)
{
    flock3d::sim::BoidSimulation simulation{noise_parameters(boid_count, variant)};

    const auto warmup_start = Clock::now();
    while (std::chrono::duration<double>(Clock::now() - warmup_start).count() < options.warmup_seconds) {
        simulation.update(flock3d::bench::fixed_dt, nullptr);
    }

    const auto benchmark_start = Clock::now();
    auto sample_start = benchmark_start;
    int sample_index = 0;

    while (std::chrono::duration<double>(Clock::now() - benchmark_start).count() < options.duration_seconds) {
        UpdateStats stats{};
        while (true) {
            const auto now = Clock::now();
            const double elapsed = std::chrono::duration<double>(now - benchmark_start).count();
            const double sample_elapsed = std::chrono::duration<double>(now - sample_start).count();
            if (elapsed >= options.duration_seconds || (stats.count > 0 && sample_elapsed >= options.sample_seconds)) {
                break;
            }

            const double milliseconds = flock3d::bench::time_ms([&simulation]() {
                simulation.update(flock3d::bench::fixed_dt, nullptr);
            });
            stats.record(milliseconds);

            if ((stats.count % 64U) == 0U) {
                progress.update("noise", variant.name, boid_count, elapsed, options.duration_seconds);
            }
        }

        const double elapsed = std::chrono::duration<double>(Clock::now() - benchmark_start).count();
        std::cout << "baseline," << variant.name << ',' << boid_count << ',' << std::fixed << std::setprecision(3)
                  << elapsed << ',' << sample_index << ',' << stats.count << ',' << stats.mean_ms() << ','
                  << stats.min_or_zero() << ',' << stats.max_ms << ',' << variant.steering << ',' << variant.perception
                  << ',' << variant.velocity << '\n';
        ++sample_index;
        sample_start = Clock::now();
    }
    progress.finish();
}

} // namespace

int main(int argc, char** argv)
{
    const BenchmarkOptions options = flock3d::bench::parse_options(argc, argv);
    ProgressBar progress{flock3d::bench::progress_enabled()};
    const NoiseVariant variants[] = {
        {"zero_noise", 0.0F, 0.0F, 0.0F},
        {"steering_noise", 0.25F, 0.0F, 0.0F},
        {"perception_noise", 0.0F, 0.25F, 0.0F},
        {"velocity_noise", 0.0F, 0.0F, 0.25F},
        {"all_noise", 0.25F, 0.25F, 0.25F},
    };

    std::cout << "scenario,noise_mode,boid_count,elapsed_seconds,sample_index,iterations_in_sample,mean_update_ms,min_update_ms,max_update_ms,steering_noise_strength,perception_noise_strength,velocity_noise_strength\n";
    for (const std::uint32_t boid_count : flock3d::bench::benchmark_boid_counts()) {
        for (const NoiseVariant& variant : variants) {
            run_scenario(variant, boid_count, options, progress);
        }
    }

    return 0;
}
