#include "benchmark_common.hpp"

#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <string_view>
#include <vector>

#include <flock3d/sim/BoidSimulation.hpp>
#include <flock3d/sim/SimulationParameters.hpp>

namespace {

using flock3d::bench::BenchmarkOptions;
using flock3d::bench::ProgressBar;
using flock3d::bench::UpdateStats;

struct NeighborMode {
    std::string_view name;
    bool adaptive_perception_enabled{};
    std::size_t max_selected_neighbors{};
    flock3d::sim::NeighborMode mode{};
};

void apply_neighbor_mode(flock3d::sim::SimulationParameters& parameters, NeighborMode mode)
{
    parameters.base_perception_radius = parameters.neighbor_radius;
    parameters.min_perception_radius = parameters.neighbor_radius * 0.5F;
    parameters.max_perception_radius = parameters.neighbor_radius * 1.5F;
    parameters.target_neighbor_count = 32U;
    parameters.max_selected_neighbors = mode.max_selected_neighbors;
    parameters.adaptive_perception_enabled = mode.adaptive_perception_enabled;
    parameters.neighbor_mode = mode.mode;
    flock3d::sim::sync_spatial_cell_size_to_query_radius(parameters);
}

void run_scenario(
    flock3d::sim::SimulationModel model,
    std::uint32_t boid_count,
    NeighborMode neighbor_mode,
    const BenchmarkOptions& options,
    ProgressBar& progress,
    std::uint32_t thread_count,
    std::vector<double>& single_thread_sample_means)
{
    auto parameters = flock3d::bench::parameters_for_model(model, boid_count, 12'345U + boid_count);
    apply_neighbor_mode(parameters, neighbor_mode);
    parameters.thread_count = thread_count;
    flock3d::sim::BoidSimulation simulation{parameters};
    const std::size_t warmup_ticks = flock3d::bench::simulated_seconds_to_ticks(options.warmup_seconds);
    for (std::size_t tick = 0; tick < warmup_ticks; ++tick) {
        simulation.update(flock3d::bench::fixed_dt, nullptr);
    }

    const std::size_t total_ticks = flock3d::bench::simulated_seconds_to_ticks(options.duration_seconds);
    const std::size_t sample_ticks = std::max<std::size_t>(1U, flock3d::bench::simulated_seconds_to_ticks(options.sample_seconds));
    std::size_t completed_ticks = 0U;
    int sample_index = 0;

    while (completed_ticks < total_ticks) {
        UpdateStats stats{};
        stats.samples_ms.reserve(sample_ticks);
        const std::size_t sample_start_tick = completed_ticks;
        while (completed_ticks < total_ticks && (stats.count == 0U || completed_ticks - sample_start_tick < sample_ticks)) {
            const double milliseconds = flock3d::bench::time_ms([&]() {
                simulation.update(flock3d::bench::fixed_dt, nullptr);
            });
            stats.record(milliseconds);
            ++completed_ticks;

            const double elapsed = flock3d::bench::ticks_to_simulated_seconds(completed_ticks);
            if ((stats.count % 64U) == 0U) {
                progress.update("simulation_update", neighbor_mode.name, boid_count, elapsed, options.duration_seconds);
            }
        }

        const double elapsed = flock3d::bench::ticks_to_simulated_seconds(completed_ticks);
        if (thread_count == 1U) {
            single_thread_sample_means.push_back(stats.mean_ms());
        }
        const double single_thread_mean = static_cast<std::size_t>(sample_index) < single_thread_sample_means.size()
            ? single_thread_sample_means[static_cast<std::size_t>(sample_index)]
            : stats.mean_ms();
        const double speedup = stats.mean_ms() > 0.0 ? single_thread_mean / stats.mean_ms() : 0.0;
        std::cout << "baseline," << flock3d::bench::model_name(model) << ',' << neighbor_mode.name << ',' << boid_count
                  << ',' << thread_count << ',' << std::fixed << std::setprecision(3) << elapsed << ',' << sample_index
                  << ',' << stats.count << ',' << stats.mean_ms() << ',' << stats.min_or_zero() << ',' << stats.max_ms
                  << ',' << speedup << '\n';
        ++sample_index;
    }
    progress.finish();
}

} // namespace

int main(int argc, char** argv)
{
    const BenchmarkOptions options = flock3d::bench::parse_options(argc, argv);
    ProgressBar progress{flock3d::bench::progress_enabled()};

    constexpr NeighborMode neighbor_modes[] = {
        NeighborMode{"fixed_radius_uncapped", false, 0U, flock3d::sim::NeighborMode::FixedRadiusUncapped},
        NeighborMode{"fixed_radius_closest_k", false, 32U, flock3d::sim::NeighborMode::FixedRadiusClosestK},
        NeighborMode{"adaptive_radius_closest_k", true, 32U, flock3d::sim::NeighborMode::AdaptiveRadiusClosestK},
    };

    std::cout << "scenario,model,neighbor_mode,boid_count,thread_count,elapsed_seconds,sample_index,"
                 "iterations_in_sample,mean_update_ms,min_update_ms,max_update_ms,speedup_vs_single_thread\n";
    for (const auto model : {
             flock3d::sim::SimulationModel::ClassicBoids,
             flock3d::sim::SimulationModel::BirdFlight,
             flock3d::sim::SimulationModel::FishSchool,
             flock3d::sim::SimulationModel::NoiseExperiment,
         }) {
        for (const std::uint32_t boid_count : flock3d::bench::benchmark_boid_counts()) {
            for (const NeighborMode neighbor_mode : neighbor_modes) {
                std::vector<double> single_thread_sample_means;
                for (const std::uint32_t thread_count : options.thread_counts) {
                    run_scenario(model, boid_count, neighbor_mode, options, progress, thread_count, single_thread_sample_means);
                }
            }
        }
    }

    return 0;
}
