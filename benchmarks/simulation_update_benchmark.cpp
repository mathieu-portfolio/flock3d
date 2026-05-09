#include "benchmark_common.hpp"

#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <string_view>

#include <flock3d/sim/BoidSimulation.hpp>
#include <flock3d/sim/SimulationMetrics.hpp>
#include <flock3d/sim/SimulationParameters.hpp>

namespace {

using flock3d::bench::BenchmarkOptions;
using flock3d::bench::Clock;
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
    ProgressBar& progress)
{
    auto parameters = flock3d::bench::parameters_for_model(model, boid_count, 12'345U + boid_count);
    apply_neighbor_mode(parameters, neighbor_mode);
    flock3d::sim::BoidSimulation simulation{parameters};
    flock3d::sim::SimulationMetrics metrics{};

    const auto warmup_start = Clock::now();
    while (std::chrono::duration<double>(Clock::now() - warmup_start).count() < options.warmup_seconds) {
        simulation.update(flock3d::bench::fixed_dt, &metrics);
    }

    const auto benchmark_start = Clock::now();
    auto sample_start = benchmark_start;
    int sample_index = 0;

    while (std::chrono::duration<double>(Clock::now() - benchmark_start).count() < options.duration_seconds) {
        UpdateStats stats{};
        flock3d::sim::SimulationMetrics sample_metrics{};
        while (true) {
            const auto now = Clock::now();
            const double elapsed = std::chrono::duration<double>(now - benchmark_start).count();
            const double sample_elapsed = std::chrono::duration<double>(now - sample_start).count();
            if (elapsed >= options.duration_seconds || (stats.count > 0 && sample_elapsed >= options.sample_seconds)) {
                break;
            }

            const double milliseconds = flock3d::bench::time_ms([&simulation, &sample_metrics]() {
                simulation.update(flock3d::bench::fixed_dt, &sample_metrics);
            });
            stats.record(milliseconds);

            if ((stats.count % 64U) == 0U) {
                progress.update(
                    "simulation_update",
                    neighbor_mode.name,
                    boid_count,
                    elapsed,
                    options.duration_seconds);
            }
        }

        const double elapsed = std::chrono::duration<double>(Clock::now() - benchmark_start).count();
        std::cout << "baseline," << flock3d::bench::model_name(model) << ',' << neighbor_mode.name << ','
                  << (neighbor_mode.adaptive_perception_enabled ? "true" : "false") << ',' << boid_count << ','
                  << std::fixed << std::setprecision(3) << elapsed << ',' << sample_index << ',' << stats.count << ','
                  << parameters.base_perception_radius << ',' << sample_metrics.effective_radius_mean << ','
                  << sample_metrics.selected_neighbors_mean << ',' << sample_metrics.avg_candidates_per_query << ','
                  << sample_metrics.avg_effective_neighbors_per_query << ','
                  << sample_metrics.exact_separation_neighbors_mean << ','
                  << sample_metrics.aggregate_cells_used_mean << ','
                  << sample_metrics.social_weight_sum_mean << ','
                  << sample_metrics.polarization << ',' << sample_metrics.cohesion << ','
                  << sample_metrics.nearest_neighbor_average_distance << ',' << sample_metrics.average_speed << ','
                  << stats.mean_ms() << ','
                  << stats.min_or_zero() << ',' << stats.max_ms << '\n';
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

    constexpr NeighborMode neighbor_modes[] = {
        NeighborMode{"fixed_radius_uncapped", false, 0U, flock3d::sim::NeighborMode::FixedRadiusUncapped},
        NeighborMode{"fixed_radius_closest_k", false, 32U, flock3d::sim::NeighborMode::FixedRadiusClosestK},
        NeighborMode{"adaptive_radius_closest_k", true, 32U, flock3d::sim::NeighborMode::AdaptiveRadiusClosestK},
        NeighborMode{"cell_aggregate_social", false, 0U, flock3d::sim::NeighborMode::CellAggregateSocial},
    };

    std::cout << "scenario,model,neighbor_mode,adaptive_perception_enabled,boid_count,elapsed_seconds,sample_index,"
                 "iterations_in_sample,base_perception_radius,effective_radius_mean,selected_neighbors_mean,"
                 "candidates_per_query,effective_neighbors_per_query,exact_separation_neighbors_mean,"
                 "aggregate_cells_used_mean,social_weight_sum_mean,polarization,flock_spread,"
                 "nearest_neighbor_distance,average_speed,mean_update_ms,min_update_ms,max_update_ms\n";
    for (const auto model : {
             flock3d::sim::SimulationModel::ClassicBoids,
             flock3d::sim::SimulationModel::BirdFlight,
             flock3d::sim::SimulationModel::FishSchool,
             flock3d::sim::SimulationModel::NoiseExperiment,
         }) {
        for (const std::uint32_t boid_count : flock3d::bench::benchmark_boid_counts()) {
            for (const NeighborMode neighbor_mode : neighbor_modes) {
                run_scenario(model, boid_count, neighbor_mode, options, progress);
            }
        }
    }

    return 0;
}
