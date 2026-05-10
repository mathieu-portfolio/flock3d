#include "benchmark_common.hpp"

#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <string_view>

#include <flock3d/sim/BoidSimulation.hpp>
#include <flock3d/sim/SimulationMetrics.hpp>
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
    ProgressBar& progress)
{
    auto parameters = flock3d::bench::parameters_for_model(model, boid_count, 12'345U + boid_count);
    apply_neighbor_mode(parameters, neighbor_mode);
    flock3d::sim::BoidSimulation simulation{parameters};
    flock3d::sim::SimulationMetrics metrics{};

    const std::size_t warmup_ticks = flock3d::bench::simulated_seconds_to_ticks(options.warmup_seconds);
    for (std::size_t tick = 0; tick < warmup_ticks; ++tick) {
        simulation.update(flock3d::bench::fixed_dt, &metrics);
    }

    const std::size_t total_ticks = flock3d::bench::simulated_seconds_to_ticks(options.duration_seconds);
    const std::size_t sample_ticks = std::max<std::size_t>(1U, flock3d::bench::simulated_seconds_to_ticks(options.sample_seconds));
    std::size_t completed_ticks = 0U;
    int sample_index = 0;

    while (completed_ticks < total_ticks) {
        UpdateStats stats{};
        stats.samples_ms.reserve(sample_ticks);
        flock3d::sim::SimulationMetrics sample_metrics{};
        const std::size_t sample_start_tick = completed_ticks;
        while (completed_ticks < total_ticks && (stats.count == 0U || completed_ticks - sample_start_tick < sample_ticks)) {
            const double milliseconds = flock3d::bench::time_ms([&]() {
                simulation.update(flock3d::bench::fixed_dt, &sample_metrics);
            });
            stats.record(milliseconds);
            ++completed_ticks;

            const double elapsed = flock3d::bench::ticks_to_simulated_seconds(completed_ticks);
            if ((stats.count % 64U) == 0U) {
                progress.update("simulation_update", neighbor_mode.name, boid_count, elapsed, options.duration_seconds);
            }
        }

        const double elapsed = flock3d::bench::ticks_to_simulated_seconds(completed_ticks);
        const std::size_t ticks_in_sample = stats.count;
        const std::size_t simulated_ticks = completed_ticks;
        std::cout << "baseline," << flock3d::bench::model_name(model) << ',' << neighbor_mode.name << ','
                  << (neighbor_mode.adaptive_perception_enabled ? "true" : "false") << ',' << boid_count << ','
                  << std::fixed << std::setprecision(3) << elapsed << ',' << sample_index << ',' << stats.count << ','
                  << parameters.base_perception_radius << ',' << flock3d::sim::effective_query_radius(parameters) << ','
                  << parameters.spatial_cell_size << ',' << parameters.max_selected_neighbors << ','
                  << parameters.target_neighbor_count << ',' << parameters.field_of_view_degrees << ','
                  << parameters.max_turn_rate << ',' << parameters.drag_coefficient << ','
                  << parameters.steering_noise_strength << ',' << parameters.perception_noise_strength << ','
                  << parameters.velocity_noise_strength << ',' << sample_metrics.effective_radius_mean << ','
                  << sample_metrics.selected_neighbors_mean << ',' << sample_metrics.accepted_neighbors_before_topology_mean << ','
                  << sample_metrics.topology_truncated_neighbors_mean << ',' << sample_metrics.topology_truncation_rate << ','
                  << sample_metrics.fov_rejected_neighbors_mean << ',' << sample_metrics.radius_rejected_neighbors_mean << ','
                  << sample_metrics.avg_candidates_per_query << ',' << sample_metrics.max_candidates_per_query << ','
                  << sample_metrics.avg_effective_neighbors_per_query << ','
                  << sample_metrics.max_effective_neighbors_per_query << ',' << sample_metrics.visited_cells_per_query << ','
                  << sample_metrics.spatial_cell_count << ',' << sample_metrics.avg_cell_occupancy << ','
                  << sample_metrics.max_cell_occupancy << ',' << sample_metrics.exact_separation_neighbors_mean << ','
                  << sample_metrics.aggregate_visited_cells_per_query << ','
                  << sample_metrics.aggregate_candidate_cells_per_query << ','
                  << sample_metrics.aggregate_radius_rejected_cells_mean << ','
                  << sample_metrics.aggregate_fov_rejected_cells_mean << ','
                  << sample_metrics.total_spatial_visited_cells_per_query << ','
                  << sample_metrics.total_spatial_candidates_per_query << ','
                  << sample_metrics.aggregate_cells_used_mean << ',' << sample_metrics.social_weight_sum_mean << ','
                  << sample_metrics.polarization << ',' << sample_metrics.cohesion << ','
                  << sample_metrics.nearest_neighbor_average_distance << ',' << sample_metrics.average_speed << ','
                  << sample_metrics.altitude_variance << ',' << sample_metrics.stall_count << ','
                  << sample_metrics.near_ground_count << ',' << sample_metrics.depth_variance << ','
                  << stats.mean_ms() << ','
                  << stats.min_or_zero() << ',' << stats.max_ms << ',' << elapsed << ',' << simulated_ticks << ','
                  << ticks_in_sample << ',' << stats.wall_seconds() << ',' << stats.mean_ns() << ','
                  << stats.p50_ms() << ',' << stats.p95_ms() << ',' << stats.p99_ms() << ','
                  << stats.ticks_per_second() << ',' << stats.ticks_per_second() << ',' << stats.real_time_factor() << '\n';
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
        NeighborMode{"cell_aggregate_social", false, 0U, flock3d::sim::NeighborMode::CellAggregateSocial},
    };

    std::cout << "scenario,model,neighbor_mode,adaptive_perception_enabled,boid_count,elapsed_seconds,sample_index,"
                 "iterations_in_sample,base_perception_radius,query_radius,spatial_cell_size,max_selected_neighbors,"
                 "target_neighbor_count,field_of_view_degrees,max_turn_rate,drag_coefficient,steering_noise_strength,"
                 "perception_noise_strength,velocity_noise_strength,effective_radius_mean,selected_neighbors_mean,"
                 "accepted_before_topology_mean,topology_truncated_mean,topology_truncation_rate,"
                 "fov_rejected_mean,radius_rejected_mean,candidates_per_query,max_candidates_per_query,"
                 "effective_neighbors_per_query,max_effective_neighbors_per_query,visited_cells_per_query,"
                 "spatial_cell_count,avg_cell_occupancy,max_cell_occupancy,exact_separation_neighbors_mean,"
                 "aggregate_visited_cells_per_query,aggregate_candidate_cells_per_query,"
                 "aggregate_radius_rejected_cells_mean,aggregate_fov_rejected_cells_mean,"
                 "total_spatial_visited_cells_per_query,total_spatial_candidates_per_query,"
                 "aggregate_cells_used_mean,social_weight_sum_mean,polarization,flock_spread,"
                 "nearest_neighbor_distance,average_speed,altitude_variance,stall_count,near_ground_count,"
                 "depth_variance,mean_update_ms,min_update_ms,max_update_ms,simulated_seconds,simulated_ticks,"
                 "ticks_in_sample,sample_wall_seconds,mean_ns_per_tick,p50_update_ms,p95_update_ms,"
                 "p99_update_ms,ticks_per_second,updates_per_second,real_time_factor\n";
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
