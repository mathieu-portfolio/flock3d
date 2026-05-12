#include "benchmark_common.hpp"

#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <string_view>
#include <vector>
#include <cstdlib>

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


constexpr NeighborMode all_neighbor_modes[] = {
    NeighborMode{"fixed_radius_uncapped", false, 0U, flock3d::sim::NeighborMode::FixedRadiusUncapped},
    NeighborMode{"fixed_radius_closest_k", false, 32U, flock3d::sim::NeighborMode::FixedRadiusClosestK},
    NeighborMode{"adaptive_radius_closest_k", true, 32U, flock3d::sim::NeighborMode::AdaptiveRadiusClosestK},
    NeighborMode{"aggregate_social", true, 32U, flock3d::sim::NeighborMode::CellAggregateSocial},
};

std::vector<flock3d::sim::SimulationModel> selected_models(const BenchmarkOptions& options)
{
    return flock3d::bench::selected_models_or_exit(options, "flock3d_simulation_update_benchmark");
}

std::vector<NeighborMode> selected_neighbor_modes(const BenchmarkOptions& options)
{
    if (options.mode_filters.empty()) {
        if (options.full_matrix) {
            return {std::begin(all_neighbor_modes), std::end(all_neighbor_modes)};
        }
        return {all_neighbor_modes[2]};
    }

    std::vector<NeighborMode> modes;
    for (const std::string& filter : options.mode_filters) {
        const auto match = std::find_if(std::begin(all_neighbor_modes), std::end(all_neighbor_modes), [&](NeighborMode mode) {
            return flock3d::bench::normalized_name_equal(filter, mode.name);
        });
        if (match == std::end(all_neighbor_modes)) {
            std::cerr << "Unknown mode '" << filter << "'. Known modes: fixed_radius_uncapped, fixed_radius_closest_k, adaptive_radius_closest_k, aggregate_social\n";
            flock3d::bench::print_usage("flock3d_simulation_update_benchmark");
            std::exit(EXIT_FAILURE);
        }
        if (std::find_if(modes.begin(), modes.end(), [&](NeighborMode mode) { return mode.name == match->name; }) == modes.end()) {
            modes.push_back(*match);
        }
    }
    return modes;
}

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
    const std::uint32_t seed = options.seed.value_or(12'345U + boid_count);
    auto parameters = flock3d::bench::parameters_for_model(model, boid_count, seed);
    apply_neighbor_mode(parameters, neighbor_mode);
    flock3d::bench::apply_parameter_overrides(parameters, options);
    parameters.thread_count = thread_count;
    parameters.thread_chunk_size = options.thread_chunk_size;
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
        UpdateStats rebuild_stats{};
        UpdateStats model_update_stats{};
        UpdateStats integration_stats{};
        UpdateStats metrics_stats{};
        UpdateStats instrumented_update_stats{};
        UpdateStats parallel_workspace_stats{};
        UpdateStats parallel_dispatch_stats{};
        double parallel_for_calls_total = 0.0;
        double parallel_worker_count_total = 0.0;
        const bool collect_timing_diagnostics = options.diagnostics_level != flock3d::bench::DiagnosticsLevel::None;
        stats.samples_ms.reserve(sample_ticks);
        const std::size_t sample_start_tick = completed_ticks;
        while (completed_ticks < total_ticks && (stats.count == 0U || completed_ticks - sample_start_tick < sample_ticks)) {
            flock3d::sim::SimulationTimingDiagnostics timing_diagnostics{};
            const double milliseconds = flock3d::bench::time_ms([&]() {
                simulation.update(flock3d::bench::fixed_dt, nullptr, collect_timing_diagnostics ? &timing_diagnostics : nullptr);
            });
            stats.record(milliseconds);
            if (collect_timing_diagnostics) {
                rebuild_stats.record(timing_diagnostics.rebuild_spatial_hash_ms);
                model_update_stats.record(timing_diagnostics.model_update_ms);
                integration_stats.record(timing_diagnostics.integration_ms);
                metrics_stats.record(timing_diagnostics.metrics_ms);
                instrumented_update_stats.record(timing_diagnostics.total_update_ms);
                parallel_workspace_stats.record(timing_diagnostics.parallel_workspace_ms);
                parallel_dispatch_stats.record(timing_diagnostics.parallel_dispatch_ms);
                parallel_for_calls_total += static_cast<double>(timing_diagnostics.parallel_for_calls);
                parallel_worker_count_total += static_cast<double>(timing_diagnostics.parallel_worker_count_total);
            }
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
        const std::uint32_t effective_workers = simulation.effective_thread_count();
        const std::size_t base_boids_per_worker = effective_workers > 0U ? boid_count / effective_workers : boid_count;
        const std::size_t remainder_boids = effective_workers > 0U ? boid_count % effective_workers : 0U;
        const double boids_per_worker_mean = effective_workers > 0U
            ? static_cast<double>(boid_count) / static_cast<double>(effective_workers)
            : 0.0;
        const std::size_t boids_per_worker_min = effective_workers > 0U ? base_boids_per_worker : 0U;
        const std::size_t boids_per_worker_max = effective_workers > 0U
            ? base_boids_per_worker + (remainder_boids > 0U ? 1U : 0U)
            : 0U;
        const double parallel_for_calls_mean = stats.count > 0U ? parallel_for_calls_total / static_cast<double>(stats.count) : 0.0;
        const double parallel_worker_count_mean = stats.count > 0U ? parallel_worker_count_total / static_cast<double>(stats.count) : 0.0;
        std::cout << "baseline," << flock3d::bench::model_name(model) << ',' << neighbor_mode.name << ',' << boid_count
                  << ',' << thread_count << ',' << effective_workers << ',' << std::fixed << std::setprecision(3) << elapsed << ',' << sample_index
                  << ',' << stats.count << ',' << stats.mean_ms() << ',' << stats.min_or_zero() << ',' << stats.max_ms
                  << ',' << speedup << ',' << parameters.random_seed << ',' << parameters.world_half_extent
                  << ',' << parameters.neighbor_radius << ',' << parameters.separation_radius << ',' << parameters.max_speed
                  << ',' << parameters.max_force << ',' << parameters.max_selected_neighbors << ','
                  << parameters.target_neighbor_count << ',' << (parameters.adaptive_perception_enabled ? "true" : "false");
        if (flock3d::bench::includes_phase_diagnostics(options.diagnostics_level)) {
            std::cout << ',' << model_update_stats.mean_ms() << ',' << integration_stats.mean_ms() << ',' << metrics_stats.mean_ms()
                      << ',' << rebuild_stats.mean_ms() << ',' << model_update_stats.mean_ms() << ',' << integration_stats.mean_ms()
                      << ',' << metrics_stats.mean_ms() << ',' << instrumented_update_stats.mean_ms();
        }
        if (flock3d::bench::includes_worker_diagnostics(options.diagnostics_level)) {
            std::cout << ',' << boids_per_worker_mean << ',' << boids_per_worker_min << ','
                      << boids_per_worker_max << ',' << parameters.thread_chunk_size << ',' << parallel_workspace_stats.mean_ms()
                      << ',' << parallel_dispatch_stats.mean_ms() << ',' << parallel_for_calls_mean << ',' << parallel_worker_count_mean;
        }
        std::cout << '\n';
        ++sample_index;
    }
    progress.finish();
}

} // namespace

int main(int argc, char** argv)
{
    const BenchmarkOptions options = flock3d::bench::parse_options(argc, argv);
    ProgressBar progress{flock3d::bench::progress_enabled()};

    const auto models = selected_models(options);
    const auto neighbor_modes = selected_neighbor_modes(options);

    std::cout << "scenario,model,neighbor_mode,boid_count,thread_count,worker_count_effective,elapsed_seconds,sample_index,"
                 "iterations_in_sample,mean_update_ms,min_update_ms,max_update_ms,speedup_vs_single_thread,"
                 "random_seed,world_half_extent,neighbor_radius,separation_radius,max_speed,max_force,"
                 "max_selected_neighbors,target_neighbor_count,adaptive_perception_enabled";
    if (flock3d::bench::includes_phase_diagnostics(options.diagnostics_level)) {
        std::cout << ",update_parallel_ms,integration_parallel_ms,serial_metrics_ms,rebuild_spatial_hash_ms,"
                     "model_update_ms,integration_ms,metrics_ms,instrumented_update_ms";
    }
    if (flock3d::bench::includes_worker_diagnostics(options.diagnostics_level)) {
        std::cout << ",boids_per_worker_mean,boids_per_worker_min,boids_per_worker_max,"
                     "chunk_size,parallel_workspace_ms,parallel_dispatch_ms,parallel_for_calls_mean,"
                     "parallel_worker_count_mean";
    }
    std::cout << '\n';
    for (const auto model : models) {
        for (const std::uint32_t boid_count : options.boid_counts) {
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
