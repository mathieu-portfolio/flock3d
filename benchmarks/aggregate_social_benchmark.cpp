#include "benchmark_common.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string_view>
#include <cstdlib>
#include <vector>

#include <flock3d/sim/BoidSimulation.hpp>
#include <flock3d/sim/SimulationMetrics.hpp>
#include <flock3d/sim/SimulationParameters.hpp>

namespace {

using flock3d::bench::BenchmarkOptions;
using flock3d::bench::ProgressBar;
using flock3d::bench::UpdateStats;

struct AggregateSocialVariant {
    std::string_view name;
    bool social_fov_enabled{};
    bool adaptive_social_radius_enabled{};
};


constexpr AggregateSocialVariant all_variants[] = {
    {"cell_aggregate_social_no_fov", false, false},
    {"cell_aggregate_social_fov", true, false},
    {"cell_aggregate_social_adaptive_radius", false, true},
    {"cell_aggregate_social_fov_adaptive_radius", true, true},
};

std::vector<flock3d::sim::SimulationModel> selected_models(const BenchmarkOptions& options)
{
    return flock3d::bench::selected_models_or_exit(options, "flock3d_aggregate_social_benchmark");
}

std::vector<AggregateSocialVariant> selected_variants(const BenchmarkOptions& options)
{
    if (options.mode_filters.empty()) {
        if (options.full_matrix) {
            return {std::begin(all_variants), std::end(all_variants)};
        }
        return {all_variants[2]};
    }

    std::vector<AggregateSocialVariant> variants;
    for (const std::string& filter : options.mode_filters) {
        const auto match = std::find_if(std::begin(all_variants), std::end(all_variants), [&](AggregateSocialVariant variant) {
            return flock3d::bench::normalized_name_equal(filter, variant.name);
        });
        if (match == std::end(all_variants)) {
            std::cerr << "Unknown aggregate social mode '" << filter << "'. Known modes: cell_aggregate_social_no_fov, cell_aggregate_social_fov, cell_aggregate_social_adaptive_radius, cell_aggregate_social_fov_adaptive_radius\n";
            flock3d::bench::print_usage("flock3d_aggregate_social_benchmark");
            std::exit(EXIT_FAILURE);
        }
        if (std::find_if(variants.begin(), variants.end(), [&](AggregateSocialVariant variant) { return variant.name == match->name; }) == variants.end()) {
            variants.push_back(*match);
        }
    }
    return variants;
}

struct AggregateSocialDiagnostics {
    double aggregate_query_radius_total{};
    double aggregate_query_radius_min{std::numeric_limits<double>::max()};
    double aggregate_query_radius_max{};
    double visible_aggregate_cells_mean{};
    double rejected_aggregate_cells_mean{};
    double aggregate_cells_used_mean{};
    double exact_separation_neighbors_mean{};
    std::size_t exact_separation_neighbors_max{};
    double social_weight_sum_mean{};
    float flock_spread{};
    float polarization{};
    std::size_t count{};

    void record(const flock3d::sim::SimulationMetrics& metrics) noexcept
    {
        aggregate_query_radius_total += metrics.effective_radius_mean;
        aggregate_query_radius_min = std::min(aggregate_query_radius_min, metrics.effective_radius_mean);
        aggregate_query_radius_max = std::max(aggregate_query_radius_max, metrics.effective_radius_mean);
        visible_aggregate_cells_mean += metrics.aggregate_candidate_cells_per_query;
        rejected_aggregate_cells_mean += metrics.aggregate_radius_rejected_cells_mean + metrics.aggregate_fov_rejected_cells_mean;
        aggregate_cells_used_mean += metrics.aggregate_cells_used_mean;
        exact_separation_neighbors_mean += metrics.exact_separation_neighbors_mean;
        exact_separation_neighbors_max = std::max(exact_separation_neighbors_max, metrics.exact_separation_neighbors_max);
        social_weight_sum_mean += metrics.social_weight_sum_mean;
        flock_spread = metrics.cohesion;
        polarization = metrics.polarization;
        ++count;
    }

    [[nodiscard]] double mean(double total) const noexcept
    {
        return count > 0U ? total / static_cast<double>(count) : 0.0;
    }

    [[nodiscard]] double aggregate_query_radius_mean() const noexcept { return mean(aggregate_query_radius_total); }
    [[nodiscard]] double aggregate_query_radius_min_or_zero() const noexcept
    {
        return count > 0U ? aggregate_query_radius_min : 0.0;
    }
    [[nodiscard]] double visible_aggregate_cells_mean_value() const noexcept { return mean(visible_aggregate_cells_mean); }
    [[nodiscard]] double rejected_aggregate_cells_mean_value() const noexcept { return mean(rejected_aggregate_cells_mean); }
    [[nodiscard]] double aggregate_cells_used_mean_value() const noexcept { return mean(aggregate_cells_used_mean); }
    [[nodiscard]] double exact_separation_neighbors_mean_value() const noexcept { return mean(exact_separation_neighbors_mean); }
    [[nodiscard]] double social_weight_sum_mean_value() const noexcept { return mean(social_weight_sum_mean); }
};

flock3d::sim::SimulationParameters aggregate_social_parameters(
    flock3d::sim::SimulationModel model,
    std::uint32_t boid_count,
    std::uint32_t seed,
    const AggregateSocialVariant& variant)
{
    auto parameters = flock3d::bench::parameters_for_model(model, boid_count, seed);
    parameters.neighbor_mode = flock3d::sim::NeighborMode::CellAggregateSocial;
    parameters.base_perception_radius = parameters.neighbor_radius;
    parameters.min_perception_radius = parameters.neighbor_radius * 0.5F;
    parameters.max_perception_radius = parameters.neighbor_radius * 1.5F;
    parameters.target_neighbor_count = 32U;
    parameters.max_selected_neighbors = 0U;
    parameters.adaptive_perception_enabled = variant.adaptive_social_radius_enabled;
    parameters.aggregate_social_field_of_view_enabled = variant.social_fov_enabled;
    parameters.field_of_view_degrees = variant.social_fov_enabled ? 220.0F : 360.0F;
    flock3d::sim::sync_spatial_cell_size_to_query_radius(parameters);
    return parameters;
}

void run_scenario(
    flock3d::sim::SimulationModel model,
    const AggregateSocialVariant& variant,
    std::uint32_t boid_count,
    const BenchmarkOptions& options,
    ProgressBar& progress,
    std::uint32_t thread_count,
    std::vector<double>& single_thread_sample_means)
{
    const std::uint32_t seed = options.seed.value_or(44'000U + boid_count);
    auto parameters = aggregate_social_parameters(model, boid_count, seed, variant);
    flock3d::bench::apply_parameter_overrides(parameters, options);
    parameters.thread_count = thread_count;
    parameters.thread_chunk_size = options.thread_chunk_size;
    flock3d::sim::BoidSimulation simulation{parameters};
    const bool collect_internal_diagnostics = flock3d::bench::includes_internal_diagnostics(options.diagnostics_level);
    // SimulationMetrics recording is intentionally serial inside BoidSimulation to avoid
    // contended metric writes. Keep diagnostics on a separate simulation so the timed
    // aggregate-social update exercises the requested thread count instead of measuring
    // serial instrumentation plus parallel integration overhead.
    auto diagnostic_simulation = collect_internal_diagnostics
        ? std::make_unique<flock3d::sim::BoidSimulation>(parameters)
        : nullptr;
    flock3d::sim::SimulationMetrics metrics{};

    const std::size_t warmup_ticks = flock3d::bench::simulated_seconds_to_ticks(options.warmup_seconds);
    for (std::size_t tick = 0; tick < warmup_ticks; ++tick) {
        simulation.update(flock3d::bench::fixed_dt, nullptr);
        if (diagnostic_simulation != nullptr) {
            diagnostic_simulation->update(flock3d::bench::fixed_dt, &metrics);
        }
    }

    const std::size_t total_ticks = flock3d::bench::simulated_seconds_to_ticks(options.duration_seconds);
    const std::size_t sample_ticks = std::max<std::size_t>(1U, flock3d::bench::simulated_seconds_to_ticks(options.sample_seconds));
    std::size_t completed_ticks = 0U;
    int sample_index = 0;

    while (completed_ticks < total_ticks) {
        UpdateStats stats{};
        UpdateStats metrics_update_stats{};
        stats.samples_ms.reserve(sample_ticks);
        AggregateSocialDiagnostics diagnostics{};
        const std::size_t sample_start_tick = completed_ticks;
        while (completed_ticks < total_ticks && (stats.count == 0U || completed_ticks - sample_start_tick < sample_ticks)) {
            const double milliseconds = flock3d::bench::time_ms([&]() {
                simulation.update(flock3d::bench::fixed_dt, nullptr);
            });
            stats.record(milliseconds);
            if (diagnostic_simulation != nullptr) {
                const double metrics_milliseconds = flock3d::bench::time_ms([&]() {
                    diagnostic_simulation->update(flock3d::bench::fixed_dt, &metrics);
                });
                metrics_update_stats.record(metrics_milliseconds);
                diagnostics.record(metrics);
            }
            ++completed_ticks;

            const double elapsed = flock3d::bench::ticks_to_simulated_seconds(completed_ticks);
            if ((stats.count % 64U) == 0U) {
                progress.update("aggregate_social", variant.name, boid_count, elapsed, options.duration_seconds);
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
        std::cout << flock3d::bench::model_name(model) << ',' << variant.name << ',' << boid_count << ',' << thread_count << ','
                  << effective_workers << ',' << std::fixed << std::setprecision(3) << elapsed << ',' << sample_index << ',' << stats.count << ','
                  << stats.mean_ms() << ',' << stats.min_or_zero() << ',' << stats.max_ms << ',' << speedup << ','
                  << (variant.social_fov_enabled ? "true" : "false") << ','
                  << (variant.adaptive_social_radius_enabled ? "true" : "false");
        if (flock3d::bench::includes_internal_diagnostics(options.diagnostics_level)) {
            std::cout << ',' << metrics_update_stats.mean_ms() << ",true,"
                      << diagnostics.visible_aggregate_cells_mean_value() << ','
                      << diagnostics.rejected_aggregate_cells_mean_value() << ','
                      << diagnostics.aggregate_cells_used_mean_value() << ',' << diagnostics.aggregate_query_radius_mean() << ','
                      << diagnostics.aggregate_query_radius_min_or_zero() << ',' << diagnostics.aggregate_query_radius_max << ','
                      << diagnostics.exact_separation_neighbors_mean_value() << ','
                      << diagnostics.exact_separation_neighbors_max << ',' << diagnostics.social_weight_sum_mean_value() << ','
                      << diagnostics.flock_spread << ',' << diagnostics.polarization << ',' << parameters.random_seed
                      << ',' << parameters.world_half_extent << ',' << parameters.neighbor_radius << ','
                      << parameters.separation_radius << ',' << parameters.max_speed << ',' << parameters.max_force
                      << ',' << parameters.max_selected_neighbors << ',' << parameters.target_neighbor_count << ','
                      << (parameters.adaptive_perception_enabled ? "true" : "false");
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
    const auto variants = selected_variants(options);

    std::cout << "scenario,aggregate_social_mode,boid_count,thread_count,worker_count_effective,elapsed_seconds,sample_index,"
                 "iterations_in_sample,mean_update_ms,min_update_ms,max_update_ms,speedup_vs_single_thread,"
                 "social_fov_enabled,adaptive_social_radius_enabled";
    if (flock3d::bench::includes_internal_diagnostics(options.diagnostics_level)) {
        std::cout << ",mean_metrics_update_ms,aggregate_social_enabled,visible_aggregate_cells_mean,"
                     "rejected_aggregate_cells_mean,aggregate_cells_used_mean,aggregate_query_radius_mean,"
                     "aggregate_query_radius_min,aggregate_query_radius_max,exact_separation_neighbors_mean,"
                     "exact_separation_neighbors_max,social_weight_sum_mean,flock_spread,polarization,random_seed,"
                     "world_half_extent,neighbor_radius,separation_radius,max_speed,max_force,max_selected_neighbors,"
                     "target_neighbor_count,adaptive_perception_enabled";
    }
    std::cout << '\n';
    for (const auto model : models) {
        for (const std::uint32_t boid_count : options.boid_counts) {
            for (const AggregateSocialVariant& variant : variants) {
                std::vector<double> single_thread_sample_means;
                for (const std::uint32_t thread_count : options.thread_counts) {
                    run_scenario(model, variant, boid_count, options, progress, thread_count, single_thread_sample_means);
                }
            }
        }
    }

    return 0;
}
