#include "benchmark_common.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string_view>

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

flock3d::sim::SimulationParameters aggregate_social_parameters(std::uint32_t boid_count, const AggregateSocialVariant& variant)
{
    auto parameters = flock3d::bench::parameters_for_model(
        flock3d::sim::SimulationModel::ClassicBoids,
        boid_count,
        44'000U + boid_count);
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
    const AggregateSocialVariant& variant,
    std::uint32_t boid_count,
    const BenchmarkOptions& options,
    ProgressBar& progress,
    std::uint32_t thread_count)
{
    auto parameters = aggregate_social_parameters(boid_count, variant);
    parameters.thread_count = thread_count;
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
        AggregateSocialDiagnostics diagnostics{};
        const std::size_t sample_start_tick = completed_ticks;
        while (completed_ticks < total_ticks && (stats.count == 0U || completed_ticks - sample_start_tick < sample_ticks)) {
            const double milliseconds = flock3d::bench::time_ms([&]() {
                simulation.update(flock3d::bench::fixed_dt, &metrics);
            });
            stats.record(milliseconds);
            diagnostics.record(metrics);
            ++completed_ticks;

            const double elapsed = flock3d::bench::ticks_to_simulated_seconds(completed_ticks);
            if ((stats.count % 64U) == 0U) {
                progress.update("aggregate_social", variant.name, boid_count, elapsed, options.duration_seconds);
            }
        }

        const double elapsed = flock3d::bench::ticks_to_simulated_seconds(completed_ticks);
        std::cout << "baseline," << variant.name << ',' << boid_count << ',' << thread_count << ',' << std::fixed
                  << std::setprecision(3) << elapsed << ',' << sample_index << ',' << stats.count << ','
                  << stats.mean_ms() << ',' << stats.min_or_zero() << ',' << stats.max_ms << ",true,"
                  << (variant.social_fov_enabled ? "true" : "false") << ','
                  << (variant.adaptive_social_radius_enabled ? "true" : "false") << ','
                  << diagnostics.visible_aggregate_cells_mean_value() << ','
                  << diagnostics.rejected_aggregate_cells_mean_value() << ','
                  << diagnostics.aggregate_cells_used_mean_value() << ',' << diagnostics.aggregate_query_radius_mean() << ','
                  << diagnostics.aggregate_query_radius_min_or_zero() << ',' << diagnostics.aggregate_query_radius_max << ','
                  << diagnostics.exact_separation_neighbors_mean_value() << ','
                  << diagnostics.exact_separation_neighbors_max << ',' << diagnostics.social_weight_sum_mean_value() << ','
                  << diagnostics.flock_spread << ',' << diagnostics.polarization << '\n';
        ++sample_index;
    }
    progress.finish();
}

} // namespace

int main(int argc, char** argv)
{
    const BenchmarkOptions options = flock3d::bench::parse_options(argc, argv);
    ProgressBar progress{flock3d::bench::progress_enabled()};
    constexpr AggregateSocialVariant variants[] = {
        {"cell_aggregate_social_no_fov", false, false},
        {"cell_aggregate_social_fov", true, false},
        {"cell_aggregate_social_adaptive_radius", false, true},
        {"cell_aggregate_social_fov_adaptive_radius", true, true},
    };

    std::cout << "scenario,aggregate_social_mode,boid_count,thread_count,elapsed_seconds,sample_index,"
                 "iterations_in_sample,mean_update_ms,min_update_ms,max_update_ms,aggregate_social_enabled,social_fov_enabled,"
                 "adaptive_social_radius_enabled,visible_aggregate_cells_mean,rejected_aggregate_cells_mean,"
                 "aggregate_cells_used_mean,aggregate_query_radius_mean,aggregate_query_radius_min,"
                 "aggregate_query_radius_max,exact_separation_neighbors_mean,exact_separation_neighbors_max,"
                 "social_weight_sum_mean,flock_spread,polarization\n";
    for (const std::uint32_t boid_count : flock3d::bench::benchmark_boid_counts()) {
        for (const AggregateSocialVariant& variant : variants) {
            for (const std::uint32_t thread_count : options.thread_counts) {
                run_scenario(variant, boid_count, options, progress, thread_count);
            }
        }
    }

    return 0;
}
