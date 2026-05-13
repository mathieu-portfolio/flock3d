#include "benchmark_common.hpp"

#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <string_view>
#include <vector>

#include <raylib.h>

#include <flock3d/math/Vec3.hpp>
#include <flock3d/sim/BoidSimulation.hpp>
#include <flock3d/sim/SpatialGrid3D.hpp>
#include <flock3d/sim/SpatialHash3D.hpp>

namespace {

using flock3d::bench::BenchmarkOptions;
using flock3d::bench::ProgressBar;
using flock3d::bench::UpdateStats;

struct SpatialScenario {
    std::string_view name;
    float world_half_extent;
    float neighbor_radius;
    float separation_radius;
};

struct SpatialSampleStats {
    UpdateStats rebuild;
    UpdateStats spatial_query;
    UpdateStats aggregate_query;
    UpdateStats naive_query;
    double candidates_per_query_total{};
    double visited_cells_per_query_total{};
    double cell_lookups_per_query_total{};
    double occupied_lookup_cells_per_query_total{};
    double aggregate_candidates_per_query_total{};
    double aggregate_visited_cells_per_query_total{};
    double aggregate_cell_lookups_per_query_total{};
    double aggregate_occupied_cells_per_query_total{};
    double aggregate_results_per_query_total{};
    double effective_neighbors_per_query_total{};
    double naive_neighbors_per_query_total{};
    double occupied_cells_total{};
    double max_cell_occupancy_total{};
    double average_cell_occupancy_total{};
    std::size_t count_mismatches{};

    void record_iteration(
        double candidates_per_query,
        double visited_cells_per_query,
        double cell_lookups_per_query,
        double occupied_lookup_cells_per_query,
        double aggregate_candidates_per_query,
        double aggregate_visited_cells_per_query,
        double aggregate_cell_lookups_per_query,
        double aggregate_occupied_cells_per_query,
        double aggregate_results_per_query,
        double effective_neighbors_per_query,
        double naive_neighbors_per_query,
        std::size_t occupied_cells,
        std::size_t max_cell_occupancy,
        double average_cell_occupancy,
        bool counts_match) noexcept
    {
        candidates_per_query_total += candidates_per_query;
        visited_cells_per_query_total += visited_cells_per_query;
        cell_lookups_per_query_total += cell_lookups_per_query;
        occupied_lookup_cells_per_query_total += occupied_lookup_cells_per_query;
        aggregate_candidates_per_query_total += aggregate_candidates_per_query;
        aggregate_visited_cells_per_query_total += aggregate_visited_cells_per_query;
        aggregate_cell_lookups_per_query_total += aggregate_cell_lookups_per_query;
        aggregate_occupied_cells_per_query_total += aggregate_occupied_cells_per_query;
        aggregate_results_per_query_total += aggregate_results_per_query;
        effective_neighbors_per_query_total += effective_neighbors_per_query;
        naive_neighbors_per_query_total += naive_neighbors_per_query;
        occupied_cells_total += static_cast<double>(occupied_cells);
        max_cell_occupancy_total += static_cast<double>(max_cell_occupancy);
        average_cell_occupancy_total += average_cell_occupancy;
        if (!counts_match) {
            ++count_mismatches;
        }
    }

    [[nodiscard]] double average(double total) const noexcept
    {
        return rebuild.count > 0 ? total / static_cast<double>(rebuild.count) : 0.0;
    }
};

std::size_t count_neighbors_naive(const std::vector<Vector3>& positions, float radius)
{
    const float radius_squared = radius * radius;
    std::size_t total = 0;
    for (std::size_t i = 0; i < positions.size(); ++i) {
        for (std::size_t j = 0; j < positions.size(); ++j) {
            if (i == j) {
                continue;
            }
            const auto offset = flock3d::math::subtract(positions[j], positions[i]);
            if (flock3d::math::length_squared(offset) <= radius_squared) {
                ++total;
            }
        }
    }
    return total;
}

struct SpatialQueryResult {
    std::size_t effective_neighbors{};
    std::size_t candidates{};
    std::size_t visited_cells{};
    std::size_t cell_lookups{};
    std::size_t occupied_cells{};
};

template <typename SpatialIndex>
SpatialQueryResult count_neighbors_spatial(
    const SpatialIndex& index,
    const std::vector<Vector3>& positions,
    float radius,
    std::vector<std::size_t>& neighbors)
{
    SpatialQueryResult result{};
    for (std::size_t i = 0; i < positions.size(); ++i) {
        flock3d::sim::NeighborQueryDiagnostics diagnostics{};
        index.query_neighbors(positions[i], radius, neighbors, diagnostics);
        result.candidates += diagnostics.candidates_tested;
        result.visited_cells += diagnostics.visited_cells;
        result.cell_lookups += diagnostics.cell_lookups;
        result.occupied_cells += diagnostics.occupied_cells;
        for (const std::size_t neighbor_index : neighbors) {
            if (neighbor_index != i) {
                ++result.effective_neighbors;
            }
        }
    }
    return result;
}

template <typename SpatialIndex>
SpatialQueryResult count_aggregates_spatial(
    const SpatialIndex& index,
    const std::vector<Vector3>& positions,
    float radius,
    std::vector<flock3d::sim::CellAggregate>& aggregates)
{
    SpatialQueryResult result{};
    for (const Vector3 position : positions) {
        flock3d::sim::NeighborQueryDiagnostics diagnostics{};
        index.query_cell_aggregates(position, radius, aggregates, diagnostics);
        result.effective_neighbors += aggregates.size();
        result.candidates += diagnostics.candidates_tested;
        result.visited_cells += diagnostics.visited_cells;
        result.cell_lookups += diagnostics.cell_lookups;
        result.occupied_cells += diagnostics.occupied_cells;
    }
    return result;
}

flock3d::sim::SimulationParameters scenario_parameters(const SpatialScenario& scenario, std::uint32_t boid_count)
{
    auto parameters = flock3d::bench::parameters_for_model(
        flock3d::sim::SimulationModel::ClassicBoids,
        boid_count,
        44'000U + boid_count);
    parameters.world_half_extent = scenario.world_half_extent;
    parameters.neighbor_radius = scenario.neighbor_radius;
    parameters.separation_radius = scenario.separation_radius;
    flock3d::sim::sync_spatial_cell_size_to_query_radius(parameters);
    return parameters;
}

template <typename SpatialIndex, typename RebuildIndex>
void run_scenario_backend(
    const SpatialScenario& scenario,
    std::uint32_t boid_count,
    const BenchmarkOptions& options,
    ProgressBar& progress,
    std::string_view backend_name,
    RebuildIndex rebuild_index)
{
    flock3d::sim::BoidSimulation simulation{scenario_parameters(scenario, boid_count)};
    std::vector<std::size_t> neighbors;
    neighbors.reserve(boid_count);
    std::vector<flock3d::sim::CellAggregate> aggregates;
    aggregates.reserve(boid_count);

    const std::size_t warmup_ticks = flock3d::bench::simulated_seconds_to_ticks(options.warmup_seconds);
    for (std::size_t tick = 0; tick < warmup_ticks; ++tick) {
        simulation.update(flock3d::bench::fixed_dt, nullptr);
    }

    const std::size_t total_ticks = flock3d::bench::simulated_seconds_to_ticks(options.duration_seconds);
    const std::size_t sample_ticks = std::max<std::size_t>(1U, flock3d::bench::simulated_seconds_to_ticks(options.sample_seconds));
    std::size_t completed_ticks = 0U;
    int sample_index = 0;

    while (completed_ticks < total_ticks) {
        SpatialSampleStats stats{};
        stats.rebuild.samples_ms.reserve(sample_ticks);
        stats.spatial_query.samples_ms.reserve(sample_ticks);
        stats.aggregate_query.samples_ms.reserve(sample_ticks);
        stats.naive_query.samples_ms.reserve(sample_ticks);
        const std::size_t sample_start_tick = completed_ticks;
        while (completed_ticks < total_ticks && (stats.rebuild.count == 0U || completed_ticks - sample_start_tick < sample_ticks)) {
            const auto& positions = simulation.positions();
            SpatialIndex index{flock3d::sim::effective_query_radius(simulation.parameters())};
            const double rebuild_ms = flock3d::bench::time_ms([&]() { rebuild_index(index, positions); });
            stats.rebuild.record(rebuild_ms);

            SpatialQueryResult spatial_result{};
            const double spatial_query_ms = flock3d::bench::time_ms([&]() {
                spatial_result = count_neighbors_spatial(index, positions, simulation.parameters().neighbor_radius, neighbors);
            });
            stats.spatial_query.record(spatial_query_ms);

            SpatialQueryResult aggregate_result{};
            const double aggregate_query_ms = flock3d::bench::time_ms([&]() {
                aggregate_result = count_aggregates_spatial(index, positions, simulation.parameters().neighbor_radius, aggregates);
            });
            stats.aggregate_query.record(aggregate_query_ms);

            std::size_t naive_neighbors = 0;
            const double naive_query_ms = flock3d::bench::time_ms([&]() {
                naive_neighbors = count_neighbors_naive(positions, simulation.parameters().neighbor_radius);
            });
            stats.naive_query.record(naive_query_ms);

            const double query_count = positions.empty() ? 1.0 : static_cast<double>(positions.size());
            stats.record_iteration(
                static_cast<double>(spatial_result.candidates) / query_count,
                static_cast<double>(spatial_result.visited_cells) / query_count,
                static_cast<double>(spatial_result.cell_lookups) / query_count,
                static_cast<double>(spatial_result.occupied_cells) / query_count,
                static_cast<double>(aggregate_result.candidates) / query_count,
                static_cast<double>(aggregate_result.visited_cells) / query_count,
                static_cast<double>(aggregate_result.cell_lookups) / query_count,
                static_cast<double>(aggregate_result.occupied_cells) / query_count,
                static_cast<double>(aggregate_result.effective_neighbors) / query_count,
                static_cast<double>(spatial_result.effective_neighbors) / query_count,
                static_cast<double>(naive_neighbors) / query_count,
                index.cell_count(),
                index.max_cell_occupancy(),
                index.average_cell_occupancy(),
                spatial_result.effective_neighbors == naive_neighbors);

            simulation.update(flock3d::bench::fixed_dt, nullptr);
            ++completed_ticks;

            const double elapsed = flock3d::bench::ticks_to_simulated_seconds(completed_ticks);
            if ((stats.rebuild.count % 16U) == 0U) {
                progress.update(backend_name, scenario.name, boid_count, elapsed, options.duration_seconds);
            }
        }

        const double elapsed = flock3d::bench::ticks_to_simulated_seconds(completed_ticks);
        const std::size_t ticks_in_sample = stats.rebuild.count;
        const std::size_t simulated_ticks = completed_ticks;
        const double sample_wall_seconds = stats.rebuild.wall_seconds() + stats.spatial_query.wall_seconds() + stats.aggregate_query.wall_seconds() + stats.naive_query.wall_seconds();
        const double ticks_per_second = sample_wall_seconds > 0.0 ? static_cast<double>(ticks_in_sample) / sample_wall_seconds : 0.0;
        const double real_time_factor = sample_wall_seconds > 0.0 ? (static_cast<double>(ticks_in_sample) * flock3d::bench::fixed_dt_seconds) / sample_wall_seconds : 0.0;
        std::cout << scenario.name << ',' << backend_name << ',' << boid_count << ',' << std::fixed << std::setprecision(3) << elapsed << ','
                  << sample_index << ',' << stats.rebuild.count << ',' << stats.rebuild.mean_ms() << ','
                  << stats.rebuild.min_or_zero() << ',' << stats.rebuild.max_ms << ',' << stats.spatial_query.mean_ms()
                  << ',' << stats.spatial_query.min_or_zero() << ',' << stats.spatial_query.max_ms << ','
                  << stats.naive_query.mean_ms() << ',' << stats.naive_query.min_or_zero() << ','
                  << stats.naive_query.max_ms << ',' << stats.average(stats.candidates_per_query_total) << ','
                  << stats.average(stats.visited_cells_per_query_total) << ','
                  << stats.average(stats.cell_lookups_per_query_total) << ','
                  << stats.average(stats.occupied_lookup_cells_per_query_total) << ','
                  << stats.aggregate_query.mean_ms() << ',' << stats.aggregate_query.min_or_zero() << ','
                  << stats.aggregate_query.max_ms << ',' << stats.average(stats.aggregate_candidates_per_query_total) << ','
                  << stats.average(stats.aggregate_visited_cells_per_query_total) << ','
                  << stats.average(stats.aggregate_cell_lookups_per_query_total) << ','
                  << stats.average(stats.aggregate_occupied_cells_per_query_total) << ','
                  << stats.average(stats.aggregate_results_per_query_total) << ','
                  << stats.average(stats.effective_neighbors_per_query_total) << ','
                  << stats.average(stats.naive_neighbors_per_query_total) << ','
                  << stats.average(stats.occupied_cells_total) << ',' << stats.average(stats.max_cell_occupancy_total)
                  << ',' << stats.average(stats.average_cell_occupancy_total) << ',' << stats.count_mismatches << ','
                  << elapsed << ',' << simulated_ticks << ',' << ticks_in_sample << ',' << sample_wall_seconds
                  << ',' << stats.rebuild.mean_ns() << ',' << stats.rebuild.p50_ms() << ',' << stats.rebuild.p95_ms()
                  << ',' << stats.rebuild.p99_ms() << ',' << stats.spatial_query.mean_ns() << ','
                  << stats.spatial_query.p50_ms() << ',' << stats.spatial_query.p95_ms() << ','
                  << stats.spatial_query.p99_ms() << ',' << stats.aggregate_query.mean_ns() << ','
                  << stats.aggregate_query.p50_ms() << ',' << stats.aggregate_query.p95_ms() << ','
                  << stats.aggregate_query.p99_ms() << ',' << stats.naive_query.mean_ns() << ','
                  << stats.naive_query.p50_ms() << ',' << stats.naive_query.p95_ms() << ','
                  << stats.naive_query.p99_ms() << ','
                  << ticks_per_second << ',' << ticks_per_second << ',' << real_time_factor << '\n';
        ++sample_index;
    }
    progress.finish();
}

void rebuild_hash_index(flock3d::sim::SpatialHash3D& hash, const std::vector<Vector3>& positions)
{
    for (std::size_t i = 0; i < positions.size(); ++i) {
        hash.insert(i, positions[i]);
    }
}

void rebuild_grid_index(flock3d::sim::SpatialGrid3D& grid, const std::vector<Vector3>& positions)
{
    grid.rebuild(positions);
}

void run_scenario(const SpatialScenario& scenario, std::uint32_t boid_count, const BenchmarkOptions& options, ProgressBar& progress)
{
    run_scenario_backend<flock3d::sim::SpatialHash3D>(
        scenario,
        boid_count,
        options,
        progress,
        "spatial_hash",
        rebuild_hash_index);
    run_scenario_backend<flock3d::sim::SpatialGrid3D>(
        scenario,
        boid_count,
        options,
        progress,
        "spatial_grid",
        rebuild_grid_index);
}

} // namespace

int main(int argc, char** argv)
{
    const BenchmarkOptions options = flock3d::bench::parse_options(argc, argv);
    ProgressBar progress{flock3d::bench::progress_enabled()};
    const SpatialScenario scenarios[] = {
        {"low_density", 60.0F, 4.0F, 2.0F},
        {"baseline_density", 40.0F, 4.0F, 2.0F},
        {"high_density", 24.0F, 4.0F, 2.0F},
        {"small_radius", 40.0F, 2.5F, 1.25F},
        {"large_radius", 40.0F, 7.0F, 2.0F},
    };

    std::cout << "scenario,backend,boid_count,elapsed_seconds,sample_index,iterations_in_sample,mean_rebuild_ms,min_rebuild_ms,max_rebuild_ms,mean_spatial_query_ms,min_spatial_query_ms,max_spatial_query_ms,mean_naive_query_ms,min_naive_query_ms,max_naive_query_ms,candidates_per_query,visited_cells_per_query,cell_lookups_per_query,occupied_lookup_cells_per_query,mean_aggregate_query_ms,min_aggregate_query_ms,max_aggregate_query_ms,aggregate_candidates_per_query,aggregate_visited_cells_per_query,aggregate_cell_lookups_per_query,aggregate_occupied_cells_per_query,aggregate_results_per_query,effective_neighbors_per_query,naive_neighbors_per_query,occupied_cell_count,max_cell_occupancy,average_cell_occupancy,count_mismatches,simulated_seconds,simulated_ticks,ticks_in_sample,sample_wall_seconds,mean_rebuild_ns_per_tick,p50_rebuild_ms,p95_rebuild_ms,p99_rebuild_ms,mean_spatial_query_ns_per_tick,p50_spatial_query_ms,p95_spatial_query_ms,p99_spatial_query_ms,mean_aggregate_query_ns_per_tick,p50_aggregate_query_ms,p95_aggregate_query_ms,p99_aggregate_query_ms,mean_naive_query_ns_per_tick,p50_naive_query_ms,p95_naive_query_ms,p99_naive_query_ms,ticks_per_second,updates_per_second,real_time_factor\n";
    for (const SpatialScenario& scenario : scenarios) {
        for (const std::uint32_t boid_count : options.boid_counts) {
            run_scenario(scenario, boid_count, options, progress);
        }
    }

    return 0;
}
