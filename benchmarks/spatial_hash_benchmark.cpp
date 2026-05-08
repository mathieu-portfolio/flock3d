#include "benchmark_common.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <string_view>
#include <vector>

#include <raylib.h>

#include <flock3d/math/Vec3.hpp>
#include <flock3d/sim/BoidSimulation.hpp>
#include <flock3d/sim/SpatialHash3D.hpp>

namespace {

using flock3d::bench::BenchmarkOptions;
using flock3d::bench::Clock;
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
    UpdateStats naive_query;
    double candidates_per_query_total{};
    double effective_neighbors_per_query_total{};
    double naive_neighbors_per_query_total{};
    double occupied_cells_total{};
    double max_cell_occupancy_total{};
    double average_cell_occupancy_total{};
    std::size_t count_mismatches{};

    void record_iteration(
        double candidates_per_query,
        double effective_neighbors_per_query,
        double naive_neighbors_per_query,
        std::size_t occupied_cells,
        std::size_t max_cell_occupancy,
        double average_cell_occupancy,
        bool counts_match) noexcept
    {
        candidates_per_query_total += candidates_per_query;
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
};

SpatialQueryResult count_neighbors_spatial(
    const flock3d::sim::SpatialHash3D& hash,
    const std::vector<Vector3>& positions,
    float radius,
    std::vector<std::size_t>& neighbors)
{
    SpatialQueryResult result{};
    for (std::size_t i = 0; i < positions.size(); ++i) {
        flock3d::sim::NeighborQueryDiagnostics diagnostics{};
        hash.query_neighbors(positions[i], radius, neighbors, diagnostics);
        result.candidates += diagnostics.candidates_tested;
        for (const std::size_t neighbor_index : neighbors) {
            if (neighbor_index != i) {
                ++result.effective_neighbors;
            }
        }
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

void run_scenario(const SpatialScenario& scenario, std::uint32_t boid_count, const BenchmarkOptions& options, ProgressBar& progress)
{
    flock3d::sim::BoidSimulation simulation{scenario_parameters(scenario, boid_count)};
    std::vector<std::size_t> neighbors;
    neighbors.reserve(boid_count);

    const auto warmup_start = Clock::now();
    while (std::chrono::duration<double>(Clock::now() - warmup_start).count() < options.warmup_seconds) {
        simulation.update(flock3d::bench::fixed_dt, nullptr);
    }

    const auto benchmark_start = Clock::now();
    auto sample_start = benchmark_start;
    int sample_index = 0;

    while (std::chrono::duration<double>(Clock::now() - benchmark_start).count() < options.duration_seconds) {
        SpatialSampleStats stats{};
        while (true) {
            const auto now = Clock::now();
            const double elapsed = std::chrono::duration<double>(now - benchmark_start).count();
            const double sample_elapsed = std::chrono::duration<double>(now - sample_start).count();
            if (elapsed >= options.duration_seconds || (stats.rebuild.count > 0 && sample_elapsed >= options.sample_seconds)) {
                break;
            }

            const auto& positions = simulation.positions();
            flock3d::sim::SpatialHash3D hash{flock3d::sim::effective_query_radius(simulation.parameters())};
            const double rebuild_ms = flock3d::bench::time_ms([&hash, &positions]() {
                for (std::size_t i = 0; i < positions.size(); ++i) {
                    hash.insert(i, positions[i]);
                }
            });
            stats.rebuild.record(rebuild_ms);

            SpatialQueryResult spatial_result{};
            const double spatial_query_ms = flock3d::bench::time_ms([&]() {
                spatial_result = count_neighbors_spatial(hash, positions, simulation.parameters().neighbor_radius, neighbors);
            });
            stats.spatial_query.record(spatial_query_ms);

            std::size_t naive_neighbors = 0;
            const double naive_query_ms = flock3d::bench::time_ms([&]() {
                naive_neighbors = count_neighbors_naive(positions, simulation.parameters().neighbor_radius);
            });
            stats.naive_query.record(naive_query_ms);

            const double query_count = positions.empty() ? 1.0 : static_cast<double>(positions.size());
            stats.record_iteration(
                static_cast<double>(spatial_result.candidates) / query_count,
                static_cast<double>(spatial_result.effective_neighbors) / query_count,
                static_cast<double>(naive_neighbors) / query_count,
                hash.cell_count(),
                hash.max_cell_occupancy(),
                hash.average_cell_occupancy(),
                spatial_result.effective_neighbors == naive_neighbors);

            simulation.update(flock3d::bench::fixed_dt, nullptr);

            if ((stats.rebuild.count % 16U) == 0U) {
                progress.update("spatial_hash", scenario.name, boid_count, elapsed, options.duration_seconds);
            }
        }

        const double elapsed = std::chrono::duration<double>(Clock::now() - benchmark_start).count();
        std::cout << scenario.name << ',' << boid_count << ',' << std::fixed << std::setprecision(3) << elapsed << ','
                  << sample_index << ',' << stats.rebuild.count << ',' << stats.rebuild.mean_ms() << ','
                  << stats.rebuild.min_or_zero() << ',' << stats.rebuild.max_ms << ',' << stats.spatial_query.mean_ms()
                  << ',' << stats.spatial_query.min_or_zero() << ',' << stats.spatial_query.max_ms << ','
                  << stats.naive_query.mean_ms() << ',' << stats.naive_query.min_or_zero() << ','
                  << stats.naive_query.max_ms << ',' << stats.average(stats.candidates_per_query_total) << ','
                  << stats.average(stats.effective_neighbors_per_query_total) << ','
                  << stats.average(stats.naive_neighbors_per_query_total) << ','
                  << stats.average(stats.occupied_cells_total) << ',' << stats.average(stats.max_cell_occupancy_total)
                  << ',' << stats.average(stats.average_cell_occupancy_total) << ',' << stats.count_mismatches << '\n';
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
    const SpatialScenario scenarios[] = {
        {"low_density", 60.0F, 4.0F, 2.0F},
        {"baseline_density", 40.0F, 4.0F, 2.0F},
        {"high_density", 24.0F, 4.0F, 2.0F},
        {"small_radius", 40.0F, 2.5F, 1.25F},
        {"large_radius", 40.0F, 7.0F, 2.0F},
    };

    std::cout << "scenario,boid_count,elapsed_seconds,sample_index,iterations_in_sample,mean_rebuild_ms,min_rebuild_ms,max_rebuild_ms,mean_spatial_query_ms,min_spatial_query_ms,max_spatial_query_ms,mean_naive_query_ms,min_naive_query_ms,max_naive_query_ms,candidates_per_query,effective_neighbors_per_query,naive_neighbors_per_query,occupied_cell_count,max_cell_occupancy,average_cell_occupancy,count_mismatches\n";
    for (const SpatialScenario& scenario : scenarios) {
        for (const std::uint32_t boid_count : flock3d::bench::benchmark_boid_counts()) {
            run_scenario(scenario, boid_count, options, progress);
        }
    }

    return 0;
}
