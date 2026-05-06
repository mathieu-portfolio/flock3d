#pragma once

#include <cstddef>
#include <cstdint>

namespace flock3d::sim {

struct SimulationMetrics {
    double simulation_update_ms{};
    double render_ms{};
    std::uint64_t neighbor_queries{};
    std::uint64_t neighbor_candidates{};
    std::uint64_t neighbor_total{};
    float average_neighbors_per_boid{};
    float polarization{};
    float cohesion{};
    float dispersion{};
    float average_speed{};
    float nearest_neighbor_average_distance{};
    std::size_t cluster_count{};
    std::size_t spatial_hash_cell_count{};

    double nearest_neighbor_distance_total{};
    std::uint64_t nearest_neighbor_distance_count{};

    constexpr void begin_simulation_step() noexcept
    {
        neighbor_queries = 0;
        neighbor_candidates = 0;
        neighbor_total = 0;
        average_neighbors_per_boid = 0.0F;
        polarization = 0.0F;
        cohesion = 0.0F;
        dispersion = 0.0F;
        average_speed = 0.0F;
        nearest_neighbor_average_distance = 0.0F;
        cluster_count = 0;
        spatial_hash_cell_count = 0;
        nearest_neighbor_distance_total = 0.0;
        nearest_neighbor_distance_count = 0;
    }

    constexpr void record_neighbor_query(std::size_t candidate_count) noexcept
    {
        ++neighbor_queries;
        neighbor_candidates += candidate_count;
    }

    constexpr void record_effective_neighbors(std::size_t neighbor_count) noexcept
    {
        neighbor_total += neighbor_count;
    }

    constexpr void record_nearest_neighbor_distance(float distance) noexcept
    {
        nearest_neighbor_distance_total += static_cast<double>(distance);
        ++nearest_neighbor_distance_count;
    }

    constexpr void record_collective_behavior(
        float step_polarization,
        float step_cohesion,
        float step_dispersion,
        float step_average_speed) noexcept
    {
        polarization = step_polarization;
        cohesion = step_cohesion;
        dispersion = step_dispersion;
        average_speed = step_average_speed;
    }

    constexpr void finish_simulation_step(std::size_t boid_count, std::size_t cell_count) noexcept
    {
        spatial_hash_cell_count = cell_count;
        average_neighbors_per_boid = boid_count > 0
            ? static_cast<float>(neighbor_total) / static_cast<float>(boid_count)
            : 0.0F;
        nearest_neighbor_average_distance = nearest_neighbor_distance_count > 0
            ? static_cast<float>(nearest_neighbor_distance_total / static_cast<double>(nearest_neighbor_distance_count))
            : 0.0F;
        cluster_count = 0; // TODO: implement cluster detection once scenario-specific connectivity is available.
    }
};

} // namespace flock3d::sim
