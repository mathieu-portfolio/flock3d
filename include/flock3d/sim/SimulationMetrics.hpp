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
    std::size_t spatial_hash_cell_count{};

    constexpr void begin_simulation_step() noexcept
    {
        neighbor_queries = 0;
        neighbor_candidates = 0;
        neighbor_total = 0;
        average_neighbors_per_boid = 0.0F;
        spatial_hash_cell_count = 0;
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

    constexpr void finish_simulation_step(std::size_t boid_count, std::size_t cell_count) noexcept
    {
        spatial_hash_cell_count = cell_count;
        average_neighbors_per_boid = boid_count > 0
            ? static_cast<float>(neighbor_total) / static_cast<float>(boid_count)
            : 0.0F;
    }
};

} // namespace flock3d::sim
