#pragma once

#include <cstddef>
#include <vector>

#include <raylib.h>

namespace flock3d::sim {

struct SimulationParameters;

struct NeighborCandidate {
    std::size_t boid_index{};
    float distance_squared{};
    Vector3 offset{};
};

[[nodiscard]] float base_perception_radius(const SimulationParameters& parameters) noexcept;
[[nodiscard]] float min_perception_radius(const SimulationParameters& parameters) noexcept;
[[nodiscard]] float max_perception_radius(const SimulationParameters& parameters) noexcept;
[[nodiscard]] float compute_effective_perception_radius(
    const SimulationParameters& parameters,
    std::size_t local_candidate_count) noexcept;
void select_closest_neighbors(std::vector<NeighborCandidate>& candidates, std::size_t max_selected_neighbors);

} // namespace flock3d::sim
