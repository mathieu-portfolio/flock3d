#include <flock3d/sim/NeighborSelection.hpp>

#include <algorithm>
#include <cmath>

#include <flock3d/sim/SimulationParameters.hpp>

namespace flock3d::sim {

float base_perception_radius(const SimulationParameters& parameters) noexcept
{
    return parameters.base_perception_radius > 0.0F ? parameters.base_perception_radius : parameters.neighbor_radius;
}

float min_perception_radius(const SimulationParameters& parameters) noexcept
{
    return parameters.min_perception_radius > 0.0F ? parameters.min_perception_radius : base_perception_radius(parameters);
}

float max_perception_radius(const SimulationParameters& parameters) noexcept
{
    return parameters.max_perception_radius > 0.0F ? parameters.max_perception_radius : base_perception_radius(parameters);
}

float compute_effective_perception_radius(const SimulationParameters& parameters, std::size_t local_candidate_count) noexcept
{
    const float base_radius = base_perception_radius(parameters);
    if (!parameters.adaptive_perception_enabled) {
        return base_radius;
    }

    const float target_count = std::max(1.0F, static_cast<float>(parameters.target_neighbor_count));
    const float local_count = std::max(1.0F, static_cast<float>(local_candidate_count));

    // Adaptive metric-topological perception: shrink dense neighborhoods and expand sparse ones by scaling
    // base_radius with sqrt(target_neighbor_count / max(local_candidate_count, 1)), then clamp to tunable bounds.
    const float unclamped_radius = base_radius * std::sqrt(target_count / local_count);
    const float minimum_configured_radius = min_perception_radius(parameters);
    const float maximum_configured_radius = max_perception_radius(parameters);
    const auto [minimum_radius, maximum_radius] =
        std::minmax(minimum_configured_radius, maximum_configured_radius);
    return std::clamp(unclamped_radius, minimum_radius, maximum_radius);
}

void select_closest_neighbors(std::vector<NeighborCandidate>& candidates, std::size_t max_selected_neighbors)
{
    const auto by_distance_then_index = [](const NeighborCandidate& lhs, const NeighborCandidate& rhs) noexcept {
        if (lhs.distance_squared == rhs.distance_squared) {
            return lhs.boid_index < rhs.boid_index;
        }
        return lhs.distance_squared < rhs.distance_squared;
    };

    if (max_selected_neighbors > 0U && candidates.size() > max_selected_neighbors) {
        const auto selected_end = candidates.begin() + static_cast<std::ptrdiff_t>(max_selected_neighbors);
        std::nth_element(candidates.begin(), selected_end, candidates.end(), by_distance_then_index);
        candidates.erase(selected_end, candidates.end());
    }

    std::sort(candidates.begin(), candidates.end(), by_distance_then_index);
}

} // namespace flock3d::sim
