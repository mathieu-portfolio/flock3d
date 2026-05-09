#pragma once

#include <algorithm>
#include <cstdint>
#include <cstddef>

#include <raylib.h>

namespace flock3d::sim {

// Add new model behavior by extending BoidSimulation's model dispatch first,
// then keep shared flocking in the reusable update path whenever possible.
enum class SimulationModel : std::uint8_t {
    ClassicBoids = 0,
    BirdFlight,
    FishSchool,
    NoiseExperiment,
};

enum class NeighborMode : std::uint8_t {
    FixedRadiusUncapped = 0,
    FixedRadiusClosestK,
    AdaptiveRadiusClosestK,
    CellAggregateSocial,
};

struct SimulationParameters {
    std::uint32_t boid_count{512};
    float world_half_extent{40.0F};
    float min_initial_speed{2.0F};
    float max_initial_speed{8.0F};
    float max_speed{10.0F};
    float neighbor_radius{4.0F};
    // A zero base perception radius keeps legacy fixed-radius behavior by using neighbor_radius.
    float base_perception_radius{0.0F};
    float min_perception_radius{0.0F};
    float max_perception_radius{0.0F};
    std::uint32_t target_neighbor_count{32U};
    // A zero max keeps legacy uncapped metric-neighbor behavior.
    std::size_t max_selected_neighbors{0U};
    bool adaptive_perception_enabled{false};
    NeighborMode neighbor_mode{NeighborMode::FixedRadiusUncapped};
    float separation_radius{2.0F};
    float separation_weight{1.5F};
    float alignment_weight{1.0F};
    float cohesion_weight{1.0F};
    float max_force{12.0F};
    float spatial_cell_size{4.0F};
    float boid_scale{0.45F};
    std::uint32_t random_seed{1337U};
    SimulationModel model{SimulationModel::ClassicBoids};
    float gravity{0.0F};
    float lift_strength{0.0F};
    float altitude_target{0.0F};
    float altitude_band{0.0F};
    float altitude_correction_strength{0.0F};
    float min_speed{0.0F};
    float max_climb_rate{0.0F};
    float max_turn_rate{0.0F};
    float field_of_view_degrees{360.0F};
    float drag_coefficient{0.0F};
    float buoyancy_strength{0.0F};
    float target_depth{0.0F};
    float depth_band{0.0F};
    float depth_correction_strength{0.0F};
    float current_strength{0.0F};
    Vector3 current_direction{1.0F, 0.0F, 0.0F};
    float perception_noise_strength{0.0F};
    float steering_noise_strength{0.0F};
    float velocity_noise_strength{0.0F};
    std::uint32_t noise_seed_offset{10'000U};
    bool noise_enabled{false};
};

[[nodiscard]] constexpr float effective_query_radius(const SimulationParameters& parameters) noexcept
{
    const float base_radius = parameters.base_perception_radius > 0.0F
        ? parameters.base_perception_radius
        : parameters.neighbor_radius;
    const float configured_max_radius = parameters.max_perception_radius > 0.0F
        ? parameters.max_perception_radius
        : base_radius;
    const float perception_query_radius = parameters.adaptive_perception_enabled
        ? std::max(base_radius, configured_max_radius)
        : base_radius;
    return std::max(perception_query_radius, parameters.separation_radius);
}

constexpr void sync_spatial_cell_size_to_query_radius(SimulationParameters& parameters) noexcept
{
    parameters.spatial_cell_size = effective_query_radius(parameters);
}

class FixedTimestepAccumulator {
public:
    explicit constexpr FixedTimestepAccumulator(double fixed_dt = 1.0 / 120.0) noexcept
        : fixed_dt_{fixed_dt}
    {
    }

    [[nodiscard]] constexpr double fixed_dt() const noexcept { return fixed_dt_; }
    [[nodiscard]] constexpr double accumulated() const noexcept { return accumulator_; }

    constexpr void add_frame_time(double frame_time_seconds) noexcept
    {
        constexpr double max_frame_time = 0.25;
        accumulator_ += std::clamp(frame_time_seconds, 0.0, max_frame_time);
    }

    constexpr bool consume_step() noexcept
    {
        if (accumulator_ < fixed_dt_) {
            return false;
        }

        accumulator_ -= fixed_dt_;
        return true;
    }

private:
    double fixed_dt_{1.0 / 120.0};
    double accumulator_{0.0};
};

} // namespace flock3d::sim
