#pragma once

#include <algorithm>
#include <cstdint>

namespace flock3d::sim {

enum class SimulationModel : std::uint8_t {
    ClassicBoids = 0,
    BirdFlight,
};

struct SimulationParameters {
    std::uint32_t boid_count{512};
    float world_half_extent{40.0F};
    float min_initial_speed{2.0F};
    float max_initial_speed{8.0F};
    float max_speed{10.0F};
    float neighbor_radius{4.0F};
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
};

[[nodiscard]] constexpr float effective_query_radius(const SimulationParameters& parameters) noexcept
{
    return std::max(parameters.neighbor_radius, parameters.separation_radius);
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
