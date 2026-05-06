#pragma once

#include <algorithm>
#include <cstdint>

namespace flock3d::sim {

struct SimulationParameters {
    std::uint32_t boid_count{512};
    float world_half_extent{40.0F};
    float min_initial_speed{2.0F};
    float max_initial_speed{8.0F};
    float max_speed{10.0F};
    float neighbor_radius{4.0F};
    float spatial_cell_size{4.0F};
    std::uint32_t random_seed{1337U};
};

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
