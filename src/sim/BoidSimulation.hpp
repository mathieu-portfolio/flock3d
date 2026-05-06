#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <raylib.h>

#include "sim/SimulationParameters.hpp"
#include "sim/SpatialHash3D.hpp"

namespace flock3d::sim {

class BoidSimulation {
public:
    explicit BoidSimulation(SimulationParameters parameters = {});

    void reset(std::uint32_t boid_count);
    void update(float dt);
    void add_boid(Vector3 position, Vector3 velocity);

    [[nodiscard]] const std::vector<Vector3>& positions() const noexcept { return positions_; }
    [[nodiscard]] const std::vector<Vector3>& velocities() const noexcept { return velocities_; }
    [[nodiscard]] const std::vector<Vector3>& accelerations() const noexcept { return accelerations_; }
    [[nodiscard]] const SimulationParameters& parameters() const noexcept { return parameters_; }
    [[nodiscard]] std::size_t size() const noexcept { return positions_.size(); }

private:
    void spawn_random(std::uint32_t boid_count);
    void wrap_position(Vector3& position) const noexcept;
    void update_neighbors();
    void apply_separation();
    void apply_alignment();
    void apply_cohesion();

    SimulationParameters parameters_;
    SpatialHash3D spatial_hash_;
    std::vector<Vector3> positions_;
    std::vector<Vector3> velocities_;
    std::vector<Vector3> accelerations_;
};

} // namespace flock3d::sim
