#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <raylib.h>

#include <flock3d/sim/SimulationMetrics.hpp>
#include <flock3d/sim/SimulationParameters.hpp>
#include <flock3d/sim/SpatialHash3D.hpp>

namespace flock3d::sim {

class BoidSimulation {
public:
    explicit BoidSimulation(SimulationParameters parameters = {});

    void reset();
    void reset(std::uint32_t boid_count);
    void apply_parameters(const SimulationParameters& parameters);
    void update(float dt, SimulationMetrics* metrics = nullptr);
    void add_boid(Vector3 position, Vector3 velocity);

    [[nodiscard]] const std::vector<Vector3>& positions() const noexcept { return positions_; }
    [[nodiscard]] const std::vector<Vector3>& velocities() const noexcept { return velocities_; }
    [[nodiscard]] const std::vector<Vector3>& accelerations() const noexcept { return accelerations_; }
    [[nodiscard]] const SimulationParameters& parameters() const noexcept { return parameters_; }
    [[nodiscard]] SimulationParameters& parameters() noexcept { return parameters_; }
    [[nodiscard]] std::size_t size() const noexcept { return positions_.size(); }

private:
    struct ModelBehavior {
        bool filter_neighbors_by_field_of_view{false};
        bool add_bird_altitude_acceleration{false};
        bool apply_bird_velocity_constraints{false};
    };

    void update_model(float dt, SimulationMetrics* metrics);
    void update_classic_boids(float dt, SimulationMetrics* metrics);
    void update_bird_flight(float dt, SimulationMetrics* metrics);
    void update_fish_school(float dt, SimulationMetrics* metrics);
    void update_shared_flocking(float dt, SimulationMetrics* metrics, ModelBehavior behavior);
    void spawn_random(std::uint32_t boid_count);
    void wrap_position(Vector3& position) const noexcept;
    void rebuild_spatial_hash();
    [[nodiscard]] bool neighbor_in_field_of_view(Vector3 velocity, Vector3 offset) const noexcept;
    [[nodiscard]] Vector3 bird_altitude_acceleration(Vector3 position) const noexcept;
    [[nodiscard]] Vector3 enforce_min_speed(Vector3 velocity) const noexcept;
    [[nodiscard]] Vector3 limit_turn_rate(Vector3 previous_velocity, Vector3 desired_velocity, float dt) const noexcept;
    [[nodiscard]] Vector3 seek(Vector3 position, Vector3 velocity, Vector3 target) const noexcept;
    void record_collective_metrics(SimulationMetrics& metrics) const noexcept;

    SimulationParameters parameters_;
    SpatialHash3D spatial_hash_;
    std::vector<Vector3> positions_;
    std::vector<Vector3> velocities_;
    std::vector<Vector3> accelerations_;
    std::vector<std::size_t> neighbor_indices_;
};

} // namespace flock3d::sim
