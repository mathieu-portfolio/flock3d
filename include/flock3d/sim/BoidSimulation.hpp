#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include <raylib.h>

#include <flock3d/sim/NeighborSelection.hpp>
#include <flock3d/sim/SimulationMetrics.hpp>
#include <flock3d/sim/SimulationParameters.hpp>
#include <flock3d/sim/SpatialHash3D.hpp>

namespace flock3d::sim {

class BoidSimulation {
public:
    explicit BoidSimulation(SimulationParameters parameters = {});
    ~BoidSimulation();

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
    [[nodiscard]] std::uint32_t effective_thread_count() const noexcept;

private:
    struct WorkerScratch {
        std::vector<std::size_t> neighbor_indices{};
        std::vector<NeighborCandidate> selected_neighbors{};
        std::vector<CellAggregate> aggregate_cells{};
    };

    class ParallelExecutor;

    struct ModelBehavior {
        bool filter_neighbors_by_field_of_view{false};
        bool add_bird_altitude_acceleration{false};
        bool apply_bird_velocity_constraints{false};
        bool add_fish_medium_acceleration{false};
        bool apply_fish_velocity_constraints{false};
        bool apply_noise{false};
    };

    void update_model(float dt, SimulationMetrics* metrics);
    void update_classic_boids(float dt, SimulationMetrics* metrics);
    void update_bird_flight(float dt, SimulationMetrics* metrics);
    void update_fish_school(float dt, SimulationMetrics* metrics);
    void update_noise_experiment(float dt, SimulationMetrics* metrics);
    void update_shared_flocking(float dt, SimulationMetrics* metrics, ModelBehavior behavior);
    void update_cell_aggregate_social(float dt, SimulationMetrics* metrics, ModelBehavior behavior);
    void integrate(float dt, ModelBehavior behavior, bool noise_active, std::uint64_t noise_step);
    void spawn_random(std::uint32_t boid_count);
    void wrap_position(Vector3& position) const noexcept;
    void rebuild_spatial_hash();
    [[nodiscard]] bool neighbor_in_field_of_view(Vector3 velocity, Vector3 offset) const noexcept;
    [[nodiscard]] float field_of_view_minimum_dot() const noexcept;
    [[nodiscard]] float social_perception_weight(
        Vector3 normalized_velocity,
        Vector3 offset,
        float distance_squared,
        float inverse_social_radius,
        float minimum_field_of_view_dot,
        bool has_forward_direction,
        bool use_front_weighting) const noexcept;
    [[nodiscard]] Vector3 bird_altitude_acceleration(Vector3 position) const noexcept;
    [[nodiscard]] Vector3 fish_medium_acceleration(Vector3 position, Vector3 velocity) const noexcept;
    [[nodiscard]] Vector3 enforce_min_speed(Vector3 velocity) const noexcept;
    [[nodiscard]] Vector3 limit_turn_rate(Vector3 previous_velocity, Vector3 desired_velocity, float dt) const noexcept;
    [[nodiscard]] Vector3 seek(Vector3 position, Vector3 velocity, Vector3 target) const noexcept;
    [[nodiscard]] Vector3 deterministic_noise_vector(std::size_t boid_index, std::uint32_t channel, std::uint64_t step) const;
    [[nodiscard]] float combined_noise_strength() const noexcept;
    void record_collective_metrics(SimulationMetrics& metrics) const noexcept;
    void prepare_parallel_workspaces(std::uint32_t worker_count);

    template <typename Fn>
    void parallel_for_ranges(std::size_t item_count, std::uint32_t requested_thread_count, Fn&& function);

    SimulationParameters parameters_;
    SpatialHash3D spatial_hash_;
    std::vector<Vector3> positions_;
    std::vector<Vector3> velocities_;
    std::vector<Vector3> accelerations_;
    std::vector<std::size_t> neighbor_indices_;
    std::vector<NeighborCandidate> selected_neighbors_;
    std::vector<CellAggregate> aggregate_cells_;
    std::vector<WorkerScratch> worker_scratch_;
    std::unique_ptr<ParallelExecutor> parallel_executor_;
    std::uint64_t noise_step_{};
};

} // namespace flock3d::sim
