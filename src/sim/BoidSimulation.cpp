#include "sim/BoidSimulation.hpp"

#include <algorithm>
#include <random>

#include "math/Vec3.hpp"

namespace flock3d::sim {
namespace {

[[nodiscard]] Vector3 random_direction(std::mt19937& rng)
{
    std::uniform_real_distribution<float> direction{-1.0F, 1.0F};
    Vector3 value{};

    do {
        value = Vector3{direction(rng), direction(rng), direction(rng)};
    } while (math::length_squared(value) < 0.0001F);

    return math::scale(value, 1.0F / math::length(value));
}

} // namespace

BoidSimulation::BoidSimulation(SimulationParameters parameters)
    : parameters_{parameters}
    , spatial_hash_{parameters.spatial_cell_size}
{
    reset(parameters_.boid_count);
}

void BoidSimulation::reset(std::uint32_t boid_count)
{
    positions_.clear();
    velocities_.clear();
    accelerations_.clear();
    positions_.reserve(boid_count);
    velocities_.reserve(boid_count);
    accelerations_.reserve(boid_count);
    spawn_random(boid_count);
}

void BoidSimulation::update(float dt)
{
    update_neighbors();
    apply_separation();
    apply_alignment();
    apply_cohesion();

    for (std::size_t i = 0; i < positions_.size(); ++i) {
        velocities_[i] = math::add(velocities_[i], math::scale(accelerations_[i], dt));
        velocities_[i] = math::clamp_length(velocities_[i], parameters_.max_speed);
        positions_[i] = math::add(positions_[i], math::scale(velocities_[i], dt));
        wrap_position(positions_[i]);
        accelerations_[i] = Vector3{};
    }
}

void BoidSimulation::add_boid(Vector3 position, Vector3 velocity)
{
    positions_.push_back(position);
    velocities_.push_back(velocity);
    accelerations_.push_back(Vector3{});
}

void BoidSimulation::spawn_random(std::uint32_t boid_count)
{
    std::mt19937 rng{parameters_.random_seed};
    std::uniform_real_distribution<float> position{-parameters_.world_half_extent, parameters_.world_half_extent};
    std::uniform_real_distribution<float> speed{parameters_.min_initial_speed, parameters_.max_initial_speed};

    for (std::uint32_t i = 0; i < boid_count; ++i) {
        add_boid(
            Vector3{position(rng), position(rng), position(rng)},
            math::scale(random_direction(rng), speed(rng)));
    }
}

void BoidSimulation::wrap_position(Vector3& position) const noexcept
{
    const float extent = parameters_.world_half_extent;

    if (position.x > extent) {
        position.x = -extent;
    } else if (position.x < -extent) {
        position.x = extent;
    }

    if (position.y > extent) {
        position.y = -extent;
    } else if (position.y < -extent) {
        position.y = extent;
    }

    if (position.z > extent) {
        position.z = -extent;
    } else if (position.z < -extent) {
        position.z = extent;
    }
}

void BoidSimulation::update_neighbors()
{
    spatial_hash_.clear();
    for (std::size_t i = 0; i < positions_.size(); ++i) {
        spatial_hash_.insert(i, positions_[i]);
    }
}

void BoidSimulation::apply_separation()
{
    // Flocking forces will be implemented in a later iteration.
}

void BoidSimulation::apply_alignment()
{
    // Flocking forces will be implemented in a later iteration.
}

void BoidSimulation::apply_cohesion()
{
    // Flocking forces will be implemented in a later iteration.
}

} // namespace flock3d::sim
