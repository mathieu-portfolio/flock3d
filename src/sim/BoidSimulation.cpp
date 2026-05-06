#include <flock3d/sim/BoidSimulation.hpp>

#include <algorithm>
#include <cmath>
#include <random>

#include <flock3d/math/Vec3.hpp>

namespace flock3d::sim {
namespace {

[[nodiscard]] Vector3 random_direction(std::mt19937& rng)
{
    std::uniform_real_distribution<float> direction{-1.0F, 1.0F};
    Vector3 value{};

    do {
        value = Vector3{direction(rng), direction(rng), direction(rng)};
    } while (math::length_squared(value) < 0.0001F);

    return math::normalize_safe(value);
}

} // namespace

BoidSimulation::BoidSimulation(SimulationParameters parameters)
    : parameters_{parameters}
    , spatial_hash_{parameters.spatial_cell_size}
{
    reset(parameters_.boid_count);
}

void BoidSimulation::reset()
{
    reset(parameters_.boid_count);
}

void BoidSimulation::reset(std::uint32_t boid_count)
{
    positions_.clear();
    velocities_.clear();
    accelerations_.clear();
    neighbor_indices_.clear();
    positions_.reserve(boid_count);
    velocities_.reserve(boid_count);
    accelerations_.reserve(boid_count);
    neighbor_indices_.reserve(boid_count);
    spawn_random(boid_count);
}

void BoidSimulation::apply_parameters(const SimulationParameters& parameters)
{
    parameters_ = parameters;
    spatial_hash_ = SpatialHash3D{parameters_.spatial_cell_size};
    reset(parameters_.boid_count);
}

void BoidSimulation::update(float dt, SimulationMetrics* metrics)
{
    if (metrics != nullptr) {
        metrics->begin_simulation_step();
    }

    rebuild_spatial_hash();

    const float neighbor_radius_squared = parameters_.neighbor_radius * parameters_.neighbor_radius;
    const float separation_radius_squared = parameters_.separation_radius * parameters_.separation_radius;
    const float query_radius = std::max(parameters_.neighbor_radius, parameters_.separation_radius);

    for (std::size_t i = 0; i < positions_.size(); ++i) {
        const Vector3 position = positions_[i];
        const Vector3 velocity = velocities_[i];
        spatial_hash_.query_neighbors(position, query_radius, neighbor_indices_);
        if (metrics != nullptr) {
            metrics->record_neighbor_query(neighbor_indices_.size());
        }

        Vector3 separation_sum{};
        Vector3 alignment_sum{};
        Vector3 cohesion_sum{};
        std::size_t separation_count = 0;
        std::size_t flock_count = 0;
        float nearest_neighbor_distance_squared = 0.0F;
        bool has_nearest_neighbor = false;

        for (const std::size_t neighbor_index : neighbor_indices_) {
            if (neighbor_index == i) {
                continue;
            }

            const Vector3 offset = math::subtract(positions_[neighbor_index], position);
            const float distance_squared = math::length_squared(offset);
            if (distance_squared <= 0.000001F) {
                continue;
            }

            if (!has_nearest_neighbor || distance_squared < nearest_neighbor_distance_squared) {
                nearest_neighbor_distance_squared = distance_squared;
                has_nearest_neighbor = true;
            }

            if (distance_squared <= separation_radius_squared) {
                const float distance = std::sqrt(distance_squared);
                separation_sum = math::add(separation_sum, math::scale(offset, -1.0F / distance));
                ++separation_count;
            }

            if (distance_squared <= neighbor_radius_squared) {
                alignment_sum = math::add(alignment_sum, velocities_[neighbor_index]);
                cohesion_sum = math::add(cohesion_sum, positions_[neighbor_index]);
                ++flock_count;
            }
        }

        Vector3 acceleration{};
        if (separation_count > 0) {
            const Vector3 average = math::scale(separation_sum, 1.0F / static_cast<float>(separation_count));
            const Vector3 desired = math::scale(math::normalize_safe(average), parameters_.max_speed);
            const Vector3 steering = math::clamp_length(math::subtract(desired, velocity), parameters_.max_force);
            acceleration = math::add(acceleration, math::scale(steering, parameters_.separation_weight));
        }

        if (metrics != nullptr) {
            metrics->record_effective_neighbors(flock_count);
            if (has_nearest_neighbor) {
                metrics->record_nearest_neighbor_distance(std::sqrt(nearest_neighbor_distance_squared));
            }
        }

        if (flock_count > 0) {
            const Vector3 average_velocity = math::scale(alignment_sum, 1.0F / static_cast<float>(flock_count));
            const Vector3 desired = math::scale(math::normalize_safe(average_velocity), parameters_.max_speed);
            const Vector3 steering = math::clamp_length(math::subtract(desired, velocity), parameters_.max_force);
            acceleration = math::add(acceleration, math::scale(steering, parameters_.alignment_weight));

            const Vector3 center = math::scale(cohesion_sum, 1.0F / static_cast<float>(flock_count));
            acceleration = math::add(acceleration, math::scale(seek(position, velocity, center), parameters_.cohesion_weight));
        }

        accelerations_[i] = math::clamp_length(acceleration, parameters_.max_force);
    }

    for (std::size_t i = 0; i < positions_.size(); ++i) {
        velocities_[i] = math::add(velocities_[i], math::scale(accelerations_[i], dt));
        velocities_[i] = math::clamp_length(velocities_[i], parameters_.max_speed);
        positions_[i] = math::add(positions_[i], math::scale(velocities_[i], dt));
        wrap_position(positions_[i]);
    }

    if (metrics != nullptr) {
        record_collective_metrics(*metrics);
        metrics->finish_simulation_step(positions_.size(), spatial_hash_.cell_count());
    }
}

void BoidSimulation::add_boid(Vector3 position, Vector3 velocity)
{
    positions_.push_back(position);
    velocities_.push_back(velocity);
    accelerations_.push_back(Vector3{});

    if (neighbor_indices_.capacity() < positions_.size()) {
        neighbor_indices_.reserve(positions_.size());
    }
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

void BoidSimulation::rebuild_spatial_hash()
{
    spatial_hash_.clear();
    for (std::size_t i = 0; i < positions_.size(); ++i) {
        spatial_hash_.insert(i, positions_[i]);
    }
}

void BoidSimulation::record_collective_metrics(SimulationMetrics& metrics) const noexcept
{
    if (positions_.empty()) {
        metrics.record_collective_behavior(0.0F, 0.0F, 0.0F, 0.0F);
        return;
    }

    Vector3 center{};
    Vector3 normalized_velocity_sum{};
    double speed_total = 0.0;

    for (std::size_t i = 0; i < positions_.size(); ++i) {
        center = math::add(center, positions_[i]);
        normalized_velocity_sum = math::add(normalized_velocity_sum, math::normalize_safe(velocities_[i]));
        speed_total += static_cast<double>(math::length(velocities_[i]));
    }

    const auto boid_count = static_cast<float>(positions_.size());
    center = math::scale(center, 1.0F / boid_count);

    double distance_total = 0.0;
    double distance_squared_total = 0.0;
    for (const Vector3 position : positions_) {
        const float distance_squared = math::length_squared(math::subtract(position, center));
        const float distance = std::sqrt(distance_squared);
        distance_total += static_cast<double>(distance);
        distance_squared_total += static_cast<double>(distance_squared);
    }

    const float polarization = math::length(math::scale(normalized_velocity_sum, 1.0F / boid_count));
    const float cohesion = static_cast<float>(distance_total / static_cast<double>(positions_.size()));
    const float dispersion = static_cast<float>(std::sqrt(distance_squared_total / static_cast<double>(positions_.size())));
    const float average_speed = static_cast<float>(speed_total / static_cast<double>(positions_.size()));
    metrics.record_collective_behavior(polarization, cohesion, dispersion, average_speed);
}

Vector3 BoidSimulation::seek(Vector3 position, Vector3 velocity, Vector3 target) const noexcept
{
    const Vector3 desired_direction = math::normalize_safe(math::subtract(target, position));
    const Vector3 desired_velocity = math::scale(desired_direction, parameters_.max_speed);
    return math::clamp_length(math::subtract(desired_velocity, velocity), parameters_.max_force);
}

} // namespace flock3d::sim
