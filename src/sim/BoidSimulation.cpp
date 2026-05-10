#include <flock3d/sim/BoidSimulation.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <random>

#include <flock3d/math/Vec3.hpp>
#include <flock3d/sim/NeighborSelection.hpp>

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
    , spatial_hash_{effective_query_radius(parameters)}
{
    sync_spatial_cell_size_to_query_radius(parameters_);
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
    selected_neighbors_.clear();
    aggregate_cells_.clear();
    noise_step_ = 0;
    positions_.reserve(boid_count);
    velocities_.reserve(boid_count);
    accelerations_.reserve(boid_count);
    neighbor_indices_.reserve(boid_count);
    selected_neighbors_.reserve(boid_count);
    aggregate_cells_.reserve(boid_count);
    spawn_random(boid_count);
}

void BoidSimulation::apply_parameters(const SimulationParameters& parameters)
{
    parameters_ = parameters;
    sync_spatial_cell_size_to_query_radius(parameters_);
    spatial_hash_ = SpatialHash3D{parameters_.spatial_cell_size};
    reset(parameters_.boid_count);
}

void BoidSimulation::update(float dt, SimulationMetrics* metrics)
{
    if (metrics != nullptr) {
        metrics->begin_simulation_step();
    }

    const float query_radius = effective_query_radius(parameters_);
    if (parameters_.spatial_cell_size != query_radius) {
        parameters_.spatial_cell_size = query_radius;
        spatial_hash_ = SpatialHash3D{parameters_.spatial_cell_size};
    }

    rebuild_spatial_hash();
    update_model(dt, metrics);

    if (metrics != nullptr) {
        record_collective_metrics(*metrics);
        metrics->finish_simulation_step(
            positions_.size(),
            spatial_hash_.cell_count(),
            spatial_hash_.average_cell_occupancy(),
            spatial_hash_.max_cell_occupancy());
    }
}

void BoidSimulation::update_model(float dt, SimulationMetrics* metrics)
{
    switch (parameters_.model) {
    case SimulationModel::ClassicBoids:
        update_classic_boids(dt, metrics);
        break;
    case SimulationModel::BirdFlight:
        update_bird_flight(dt, metrics);
        break;
    case SimulationModel::FishSchool:
        update_fish_school(dt, metrics);
        break;
    case SimulationModel::NoiseExperiment:
        update_noise_experiment(dt, metrics);
        break;
    default:
        update_classic_boids(dt, metrics);
        break;
    }
}

void BoidSimulation::update_classic_boids(float dt, SimulationMetrics* metrics)
{
    update_shared_flocking(dt, metrics, ModelBehavior{});
}

void BoidSimulation::update_bird_flight(float dt, SimulationMetrics* metrics)
{
    update_shared_flocking(
        dt,
        metrics,
        ModelBehavior{
            true,
            true,
            true,
        });
}

void BoidSimulation::update_fish_school(float dt, SimulationMetrics* metrics)
{
    update_shared_flocking(
        dt,
        metrics,
        ModelBehavior{
            true,
            false,
            false,
            true,
            true,
        });
}

void BoidSimulation::update_noise_experiment(float dt, SimulationMetrics* metrics)
{
    update_shared_flocking(
        dt,
        metrics,
        ModelBehavior{
            false,
            false,
            false,
            false,
            false,
            true,
        });
}

void BoidSimulation::update_shared_flocking(float dt, SimulationMetrics* metrics, ModelBehavior behavior)
{
    if (parameters_.neighbor_mode == NeighborMode::CellAggregateSocial) {
        update_cell_aggregate_social(dt, metrics, behavior);
        return;
    }
    const float query_radius = effective_query_radius(parameters_);
    const float perception_radius = base_perception_radius(parameters_);
    const float separation_radius_squared = parameters_.separation_radius * parameters_.separation_radius;
    const bool noise_active = behavior.apply_noise && parameters_.noise_enabled;
    const std::uint64_t noise_step = noise_step_;

    for (std::size_t i = 0; i < positions_.size(); ++i) {
        const Vector3 position = positions_[i];
        const Vector3 velocity = velocities_[i];
        NeighborQueryDiagnostics query_diagnostics{};
        spatial_hash_.query_neighbors(position, query_radius, neighbor_indices_, query_diagnostics);
        if (metrics != nullptr) {
            metrics->record_neighbor_query(query_diagnostics.candidates_tested, query_diagnostics.visited_cells);
        }

        const auto local_candidate_count = static_cast<std::size_t>(std::count_if(
            neighbor_indices_.begin(),
            neighbor_indices_.end(),
            [i](std::size_t neighbor_index) noexcept { return neighbor_index != i; }));
        const float effective_perception_radius = compute_effective_perception_radius(parameters_, local_candidate_count);
        const float effective_perception_radius_squared = effective_perception_radius * effective_perception_radius;

        selected_neighbors_.clear();
        std::size_t fov_rejected_count = 0;
        std::size_t radius_rejected_count = 0;
        for (const std::size_t neighbor_index : neighbor_indices_) {
            if (neighbor_index == i) {
                continue;
            }

            Vector3 offset = math::subtract(positions_[neighbor_index], position);
            if (behavior.filter_neighbors_by_field_of_view && !neighbor_in_field_of_view(velocity, offset)) {
                ++fov_rejected_count;
                continue;
            }
            if (noise_active && parameters_.perception_noise_strength > 0.0F) {
                const auto channel = static_cast<std::uint32_t>((neighbor_index * 2U) + 1U);
                offset = math::add(
                    offset,
                    math::scale(
                        deterministic_noise_vector(i, channel, noise_step),
                        parameters_.perception_noise_strength * perception_radius));
            }
            const float distance_squared = math::length_squared(offset);
            if (distance_squared <= 0.000001F || distance_squared > effective_perception_radius_squared) {
                ++radius_rejected_count;
                continue;
            }

            selected_neighbors_.push_back(NeighborCandidate{neighbor_index, distance_squared});
        }
        const std::size_t accepted_before_topology = selected_neighbors_.size();
        select_closest_neighbors(selected_neighbors_, parameters_.max_selected_neighbors);

        Vector3 separation_sum{};
        Vector3 alignment_sum{};
        Vector3 cohesion_sum{};
        std::size_t separation_count = 0;
        std::size_t flock_count = 0;
        float nearest_neighbor_distance_squared = 0.0F;
        bool has_nearest_neighbor = false;

        for (const NeighborCandidate& neighbor : selected_neighbors_) {
            const std::size_t neighbor_index = neighbor.boid_index;
            Vector3 offset = math::subtract(positions_[neighbor_index], position);
            if (noise_active && parameters_.perception_noise_strength > 0.0F) {
                const auto channel = static_cast<std::uint32_t>((neighbor_index * 2U) + 1U);
                offset = math::add(
                    offset,
                    math::scale(
                        deterministic_noise_vector(i, channel, noise_step),
                        parameters_.perception_noise_strength * perception_radius));
            }
            const float distance_squared = neighbor.distance_squared;

            if (!has_nearest_neighbor || distance_squared < nearest_neighbor_distance_squared) {
                nearest_neighbor_distance_squared = distance_squared;
                has_nearest_neighbor = true;
            }

            if (distance_squared <= separation_radius_squared) {
                const float distance = std::sqrt(distance_squared);
                separation_sum = math::add(separation_sum, math::scale(offset, -1.0F / distance));
                ++separation_count;
            }

            Vector3 perceived_velocity = velocities_[neighbor_index];
            if (noise_active && parameters_.perception_noise_strength > 0.0F) {
                const auto channel = static_cast<std::uint32_t>((neighbor_index * 2U) + 2U);
                perceived_velocity = math::add(
                    perceived_velocity,
                    math::scale(
                        deterministic_noise_vector(i, channel, noise_step),
                        parameters_.perception_noise_strength * parameters_.max_speed));
            }
            alignment_sum = math::add(alignment_sum, perceived_velocity);
            cohesion_sum = math::add(cohesion_sum, math::add(position, offset));
            ++flock_count;
        }

        Vector3 acceleration{};
        if (separation_count > 0) {
            const Vector3 average = math::scale(separation_sum, 1.0F / static_cast<float>(separation_count));
            const Vector3 desired = math::scale(math::normalize_safe(average), parameters_.max_speed);
            const Vector3 steering = math::clamp_length(math::subtract(desired, velocity), parameters_.max_force);
            acceleration = math::add(acceleration, math::scale(steering, parameters_.separation_weight));
        }

        if (metrics != nullptr) {
            metrics->record_effective_radius(effective_perception_radius);
            metrics->record_effective_neighbors(flock_count);
            metrics->record_neighbor_filtering(
                accepted_before_topology,
                selected_neighbors_.size(),
                fov_rejected_count,
                radius_rejected_count);
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

        if (behavior.add_bird_altitude_acceleration) {
            acceleration = math::add(acceleration, bird_altitude_acceleration(position));
        }
        if (behavior.add_fish_medium_acceleration) {
            acceleration = math::add(acceleration, fish_medium_acceleration(position, velocity));
        }
        if (noise_active && parameters_.steering_noise_strength > 0.0F) {
            acceleration = math::add(
                acceleration,
                math::scale(
                    deterministic_noise_vector(i, 10'001U, noise_step),
                    parameters_.steering_noise_strength * parameters_.max_force));
        }

        accelerations_[i] = math::clamp_length(acceleration, parameters_.max_force);
    }

    integrate(dt, behavior, noise_active, noise_step);
}

void BoidSimulation::update_cell_aggregate_social(float dt, SimulationMetrics* metrics, ModelBehavior behavior)
{
    const float social_radius = base_perception_radius(parameters_);
    const float social_radius_squared = social_radius * social_radius;
    const float separation_radius_squared = parameters_.separation_radius * parameters_.separation_radius;
    const bool noise_active = behavior.apply_noise && parameters_.noise_enabled;
    const std::uint64_t noise_step = noise_step_;

    for (std::size_t i = 0; i < positions_.size(); ++i) {
        const Vector3 position = positions_[i];
        const Vector3 velocity = velocities_[i];

        NeighborQueryDiagnostics separation_diagnostics{};
        spatial_hash_.query_neighbors(position, parameters_.separation_radius, neighbor_indices_, separation_diagnostics);
        if (metrics != nullptr) {
            metrics->record_neighbor_query(separation_diagnostics.candidates_tested, separation_diagnostics.visited_cells);
        }

        Vector3 separation_sum{};
        std::size_t separation_count = 0;
        float nearest_neighbor_distance_squared = 0.0F;
        bool has_nearest_neighbor = false;
        for (const std::size_t neighbor_index : neighbor_indices_) {
            if (neighbor_index == i) {
                continue;
            }
            const Vector3 offset = math::subtract(positions_[neighbor_index], position);
            const float distance_squared = math::length_squared(offset);
            if (distance_squared <= 0.000001F || distance_squared > separation_radius_squared) {
                continue;
            }
            if (!has_nearest_neighbor || distance_squared < nearest_neighbor_distance_squared) {
                nearest_neighbor_distance_squared = distance_squared;
                has_nearest_neighbor = true;
            }
            const float distance = std::sqrt(distance_squared);
            separation_sum = math::add(separation_sum, math::scale(offset, -1.0F / distance));
            ++separation_count;
        }

        NeighborQueryDiagnostics aggregate_diagnostics{};
        spatial_hash_.query_cell_aggregates(position, social_radius, aggregate_cells_, aggregate_diagnostics);

        Vector3 weighted_velocity_sum{};
        Vector3 weighted_centroid_sum{};
        double social_weight_sum = 0.0;
        std::size_t aggregate_cells_used = 0;
        std::size_t aggregate_radius_rejected_count = 0;
        std::size_t aggregate_fov_rejected_count = 0;
        const CellCoord own_cell = spatial_hash_.cell_for(position);
        for (CellAggregate aggregate : aggregate_cells_) {
            if (aggregate.coord == own_cell) {
                if (aggregate.count <= 1U) {
                    continue;
                }
                --aggregate.count;
                aggregate.sum_position = math::subtract(aggregate.sum_position, position);
                aggregate.sum_velocity = math::subtract(aggregate.sum_velocity, velocity);
                const float inverse_count = 1.0F / static_cast<float>(aggregate.count);
                aggregate.centroid = math::scale(aggregate.sum_position, inverse_count);
                aggregate.average_velocity = math::scale(aggregate.sum_velocity, inverse_count);
            }

            const Vector3 offset = math::subtract(aggregate.centroid, position);
            const float distance_squared = math::length_squared(offset);
            if (distance_squared <= 0.000001F || distance_squared > social_radius_squared) {
                ++aggregate_radius_rejected_count;
                continue;
            }
            if (behavior.filter_neighbors_by_field_of_view && !neighbor_in_field_of_view(velocity, offset)) {
                ++aggregate_fov_rejected_count;
                continue;
            }

            const float weight = static_cast<float>(aggregate.count);
            weighted_centroid_sum = math::add(weighted_centroid_sum, math::scale(aggregate.centroid, weight));
            weighted_velocity_sum = math::add(weighted_velocity_sum, math::scale(aggregate.average_velocity, weight));
            social_weight_sum += static_cast<double>(weight);
            ++aggregate_cells_used;
        }

        Vector3 acceleration{};
        if (separation_count > 0) {
            const Vector3 average = math::scale(separation_sum, 1.0F / static_cast<float>(separation_count));
            const Vector3 desired = math::scale(math::normalize_safe(average), parameters_.max_speed);
            const Vector3 steering = math::clamp_length(math::subtract(desired, velocity), parameters_.max_force);
            acceleration = math::add(acceleration, math::scale(steering, parameters_.separation_weight));
        }

        if (metrics != nullptr) {
            metrics->record_effective_radius(social_radius);
            metrics->record_effective_neighbors(static_cast<std::size_t>(social_weight_sum));
            metrics->record_neighbor_filtering(
                aggregate_cells_used,
                aggregate_cells_used,
                aggregate_fov_rejected_count,
                aggregate_radius_rejected_count);
            metrics->record_cell_aggregate_query(
                aggregate_diagnostics.visited_cells,
                aggregate_diagnostics.candidates_tested,
                aggregate_radius_rejected_count,
                aggregate_fov_rejected_count);
            metrics->record_cell_aggregate_social(separation_count, aggregate_cells_used, social_weight_sum);
            if (has_nearest_neighbor) {
                metrics->record_nearest_neighbor_distance(std::sqrt(nearest_neighbor_distance_squared));
            }
        }

        if (social_weight_sum > 0.0) {
            const float inverse_weight = 1.0F / static_cast<float>(social_weight_sum);
            const Vector3 average_velocity = math::scale(weighted_velocity_sum, inverse_weight);
            const Vector3 desired = math::scale(math::normalize_safe(average_velocity), parameters_.max_speed);
            const Vector3 steering = math::clamp_length(math::subtract(desired, velocity), parameters_.max_force);
            acceleration = math::add(acceleration, math::scale(steering, parameters_.alignment_weight));

            const Vector3 center = math::scale(weighted_centroid_sum, inverse_weight);
            acceleration = math::add(acceleration, math::scale(seek(position, velocity, center), parameters_.cohesion_weight));
        }

        if (behavior.add_bird_altitude_acceleration) {
            acceleration = math::add(acceleration, bird_altitude_acceleration(position));
        }
        if (behavior.add_fish_medium_acceleration) {
            acceleration = math::add(acceleration, fish_medium_acceleration(position, velocity));
        }
        if (noise_active && parameters_.steering_noise_strength > 0.0F) {
            acceleration = math::add(
                acceleration,
                math::scale(
                    deterministic_noise_vector(i, 10'001U, noise_step),
                    parameters_.steering_noise_strength * parameters_.max_force));
        }

        accelerations_[i] = math::clamp_length(acceleration, parameters_.max_force);
    }

    integrate(dt, behavior, noise_active, noise_step);
}

void BoidSimulation::integrate(float dt, ModelBehavior behavior, bool noise_active, std::uint64_t noise_step)
{
    for (std::size_t i = 0; i < positions_.size(); ++i) {
        const Vector3 previous_velocity = velocities_[i];
        Vector3 desired_velocity = math::add(previous_velocity, math::scale(accelerations_[i], dt));
        if (behavior.apply_bird_velocity_constraints || behavior.apply_fish_velocity_constraints) {
            desired_velocity = limit_turn_rate(previous_velocity, desired_velocity, dt);
        }
        if (behavior.apply_fish_velocity_constraints && parameters_.drag_coefficient > 0.0F) {
            const float drag_factor = std::max(0.0F, 1.0F - parameters_.drag_coefficient * dt);
            desired_velocity = math::scale(desired_velocity, drag_factor);
        }
        desired_velocity = math::clamp_length(desired_velocity, parameters_.max_speed);
        if (behavior.apply_bird_velocity_constraints) {
            desired_velocity = enforce_min_speed(desired_velocity);
            if (parameters_.max_climb_rate > 0.0F) {
                desired_velocity.y = std::clamp(desired_velocity.y, -parameters_.max_climb_rate, parameters_.max_climb_rate);
                desired_velocity = enforce_min_speed(desired_velocity);
            }
        }
        if (noise_active && parameters_.velocity_noise_strength > 0.0F) {
            desired_velocity = math::add(
                desired_velocity,
                math::scale(
                    deterministic_noise_vector(i, 20'001U, noise_step),
                    parameters_.velocity_noise_strength * parameters_.max_speed));
            desired_velocity = math::clamp_length(desired_velocity, parameters_.max_speed);
        }
        velocities_[i] = desired_velocity;
        positions_[i] = math::add(positions_[i], math::scale(velocities_[i], dt));
        wrap_position(positions_[i]);
    }

    if (behavior.apply_noise) {
        ++noise_step_;
    }
}

void BoidSimulation::add_boid(Vector3 position, Vector3 velocity)
{
    positions_.push_back(position);
    velocities_.push_back(velocity);
    accelerations_.push_back(Vector3{});

    if (neighbor_indices_.capacity() < positions_.size()) {
        neighbor_indices_.reserve(positions_.size());
        selected_neighbors_.reserve(positions_.size());
        aggregate_cells_.reserve(positions_.size());
    }
}

bool BoidSimulation::neighbor_in_field_of_view(Vector3 velocity, Vector3 offset) const noexcept
{
    if (parameters_.field_of_view_degrees >= 359.999F) {
        return true;
    }
    if (parameters_.field_of_view_degrees <= 0.0F) {
        return false;
    }

    const Vector3 forward = math::normalize_safe(velocity);
    const Vector3 direction = math::normalize_safe(offset);
    if (math::length_squared(forward) <= 0.000001F || math::length_squared(direction) <= 0.000001F) {
        return true;
    }

    const float half_angle_radians = parameters_.field_of_view_degrees * (std::numbers::pi_v<float> / 180.0F) * 0.5F;
    const float minimum_dot = std::cos(half_angle_radians);
    return math::dot(forward, direction) >= minimum_dot;
}

Vector3 BoidSimulation::bird_altitude_acceleration(Vector3 position) const noexcept
{
    Vector3 acceleration{0.0F, parameters_.lift_strength - parameters_.gravity, 0.0F};
    const float lower = parameters_.altitude_target - parameters_.altitude_band;
    const float upper = parameters_.altitude_target + parameters_.altitude_band;
    if (position.y < lower) {
        acceleration.y += (lower - position.y) * parameters_.altitude_correction_strength;
    } else if (position.y > upper) {
        acceleration.y -= (position.y - upper) * parameters_.altitude_correction_strength;
    }
    return acceleration;
}

Vector3 BoidSimulation::fish_medium_acceleration(Vector3 position, Vector3 velocity) const noexcept
{
    Vector3 acceleration{0.0F, parameters_.buoyancy_strength, 0.0F};

    const float lower = parameters_.target_depth - parameters_.depth_band;
    const float upper = parameters_.target_depth + parameters_.depth_band;
    if (position.y < lower) {
        acceleration.y += (lower - position.y) * parameters_.depth_correction_strength;
    } else if (position.y > upper) {
        acceleration.y -= (position.y - upper) * parameters_.depth_correction_strength;
    }

    if (parameters_.current_strength > 0.0F && math::length_squared(parameters_.current_direction) > 0.000001F) {
        const Vector3 current = math::scale(math::normalize_safe(parameters_.current_direction), parameters_.current_strength);
        acceleration = math::add(acceleration, math::subtract(current, velocity));
    }

    return acceleration;
}

Vector3 BoidSimulation::enforce_min_speed(Vector3 velocity) const noexcept
{
    if (parameters_.min_speed <= 0.0F) {
        return velocity;
    }

    const float speed = math::length(velocity);
    if (speed >= parameters_.min_speed) {
        return velocity;
    }

    const Vector3 direction = speed > 0.000001F ? math::scale(velocity, 1.0F / speed) : Vector3{1.0F, 0.0F, 0.0F};
    const float target_speed = std::min(parameters_.min_speed, parameters_.max_speed);
    return math::scale(direction, target_speed);
}

Vector3 BoidSimulation::limit_turn_rate(Vector3 previous_velocity, Vector3 desired_velocity, float dt) const noexcept
{
    if (parameters_.max_turn_rate <= 0.0F || dt <= 0.0F) {
        return desired_velocity;
    }

    const float previous_speed = math::length(previous_velocity);
    const float desired_speed = math::length(desired_velocity);
    if (previous_speed <= 0.000001F || desired_speed <= 0.000001F) {
        return desired_velocity;
    }

    const Vector3 from = math::scale(previous_velocity, 1.0F / previous_speed);
    const Vector3 to = math::scale(desired_velocity, 1.0F / desired_speed);
    const float dot = std::clamp(math::dot(from, to), -1.0F, 1.0F);
    const float angle = std::acos(dot);
    const float max_angle = parameters_.max_turn_rate * (std::numbers::pi_v<float> / 180.0F) * dt;
    if (angle <= max_angle || angle <= 0.000001F) {
        return desired_velocity;
    }

    const float blend = max_angle / angle;
    const Vector3 direction = math::normalize_safe(math::add(math::scale(from, 1.0F - blend), math::scale(to, blend)));
    return math::scale(direction, desired_speed);
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
        spatial_hash_.insert(i, positions_[i], velocities_[i]);
    }
}

void BoidSimulation::record_collective_metrics(SimulationMetrics& metrics) const noexcept
{
    metrics.noise_strength = combined_noise_strength();
    if (positions_.empty()) {
        metrics.record_collective_behavior(0.0F, 0.0F, 0.0F, 0.0F);
        metrics.order_loss = 1.0F;
        return;
    }

    Vector3 center{};
    Vector3 normalized_velocity_sum{};
    double speed_total = 0.0;
    double altitude_total = 0.0;
    double depth_total = 0.0;
    std::size_t stall_count = 0;
    std::size_t near_ground_count = 0;

    for (std::size_t i = 0; i < positions_.size(); ++i) {
        center = math::add(center, positions_[i]);
        normalized_velocity_sum = math::add(normalized_velocity_sum, math::normalize_safe(velocities_[i]));
        const float speed = math::length(velocities_[i]);
        speed_total += static_cast<double>(speed);
        altitude_total += static_cast<double>(positions_[i].y);
        depth_total += static_cast<double>(positions_[i].y);
        if (parameters_.min_speed > 0.0F && speed < parameters_.min_speed) {
            ++stall_count;
        }
        if (positions_[i].y <= 0.0F) {
            ++near_ground_count;
        }
    }

    const auto boid_count = static_cast<float>(positions_.size());
    center = math::scale(center, 1.0F / boid_count);

    const double mean_altitude = altitude_total / static_cast<double>(positions_.size());
    const double mean_depth = depth_total / static_cast<double>(positions_.size());
    double altitude_variance_total = 0.0;
    double depth_variance_total = 0.0;
    double distance_total = 0.0;
    double distance_squared_total = 0.0;
    for (const Vector3 position : positions_) {
        const double altitude_error = static_cast<double>(position.y) - mean_altitude;
        altitude_variance_total += altitude_error * altitude_error;
        const double depth_error = static_cast<double>(position.y) - mean_depth;
        depth_variance_total += depth_error * depth_error;
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
    metrics.order_loss = 1.0F - polarization;
    metrics.record_fish_metrics(
        static_cast<float>(mean_depth),
        static_cast<float>(depth_variance_total / static_cast<double>(positions_.size())));
    metrics.record_flight_metrics(
        static_cast<float>(mean_altitude),
        static_cast<float>(altitude_variance_total / static_cast<double>(positions_.size())),
        stall_count,
        near_ground_count);
}

Vector3 BoidSimulation::deterministic_noise_vector(std::size_t boid_index, std::uint32_t channel, std::uint64_t step) const
{
    const auto seed = static_cast<std::uint32_t>(
        parameters_.random_seed
        + parameters_.noise_seed_offset
        + (static_cast<std::uint32_t>(boid_index) * 747'796'405U)
        + (channel * 2'891'336'453U)
        + (static_cast<std::uint32_t>(step) * 277'803'737U));
    std::mt19937 rng{seed};
    return random_direction(rng);
}

float BoidSimulation::combined_noise_strength() const noexcept
{
    if (!parameters_.noise_enabled || parameters_.model != SimulationModel::NoiseExperiment) {
        return 0.0F;
    }
    return std::max({
        parameters_.perception_noise_strength,
        parameters_.steering_noise_strength,
        parameters_.velocity_noise_strength,
    });
}

Vector3 BoidSimulation::seek(Vector3 position, Vector3 velocity, Vector3 target) const noexcept
{
    const Vector3 desired_direction = math::normalize_safe(math::subtract(target, position));
    const Vector3 desired_velocity = math::scale(desired_direction, parameters_.max_speed);
    return math::clamp_length(math::subtract(desired_velocity, velocity), parameters_.max_force);
}

} // namespace flock3d::sim
