#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <cstddef>
#include <cmath>
#include <numbers>
#include <vector>

#include <raylib.h>

#include <flock3d/math/Vec3.hpp>
#include <flock3d/sim/BoidSimulation.hpp>
#include <flock3d/sim/NeighborSelection.hpp>
#include <flock3d/sim/Scenario.hpp>
#include <flock3d/sim/SimulationMetrics.hpp>
#include <flock3d/sim/SimulationParameters.hpp>

namespace {

[[nodiscard]] flock3d::sim::SimulationParameters steering_test_parameters()
{
    flock3d::sim::SimulationParameters parameters{};
    parameters.boid_count = 0;
    parameters.world_half_extent = 100.0F;
    parameters.max_speed = 10.0F;
    parameters.neighbor_radius = 10.0F;
    parameters.separation_radius = 2.0F;
    parameters.separation_weight = 0.0F;
    parameters.alignment_weight = 0.0F;
    parameters.cohesion_weight = 0.0F;
    parameters.max_force = 10.0F;
    parameters.spatial_cell_size = 4.0F;
    return parameters;
}


[[nodiscard]] double trajectory_distance(const flock3d::sim::BoidSimulation& lhs, const flock3d::sim::BoidSimulation& rhs)
{
    double total = 0.0;
    for (std::size_t i = 0; i < lhs.positions().size(); ++i) {
        const auto position_delta = flock3d::math::subtract(lhs.positions()[i], rhs.positions()[i]);
        const auto velocity_delta = flock3d::math::subtract(lhs.velocities()[i], rhs.velocities()[i]);
        total += flock3d::math::length(position_delta);
        total += flock3d::math::length(velocity_delta);
    }
    return total;
}

void run_steps(flock3d::sim::BoidSimulation& simulation, int steps, flock3d::sim::SimulationMetrics& metrics)
{
    for (int step = 0; step < steps; ++step) {
        simulation.update(1.0F / 120.0F, &metrics);
    }
}

} // namespace

TEST_CASE("BoidSimulation wraps positions at world bounds", "[simulation]")
{
    flock3d::sim::SimulationParameters parameters{};
    parameters.boid_count = 0;
    parameters.world_half_extent = 10.0F;
    parameters.max_speed = 100.0F;

    flock3d::sim::BoidSimulation simulation{parameters};
    simulation.add_boid(Vector3{9.9F, 0.0F, 0.0F}, Vector3{2.0F, 0.0F, 0.0F});

    simulation.update(0.1F);

    REQUIRE(simulation.positions().size() == 1);
    CHECK(simulation.positions().front().x == -10.0F);
}

TEST_CASE("BoidSimulation separation steers away from close neighbor", "[simulation]")
{
    auto parameters = steering_test_parameters();
    parameters.separation_weight = 1.0F;

    flock3d::sim::BoidSimulation simulation{parameters};
    simulation.add_boid(Vector3{0.0F, 0.0F, 0.0F}, Vector3{});
    simulation.add_boid(Vector3{1.0F, 0.0F, 0.0F}, Vector3{});

    simulation.update(1.0F);

    REQUIRE(simulation.velocities().size() == 2);
    CHECK(simulation.velocities()[0].x < 0.0F);
    CHECK(simulation.velocities()[1].x > 0.0F);
}

TEST_CASE("BoidSimulation cohesion steers toward neighbor center", "[simulation]")
{
    auto parameters = steering_test_parameters();
    parameters.cohesion_weight = 1.0F;
    parameters.separation_radius = 0.25F;

    flock3d::sim::BoidSimulation simulation{parameters};
    simulation.add_boid(Vector3{0.0F, 0.0F, 0.0F}, Vector3{});
    simulation.add_boid(Vector3{4.0F, 0.0F, 0.0F}, Vector3{});

    simulation.update(1.0F);

    REQUIRE(simulation.velocities().size() == 2);
    CHECK(simulation.velocities()[0].x > 0.0F);
    CHECK(simulation.velocities()[1].x < 0.0F);
}

TEST_CASE("BoidSimulation clamps velocity to max speed", "[simulation]")
{
    auto parameters = steering_test_parameters();
    parameters.max_speed = 3.0F;

    flock3d::sim::BoidSimulation simulation{parameters};
    simulation.add_boid(Vector3{}, Vector3{30.0F, 0.0F, 0.0F});

    simulation.update(0.0F);

    REQUIRE(simulation.velocities().size() == 1);
    CHECK(flock3d::math::length(simulation.velocities().front()) <= 3.0001F);
}


TEST_CASE("Neighbor selection excludes self in fixed-radius uncapped mode", "[simulation][neighbors]")
{
    auto parameters = steering_test_parameters();
    parameters.max_selected_neighbors = 0U;

    flock3d::sim::BoidSimulation simulation{parameters};
    simulation.add_boid(Vector3{0.0F, 0.0F, 0.0F}, Vector3{});

    flock3d::sim::SimulationMetrics metrics{};
    simulation.update(0.0F, &metrics);

    CHECK(metrics.neighbor_queries == 1);
    CHECK(metrics.neighbor_candidates == 1);
    CHECK(metrics.neighbor_total == 0);
    CHECK(metrics.selected_neighbors_mean == Catch::Approx(0.0));
}

TEST_CASE("Neighbor selection respects max_selected_neighbors", "[simulation][neighbors]")
{
    auto parameters = steering_test_parameters();
    parameters.max_selected_neighbors = 2U;

    flock3d::sim::BoidSimulation simulation{parameters};
    simulation.add_boid(Vector3{0.0F, 0.0F, 0.0F}, Vector3{});
    simulation.add_boid(Vector3{1.0F, 0.0F, 0.0F}, Vector3{});
    simulation.add_boid(Vector3{2.0F, 0.0F, 0.0F}, Vector3{});
    simulation.add_boid(Vector3{3.0F, 0.0F, 0.0F}, Vector3{});

    flock3d::sim::SimulationMetrics metrics{};
    simulation.update(0.0F, &metrics);

    CHECK(metrics.max_effective_neighbors_per_query <= 2U);
    CHECK(metrics.selected_neighbors_mean == Catch::Approx(2.0));
}

TEST_CASE("Neighbor selection keeps closest deterministic candidates", "[simulation][neighbors]")
{
    std::vector<flock3d::sim::NeighborCandidate> candidates{
        {4U, 4.0F},
        {3U, 1.0F},
        {2U, 1.0F},
        {1U, 9.0F},
    };

    flock3d::sim::select_closest_neighbors(candidates, 3U);

    REQUIRE(candidates.size() == 3U);
    CHECK(candidates[0].boid_index == 2U);
    CHECK(candidates[1].boid_index == 3U);
    CHECK(candidates[2].boid_index == 4U);
}

TEST_CASE("Adaptive perception radius clamps to configured bounds", "[simulation][neighbors]")
{
    auto parameters = steering_test_parameters();
    parameters.base_perception_radius = 10.0F;
    parameters.min_perception_radius = 5.0F;
    parameters.max_perception_radius = 15.0F;
    parameters.target_neighbor_count = 4U;
    parameters.adaptive_perception_enabled = true;

    CHECK(flock3d::sim::compute_effective_perception_radius(parameters, 100U) == Catch::Approx(5.0F));
    CHECK(flock3d::sim::compute_effective_perception_radius(parameters, 1U) == Catch::Approx(15.0F));
}

TEST_CASE("Adaptive perception radius shrinks when local density exceeds target", "[simulation][neighbors]")
{
    auto parameters = steering_test_parameters();
    parameters.base_perception_radius = 10.0F;
    parameters.min_perception_radius = 1.0F;
    parameters.max_perception_radius = 20.0F;
    parameters.target_neighbor_count = 4U;
    parameters.adaptive_perception_enabled = true;

    CHECK(flock3d::sim::compute_effective_perception_radius(parameters, 16U) == Catch::Approx(5.0F));
}

TEST_CASE("Adaptive perception radius expands in sparse neighborhoods", "[simulation][neighbors]")
{
    auto parameters = steering_test_parameters();
    parameters.base_perception_radius = 10.0F;
    parameters.min_perception_radius = 1.0F;
    parameters.max_perception_radius = 20.0F;
    parameters.target_neighbor_count = 16U;
    parameters.adaptive_perception_enabled = true;

    CHECK(flock3d::sim::compute_effective_perception_radius(parameters, 4U) == Catch::Approx(20.0F));
}

TEST_CASE("Fixed-radius uncapped neighbor mode remains available", "[simulation][neighbors]")
{
    auto parameters = steering_test_parameters();
    parameters.neighbor_radius = 10.0F;
    parameters.max_selected_neighbors = 0U;
    parameters.adaptive_perception_enabled = false;
    flock3d::sim::sync_spatial_cell_size_to_query_radius(parameters);

    flock3d::sim::BoidSimulation simulation{parameters};
    simulation.add_boid(Vector3{0.0F, 0.0F, 0.0F}, Vector3{});
    simulation.add_boid(Vector3{1.0F, 0.0F, 0.0F}, Vector3{});
    simulation.add_boid(Vector3{2.0F, 0.0F, 0.0F}, Vector3{});
    simulation.add_boid(Vector3{3.0F, 0.0F, 0.0F}, Vector3{});

    flock3d::sim::SimulationMetrics metrics{};
    simulation.update(0.0F, &metrics);

    CHECK(metrics.max_effective_neighbors_per_query == 3U);
    CHECK(metrics.avg_effective_neighbors_per_query == Catch::Approx(3.0));
    CHECK(metrics.effective_radius_mean == Catch::Approx(10.0F));
}


TEST_CASE("Cell aggregate social uses visibility-weighted social perception", "[simulation][neighbors][aggregates]")
{
    auto parameters = steering_test_parameters();
    parameters.model = flock3d::sim::SimulationModel::BirdFlight;
    parameters.neighbor_mode = flock3d::sim::NeighborMode::CellAggregateSocial;
    parameters.field_of_view_degrees = 180.0F;
    parameters.cohesion_weight = 1.0F;
    parameters.max_force = 10.0F;
    parameters.separation_radius = 0.5F;
    flock3d::sim::sync_spatial_cell_size_to_query_radius(parameters);

    flock3d::sim::BoidSimulation simulation{parameters};
    simulation.add_boid(Vector3{0.0F, 10.0F, 0.0F}, Vector3{2.0F, 0.0F, 0.0F});
    simulation.add_boid(Vector3{-4.0F, 10.0F, 0.0F}, Vector3{2.0F, 0.0F, 0.0F});

    flock3d::sim::SimulationMetrics metrics{};
    simulation.update(1.0F, &metrics);

    CHECK(simulation.velocities()[0].x == Catch::Approx(2.0F));
    CHECK(metrics.exact_separation_neighbors_total == 0U);
}

TEST_CASE("Cell aggregate social applies distance falloff to aggregate weights", "[simulation][neighbors][aggregates]")
{
    auto parameters = steering_test_parameters();
    parameters.neighbor_mode = flock3d::sim::NeighborMode::CellAggregateSocial;
    parameters.neighbor_radius = 10.0F;
    parameters.base_perception_radius = 10.0F;
    parameters.separation_radius = 0.5F;
    flock3d::sim::sync_spatial_cell_size_to_query_radius(parameters);

    flock3d::sim::BoidSimulation simulation{parameters};
    simulation.add_boid(Vector3{0.0F, 0.0F, 0.0F}, Vector3{});
    simulation.add_boid(Vector3{5.0F, 0.0F, 0.0F}, Vector3{});

    flock3d::sim::SimulationMetrics metrics{};
    simulation.update(0.0F, &metrics);

    CHECK(metrics.social_weight_sum_mean > 0.0);
    CHECK(metrics.social_weight_sum_mean < 1.0);
    CHECK(metrics.exact_separation_neighbors_total == 0U);
}

TEST_CASE("Cell aggregate social can adapt social radius from local density", "[simulation][neighbors][aggregates]")
{
    auto parameters = steering_test_parameters();
    parameters.neighbor_mode = flock3d::sim::NeighborMode::CellAggregateSocial;
    parameters.base_perception_radius = 10.0F;
    parameters.min_perception_radius = 2.0F;
    parameters.max_perception_radius = 20.0F;
    parameters.target_neighbor_count = 1U;
    parameters.adaptive_perception_enabled = true;
    parameters.separation_radius = 0.5F;
    flock3d::sim::sync_spatial_cell_size_to_query_radius(parameters);

    flock3d::sim::BoidSimulation simulation{parameters};
    simulation.add_boid(Vector3{-2.0F, 0.0F, 0.0F}, Vector3{});
    simulation.add_boid(Vector3{-1.0F, 0.0F, 0.0F}, Vector3{});
    simulation.add_boid(Vector3{0.0F, 0.0F, 0.0F}, Vector3{});
    simulation.add_boid(Vector3{1.0F, 0.0F, 0.0F}, Vector3{});
    simulation.add_boid(Vector3{2.0F, 0.0F, 0.0F}, Vector3{});

    flock3d::sim::SimulationMetrics metrics{};
    simulation.update(0.0F, &metrics);

    CHECK(metrics.effective_radius_mean < 10.0);
    CHECK(metrics.effective_radius_mean >= 2.0);
}

TEST_CASE("BoidSimulation records neighbor metrics", "[simulation]")
{
    auto parameters = steering_test_parameters();
    flock3d::sim::BoidSimulation simulation{parameters};
    simulation.add_boid(Vector3{0.0F, 0.0F, 0.0F}, Vector3{});
    simulation.add_boid(Vector3{1.0F, 0.0F, 0.0F}, Vector3{});
    simulation.add_boid(Vector3{50.0F, 0.0F, 0.0F}, Vector3{});

    flock3d::sim::SimulationMetrics metrics{};
    simulation.update(0.0F, &metrics);

    CHECK(metrics.neighbor_queries == 3);
    CHECK(metrics.neighbor_candidates == 5);
    CHECK(metrics.neighbor_total == 2);
    CHECK(metrics.avg_candidates_per_query == Catch::Approx(5.0 / 3.0));
    CHECK(metrics.max_candidates_per_query == 2);
    CHECK(metrics.avg_effective_neighbors_per_query == Catch::Approx(2.0 / 3.0));
    CHECK(metrics.max_effective_neighbors_per_query == 1);
    CHECK(metrics.average_neighbors_per_boid == 2.0F / 3.0F);
    CHECK(metrics.spatial_cell_count == 2);
    CHECK(metrics.spatial_hash_cell_count == 2);
    CHECK(metrics.avg_cell_occupancy == Catch::Approx(1.5));
    CHECK(metrics.max_cell_occupancy == 2);
}

TEST_CASE("BoidSimulation synchronizes spatial cells before measuring hash candidates", "[simulation][metrics]")
{
    auto parameters = steering_test_parameters();
    parameters.neighbor_radius = 1.0F;
    parameters.separation_radius = 0.25F;
    parameters.spatial_cell_size = 10.0F;

    flock3d::sim::BoidSimulation simulation{parameters};
    simulation.add_boid(Vector3{0.0F, 0.0F, 0.0F}, Vector3{});
    simulation.add_boid(Vector3{2.0F, 0.0F, 0.0F}, Vector3{});
    simulation.add_boid(Vector3{3.0F, 0.0F, 0.0F}, Vector3{});

    flock3d::sim::SimulationMetrics metrics{};
    simulation.update(0.0F, &metrics);

    CHECK(simulation.parameters().spatial_cell_size == Catch::Approx(1.0F));
    CHECK(metrics.neighbor_queries == 3);
    CHECK(metrics.neighbor_candidates == 5);
    CHECK(metrics.neighbor_total == 2);
    CHECK(metrics.avg_candidates_per_query == Catch::Approx(5.0 / 3.0));
    CHECK(metrics.max_candidates_per_query == 2);
    CHECK(metrics.avg_effective_neighbors_per_query == Catch::Approx(2.0 / 3.0));
    CHECK(metrics.max_effective_neighbors_per_query == 1);
    CHECK(metrics.spatial_cell_count == 3);
    CHECK(metrics.avg_cell_occupancy == Catch::Approx(1.0));
    CHECK(metrics.max_cell_occupancy == 1);
}

TEST_CASE("BoidSimulation repairs direct radius edits before update", "[simulation][parameters]")
{
    flock3d::sim::SimulationParameters parameters{};
    parameters.boid_count = 0;
    parameters.neighbor_radius = 4.0F;
    parameters.separation_radius = 2.0F;
    parameters.spatial_cell_size = 4.0F;

    flock3d::sim::BoidSimulation simulation{parameters};
    simulation.parameters().neighbor_radius = 5.0F;

    flock3d::sim::SimulationMetrics metrics{};
    simulation.update(0.0F, &metrics);

    CHECK(simulation.parameters().spatial_cell_size == Catch::Approx(5.0F));
}

TEST_CASE("FixedTimestepAccumulator consumes deterministic 120 Hz steps", "[time]")
{
    flock3d::sim::FixedTimestepAccumulator accumulator{1.0 / 120.0};

    accumulator.add_frame_time(1.0 / 60.0);

    CHECK(accumulator.consume_step());
    CHECK(accumulator.consume_step());
    CHECK_FALSE(accumulator.consume_step());
}

TEST_CASE("FixedTimestepAccumulator clamps very large frame deltas", "[time]")
{
    flock3d::sim::FixedTimestepAccumulator accumulator{1.0 / 120.0};
    accumulator.add_frame_time(10.0);

    int steps = 0;
    while (accumulator.consume_step()) {
        ++steps;
    }

    CHECK(steps == 30);
}


TEST_CASE("BoidSimulation reset is reproducible for the same seed", "[simulation][seed]")
{
    flock3d::sim::SimulationParameters parameters{};
    parameters.boid_count = 16;
    parameters.random_seed = 4242U;

    flock3d::sim::BoidSimulation first{parameters};
    flock3d::sim::BoidSimulation second{parameters};

    REQUIRE(first.positions().size() == second.positions().size());
    REQUIRE(first.velocities().size() == second.velocities().size());
    CHECK(first.positions().front().x == second.positions().front().x);
    CHECK(first.positions().front().y == second.positions().front().y);
    CHECK(first.positions().front().z == second.positions().front().z);
    CHECK(first.velocities().front().x == second.velocities().front().x);
    CHECK(first.velocities().front().y == second.velocities().front().y);
    CHECK(first.velocities().front().z == second.velocities().front().z);

    first.update(0.25F);
    first.reset();
    CHECK(first.positions().front().x == second.positions().front().x);
    CHECK(first.velocities().front().z == second.velocities().front().z);
}

TEST_CASE("BoidSimulation uses different initial state for different seeds", "[simulation][seed]")
{
    flock3d::sim::SimulationParameters parameters{};
    parameters.boid_count = 16;
    parameters.random_seed = 111U;
    flock3d::sim::BoidSimulation first{parameters};

    parameters.random_seed = 222U;
    flock3d::sim::BoidSimulation second{parameters};

    REQUIRE(first.positions().size() == second.positions().size());
    const bool different_position = first.positions().front().x != second.positions().front().x
        || first.positions().front().y != second.positions().front().y
        || first.positions().front().z != second.positions().front().z;
    const bool different_velocity = first.velocities().front().x != second.velocities().front().x
        || first.velocities().front().y != second.velocities().front().y
        || first.velocities().front().z != second.velocities().front().z;
    CHECK((different_position || different_velocity));
}

TEST_CASE("SimulationMetrics polarization is near one for aligned velocities", "[simulation][metrics]")
{
    auto parameters = steering_test_parameters();
    flock3d::sim::BoidSimulation simulation{parameters};
    simulation.add_boid(Vector3{-1.0F, 0.0F, 0.0F}, Vector3{2.0F, 0.0F, 0.0F});
    simulation.add_boid(Vector3{1.0F, 0.0F, 0.0F}, Vector3{4.0F, 0.0F, 0.0F});

    flock3d::sim::SimulationMetrics metrics{};
    simulation.update(0.0F, &metrics);

    CHECK(metrics.polarization == Catch::Approx(1.0F).margin(0.0001F));
    CHECK(metrics.average_speed == Catch::Approx(3.0F).margin(0.0001F));
}

TEST_CASE("SimulationMetrics polarization is lower for opposing velocities", "[simulation][metrics]")
{
    auto parameters = steering_test_parameters();
    flock3d::sim::BoidSimulation simulation{parameters};
    simulation.add_boid(Vector3{-1.0F, 0.0F, 0.0F}, Vector3{2.0F, 0.0F, 0.0F});
    simulation.add_boid(Vector3{1.0F, 0.0F, 0.0F}, Vector3{-2.0F, 0.0F, 0.0F});

    flock3d::sim::SimulationMetrics metrics{};
    simulation.update(0.0F, &metrics);

    CHECK(metrics.polarization < 0.25F);
}

TEST_CASE("Scenario factory returns valid definitions for every scenario type", "[scenario]")
{
    for (const auto type : flock3d::sim::scenario_types) {
        const auto definition = flock3d::sim::build_scenario(type);
        CHECK(definition.type == type);
        CHECK_FALSE(definition.display_name.empty());
        CHECK_FALSE(definition.description.empty());
        CHECK(definition.simulation_parameters.boid_count > 0);
        CHECK(definition.simulation_parameters.random_seed > 0U);
        CHECK(definition.environment.world_half_extent > 0.0F);
        CHECK(definition.constraints.max_speed > 0.0F);
        CHECK(definition.behavior.neighbor_radius > 0.0F);
        CHECK(definition.simulation_parameters.spatial_cell_size
            == Catch::Approx(flock3d::sim::effective_query_radius(definition.simulation_parameters)));
    }
}

TEST_CASE("NoiseExperiment keeps spatial cells aligned with perception radius", "[scenario][noise]")
{
    const auto definition = flock3d::sim::build_scenario(flock3d::sim::ScenarioType::NoiseExperiment);
    const auto& parameters = definition.simulation_parameters;

    CHECK(parameters.neighbor_radius == Catch::Approx(5.0F));
    CHECK(parameters.spatial_cell_size == Catch::Approx(parameters.neighbor_radius));
}

TEST_CASE("NoiseExperiment zero noise matches ClassicBoids trajectory", "[simulation][noise]")
{
    auto classic = flock3d::sim::build_scenario(flock3d::sim::ScenarioType::ClassicBoids).simulation_parameters;
    classic.boid_count = 48U;
    classic.random_seed = 2024U;
    classic.neighbor_radius = 5.0F;
    flock3d::sim::sync_spatial_cell_size_to_query_radius(classic);

    auto noise = classic;
    noise.model = flock3d::sim::SimulationModel::NoiseExperiment;
    noise.noise_enabled = true;
    noise.perception_noise_strength = 0.0F;
    noise.steering_noise_strength = 0.0F;
    noise.velocity_noise_strength = 0.0F;

    flock3d::sim::BoidSimulation classic_sim{classic};
    flock3d::sim::BoidSimulation noise_sim{noise};
    flock3d::sim::SimulationMetrics classic_metrics{};
    flock3d::sim::SimulationMetrics noise_metrics{};
    run_steps(classic_sim, 24, classic_metrics);
    run_steps(noise_sim, 24, noise_metrics);

    CHECK(trajectory_distance(classic_sim, noise_sim) == Catch::Approx(0.0).margin(0.000001));
    CHECK(noise_metrics.noise_strength == Catch::Approx(0.0F));
    CHECK(noise_metrics.order_loss == Catch::Approx(1.0F - noise_metrics.polarization));
}

TEST_CASE("NoiseExperiment noisy behavior is deterministic for the same seed", "[simulation][noise][seed]")
{
    auto parameters = flock3d::sim::build_scenario(flock3d::sim::ScenarioType::NoiseExperiment).simulation_parameters;
    parameters.boid_count = 48U;
    parameters.random_seed = 3030U;
    parameters.perception_noise_strength = 0.2F;
    parameters.steering_noise_strength = 0.25F;
    parameters.velocity_noise_strength = 0.05F;

    flock3d::sim::BoidSimulation first{parameters};
    flock3d::sim::BoidSimulation second{parameters};
    flock3d::sim::SimulationMetrics first_metrics{};
    flock3d::sim::SimulationMetrics second_metrics{};
    run_steps(first, 30, first_metrics);
    run_steps(second, 30, second_metrics);

    CHECK(trajectory_distance(first, second) == Catch::Approx(0.0).margin(0.000001));
    CHECK(first_metrics.polarization == second_metrics.polarization);
}

TEST_CASE("NoiseExperiment strength changes trajectory", "[simulation][noise]")
{
    auto low = flock3d::sim::build_scenario(flock3d::sim::ScenarioType::NoiseExperiment).simulation_parameters;
    low.boid_count = 48U;
    low.random_seed = 4040U;
    low.perception_noise_strength = 0.02F;
    low.steering_noise_strength = 0.02F;

    auto high = low;
    high.perception_noise_strength = 0.35F;
    high.steering_noise_strength = 0.35F;
    high.velocity_noise_strength = 0.08F;

    flock3d::sim::BoidSimulation low_sim{low};
    flock3d::sim::BoidSimulation high_sim{high};
    flock3d::sim::SimulationMetrics low_metrics{};
    flock3d::sim::SimulationMetrics high_metrics{};
    run_steps(low_sim, 30, low_metrics);
    run_steps(high_sim, 30, high_metrics);

    CHECK(trajectory_distance(low_sim, high_sim) > 0.01);
    CHECK(high_metrics.noise_strength > low_metrics.noise_strength);
}

TEST_CASE("NoiseExperiment high noise lowers short-run polarization", "[simulation][noise][metrics]")
{
    auto baseline = flock3d::sim::build_scenario(flock3d::sim::ScenarioType::NoiseExperiment).simulation_parameters;
    baseline.boid_count = 96U;
    baseline.random_seed = 5050U;
    baseline.perception_noise_strength = 0.0F;
    baseline.steering_noise_strength = 0.0F;
    baseline.velocity_noise_strength = 0.0F;

    auto high = baseline;
    high.perception_noise_strength = 0.5F;
    high.steering_noise_strength = 0.5F;
    high.velocity_noise_strength = 0.2F;

    flock3d::sim::BoidSimulation baseline_sim{baseline};
    flock3d::sim::BoidSimulation high_sim{high};
    flock3d::sim::SimulationMetrics baseline_metrics{};
    flock3d::sim::SimulationMetrics high_metrics{};
    run_steps(baseline_sim, 120, baseline_metrics);
    run_steps(high_sim, 120, high_metrics);

    CHECK(high_metrics.polarization < baseline_metrics.polarization);
    CHECK(high_metrics.order_loss > baseline_metrics.order_loss);
}

TEST_CASE("ClassicBoids dispatch updates shared flocking", "[simulation][dispatch]")
{
    auto parameters = steering_test_parameters();
    parameters.model = flock3d::sim::SimulationModel::ClassicBoids;
    parameters.cohesion_weight = 1.0F;
    parameters.separation_radius = 0.25F;

    flock3d::sim::BoidSimulation simulation{parameters};
    simulation.add_boid(Vector3{0.0F, 0.0F, 0.0F}, Vector3{});
    simulation.add_boid(Vector3{4.0F, 0.0F, 0.0F}, Vector3{});

    simulation.update(1.0F);

    REQUIRE(simulation.velocities().size() == 2);
    CHECK(simulation.velocities()[0].x > 0.0F);
    CHECK(simulation.velocities()[1].x < 0.0F);
}

TEST_CASE("FishSchool dispatch preserves agent arrays with medium constraints", "[simulation][dispatch][fishschool]")
{
    auto parameters = flock3d::sim::build_scenario(flock3d::sim::ScenarioType::FishSchool).simulation_parameters;
    parameters.boid_count = 0;

    CHECK(parameters.model == flock3d::sim::SimulationModel::FishSchool);

    flock3d::sim::BoidSimulation simulation{parameters};
    simulation.add_boid(Vector3{0.0F, 0.0F, 0.0F}, Vector3{});
    simulation.add_boid(Vector3{1.0F, 0.0F, 0.0F}, Vector3{});
    const auto before_count = simulation.size();

    simulation.update(1.0F);

    CHECK(simulation.size() == before_count);
    CHECK(simulation.positions().size() == before_count);
    CHECK(simulation.velocities().size() == before_count);
}

TEST_CASE("Unknown SimulationModel falls back to classic flocking", "[simulation][dispatch]")
{
    auto parameters = steering_test_parameters();
    parameters.model = static_cast<flock3d::sim::SimulationModel>(255U);
    parameters.cohesion_weight = 1.0F;
    parameters.separation_radius = 0.25F;

    flock3d::sim::BoidSimulation simulation{parameters};
    simulation.add_boid(Vector3{0.0F, 0.0F, 0.0F}, Vector3{});
    simulation.add_boid(Vector3{4.0F, 0.0F, 0.0F}, Vector3{});

    simulation.update(1.0F);

    REQUIRE(simulation.velocities().size() == 2);
    CHECK(simulation.velocities()[0].x > 0.0F);
    CHECK(simulation.velocities()[1].x < 0.0F);
}

TEST_CASE("BirdFlight enforces minimum speed", "[simulation][birdflight]")
{
    auto parameters = steering_test_parameters();
    parameters.model = flock3d::sim::SimulationModel::BirdFlight;
    parameters.min_speed = 5.0F;
    parameters.max_speed = 10.0F;

    flock3d::sim::BoidSimulation simulation{parameters};
    simulation.add_boid(Vector3{0.0F, 10.0F, 0.0F}, Vector3{1.0F, 0.0F, 0.0F});

    simulation.update(0.0F);

    REQUIRE(simulation.velocities().size() == 1);
    CHECK(flock3d::math::length(simulation.velocities().front()) == Catch::Approx(5.0F));
}

TEST_CASE("BirdFlight limits turn rate", "[simulation][birdflight]")
{
    auto parameters = steering_test_parameters();
    parameters.model = flock3d::sim::SimulationModel::BirdFlight;
    parameters.max_speed = 30.0F;
    parameters.max_force = 100.0F;
    parameters.gravity = 10.0F;
    parameters.max_turn_rate = 10.0F;

    flock3d::sim::BoidSimulation simulation{parameters};
    simulation.add_boid(Vector3{0.0F, 10.0F, 0.0F}, Vector3{10.0F, 0.0F, 0.0F});

    simulation.update(1.0F);

    const auto direction = flock3d::math::normalize_safe(simulation.velocities().front());
    const auto angle_radians = std::acos(std::clamp(direction.x, -1.0F, 1.0F));
    const auto angle_degrees = angle_radians * 180.0F / std::numbers::pi_v<float>;
    CHECK(angle_degrees <= 10.1F);
}

TEST_CASE("BirdFlight field of view excludes boids behind the agent", "[simulation][birdflight]")
{
    auto parameters = steering_test_parameters();
    parameters.model = flock3d::sim::SimulationModel::BirdFlight;
    parameters.cohesion_weight = 1.0F;
    parameters.max_force = 10.0F;
    parameters.field_of_view_degrees = 180.0F;

    flock3d::sim::BoidSimulation simulation{parameters};
    simulation.add_boid(Vector3{0.0F, 10.0F, 0.0F}, Vector3{2.0F, 0.0F, 0.0F});
    simulation.add_boid(Vector3{-4.0F, 10.0F, 0.0F}, Vector3{2.0F, 0.0F, 0.0F});

    flock3d::sim::SimulationMetrics metrics{};
    simulation.update(1.0F, &metrics);

    CHECK(simulation.velocities()[0].x == Catch::Approx(2.0F));
    CHECK(metrics.neighbor_total == 1);
}

TEST_CASE("BirdFlight remains deterministic for the same seed", "[simulation][birdflight][seed]")
{
    auto parameters = flock3d::sim::build_scenario(flock3d::sim::ScenarioType::BirdFlight).simulation_parameters;
    parameters.boid_count = 32U;
    parameters.random_seed = 9090U;

    flock3d::sim::BoidSimulation first{parameters};
    flock3d::sim::BoidSimulation second{parameters};
    flock3d::sim::SimulationMetrics first_metrics{};
    flock3d::sim::SimulationMetrics second_metrics{};

    for (int step = 0; step < 20; ++step) {
        first.update(1.0F / 120.0F, &first_metrics);
        second.update(1.0F / 120.0F, &second_metrics);
    }

    REQUIRE(first.positions().size() == second.positions().size());
    CHECK(first.positions()[3].x == second.positions()[3].x);
    CHECK(first.positions()[3].y == second.positions()[3].y);
    CHECK(first.velocities()[7].z == second.velocities()[7].z);
    CHECK(first_metrics.mean_altitude == second_metrics.mean_altitude);
    CHECK(first_metrics.stall_count == second_metrics.stall_count);
}

TEST_CASE("BirdFlight scenario produces valid constrained parameters", "[scenario][birdflight]")
{
    const auto definition = flock3d::sim::build_scenario(flock3d::sim::ScenarioType::BirdFlight);
    const auto& parameters = definition.simulation_parameters;

    CHECK(parameters.model == flock3d::sim::SimulationModel::BirdFlight);
    CHECK(parameters.gravity > 0.0F);
    CHECK(parameters.lift_strength > 0.0F);
    CHECK(parameters.altitude_target > 0.0F);
    CHECK(parameters.altitude_band > 0.0F);
    CHECK(parameters.altitude_correction_strength > 0.0F);
    CHECK(parameters.min_speed > 0.0F);
    CHECK(parameters.max_climb_rate > 0.0F);
    CHECK(parameters.max_turn_rate > 0.0F);
    CHECK(parameters.field_of_view_degrees > 0.0F);
    CHECK(parameters.field_of_view_degrees <= 360.0F);
}

TEST_CASE("FishSchool drag reduces speed over time", "[simulation][fishschool]")
{
    auto parameters = steering_test_parameters();
    parameters.model = flock3d::sim::SimulationModel::FishSchool;
    parameters.drag_coefficient = 0.5F;
    parameters.max_turn_rate = 0.0F;
    parameters.max_force = 10.0F;

    flock3d::sim::BoidSimulation simulation{parameters};
    simulation.add_boid(Vector3{0.0F, 0.0F, 0.0F}, Vector3{6.0F, 0.0F, 0.0F});

    const auto initial_speed = flock3d::math::length(simulation.velocities().front());
    simulation.update(1.0F);

    REQUIRE(simulation.velocities().size() == 1);
    CHECK(flock3d::math::length(simulation.velocities().front()) < initial_speed);
}

TEST_CASE("FishSchool depth correction moves agents toward target depth", "[simulation][fishschool]")
{
    auto parameters = steering_test_parameters();
    parameters.model = flock3d::sim::SimulationModel::FishSchool;
    parameters.drag_coefficient = 0.0F;
    parameters.max_turn_rate = 0.0F;
    parameters.target_depth = -10.0F;
    parameters.depth_band = 0.0F;
    parameters.depth_correction_strength = 1.0F;
    parameters.max_force = 100.0F;
    parameters.max_speed = 100.0F;

    flock3d::sim::BoidSimulation simulation{parameters};
    simulation.add_boid(Vector3{0.0F, 0.0F, 0.0F}, Vector3{});

    simulation.update(0.1F);

    REQUIRE(simulation.positions().size() == 1);
    CHECK(simulation.positions().front().y < 0.0F);
}

TEST_CASE("FishSchool current changes average velocity direction", "[simulation][fishschool]")
{
    auto parameters = steering_test_parameters();
    parameters.model = flock3d::sim::SimulationModel::FishSchool;
    parameters.drag_coefficient = 0.0F;
    parameters.max_turn_rate = 0.0F;
    parameters.current_strength = 4.0F;
    parameters.current_direction = Vector3{0.0F, 0.0F, 1.0F};
    parameters.max_force = 100.0F;
    parameters.max_speed = 100.0F;

    flock3d::sim::BoidSimulation simulation{parameters};
    simulation.add_boid(Vector3{}, Vector3{});
    simulation.add_boid(Vector3{10.0F, 0.0F, 0.0F}, Vector3{});

    simulation.update(0.5F);

    float average_z = 0.0F;
    for (const auto velocity : simulation.velocities()) {
        average_z += velocity.z;
    }
    average_z /= static_cast<float>(simulation.velocities().size());
    CHECK(average_z > 0.0F);
}

TEST_CASE("FishSchool remains deterministic for the same seed", "[simulation][fishschool][seed]")
{
    auto parameters = flock3d::sim::build_scenario(flock3d::sim::ScenarioType::FishSchool).simulation_parameters;
    parameters.boid_count = 32U;
    parameters.random_seed = 4040U;
    parameters.current_strength = 1.25F;

    flock3d::sim::BoidSimulation first{parameters};
    flock3d::sim::BoidSimulation second{parameters};
    flock3d::sim::SimulationMetrics first_metrics{};
    flock3d::sim::SimulationMetrics second_metrics{};

    for (int step = 0; step < 20; ++step) {
        first.update(1.0F / 120.0F, &first_metrics);
        second.update(1.0F / 120.0F, &second_metrics);
    }

    REQUIRE(first.positions().size() == second.positions().size());
    CHECK(first.positions()[3].x == second.positions()[3].x);
    CHECK(first.positions()[3].y == second.positions()[3].y);
    CHECK(first.velocities()[7].z == second.velocities()[7].z);
    CHECK(first_metrics.mean_depth == second_metrics.mean_depth);
    CHECK(first_metrics.depth_variance == second_metrics.depth_variance);
}

TEST_CASE("FishSchool scenario produces resistive-medium defaults", "[scenario][fishschool]")
{
    const auto definition = flock3d::sim::build_scenario(flock3d::sim::ScenarioType::FishSchool);
    const auto& parameters = definition.simulation_parameters;

    CHECK(parameters.model == flock3d::sim::SimulationModel::FishSchool);
    CHECK(parameters.drag_coefficient > 0.0F);
    CHECK(parameters.depth_correction_strength > 0.0F);
    CHECK(parameters.depth_band > 0.0F);
    CHECK(parameters.max_turn_rate > 0.0F);
    CHECK(parameters.max_speed < flock3d::sim::SimulationParameters{}.max_speed);
}

TEST_CASE("Cell aggregate social mode uses exact separation without self", "[simulation][neighbors][aggregates]")
{
    auto parameters = steering_test_parameters();
    parameters.neighbor_mode = flock3d::sim::NeighborMode::CellAggregateSocial;
    parameters.separation_weight = 1.0F;
    parameters.alignment_weight = 0.0F;
    parameters.cohesion_weight = 0.0F;
    parameters.separation_radius = 2.0F;
    flock3d::sim::sync_spatial_cell_size_to_query_radius(parameters);

    flock3d::sim::BoidSimulation simulation{parameters};
    simulation.add_boid(Vector3{0.0F, 0.0F, 0.0F}, Vector3{});

    flock3d::sim::SimulationMetrics metrics{};
    simulation.update(0.0F, &metrics);

    CHECK(metrics.exact_separation_neighbors_mean == Catch::Approx(0.0));
    CHECK(metrics.neighbor_total == 0U);
}

TEST_CASE("Cell aggregate social mode remains deterministic", "[simulation][neighbors][aggregates]")
{
    auto parameters = steering_test_parameters();
    parameters.neighbor_mode = flock3d::sim::NeighborMode::CellAggregateSocial;
    parameters.boid_count = 12U;
    parameters.random_seed = 42U;
    parameters.alignment_weight = 1.0F;
    parameters.cohesion_weight = 1.0F;
    parameters.separation_weight = 1.0F;

    flock3d::sim::BoidSimulation lhs{parameters};
    flock3d::sim::BoidSimulation rhs{parameters};
    flock3d::sim::SimulationMetrics lhs_metrics{};
    flock3d::sim::SimulationMetrics rhs_metrics{};

    run_steps(lhs, 8, lhs_metrics);
    run_steps(rhs, 8, rhs_metrics);

    CHECK(trajectory_distance(lhs, rhs) == Catch::Approx(0.0));
    CHECK(lhs_metrics.aggregate_cells_used_mean == Catch::Approx(rhs_metrics.aggregate_cells_used_mean));
    CHECK(lhs_metrics.social_weight_sum_mean == Catch::Approx(rhs_metrics.social_weight_sum_mean));
}

TEST_CASE("All neighbor modes remain available", "[simulation][neighbors]")
{
    CHECK(static_cast<int>(flock3d::sim::NeighborMode::FixedRadiusUncapped) == 0);
    CHECK(static_cast<int>(flock3d::sim::NeighborMode::FixedRadiusClosestK) == 1);
    CHECK(static_cast<int>(flock3d::sim::NeighborMode::AdaptiveRadiusClosestK) == 2);
    CHECK(static_cast<int>(flock3d::sim::NeighborMode::CellAggregateSocial) == 3);
}

TEST_CASE("BoidSimulation threaded updates preserve finite state and boid count", "[simulation][threads]")
{
    auto parameters = flock3d::sim::build_scenario(flock3d::sim::ScenarioType::ClassicBoids).simulation_parameters;
    parameters.boid_count = 96U;
    parameters.random_seed = 42U;
    parameters.thread_count = 4U;

    flock3d::sim::BoidSimulation simulation{parameters};
    for (int step = 0; step < 8; ++step) {
        simulation.update(1.0F / 120.0F, nullptr);
    }

    REQUIRE(simulation.positions().size() == 96U);
    REQUIRE(simulation.velocities().size() == 96U);
    for (std::size_t i = 0; i < simulation.positions().size(); ++i) {
        CHECK(std::isfinite(simulation.positions()[i].x));
        CHECK(std::isfinite(simulation.positions()[i].y));
        CHECK(std::isfinite(simulation.positions()[i].z));
        CHECK(std::isfinite(simulation.velocities()[i].x));
        CHECK(std::isfinite(simulation.velocities()[i].y));
        CHECK(std::isfinite(simulation.velocities()[i].z));
    }
}

TEST_CASE("BoidSimulation threaded updates match deterministic serial trajectory", "[simulation][threads][determinism]")
{
    auto parameters = flock3d::sim::build_scenario(flock3d::sim::ScenarioType::NoiseExperiment).simulation_parameters;
    parameters.boid_count = 80U;
    parameters.random_seed = 99U;
    parameters.thread_count = 1U;
    flock3d::sim::BoidSimulation serial{parameters};

    parameters.thread_count = 4U;
    flock3d::sim::BoidSimulation threaded{parameters};

    for (int step = 0; step < 6; ++step) {
        serial.update(1.0F / 120.0F, nullptr);
        threaded.update(1.0F / 120.0F, nullptr);
    }

    REQUIRE(serial.positions().size() == threaded.positions().size());
    CHECK(trajectory_distance(serial, threaded) == Catch::Approx(0.0).margin(0.0));
}

TEST_CASE("BoidSimulation thread count policy keeps serial and automatic paths distinct", "[simulation][threads]")
{
    auto parameters = flock3d::sim::build_scenario(flock3d::sim::ScenarioType::ClassicBoids).simulation_parameters;

    parameters.boid_count = 511U;
    parameters.thread_count = 0U;
    flock3d::sim::BoidSimulation small_auto{parameters};
    CHECK(small_auto.effective_thread_count() == 1U);

    parameters.boid_count = 512U;
    flock3d::sim::BoidSimulation medium_auto{parameters};
    CHECK(medium_auto.effective_thread_count() == 2U);

    parameters.boid_count = 1024U;
    flock3d::sim::BoidSimulation large_auto{parameters};
    CHECK(large_auto.effective_thread_count() == 4U);

    parameters.thread_count = 1U;
    flock3d::sim::BoidSimulation serial{parameters};
    CHECK(serial.effective_thread_count() == 1U);

    parameters.thread_count = 8U;
    flock3d::sim::BoidSimulation manual{parameters};
    CHECK(manual.effective_thread_count() == 8U);
}

TEST_CASE("BoidSimulation threaded updates preserve boids and finite state", "[simulation][threads]")
{
    auto parameters = flock3d::sim::build_scenario(flock3d::sim::ScenarioType::ClassicBoids).simulation_parameters;
    parameters.boid_count = 128U;
    parameters.thread_count = 4U;
    parameters.thread_chunk_size = 17U;

    flock3d::sim::BoidSimulation simulation{parameters};
    simulation.update(1.0F / 120.0F);

    REQUIRE(simulation.positions().size() == 128U);
    REQUIRE(simulation.velocities().size() == 128U);
    for (std::size_t i = 0; i < simulation.positions().size(); ++i) {
        const Vector3 position = simulation.positions()[i];
        const Vector3 velocity = simulation.velocities()[i];
        CHECK(std::isfinite(position.x));
        CHECK(std::isfinite(position.y));
        CHECK(std::isfinite(position.z));
        CHECK(std::isfinite(velocity.x));
        CHECK(std::isfinite(velocity.y));
        CHECK(std::isfinite(velocity.z));
    }
}

TEST_CASE("BoidSimulation threaded and serial updates are deterministic", "[simulation][threads]")
{
    auto serial_parameters = flock3d::sim::build_scenario(flock3d::sim::ScenarioType::ClassicBoids).simulation_parameters;
    serial_parameters.boid_count = 96U;
    serial_parameters.random_seed = 2026U;
    serial_parameters.thread_count = 1U;

    auto threaded_parameters = serial_parameters;
    threaded_parameters.thread_count = 4U;
    threaded_parameters.thread_chunk_size = 11U;

    flock3d::sim::BoidSimulation serial{serial_parameters};
    flock3d::sim::BoidSimulation threaded{threaded_parameters};

    for (int step = 0; step < 12; ++step) {
        serial.update(1.0F / 120.0F);
        threaded.update(1.0F / 120.0F);
    }

    REQUIRE(serial.positions().size() == threaded.positions().size());
    REQUIRE(serial.velocities().size() == threaded.velocities().size());
    CHECK(trajectory_distance(serial, threaded) == Catch::Approx(0.0).margin(0.000001));
}
