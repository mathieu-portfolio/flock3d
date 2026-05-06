#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <raylib.h>

#include <flock3d/math/Vec3.hpp>
#include <flock3d/sim/BoidSimulation.hpp>
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
    CHECK(metrics.average_neighbors_per_boid == 2.0F / 3.0F);
    CHECK(metrics.spatial_hash_cell_count == 2);
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
    }
}
