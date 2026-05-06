#include <catch2/catch_test_macros.hpp>
#include <raylib.h>

#include <flock3d/sim/BoidSimulation.hpp>
#include <flock3d/sim/SimulationParameters.hpp>

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
