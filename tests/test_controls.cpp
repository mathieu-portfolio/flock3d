#include <catch2/catch_test_macros.hpp>

#include <flock3d/app/ControlCommands.hpp>
#include <flock3d/app/ControlHelpers.hpp>
#include <flock3d/app/OverlayLayout.hpp>
#include <flock3d/sim/SimulationParameters.hpp>

TEST_CASE("Camera speed clamps to configured movement range", "[controls]")
{
    flock3d::app::CameraSettings settings{};
    settings.min_move_speed = 4.0F;
    settings.max_move_speed = 240.0F;

    CHECK(flock3d::app::clamped_camera_speed(1.0F, settings) == 4.0F);
    CHECK(flock3d::app::clamped_camera_speed(64.0F, settings) == 64.0F);
    CHECK(flock3d::app::clamped_camera_speed(500.0F, settings) == 240.0F);
}

TEST_CASE("Tunable parameter adjustment applies step and clamps", "[controls]")
{
    flock3d::sim::SimulationParameters parameters{};
    parameters.separation_weight = 9.95F;
    parameters.neighbor_radius = 0.55F;

    flock3d::app::adjust_parameter(parameters, flock3d::app::TunableParameter::separation_weight, 1);
    CHECK(parameters.separation_weight == 10.0F);

    flock3d::app::adjust_parameter(parameters, flock3d::app::TunableParameter::perception_radius, -1);
    CHECK(parameters.neighbor_radius == 0.5F);
}

TEST_CASE("Radius tuning keeps spatial cells aligned with query radius", "[controls]")
{
    flock3d::sim::SimulationParameters parameters{};
    parameters.neighbor_radius = 4.0F;
    parameters.separation_radius = 2.0F;
    parameters.spatial_cell_size = 4.0F;

    flock3d::app::adjust_parameter(parameters, flock3d::app::TunableParameter::perception_radius, 1);
    CHECK(parameters.neighbor_radius == 4.25F);
    CHECK(parameters.spatial_cell_size == 4.25F);

    parameters.separation_radius = 4.5F;
    flock3d::app::adjust_parameter(parameters, flock3d::app::TunableParameter::separation_radius, 1);
    CHECK(parameters.separation_radius == 4.75F);
    CHECK(parameters.spatial_cell_size == 4.75F);
}

TEST_CASE("Control command queue drains commands in input order", "[controls]")
{
    flock3d::app::ControlCommandQueue commands{};
    commands.enqueue(flock3d::app::ControlCommand::toggle_pause());
    commands.enqueue(flock3d::app::ControlCommand::adjust_boid_count(128));
    commands.enqueue(flock3d::app::ControlCommand::adjust_selected_parameter(-1));

    const auto drained = commands.drain();

    REQUIRE(drained.size() == 3);
    CHECK(drained[0].type == flock3d::app::ControlCommandType::TogglePause);
    CHECK(drained[1].type == flock3d::app::ControlCommandType::AdjustBoidCount);
    CHECK(drained[1].amount == 128);
    CHECK(drained[2].type == flock3d::app::ControlCommandType::AdjustSelectedParameter);
    CHECK(drained[2].amount == -1);
    CHECK(commands.empty());
}

TEST_CASE("Control command queue supports frame boundary batching", "[controls]")
{
    flock3d::app::ControlCommandQueue commands{};
    commands.enqueue(flock3d::app::ControlCommand::select_parameter(flock3d::app::TunableParameter::max_speed));

    const auto first_frame = commands.drain();
    const auto second_frame = commands.drain();

    REQUIRE(first_frame.size() == 1);
    CHECK(first_frame.front().type == flock3d::app::ControlCommandType::SelectParameter);
    CHECK(first_frame.front().parameter == flock3d::app::TunableParameter::max_speed);
    CHECK(second_frame.empty());
}
