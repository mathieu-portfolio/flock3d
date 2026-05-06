#include <catch2/catch_test_macros.hpp>

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

TEST_CASE("Tunable parameter index maps number keys to selected parameter", "[controls]")
{
    CHECK(flock3d::app::parameter_from_index(0) == flock3d::app::TunableParameter::separation_weight);
    CHECK(flock3d::app::parameter_from_index(7) == flock3d::app::TunableParameter::boid_scale);
    CHECK(flock3d::app::parameter_from_index(99) == flock3d::app::TunableParameter::boid_scale);
}

TEST_CASE("Tunable parameter keyboard cycling wraps around", "[controls]")
{
    CHECK(flock3d::app::offset_parameter(flock3d::app::TunableParameter::separation_weight, 1)
          == flock3d::app::TunableParameter::alignment_weight);
    CHECK(flock3d::app::offset_parameter(flock3d::app::TunableParameter::separation_weight, -1)
          == flock3d::app::TunableParameter::boid_scale);
    CHECK(flock3d::app::offset_parameter(flock3d::app::TunableParameter::boid_scale, 1)
          == flock3d::app::TunableParameter::separation_weight);
}
