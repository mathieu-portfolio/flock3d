#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

#include <flock3d/sim/SimulationParameters.hpp>

namespace flock3d::app {

struct CameraSettings {
    float base_move_speed{32.0F};
    float fast_move_multiplier{4.0F};
    float mouse_sensitivity{0.0035F};
    float min_move_speed{4.0F};
    float max_move_speed{240.0F};
};

[[nodiscard]] constexpr float clamped_camera_speed(float speed, const CameraSettings& settings) noexcept
{
    return std::clamp(speed, settings.min_move_speed, settings.max_move_speed);
}

enum class TunableParameter : std::uint8_t {
    separation_weight = 0,
    alignment_weight,
    cohesion_weight,
    perception_radius,
    separation_radius,
    max_speed,
    max_force,
    boid_scale,
};

struct TunableParameterDescriptor {
    TunableParameter parameter;
    std::string_view label;
    float sim::SimulationParameters::* value;
    float step;
    float minimum;
    float maximum;
};

inline constexpr std::array<TunableParameterDescriptor, 8> tunable_parameters{{
    {TunableParameter::separation_weight, "separation weight", &sim::SimulationParameters::separation_weight, 0.10F, 0.0F, 10.0F},
    {TunableParameter::alignment_weight, "alignment weight", &sim::SimulationParameters::alignment_weight, 0.10F, 0.0F, 10.0F},
    {TunableParameter::cohesion_weight, "cohesion weight", &sim::SimulationParameters::cohesion_weight, 0.10F, 0.0F, 10.0F},
    {TunableParameter::perception_radius, "perception radius", &sim::SimulationParameters::neighbor_radius, 0.25F, 0.5F, 80.0F},
    {TunableParameter::separation_radius, "separation radius", &sim::SimulationParameters::separation_radius, 0.25F, 0.1F, 40.0F},
    {TunableParameter::max_speed, "max speed", &sim::SimulationParameters::max_speed, 0.50F, 0.5F, 120.0F},
    {TunableParameter::max_force, "max force", &sim::SimulationParameters::max_force, 0.50F, 0.5F, 120.0F},
    {TunableParameter::boid_scale, "boid scale", &sim::SimulationParameters::boid_scale, 0.05F, 0.05F, 5.0F},
}};

[[nodiscard]] constexpr const TunableParameterDescriptor& descriptor_for(TunableParameter parameter) noexcept
{
    return tunable_parameters[static_cast<std::size_t>(parameter)];
}

[[nodiscard]] constexpr TunableParameter parameter_from_index(std::size_t zero_based_index) noexcept
{
    return tunable_parameters[std::clamp<std::size_t>(zero_based_index, 0, tunable_parameters.size() - 1)].parameter;
}

[[nodiscard]] constexpr std::size_t parameter_index(TunableParameter parameter) noexcept
{
    return static_cast<std::size_t>(parameter);
}

[[nodiscard]] constexpr TunableParameter offset_parameter(TunableParameter parameter, int offset) noexcept
{
    const auto count = static_cast<int>(tunable_parameters.size());
    const auto current = static_cast<int>(parameter_index(parameter));
    const auto wrapped = (current + offset % count + count) % count;
    return parameter_from_index(static_cast<std::size_t>(wrapped));
}

[[nodiscard]] inline float parameter_value(const sim::SimulationParameters& parameters, TunableParameter parameter) noexcept
{
    const auto& descriptor = descriptor_for(parameter);
    return parameters.*(descriptor.value);
}

inline void adjust_parameter(sim::SimulationParameters& parameters, TunableParameter parameter, int direction) noexcept
{
    const auto& descriptor = descriptor_for(parameter);
    auto& value = parameters.*(descriptor.value);
    value = std::clamp(value + static_cast<float>(direction) * descriptor.step, descriptor.minimum, descriptor.maximum);
}

} // namespace flock3d::app
