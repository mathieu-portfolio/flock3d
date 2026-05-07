#include <flock3d/sim/Scenario.hpp>

#include <array>
#include <cctype>
#include <cstddef>

namespace flock3d::sim {
namespace {


constexpr std::array<std::string_view, 7> scenario_cli_names{{
    "ClassicBoids",
    "BirdFlight",
    "FishSchool",
    "PredatorPrey",
    "ObstacleAvoidance",
    "Leadership",
    "NoiseExperiment",
}};

[[nodiscard]] constexpr bool is_name_separator(char character) noexcept
{
    return character == ' ' || character == '-' || character == '_';
}

[[nodiscard]] char normalized_character(char character) noexcept
{
    return static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
}

[[nodiscard]] bool normalized_equal(std::string_view lhs, std::string_view rhs) noexcept
{
    std::size_t lhs_index = 0;
    std::size_t rhs_index = 0;
    while (true) {
        while (lhs_index < lhs.size() && is_name_separator(lhs[lhs_index])) {
            ++lhs_index;
        }
        while (rhs_index < rhs.size() && is_name_separator(rhs[rhs_index])) {
            ++rhs_index;
        }
        if (lhs_index == lhs.size() || rhs_index == rhs.size()) {
            return lhs_index == lhs.size() && rhs_index == rhs.size();
        }
        if (normalized_character(lhs[lhs_index]) != normalized_character(rhs[rhs_index])) {
            return false;
        }
        ++lhs_index;
        ++rhs_index;
    }
}

[[nodiscard]] constexpr std::size_t scenario_index(ScenarioType type) noexcept
{
    for (std::size_t i = 0; i < scenario_types.size(); ++i) {
        if (scenario_types[i] == type) {
            return i;
        }
    }
    return 0;
}

[[nodiscard]] ScenarioDefinition classic_defaults() noexcept
{
    ScenarioDefinition definition{};
    definition.type = ScenarioType::ClassicBoids;
    definition.display_name = "Classic Boids";
    definition.description = "Baseline Reynolds-style flocking with separation, alignment, and cohesion.";
    definition.simulation_parameters = SimulationParameters{};
    definition.environment = EnvironmentSettings{definition.simulation_parameters.world_half_extent, true};
    definition.constraints = ConstraintSettings{
        definition.simulation_parameters.max_speed,
        definition.simulation_parameters.max_force,
        definition.simulation_parameters.gravity,
        definition.simulation_parameters.lift_strength,
        definition.simulation_parameters.altitude_target,
        definition.simulation_parameters.altitude_band,
        definition.simulation_parameters.altitude_correction_strength,
        definition.simulation_parameters.min_speed,
        definition.simulation_parameters.max_climb_rate,
        definition.simulation_parameters.max_turn_rate,
        definition.simulation_parameters.field_of_view_degrees,
    };
    definition.behavior = BehaviorSettings{
        definition.simulation_parameters.separation_weight,
        definition.simulation_parameters.alignment_weight,
        definition.simulation_parameters.cohesion_weight,
        definition.simulation_parameters.neighbor_radius,
        definition.simulation_parameters.separation_radius,
    };
    definition.metrics = MetricsSettings{};
    return definition;
}

void sync_plain_settings_to_parameters(ScenarioDefinition& definition) noexcept
{
    definition.simulation_parameters.world_half_extent = definition.environment.world_half_extent;
    definition.simulation_parameters.max_speed = definition.constraints.max_speed;
    definition.simulation_parameters.max_force = definition.constraints.max_force;
    definition.simulation_parameters.gravity = definition.constraints.gravity;
    definition.simulation_parameters.lift_strength = definition.constraints.lift_strength;
    definition.simulation_parameters.altitude_target = definition.constraints.altitude_target;
    definition.simulation_parameters.altitude_band = definition.constraints.altitude_band;
    definition.simulation_parameters.altitude_correction_strength = definition.constraints.altitude_correction_strength;
    definition.simulation_parameters.min_speed = definition.constraints.min_speed;
    definition.simulation_parameters.max_climb_rate = definition.constraints.max_climb_rate;
    definition.simulation_parameters.max_turn_rate = definition.constraints.max_turn_rate;
    definition.simulation_parameters.field_of_view_degrees = definition.constraints.field_of_view_degrees;
    definition.simulation_parameters.drag_coefficient = definition.constraints.drag_coefficient;
    definition.simulation_parameters.buoyancy_strength = definition.constraints.buoyancy_strength;
    definition.simulation_parameters.target_depth = definition.constraints.target_depth;
    definition.simulation_parameters.depth_band = definition.constraints.depth_band;
    definition.simulation_parameters.depth_correction_strength = definition.constraints.depth_correction_strength;
    definition.simulation_parameters.current_strength = definition.constraints.current_strength;
    definition.simulation_parameters.current_direction = definition.constraints.current_direction;
    definition.simulation_parameters.separation_weight = definition.behavior.separation_weight;
    definition.simulation_parameters.alignment_weight = definition.behavior.alignment_weight;
    definition.simulation_parameters.cohesion_weight = definition.behavior.cohesion_weight;
    definition.simulation_parameters.neighbor_radius = definition.behavior.neighbor_radius;
    definition.simulation_parameters.separation_radius = definition.behavior.separation_radius;
    sync_spatial_cell_size_to_query_radius(definition.simulation_parameters);
}

} // namespace

ScenarioDefinition build_scenario(ScenarioType type) noexcept
{
    auto definition = classic_defaults();
    definition.type = type;

    switch (type) {
    case ScenarioType::ClassicBoids:
        break;
    case ScenarioType::BirdFlight:
        definition.display_name = "Bird Flight";
        definition.description = "Flocking with gravity, lift, altitude hold, limited climb rate, turn rate, and forward field of view.";
        definition.simulation_parameters.model = SimulationModel::BirdFlight;
        definition.simulation_parameters.random_seed = 2401U;
        definition.simulation_parameters.min_initial_speed = 5.0F;
        definition.simulation_parameters.max_initial_speed = 9.0F;
        definition.constraints.max_speed = 12.0F;
        definition.constraints.max_force = 16.0F;
        definition.constraints.gravity = 9.8F;
        definition.constraints.lift_strength = 9.8F;
        definition.constraints.altitude_target = 12.0F;
        definition.constraints.altitude_band = 4.0F;
        definition.constraints.altitude_correction_strength = 1.8F;
        definition.constraints.min_speed = 4.0F;
        definition.constraints.max_climb_rate = 6.0F;
        definition.constraints.max_turn_rate = 120.0F;
        definition.constraints.field_of_view_degrees = 220.0F;
        definition.behavior.alignment_weight = 1.25F;
        break;
    case ScenarioType::FishSchool:
        definition.display_name = "Fish School";
        definition.description = "Resistive-medium schooling with drag, depth preference, smooth turning, and optional current.";
        definition.simulation_parameters.model = SimulationModel::FishSchool;
        definition.simulation_parameters.random_seed = 3109U;
        definition.simulation_parameters.min_initial_speed = 1.5F;
        definition.simulation_parameters.max_initial_speed = 4.5F;
        definition.environment.world_half_extent = 35.0F;
        definition.constraints.max_speed = 6.0F;
        definition.constraints.max_force = 7.0F;
        definition.constraints.max_turn_rate = 80.0F;
        definition.constraints.drag_coefficient = 0.35F;
        definition.constraints.buoyancy_strength = 0.15F;
        definition.constraints.target_depth = -8.0F;
        definition.constraints.depth_band = 5.0F;
        definition.constraints.depth_correction_strength = 0.8F;
        definition.constraints.current_strength = 0.0F;
        definition.constraints.current_direction = Vector3{1.0F, 0.0F, 0.0F};
        definition.behavior.alignment_weight = 1.15F;
        definition.behavior.cohesion_weight = 1.35F;
        definition.behavior.neighbor_radius = 5.0F;
        definition.behavior.separation_radius = 1.5F;
        break;
    case ScenarioType::PredatorPrey:
        definition.display_name = "Predator-Prey";
        definition.description = "Placeholder for future predator/prey roles; currently reuses classic boids behavior.";
        definition.simulation_parameters.random_seed = 4513U;
        definition.simulation_parameters.boid_count = 640U;
        definition.behavior.separation_weight = 1.8F;
        break;
    case ScenarioType::ObstacleAvoidance:
        definition.display_name = "Obstacle Avoidance";
        definition.description = "Placeholder for future obstacle fields; currently reuses classic boids behavior.";
        definition.simulation_parameters.random_seed = 5209U;
        definition.environment.world_half_extent = 45.0F;
        break;
    case ScenarioType::Leadership:
        definition.display_name = "Leadership";
        definition.description = "Placeholder for future informed leaders; currently reuses classic boids behavior.";
        definition.simulation_parameters.random_seed = 6823U;
        definition.behavior.alignment_weight = 1.4F;
        definition.behavior.cohesion_weight = 0.85F;
        break;
    case ScenarioType::NoiseExperiment:
        definition.display_name = "Noise Experiment";
        definition.description = "Placeholder for future controlled noise sweeps; currently reuses classic boids behavior.";
        definition.simulation_parameters.random_seed = 7901U;
        definition.behavior.alignment_weight = 0.75F;
        definition.behavior.neighbor_radius = 5.0F;
        break;
    }

    sync_plain_settings_to_parameters(definition);
    return definition;
}


std::string_view scenario_display_name(ScenarioType type) noexcept
{
    return build_scenario(type).display_name;
}

std::string_view scenario_cli_name(ScenarioType type) noexcept
{
    return scenario_cli_names[scenario_index(type)];
}

std::optional<ScenarioType> scenario_type_from_name(std::string_view name) noexcept
{
    for (std::size_t i = 0; i < scenario_types.size(); ++i) {
        if (normalized_equal(name, scenario_cli_names[i]) || normalized_equal(name, build_scenario(scenario_types[i]).display_name)) {
            return scenario_types[i];
        }
    }
    return std::nullopt;
}

ScenarioDefinition BuildScenario(ScenarioType type) noexcept
{
    return build_scenario(type);
}

ScenarioDefinition ScenarioFactory(ScenarioType type) noexcept
{
    return build_scenario(type);
}

ScenarioType next_scenario_type(ScenarioType type) noexcept
{
    const auto next = (scenario_index(type) + 1U) % scenario_types.size();
    return scenario_types[next];
}

ScenarioType previous_scenario_type(ScenarioType type) noexcept
{
    const auto index = scenario_index(type);
    const auto previous = (index + scenario_types.size() - 1U) % scenario_types.size();
    return scenario_types[previous];
}

} // namespace flock3d::sim
