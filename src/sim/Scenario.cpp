#include <flock3d/sim/Scenario.hpp>

#include <cstddef>

namespace flock3d::sim {
namespace {

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
    definition.simulation_parameters.separation_weight = definition.behavior.separation_weight;
    definition.simulation_parameters.alignment_weight = definition.behavior.alignment_weight;
    definition.simulation_parameters.cohesion_weight = definition.behavior.cohesion_weight;
    definition.simulation_parameters.neighbor_radius = definition.behavior.neighbor_radius;
    definition.simulation_parameters.separation_radius = definition.behavior.separation_radius;
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
        definition.description = "Placeholder for future avian flight constraints; currently reuses classic boids behavior.";
        definition.simulation_parameters.random_seed = 2401U;
        definition.constraints.max_speed = 12.0F;
        definition.behavior.alignment_weight = 1.25F;
        break;
    case ScenarioType::FishSchool:
        definition.display_name = "Fish School";
        definition.description = "Placeholder for future aquatic schooling and drag; currently reuses classic boids behavior.";
        definition.simulation_parameters.random_seed = 3109U;
        definition.environment.world_half_extent = 35.0F;
        definition.behavior.cohesion_weight = 1.25F;
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
