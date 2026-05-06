#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string_view>

#include <flock3d/sim/SimulationParameters.hpp>

namespace flock3d::sim {

enum class ScenarioType : std::uint8_t {
    ClassicBoids = 0,
    BirdFlight,
    FishSchool,
    PredatorPrey,
    ObstacleAvoidance,
    Leadership,
    NoiseExperiment,
};

struct EnvironmentSettings {
    float world_half_extent{40.0F};
    bool wrap_world{true};
};

struct ConstraintSettings {
    float max_speed{10.0F};
    float max_force{12.0F};
    float gravity{0.0F};
    float lift_strength{0.0F};
    float altitude_target{0.0F};
    float altitude_band{0.0F};
    float altitude_correction_strength{0.0F};
    float min_speed{0.0F};
    float max_climb_rate{0.0F};
    float max_turn_rate{0.0F};
    float field_of_view_degrees{360.0F};
};

struct BehaviorSettings {
    float separation_weight{1.5F};
    float alignment_weight{1.0F};
    float cohesion_weight{1.0F};
    float neighbor_radius{4.0F};
    float separation_radius{2.0F};
};

struct MetricsSettings {
    bool collective_metrics_enabled{true};
    bool nearest_neighbor_distance_enabled{true};
    bool cluster_count_enabled{false};
};

struct ScenarioDefinition {
    ScenarioType type{ScenarioType::ClassicBoids};
    std::string_view display_name{"Classic Boids"};
    std::string_view description{"Reynolds-style separation, alignment, and cohesion in a wrapped 3D world."};
    SimulationParameters simulation_parameters{};
    EnvironmentSettings environment{};
    ConstraintSettings constraints{};
    BehaviorSettings behavior{};
    MetricsSettings metrics{};
};

inline constexpr std::array<ScenarioType, 7> scenario_types{{
    ScenarioType::ClassicBoids,
    ScenarioType::BirdFlight,
    ScenarioType::FishSchool,
    ScenarioType::PredatorPrey,
    ScenarioType::ObstacleAvoidance,
    ScenarioType::Leadership,
    ScenarioType::NoiseExperiment,
}};

[[nodiscard]] ScenarioDefinition build_scenario(ScenarioType type) noexcept;
[[nodiscard]] std::string_view scenario_display_name(ScenarioType type) noexcept;
[[nodiscard]] std::string_view scenario_cli_name(ScenarioType type) noexcept;
[[nodiscard]] std::optional<ScenarioType> scenario_type_from_name(std::string_view name) noexcept;
[[nodiscard]] ScenarioDefinition BuildScenario(ScenarioType type) noexcept;
[[nodiscard]] ScenarioDefinition ScenarioFactory(ScenarioType type) noexcept;
[[nodiscard]] ScenarioType next_scenario_type(ScenarioType type) noexcept;
[[nodiscard]] ScenarioType previous_scenario_type(ScenarioType type) noexcept;

} // namespace flock3d::sim
