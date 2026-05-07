#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <flock3d/experiment/MetricsExport.hpp>
#include <flock3d/sim/Scenario.hpp>
#include <flock3d/sim/SimulationParameters.hpp>

namespace flock3d::experiment {

struct SweepDefinition {
    std::string parameter{};
    double start{};
    double end{};
    double step{};
};

struct ExperimentPreset {
    std::string_view name{};
    std::string_view description{};
    sim::ScenarioType scenario{sim::ScenarioType::BirdFlight};
    sim::SimulationParameters parameters{};
};

struct ExperimentConfig {
    sim::ScenarioType scenario{sim::ScenarioType::ClassicBoids};
    std::uint32_t seed{123U};
    std::uint32_t boids{512U};
    double duration_seconds{30.0};
    double fixed_dt{1.0 / 120.0};
    double sample_rate_hz{5.0};
    ExportMode export_mode{ExportMode::SampledTimeSeries};
    std::filesystem::path output_path{"outputs/experiment.csv"};
    std::optional<SweepDefinition> sweep{};
    std::optional<std::string> preset{};
    std::optional<sim::SimulationParameters> parameter_defaults{};
};

struct ExperimentRunResult {
    std::size_t rows_written{};
    std::filesystem::path output_path{};
};

[[nodiscard]] std::optional<ExperimentPreset> experiment_preset(std::string_view name);
[[nodiscard]] std::span<const std::string_view> experiment_preset_names() noexcept;
[[nodiscard]] bool apply_experiment_preset(ExperimentConfig& config, std::string_view name);
[[nodiscard]] sim::SimulationParameters experiment_parameters(const ExperimentConfig& config);
[[nodiscard]] std::optional<SweepDefinition> parse_sweep(std::string_view text);
[[nodiscard]] std::vector<double> sweep_values(const SweepDefinition& sweep);
[[nodiscard]] bool apply_sweep_value(sim::SimulationParameters& parameters, std::string_view parameter, double value) noexcept;
[[nodiscard]] std::optional<ExperimentConfig> parse_cli(int argc, char** argv, std::string& error);
[[nodiscard]] ExperimentRunResult run_experiment(const ExperimentConfig& config);

} // namespace flock3d::experiment
