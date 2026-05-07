#include <flock3d/experiment/ExperimentRunner.hpp>

#include <array>
#include <charconv>
#include <cmath>
#include <iostream>
#include <sstream>

#include <flock3d/sim/BoidSimulation.hpp>

namespace flock3d::experiment {
namespace {

[[nodiscard]] std::optional<double> parse_double(std::string_view text) noexcept
{
    double value{};
    const auto* begin = text.data();
    const auto* end = text.data() + text.size();
    const auto [ptr, ec] = std::from_chars(begin, end, value);
    if (ec != std::errc{} || ptr != end) {
        return std::nullopt;
    }
    return value;
}

[[nodiscard]] std::optional<std::uint32_t> parse_u32(std::string_view text) noexcept
{
    std::uint32_t value{};
    const auto* begin = text.data();
    const auto* end = text.data() + text.size();
    const auto [ptr, ec] = std::from_chars(begin, end, value);
    if (ec != std::errc{} || ptr != end) {
        return std::nullopt;
    }
    return value;
}


[[nodiscard]] sim::SimulationParameters bird_preset_parameters(
    float lift_strength,
    float gravity,
    float field_of_view_degrees,
    float max_turn_rate)
{
    auto parameters = sim::build_scenario(sim::ScenarioType::BirdFlight).simulation_parameters;
    parameters.lift_strength = lift_strength;
    parameters.gravity = gravity;
    parameters.field_of_view_degrees = field_of_view_degrees;
    parameters.max_turn_rate = max_turn_rate;
    return parameters;
}

[[nodiscard]] sim::SimulationParameters fish_preset_parameters(
    float drag_coefficient,
    float current_strength,
    float neighbor_radius,
    float field_of_view_degrees)
{
    auto parameters = sim::build_scenario(sim::ScenarioType::FishSchool).simulation_parameters;
    parameters.drag_coefficient = drag_coefficient;
    parameters.current_strength = current_strength;
    parameters.neighbor_radius = neighbor_radius;
    parameters.field_of_view_degrees = field_of_view_degrees;
    sim::sync_spatial_cell_size_to_query_radius(parameters);
    return parameters;
}

[[nodiscard]] constexpr std::array<std::string_view, 9> preset_name_storage() noexcept
{
    return {
        "bird_baseline",
        "bird_low_lift",
        "bird_high_gravity",
        "bird_narrow_fov",
        "bird_low_turn_rate",
        "fish_baseline",
        "fish_high_drag",
        "fish_strong_current",
        "fish_low_visibility",
    };
}

[[nodiscard]] std::string value_string(double value)
{
    std::ostringstream stream;
    stream << value;
    return stream.str();
}

void run_single_value(
    const ExperimentConfig& config,
    CsvMetricsWriter& writer,
    std::optional<std::pair<std::string, double>> sweep,
    std::size_t& rows_written)
{
    auto parameters = experiment_parameters(config);
    parameters.random_seed = config.seed;
    parameters.boid_count = config.boids;
    if (sweep.has_value() && !apply_sweep_value(parameters, sweep->first, sweep->second)) {
        return;
    }

    sim::BoidSimulation simulation{parameters};
    sim::SimulationMetrics metrics{};
    SampleScheduler scheduler{config.sample_rate_hz};
    SummaryAggregator summary{};
    const auto timestamp = timestamp_for_filename();
    const auto git_commit = current_git_commit();
    const auto sweep_parameter = sweep.has_value() ? std::string_view{sweep->first} : std::string_view{};
    const auto sweep_value_storage = sweep.has_value() ? value_string(sweep->second) : std::string{};
    const auto sweep_value = std::string_view{sweep_value_storage};

    double simulation_time = 0.0;
    while (simulation_time + 1.0e-12 < config.duration_seconds) {
        const auto dt = std::min(config.fixed_dt, config.duration_seconds - simulation_time);
        simulation.update(static_cast<float>(dt), &metrics);
        simulation_time += dt;

        if (!scheduler.should_sample(simulation_time)) {
            continue;
        }
        const auto sample_index = scheduler.consume_sample();
        if (config.export_mode == ExportMode::Summary) {
            summary.add_sample(metrics);
            continue;
        }
        if (config.export_mode == ExportMode::FullTrajectory) {
            continue;
        }
        writer.write_sample(
            SampleMetadata{
                sim::scenario_display_name(config.scenario),
                config.seed,
                timestamp,
                git_commit,
                config.export_mode,
                config.sample_rate_hz,
                sample_index,
                simulation_time,
                simulation.size(),
                sweep_parameter,
                sweep_value,
            },
            metrics);
        ++rows_written;
    }

    if (config.export_mode == ExportMode::Summary) {
        auto aggregate = summary.aggregate(simulation_time);
        writer.write_sample(
            SampleMetadata{
                sim::scenario_display_name(config.scenario),
                config.seed,
                timestamp,
                git_commit,
                config.export_mode,
                config.sample_rate_hz,
                summary.sample_count(),
                simulation_time,
                simulation.size(),
                sweep_parameter,
                sweep_value,
            },
            aggregate);
        ++rows_written;
    }
}

} // namespace


std::optional<ExperimentPreset> experiment_preset(std::string_view name)
{
    if (name == "bird_baseline") {
        return ExperimentPreset{
            "bird_baseline",
            "BirdFlight default stability baseline",
            sim::ScenarioType::BirdFlight,
            bird_preset_parameters(9.8F, 9.8F, 220.0F, 120.0F)};
    }
    if (name == "bird_low_lift") {
        return ExperimentPreset{
            "bird_low_lift",
            "BirdFlight with reduced lift",
            sim::ScenarioType::BirdFlight,
            bird_preset_parameters(8.2F, 9.8F, 220.0F, 120.0F)};
    }
    if (name == "bird_high_gravity") {
        return ExperimentPreset{
            "bird_high_gravity",
            "BirdFlight with stronger gravity",
            sim::ScenarioType::BirdFlight,
            bird_preset_parameters(9.8F, 12.5F, 220.0F, 120.0F)};
    }
    if (name == "bird_narrow_fov") {
        return ExperimentPreset{
            "bird_narrow_fov",
            "BirdFlight with narrower forward field of view",
            sim::ScenarioType::BirdFlight,
            bird_preset_parameters(9.8F, 9.8F, 140.0F, 120.0F)};
    }
    if (name == "bird_low_turn_rate") {
        return ExperimentPreset{
            "bird_low_turn_rate",
            "BirdFlight with limited turn response",
            sim::ScenarioType::BirdFlight,
            bird_preset_parameters(9.8F, 9.8F, 220.0F, 70.0F)};
    }
    if (name == "fish_baseline") {
        return ExperimentPreset{
            "fish_baseline",
            "FishSchool default resistive-medium baseline",
            sim::ScenarioType::FishSchool,
            fish_preset_parameters(0.35F, 0.0F, 5.0F, 360.0F)};
    }
    if (name == "fish_high_drag") {
        return ExperimentPreset{
            "fish_high_drag",
            "FishSchool with stronger velocity drag",
            sim::ScenarioType::FishSchool,
            fish_preset_parameters(0.75F, 0.0F, 5.0F, 360.0F)};
    }
    if (name == "fish_strong_current") {
        return ExperimentPreset{
            "fish_strong_current",
            "FishSchool advected by a stronger constant current",
            sim::ScenarioType::FishSchool,
            fish_preset_parameters(0.35F, 3.0F, 5.0F, 360.0F)};
    }
    if (name == "fish_low_visibility") {
        return ExperimentPreset{
            "fish_low_visibility",
            "FishSchool with shorter interaction range and restricted field of view",
            sim::ScenarioType::FishSchool,
            fish_preset_parameters(0.35F, 0.0F, 2.75F, 180.0F)};
    }
    return std::nullopt;
}

std::span<const std::string_view> experiment_preset_names() noexcept
{
    static constexpr auto names = preset_name_storage();
    return names;
}

bool apply_experiment_preset(ExperimentConfig& config, std::string_view name)
{
    const auto preset = experiment_preset(name);
    if (!preset.has_value()) {
        return false;
    }
    config.preset = std::string{name};
    config.scenario = preset->scenario;
    config.parameter_defaults = preset->parameters;
    return true;
}

sim::SimulationParameters experiment_parameters(const ExperimentConfig& config)
{
    if (config.parameter_defaults.has_value()) {
        return *config.parameter_defaults;
    }
    return sim::build_scenario(config.scenario).simulation_parameters;
}

std::optional<SweepDefinition> parse_sweep(std::string_view text)
{
    const auto equals = text.find('=');
    if (equals == std::string_view::npos || equals == 0 || equals + 1 >= text.size()) {
        return std::nullopt;
    }

    const auto parameter = text.substr(0, equals);
    const auto range = text.substr(equals + 1);
    const auto first_colon = range.find(':');
    const auto second_colon = first_colon == std::string_view::npos ? std::string_view::npos : range.find(':', first_colon + 1);
    if (first_colon == std::string_view::npos || second_colon == std::string_view::npos) {
        return std::nullopt;
    }

    auto start = parse_double(range.substr(0, first_colon));
    auto end = parse_double(range.substr(first_colon + 1, second_colon - first_colon - 1));
    auto step = parse_double(range.substr(second_colon + 1));
    if (!start.has_value() || !end.has_value() || !step.has_value() || *step == 0.0) {
        return std::nullopt;
    }
    if ((*end > *start && *step < 0.0) || (*end < *start && *step > 0.0)) {
        return std::nullopt;
    }

    return SweepDefinition{std::string{parameter}, *start, *end, *step};
}

std::vector<double> sweep_values(const SweepDefinition& sweep)
{
    std::vector<double> values{};
    const auto count_estimate = static_cast<std::size_t>(std::floor(std::abs((sweep.end - sweep.start) / sweep.step))) + 1U;
    values.reserve(count_estimate);
    constexpr double epsilon = 1.0e-9;
    if (sweep.step > 0.0) {
        for (double value = sweep.start; value <= sweep.end + epsilon; value += sweep.step) {
            values.push_back(value);
        }
    } else {
        for (double value = sweep.start; value >= sweep.end - epsilon; value += sweep.step) {
            values.push_back(value);
        }
    }
    return values;
}

bool apply_sweep_value(sim::SimulationParameters& parameters, std::string_view parameter, double value) noexcept
{
    const auto float_value = static_cast<float>(value);
    if (parameter == "perception_radius" || parameter == "neighbor_radius") {
        parameters.neighbor_radius = float_value;
        sim::sync_spatial_cell_size_to_query_radius(parameters);
        return true;
    }
    if (parameter == "alignment_weight") {
        parameters.alignment_weight = float_value;
        return true;
    }
    if (parameter == "separation_weight") {
        parameters.separation_weight = float_value;
        return true;
    }
    if (parameter == "cohesion_weight") {
        parameters.cohesion_weight = float_value;
        return true;
    }
    if (parameter == "separation_radius") {
        parameters.separation_radius = float_value;
        sim::sync_spatial_cell_size_to_query_radius(parameters);
        return true;
    }
    if (parameter == "max_speed") {
        parameters.max_speed = float_value;
        return true;
    }
    if (parameter == "max_force") {
        parameters.max_force = float_value;
        return true;
    }
    if (parameter == "gravity") {
        parameters.gravity = float_value;
        return true;
    }
    if (parameter == "lift_strength" || parameter == "lift") {
        parameters.lift_strength = float_value;
        return true;
    }
    if (parameter == "max_turn_rate") {
        parameters.max_turn_rate = float_value;
        return true;
    }
    if (parameter == "field_of_view_degrees" || parameter == "field_of_view") {
        parameters.field_of_view_degrees = float_value;
        return true;
    }
    if (parameter == "altitude_correction_strength" || parameter == "altitude_correction") {
        parameters.altitude_correction_strength = float_value;
        return true;
    }
    if (parameter == "drag_coefficient" || parameter == "drag") {
        parameters.drag_coefficient = float_value;
        return true;
    }
    if (parameter == "buoyancy_strength" || parameter == "buoyancy") {
        parameters.buoyancy_strength = float_value;
        return true;
    }
    if (parameter == "target_depth") {
        parameters.target_depth = float_value;
        return true;
    }
    if (parameter == "depth_band") {
        parameters.depth_band = float_value;
        return true;
    }
    if (parameter == "depth_correction_strength" || parameter == "depth_correction") {
        parameters.depth_correction_strength = float_value;
        return true;
    }
    if (parameter == "current_strength" || parameter == "current") {
        parameters.current_strength = float_value;
        return true;
    }
    if (parameter == "current_direction_x") {
        parameters.current_direction.x = float_value;
        return true;
    }
    if (parameter == "current_direction_y") {
        parameters.current_direction.y = float_value;
        return true;
    }
    if (parameter == "current_direction_z") {
        parameters.current_direction.z = float_value;
        return true;
    }
    return false;
}

std::optional<ExperimentConfig> parse_cli(int argc, char** argv, std::string& error)
{
    ExperimentConfig config{};
    for (int i = 1; i < argc; ++i) {
        const std::string_view arg{argv[i]};
        if (arg == "--preset") {
            if (i + 1 >= argc) {
                error = "missing value for --preset";
                return std::nullopt;
            }
            ++i;
            if (!apply_experiment_preset(config, argv[i])) {
                error = "unknown preset: ";
                error += argv[i];
                return std::nullopt;
            }
        }
    }

    for (int i = 1; i < argc; ++i) {
        const std::string_view arg{argv[i]};
        const auto read_value = [&]() -> std::optional<std::string_view> {
            if (i + 1 >= argc) {
                error = "missing value for ";
                error += arg;
                return std::nullopt;
            }
            ++i;
            return std::string_view{argv[i]};
        };

        if (arg == "--scenario") {
            const auto value = read_value();
            if (!value.has_value()) {
                return std::nullopt;
            }
            const auto scenario = sim::scenario_type_from_name(*value);
            if (!scenario.has_value()) {
                error = "unknown scenario: ";
                error += *value;
                return std::nullopt;
            }
            config.scenario = *scenario;
            config.parameter_defaults = sim::build_scenario(*scenario).simulation_parameters;
        } else if (arg == "--preset") {
            const auto value = read_value();
            if (!value.has_value()) {
                return std::nullopt;
            }
        } else if (arg == "--seed") {
            const auto value = read_value();
            const auto seed = value.has_value() ? parse_u32(*value) : std::optional<std::uint32_t>{};
            if (!seed.has_value()) {
                error = "invalid seed";
                return std::nullopt;
            }
            config.seed = *seed;
        } else if (arg == "--boids") {
            const auto value = read_value();
            const auto boids = value.has_value() ? parse_u32(*value) : std::optional<std::uint32_t>{};
            if (!boids.has_value()) {
                error = "invalid boid count";
                return std::nullopt;
            }
            config.boids = *boids;
        } else if (arg == "--duration") {
            const auto value = read_value();
            const auto duration = value.has_value() ? parse_double(*value) : std::optional<double>{};
            if (!duration.has_value() || *duration < 0.0) {
                error = "invalid duration";
                return std::nullopt;
            }
            config.duration_seconds = *duration;
        } else if (arg == "--fixed-dt") {
            const auto value = read_value();
            const auto fixed_dt = value.has_value() ? parse_double(*value) : std::optional<double>{};
            if (!fixed_dt.has_value() || *fixed_dt <= 0.0) {
                error = "invalid fixed dt";
                return std::nullopt;
            }
            config.fixed_dt = *fixed_dt;
        } else if (arg == "--sample-rate") {
            const auto value = read_value();
            const auto sample_rate = value.has_value() ? parse_double(*value) : std::optional<double>{};
            if (!sample_rate.has_value() || *sample_rate <= 0.0) {
                error = "invalid sample rate";
                return std::nullopt;
            }
            config.sample_rate_hz = *sample_rate;
        } else if (arg == "--export-mode") {
            const auto value = read_value();
            const auto mode = value.has_value() ? parse_export_mode(*value) : std::optional<ExportMode>{};
            if (!mode.has_value()) {
                error = "invalid export mode";
                return std::nullopt;
            }
            config.export_mode = *mode;
        } else if (arg == "--output") {
            const auto value = read_value();
            if (!value.has_value()) {
                return std::nullopt;
            }
            config.output_path = std::filesystem::path{std::string{*value}};
        } else if (arg == "--sweep") {
            const auto value = read_value();
            if (!value.has_value()) {
                return std::nullopt;
            }
            auto sweep = parse_sweep(*value);
            if (!sweep.has_value()) {
                error = "invalid sweep";
                return std::nullopt;
            }
            config.sweep = std::move(sweep);
        } else {
            error = "unknown argument: ";
            error += arg;
            return std::nullopt;
        }
    }
    return config;
}

ExperimentRunResult run_experiment(const ExperimentConfig& config)
{
    CsvMetricsWriter writer{config.output_path};
    std::size_t rows_written{};

    if (config.sweep.has_value()) {
        for (const double value : sweep_values(*config.sweep)) {
            run_single_value(config, writer, std::pair<std::string, double>{config.sweep->parameter, value}, rows_written);
        }
    } else {
        run_single_value(config, writer, std::nullopt, rows_written);
    }

    writer.close();
    return ExperimentRunResult{rows_written, config.output_path};
}

} // namespace flock3d::experiment
