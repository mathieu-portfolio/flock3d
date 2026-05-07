#include <flock3d/experiment/ExperimentRunner.hpp>

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
    auto scenario = sim::build_scenario(config.scenario);
    auto parameters = scenario.simulation_parameters;
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
    return false;
}

std::optional<ExperimentConfig> parse_cli(int argc, char** argv, std::string& error)
{
    ExperimentConfig config{};
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
