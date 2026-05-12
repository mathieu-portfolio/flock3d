#pragma once

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <thread>
#include <vector>

#if defined(_WIN32)
#include <io.h>
#else
#include <unistd.h>
#endif

#include <flock3d/sim/Scenario.hpp>
#include <flock3d/sim/SimulationParameters.hpp>

namespace flock3d::bench {

using Clock = std::chrono::steady_clock;

inline constexpr double default_duration_seconds = 1.0;
inline constexpr double default_sample_seconds = 1.0;
inline constexpr double default_warmup_seconds = 0.25;
inline constexpr double full_matrix_duration_seconds = 20.0;
inline constexpr double full_matrix_sample_seconds = 5.0;
inline constexpr double full_matrix_warmup_seconds = 1.0;
inline constexpr float fixed_dt = 1.0F / 120.0F;
inline constexpr double fixed_dt_seconds = static_cast<double>(fixed_dt);

using flock3d::sim::SimulationParameters;
using flock3d::sim::sync_spatial_cell_size_to_query_radius;

struct ParameterOverrides {
    std::optional<float> world_half_extent{};
    std::optional<float> neighbor_radius{};
    std::optional<float> perception_radius{};
    std::optional<float> separation_radius{};
    std::optional<float> max_speed{};
    std::optional<float> max_force{};
    std::optional<std::size_t> max_selected_neighbors{};
    std::optional<std::uint32_t> target_neighbor_count{};
    std::optional<bool> adaptive_perception_enabled{};
};

enum class DiagnosticsLevel {
    None,
    Phases,
    Workers,
    Full,
};

struct BenchmarkOptions {
    double duration_seconds{default_duration_seconds};
    double sample_seconds{default_sample_seconds};
    double warmup_seconds{default_warmup_seconds};
    std::optional<std::uint32_t> seed{};
    std::uint32_t thread_chunk_size{0U};
    std::vector<std::uint32_t> thread_counts{1U};
    std::vector<std::uint32_t> boid_counts{512U};
    std::vector<std::string> model_filters{};
    std::vector<std::string> mode_filters{};
    ParameterOverrides parameter_overrides{};
    DiagnosticsLevel diagnostics_level{DiagnosticsLevel::None};
    bool full_matrix{false};
    bool hardware_threads{false};
};

inline void append_unique(std::vector<std::uint32_t>& values, std::uint32_t value)
{
    if (std::find(values.begin(), values.end(), value) == values.end()) {
        values.push_back(value);
    }
}

inline void append_hardware_thread_count(std::vector<std::uint32_t>& thread_counts)
{
    const unsigned int hardware_threads = std::thread::hardware_concurrency();
    if (hardware_threads == 0U) {
        return;
    }
    append_unique(thread_counts, static_cast<std::uint32_t>(hardware_threads));
}

[[nodiscard]] inline std::string trim_copy(std::string_view text)
{
    const auto first = text.find_first_not_of(" \t\n\r");
    if (first == std::string_view::npos) {
        return {};
    }
    const auto last = text.find_last_not_of(" \t\n\r");
    return std::string{text.substr(first, last - first + 1U)};
}

[[nodiscard]] inline std::vector<std::string> parse_string_list(std::string_view text)
{
    std::vector<std::string> values;
    std::size_t offset = 0U;
    while (offset <= text.size()) {
        const std::size_t comma = text.find(',', offset);
        const std::string token = trim_copy(text.substr(offset, comma == std::string_view::npos ? std::string_view::npos : comma - offset));
        if (!token.empty() && std::find(values.begin(), values.end(), token) == values.end()) {
            values.push_back(token);
        }
        if (comma == std::string_view::npos) {
            break;
        }
        offset = comma + 1U;
    }
    return values;
}

[[nodiscard]] inline std::optional<std::uint32_t> parse_u32(std::string_view text) noexcept
{
    const std::string trimmed = trim_copy(text);
    if (trimmed.empty() || trimmed.front() == '-') {
        return std::nullopt;
    }
    std::uint32_t value{};
    const char* begin = trimmed.data();
    const char* end = begin + trimmed.size();
    const auto result = std::from_chars(begin, end, value);
    if (result.ec != std::errc{} || result.ptr != end) {
        return std::nullopt;
    }
    return value;
}

[[nodiscard]] inline std::optional<std::size_t> parse_size(std::string_view text) noexcept
{
    const auto value = parse_u32(text);
    if (!value.has_value()) {
        return std::nullopt;
    }
    return static_cast<std::size_t>(*value);
}

[[nodiscard]] inline std::optional<double> parse_double(std::string_view text) noexcept
{
    const std::string trimmed = trim_copy(text);
    if (trimmed.empty()) {
        return std::nullopt;
    }
    char* end = nullptr;
    const double value = std::strtod(trimmed.c_str(), &end);
    if (end == trimmed.c_str() || *end != '\0' || !std::isfinite(value)) {
        return std::nullopt;
    }
    return value;
}

[[nodiscard]] inline std::optional<float> parse_float(std::string_view text) noexcept
{
    const auto value = parse_double(text);
    if (!value.has_value()) {
        return std::nullopt;
    }
    return static_cast<float>(*value);
}

[[nodiscard]] inline std::vector<std::uint32_t> parse_positive_integer_list(std::string_view text)
{
    std::vector<std::uint32_t> values;
    for (const std::string& token : parse_string_list(text)) {
        const auto value = parse_u32(token);
        if (!value.has_value() || *value == 0U) {
            return {};
        }
        append_unique(values, *value);
    }
    return values;
}

[[nodiscard]] inline std::vector<std::uint32_t> parse_thread_counts(std::string_view text)
{
    return parse_positive_integer_list(text);
}

[[nodiscard]] inline std::vector<std::uint32_t> parse_positive_counts(std::string_view text)
{
    return parse_positive_integer_list(text);
}

[[nodiscard]] inline std::string lowercase_copy(std::string_view text)
{
    std::string value = trim_copy(text);
    for (char& character : value) {
        if (character >= 'A' && character <= 'Z') {
            character = static_cast<char>(character - 'A' + 'a');
        }
    }
    return value;
}

[[nodiscard]] inline std::optional<DiagnosticsLevel> parse_diagnostics_level(std::string_view text) noexcept
{
    const std::string value = lowercase_copy(text);
    if (value == "none" || value == "basic" || value == "compact") {
        return DiagnosticsLevel::None;
    }
    if (value == "phases") {
        return DiagnosticsLevel::Phases;
    }
    if (value == "workers") {
        return DiagnosticsLevel::Workers;
    }
    if (value == "full") {
        return DiagnosticsLevel::Full;
    }
    return std::nullopt;
}

[[nodiscard]] inline bool includes_phase_diagnostics(DiagnosticsLevel level) noexcept
{
    return level == DiagnosticsLevel::Phases || level == DiagnosticsLevel::Full;
}

[[nodiscard]] inline bool includes_worker_diagnostics(DiagnosticsLevel level) noexcept
{
    return level == DiagnosticsLevel::Workers || level == DiagnosticsLevel::Full;
}

[[nodiscard]] inline bool includes_internal_diagnostics(DiagnosticsLevel level) noexcept
{
    return level == DiagnosticsLevel::Full;
}

[[nodiscard]] inline std::optional<bool> parse_bool(std::string_view text) noexcept
{
    const std::string value = lowercase_copy(text);
    if (value == "on" || value == "true" || value == "1" || value == "yes") {
        return true;
    }
    if (value == "off" || value == "false" || value == "0" || value == "no") {
        return false;
    }
    return std::nullopt;
}

[[nodiscard]] inline char normalized_character(char character) noexcept
{
    if (character >= 'A' && character <= 'Z') {
        return static_cast<char>(character - 'A' + 'a');
    }
    return character;
}

[[nodiscard]] inline bool is_name_separator(char character) noexcept
{
    return character == ' ' || character == '-' || character == '_';
}

[[nodiscard]] inline bool normalized_name_equal(std::string_view lhs, std::string_view rhs) noexcept
{
    std::size_t lhs_index = 0U;
    std::size_t rhs_index = 0U;
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

[[nodiscard]] inline std::string join_names(const std::vector<std::string_view>& names)
{
    std::string joined;
    for (std::size_t index = 0U; index < names.size(); ++index) {
        if (index > 0U) {
            joined += ", ";
        }
        joined += names[index];
    }
    return joined;
}

inline std::string_view model_name(flock3d::sim::SimulationModel model) noexcept
{
    switch (model) {
    case flock3d::sim::SimulationModel::ClassicBoids:
        return "ClassicBoids";
    case flock3d::sim::SimulationModel::BirdFlight:
        return "BirdFlight";
    case flock3d::sim::SimulationModel::FishSchool:
        return "FishSchool";
    case flock3d::sim::SimulationModel::NoiseExperiment:
        return "NoiseExperiment";
    }
    return "Unknown";
}

[[nodiscard]] inline std::optional<flock3d::sim::SimulationModel> parse_model_name(std::string_view name) noexcept
{
    using flock3d::sim::SimulationModel;
    constexpr SimulationModel models[] = {
        SimulationModel::ClassicBoids,
        SimulationModel::BirdFlight,
        SimulationModel::FishSchool,
        SimulationModel::NoiseExperiment,
    };
    for (const SimulationModel model : models) {
        if (normalized_name_equal(name, model_name(model))) {
            return model;
        }
    }
    return std::nullopt;
}

[[nodiscard]] inline std::vector<flock3d::sim::SimulationModel> parse_model_list(std::string_view text)
{
    std::vector<flock3d::sim::SimulationModel> models;
    for (const std::string& token : parse_string_list(text)) {
        const auto model = parse_model_name(token);
        if (!model.has_value()) {
            return {};
        }
        if (std::find(models.begin(), models.end(), *model) == models.end()) {
            models.push_back(*model);
        }
    }
    return models;
}

struct UpdateStats {
    std::size_t count{};
    double total_ms{};
    double min_ms{std::numeric_limits<double>::max()};
    double max_ms{};
    std::vector<double> samples_ms{};

    void record(double milliseconds)
    {
        ++count;
        total_ms += milliseconds;
        min_ms = std::min(min_ms, milliseconds);
        max_ms = std::max(max_ms, milliseconds);
        samples_ms.push_back(milliseconds);
    }

    [[nodiscard]] double mean_ms() const noexcept { return count > 0 ? total_ms / static_cast<double>(count) : 0.0; }
    [[nodiscard]] double mean_ns() const noexcept { return mean_ms() * 1'000'000.0; }
    [[nodiscard]] double wall_seconds() const noexcept { return total_ms / 1000.0; }
    [[nodiscard]] double ticks_per_second() const noexcept { return wall_seconds() > 0.0 ? static_cast<double>(count) / wall_seconds() : 0.0; }
    [[nodiscard]] double real_time_factor(double dt = fixed_dt_seconds) const noexcept { return wall_seconds() > 0.0 ? (static_cast<double>(count) * dt) / wall_seconds() : 0.0; }
    [[nodiscard]] double min_or_zero() const noexcept { return count > 0 ? min_ms : 0.0; }

    [[nodiscard]] double percentile_ms(double percentile) const
    {
        if (samples_ms.empty()) {
            return 0.0;
        }
        auto values = samples_ms;
        std::sort(values.begin(), values.end());
        const double clamped = std::clamp(percentile, 0.0, 1.0);
        if (clamped <= 0.0) {
            return values.front();
        }
        const auto rank = static_cast<std::size_t>(std::ceil(clamped * static_cast<double>(values.size())));
        const std::size_t index = std::min(values.size() - 1U, rank > 0U ? rank - 1U : 0U);
        return values[index];
    }

    [[nodiscard]] double p50_ms() const { return percentile_ms(0.50); }
    [[nodiscard]] double p95_ms() const { return percentile_ms(0.95); }
    [[nodiscard]] double p99_ms() const { return percentile_ms(0.99); }
};

inline void print_usage(std::string_view executable)
{
    std::cerr << "Usage: " << executable
              << " [--models ClassicBoids,BirdFlight] [--modes fixed_radius_uncapped,adaptive_radius_closest_k]"
              << " [--counts 512,1024] [--threads 1,2,4] [--hardware-threads] [--full-matrix]"
              << " [--duration seconds] [--sample seconds] [--warmup seconds] [--seed value] [--chunk-size boids]"
              << " [--diagnostics none|phases|workers|full] [--profile-level none|phases|workers|full]"
              << " [--world-size half-extent] [--neighbor-radius radius] [--perception-radius radius]"
              << " [--separation-radius radius] [--max-speed speed] [--max-force force]"
              << " [--max-selected-neighbors count] [--target-neighbor-count count] [--adaptive-perception on|off]\n"
              << "CSV is printed to stdout. Default CSVs stay compact; pass --diagnostics to append investigation-only columns."
              << " Benchmarks advance simulated time as fast as possible; progress is printed to stderr only when stderr is a terminal."
              << " Comma-separated lists are accepted for models, modes, counts, and threads.\n";
}

[[noreturn]] inline void fail_usage(std::string_view executable, std::string_view message)
{
    std::cerr << "benchmark option error: " << message << "\n\n";
    print_usage(executable);
    std::exit(EXIT_FAILURE);
}

[[nodiscard]] inline std::string_view require_value(int argc, char** argv, int& index, std::string_view argument)
{
    if (index + 1 >= argc) {
        fail_usage(argv[0], std::string{argument} + " requires a value");
    }
    ++index;
    return argv[index];
}

inline void apply_full_matrix_defaults(BenchmarkOptions& options)
{
    options.duration_seconds = full_matrix_duration_seconds;
    options.sample_seconds = full_matrix_sample_seconds;
    options.warmup_seconds = full_matrix_warmup_seconds;
    options.thread_counts = {1U, 2U, 4U, 8U, 16U};
    options.boid_counts = {128U, 256U, 512U, 1024U};
}

inline BenchmarkOptions parse_options(int argc, char** argv)
{
    BenchmarkOptions options{};
    for (int index = 1; index < argc; ++index) {
        const std::string_view argument{argv[index]};
        if (argument == "--help" || argument == "-h") {
            print_usage(argv[0]);
            std::exit(EXIT_SUCCESS);
        }
        if (argument == "--full-matrix") {
            options.full_matrix = true;
            apply_full_matrix_defaults(options);
        } else if (argument == "--hardware-threads") {
            options.hardware_threads = true;
            append_hardware_thread_count(options.thread_counts);
        } else if (argument == "--duration") {
            const auto value = parse_double(require_value(argc, argv, index, argument));
            if (!value.has_value() || *value <= 0.0) {
                fail_usage(argv[0], "--duration must be a finite positive number");
            }
            options.duration_seconds = *value;
        } else if (argument == "--sample") {
            const auto value = parse_double(require_value(argc, argv, index, argument));
            if (!value.has_value() || *value <= 0.0) {
                fail_usage(argv[0], "--sample must be a finite positive number");
            }
            options.sample_seconds = *value;
        } else if (argument == "--warmup") {
            const auto value = parse_double(require_value(argc, argv, index, argument));
            if (!value.has_value() || *value < 0.0) {
                fail_usage(argv[0], "--warmup must be a finite non-negative number");
            }
            options.warmup_seconds = *value;
        } else if (argument == "--threads") {
            options.thread_counts = parse_thread_counts(require_value(argc, argv, index, argument));
            if (options.thread_counts.empty()) {
                fail_usage(argv[0], "--threads must be a comma-separated list of positive integers");
            }
        } else if (argument == "--chunk-size") {
            const auto value = parse_u32(require_value(argc, argv, index, argument));
            if (!value.has_value()) {
                fail_usage(argv[0], "--chunk-size must be a non-negative integer");
            }
            options.thread_chunk_size = *value;
        } else if (argument == "--counts" || argument == "--boid-counts") {
            options.boid_counts = parse_positive_counts(require_value(argc, argv, index, argument));
            if (options.boid_counts.empty()) {
                fail_usage(argv[0], "--counts must be a comma-separated list of positive integers");
            }
        } else if (argument == "--models" || argument == "--scenarios") {
            options.model_filters = parse_string_list(require_value(argc, argv, index, argument));
            if (options.model_filters.empty()) {
                fail_usage(argv[0], "--models must name at least one model");
            }
        } else if (argument == "--modes") {
            options.mode_filters = parse_string_list(require_value(argc, argv, index, argument));
            if (options.mode_filters.empty()) {
                fail_usage(argv[0], "--modes must name at least one mode");
            }
        } else if (argument == "--diagnostics" || argument == "--profile-level") {
            const auto value = parse_diagnostics_level(require_value(argc, argv, index, argument));
            if (!value.has_value()) {
                fail_usage(argv[0], "--diagnostics must be one of none, phases, workers, or full");
            }
            options.diagnostics_level = *value;
        } else if (argument == "--seed") {
            const auto value = parse_u32(require_value(argc, argv, index, argument));
            if (!value.has_value()) {
                fail_usage(argv[0], "--seed must be a non-negative integer");
            }
            options.seed = *value;
        } else if (argument == "--world-size" || argument == "--world-half-extent") {
            const auto value = parse_float(require_value(argc, argv, index, argument));
            if (!value.has_value() || *value <= 0.0F) {
                fail_usage(argv[0], "--world-size must be a finite positive number");
            }
            options.parameter_overrides.world_half_extent = *value;
        } else if (argument == "--neighbor-radius" || argument == "--radius") {
            const auto value = parse_float(require_value(argc, argv, index, argument));
            if (!value.has_value() || *value <= 0.0F) {
                fail_usage(argv[0], "--neighbor-radius must be a finite positive number");
            }
            options.parameter_overrides.neighbor_radius = *value;
        } else if (argument == "--perception-radius") {
            const auto value = parse_float(require_value(argc, argv, index, argument));
            if (!value.has_value() || *value <= 0.0F) {
                fail_usage(argv[0], "--perception-radius must be a finite positive number");
            }
            options.parameter_overrides.perception_radius = *value;
        } else if (argument == "--separation-radius") {
            const auto value = parse_float(require_value(argc, argv, index, argument));
            if (!value.has_value() || *value <= 0.0F) {
                fail_usage(argv[0], "--separation-radius must be a finite positive number");
            }
            options.parameter_overrides.separation_radius = *value;
        } else if (argument == "--max-speed") {
            const auto value = parse_float(require_value(argc, argv, index, argument));
            if (!value.has_value() || *value <= 0.0F) {
                fail_usage(argv[0], "--max-speed must be a finite positive number");
            }
            options.parameter_overrides.max_speed = *value;
        } else if (argument == "--max-force") {
            const auto value = parse_float(require_value(argc, argv, index, argument));
            if (!value.has_value() || *value <= 0.0F) {
                fail_usage(argv[0], "--max-force must be a finite positive number");
            }
            options.parameter_overrides.max_force = *value;
        } else if (argument == "--max-selected-neighbors") {
            const auto value = parse_size(require_value(argc, argv, index, argument));
            if (!value.has_value()) {
                fail_usage(argv[0], "--max-selected-neighbors must be a non-negative integer");
            }
            options.parameter_overrides.max_selected_neighbors = *value;
        } else if (argument == "--target-neighbor-count") {
            const auto value = parse_u32(require_value(argc, argv, index, argument));
            if (!value.has_value() || *value == 0U) {
                fail_usage(argv[0], "--target-neighbor-count must be a positive integer");
            }
            options.parameter_overrides.target_neighbor_count = *value;
        } else if (argument == "--adaptive-perception") {
            const auto value = parse_bool(require_value(argc, argv, index, argument));
            if (!value.has_value()) {
                fail_usage(argv[0], "--adaptive-perception must be one of on/off, true/false, or 1/0");
            }
            options.parameter_overrides.adaptive_perception_enabled = *value;
        } else {
            fail_usage(argv[0], std::string{"unknown option: "} + std::string{argument});
        }
    }
    return options;
}

inline void apply_parameter_overrides(SimulationParameters& parameters, const BenchmarkOptions& options)
{
    const ParameterOverrides& overrides = options.parameter_overrides;
    if (overrides.world_half_extent.has_value()) {
        parameters.world_half_extent = *overrides.world_half_extent;
    }
    if (overrides.neighbor_radius.has_value()) {
        parameters.neighbor_radius = *overrides.neighbor_radius;
    }
    if (overrides.perception_radius.has_value()) {
        parameters.base_perception_radius = *overrides.perception_radius;
        parameters.min_perception_radius = *overrides.perception_radius;
        parameters.max_perception_radius = *overrides.perception_radius;
    }
    if (overrides.separation_radius.has_value()) {
        parameters.separation_radius = *overrides.separation_radius;
    }
    if (overrides.max_speed.has_value()) {
        parameters.max_speed = *overrides.max_speed;
    }
    if (overrides.max_force.has_value()) {
        parameters.max_force = *overrides.max_force;
    }
    if (overrides.max_selected_neighbors.has_value()) {
        parameters.max_selected_neighbors = *overrides.max_selected_neighbors;
    }
    if (overrides.target_neighbor_count.has_value()) {
        parameters.target_neighbor_count = *overrides.target_neighbor_count;
    }
    if (overrides.adaptive_perception_enabled.has_value()) {
        parameters.adaptive_perception_enabled = *overrides.adaptive_perception_enabled;
    }
    sync_spatial_cell_size_to_query_radius(parameters);
}

[[nodiscard]] inline std::size_t simulated_seconds_to_ticks(double seconds, double dt = fixed_dt_seconds)
{
    if (dt <= 0.0 || !std::isfinite(dt)) {
        return 0U;
    }
    if (seconds <= 0.0) {
        return 0U;
    }
    return static_cast<std::size_t>(std::ceil(seconds / dt));
}

[[nodiscard]] inline double ticks_to_simulated_seconds(std::size_t ticks, double dt = fixed_dt_seconds) noexcept
{
    return static_cast<double>(ticks) * dt;
}

inline bool stream_is_terminal(int file_descriptor) noexcept
{
#if defined(_WIN32)
    return _isatty(file_descriptor) != 0;
#else
    return isatty(file_descriptor) != 0;
#endif
}

inline bool progress_enabled_for_streams(bool stdout_is_terminal, bool stderr_is_terminal) noexcept
{
    (void)stdout_is_terminal;
    return stderr_is_terminal;
}

inline bool progress_enabled_for_mode(std::string_view mode, bool auto_enabled) noexcept
{
    if (mode.empty() || mode == "auto") {
        return auto_enabled;
    }
    if (mode == "always" || mode == "1" || mode == "true" || mode == "yes" || mode == "on") {
        return true;
    }
    if (mode == "never" || mode == "0" || mode == "false" || mode == "no" || mode == "off") {
        return false;
    }
    return auto_enabled;
}

inline bool progress_enabled() noexcept
{
#if defined(_WIN32)
    const bool auto_enabled = progress_enabled_for_streams(stream_is_terminal(_fileno(stdout)), stream_is_terminal(_fileno(stderr)));
#else
    const bool auto_enabled = progress_enabled_for_streams(stream_is_terminal(STDOUT_FILENO), stream_is_terminal(STDERR_FILENO));
#endif
    const char* progress_mode = std::getenv("FLOCK3D_BENCHMARK_PROGRESS");
    return progress_enabled_for_mode(progress_mode != nullptr ? std::string_view{progress_mode} : std::string_view{}, auto_enabled);
}

class ProgressBar {
public:
    explicit ProgressBar(bool enabled) noexcept
        : enabled_{enabled}
    {
    }

    void update(std::string_view benchmark, std::string_view scenario, std::size_t boid_count, double elapsed, double target)
    {
        if (!enabled_) {
            return;
        }
        const double ratio = target > 0.0 ? std::clamp(elapsed / target, 0.0, 1.0) : 1.0;
        const int percent = static_cast<int>(std::round(ratio * 100.0));
        constexpr int width = 20;
        const int filled = static_cast<int>(std::round(ratio * static_cast<double>(width)));

        std::cerr << '\r' << '[';
        for (int index = 0; index < width; ++index) {
            std::cerr << (index < filled ? '#' : '-');
        }
        std::cerr << "] " << std::setw(3) << percent << "% | " << benchmark << " | " << scenario << " | "
                  << boid_count << " boids | " << std::fixed << std::setprecision(1) << elapsed << " simulated s / "
                  << target << " simulated s" << std::flush;
    }

    void finish()
    {
        if (enabled_) {
            std::cerr << "\033[2K\r" << std::flush;
        }
    }

private:
    bool enabled_{};
};

inline SimulationParameters base_parameters(std::uint32_t boid_count, std::uint32_t seed)
{
    SimulationParameters parameters{};
    parameters.boid_count = boid_count;
    parameters.random_seed = seed;
    parameters.world_half_extent = 40.0F;
    parameters.neighbor_radius = 4.0F;
    parameters.separation_radius = 2.0F;
    parameters.max_speed = 10.0F;
    parameters.max_force = 12.0F;
    sync_spatial_cell_size_to_query_radius(parameters);
    return parameters;
}

inline SimulationParameters parameters_for_model(flock3d::sim::SimulationModel model, std::uint32_t boid_count, std::uint32_t seed)
{
    using flock3d::sim::ScenarioType;
    ScenarioType scenario = ScenarioType::ClassicBoids;
    switch (model) {
    case flock3d::sim::SimulationModel::ClassicBoids:
        scenario = ScenarioType::ClassicBoids;
        break;
    case flock3d::sim::SimulationModel::BirdFlight:
        scenario = ScenarioType::BirdFlight;
        break;
    case flock3d::sim::SimulationModel::FishSchool:
        scenario = ScenarioType::FishSchool;
        break;
    case flock3d::sim::SimulationModel::NoiseExperiment:
        scenario = ScenarioType::NoiseExperiment;
        break;
    }

    SimulationParameters parameters = flock3d::sim::build_scenario(scenario).simulation_parameters;
    parameters.boid_count = boid_count;
    parameters.random_seed = seed;
    sync_spatial_cell_size_to_query_radius(parameters);
    return parameters;
}

template <typename Fn>
inline double time_ms(Fn&& function)
{
    const auto start = Clock::now();
    function();
    const auto stop = Clock::now();
    return std::chrono::duration<double, std::milli>(stop - start).count();
}

} // namespace flock3d::bench
