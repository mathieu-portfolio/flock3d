#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <string_view>

#include <flock3d/sim/Scenario.hpp>
#include <flock3d/sim/SimulationMetrics.hpp>

namespace flock3d::experiment {

enum class ExportMode {
    Summary,
    SampledTimeSeries,
    FullTrajectory,
};

struct SampleMetadata {
    std::string_view scenario{};
    std::uint32_t seed{};
    std::string_view timestamp{};
    std::string_view git_commit{};
    ExportMode export_mode{ExportMode::SampledTimeSeries};
    double sample_rate_hz{5.0};
    std::size_t sample_index{};
    double simulation_time{};
    std::size_t boid_count{};
    std::string_view sweep_parameter{};
    std::string_view sweep_value{};
};

class SampleScheduler {
public:
    explicit SampleScheduler(double sample_rate_hz = 5.0) noexcept;

    void set_sample_rate_hz(double sample_rate_hz) noexcept;
    void reset() noexcept;

    [[nodiscard]] double sample_rate_hz() const noexcept { return sample_rate_hz_; }
    [[nodiscard]] double sample_interval_seconds() const noexcept { return sample_interval_seconds_; }
    [[nodiscard]] std::size_t next_sample_index() const noexcept { return next_sample_index_; }
    [[nodiscard]] bool should_sample(double simulation_time) const noexcept;
    [[nodiscard]] std::size_t consume_sample() noexcept;

private:
    double sample_rate_hz_{5.0};
    double sample_interval_seconds_{0.2};
    double next_sample_time_{0.2};
    std::size_t next_sample_index_{};
};

struct SummaryStatistics {
    double mean_polarization{};
    double max_polarization{};
    double mean_cohesion{};
    double max_dispersion{};
    double mean_average_speed{};
    double mean_average_neighbors{};
    double mean_nearest_neighbor_distance{};
    double total_duration_seconds{};
    std::size_t sample_count{};
};

class SummaryAggregator {
public:
    void add_sample(const sim::SimulationMetrics& metrics) noexcept;
    [[nodiscard]] SummaryStatistics statistics(double total_duration_seconds) const noexcept;
    [[nodiscard]] sim::SimulationMetrics aggregate(double total_duration_seconds) const noexcept;
    [[nodiscard]] std::size_t sample_count() const noexcept { return sample_count_; }

private:
    std::size_t sample_count_{};
    double polarization_total_{};
    double polarization_max_{};
    double cohesion_total_{};
    double dispersion_max_{};
    double average_speed_total_{};
    double average_neighbors_total_{};
    double nearest_neighbor_distance_total_{};
};

class CsvMetricsWriter {
public:
    CsvMetricsWriter() = default;
    explicit CsvMetricsWriter(std::filesystem::path output_path);

    bool open(const std::filesystem::path& output_path);
    void close();
    void write_sample(const SampleMetadata& metadata, const sim::SimulationMetrics& metrics);

    [[nodiscard]] bool is_open() const noexcept { return stream_.is_open(); }
    [[nodiscard]] const std::filesystem::path& output_path() const noexcept { return output_path_; }

    static constexpr std::string_view header();

private:
    std::ofstream stream_{};
    std::filesystem::path output_path_{};
};

class MetricsRecorder {
public:
    void start(
        const std::filesystem::path& output_path,
        sim::ScenarioType scenario,
        std::uint32_t seed,
        ExportMode mode,
        double sample_rate_hz);
    void stop(double simulation_time, std::size_t boid_count, const sim::SimulationMetrics& latest_metrics);
    void record_step(double simulation_time, std::size_t boid_count, const sim::SimulationMetrics& metrics);

    void set_export_mode(ExportMode mode) noexcept;
    void set_sample_rate_hz(double sample_rate_hz) noexcept;

    [[nodiscard]] bool is_recording() const noexcept { return recording_; }
    [[nodiscard]] ExportMode export_mode() const noexcept { return export_mode_; }
    [[nodiscard]] double sample_rate_hz() const noexcept { return scheduler_.sample_rate_hz(); }
    [[nodiscard]] const std::filesystem::path& output_path() const noexcept { return writer_.output_path(); }

private:
    void write_summary(double simulation_time, std::size_t boid_count, const sim::SimulationMetrics& fallback_metrics);

    CsvMetricsWriter writer_{};
    SampleScheduler scheduler_{5.0};
    SummaryAggregator summary_{};
    sim::ScenarioType scenario_{sim::ScenarioType::ClassicBoids};
    std::uint32_t seed_{};
    ExportMode export_mode_{ExportMode::SampledTimeSeries};
    std::string timestamp_{};
    std::string git_commit_{};
    bool recording_{};
};

[[nodiscard]] std::string_view to_string(ExportMode mode) noexcept;
[[nodiscard]] std::optional<ExportMode> parse_export_mode(std::string_view text) noexcept;
[[nodiscard]] std::string timestamp_for_filename();
[[nodiscard]] std::string current_git_commit();
[[nodiscard]] std::filesystem::path default_output_path(sim::ScenarioType scenario, ExportMode mode);

} // namespace flock3d::experiment
