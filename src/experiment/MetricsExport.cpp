#include <flock3d/experiment/MetricsExport.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <iomanip>
#include <sstream>

namespace flock3d::experiment {
namespace {

constexpr std::string_view csv_header = "scenario,seed,timestamp,git_commit,export_mode,sample_rate_hz,sample_index,simulation_time,boid_count,polarization,cohesion,dispersion,average_speed,average_neighbors,nearest_neighbor_distance,simulation_update_ms,neighbor_queries,spatial_cell_count,mean_altitude,altitude_variance,stall_count,near_ground_count,sweep_parameter,sweep_value";

[[nodiscard]] double valid_sample_rate(double sample_rate_hz) noexcept
{
    return sample_rate_hz > 0.0 ? sample_rate_hz : 5.0;
}

void write_csv_value(std::ostream& stream, std::string_view value)
{
    const bool quote = value.find_first_of(",\"\n\r") != std::string_view::npos;
    if (!quote) {
        stream << value;
        return;
    }

    stream << '"';
    for (const char character : value) {
        if (character == '"') {
            stream << "\"\"";
        } else {
            stream << character;
        }
    }
    stream << '"';
}

} // namespace

SampleScheduler::SampleScheduler(double sample_rate_hz) noexcept
{
    set_sample_rate_hz(sample_rate_hz);
}

void SampleScheduler::set_sample_rate_hz(double sample_rate_hz) noexcept
{
    sample_rate_hz_ = valid_sample_rate(sample_rate_hz);
    sample_interval_seconds_ = 1.0 / sample_rate_hz_;
    next_sample_time_ = sample_interval_seconds_ * static_cast<double>(next_sample_index_ + 1U);
}

void SampleScheduler::reset() noexcept
{
    next_sample_index_ = 0;
    next_sample_time_ = sample_interval_seconds_;
}

bool SampleScheduler::should_sample(double simulation_time) const noexcept
{
    constexpr double epsilon = 1.0e-9;
    return simulation_time + epsilon >= next_sample_time_;
}

std::size_t SampleScheduler::consume_sample() noexcept
{
    const auto consumed = next_sample_index_;
    ++next_sample_index_;
    next_sample_time_ = sample_interval_seconds_ * static_cast<double>(next_sample_index_ + 1U);
    return consumed;
}

void SummaryAggregator::add_sample(const sim::SimulationMetrics& metrics) noexcept
{
    ++sample_count_;
    polarization_total_ += metrics.polarization;
    polarization_max_ = std::max(polarization_max_, static_cast<double>(metrics.polarization));
    cohesion_total_ += metrics.cohesion;
    dispersion_max_ = std::max(dispersion_max_, static_cast<double>(metrics.dispersion));
    average_speed_total_ += metrics.average_speed;
    average_neighbors_total_ += metrics.average_neighbors_per_boid;
    nearest_neighbor_distance_total_ += metrics.nearest_neighbor_average_distance;
    altitude_total_ += metrics.mean_altitude;
    altitude_variance_total_ += metrics.altitude_variance;
    stall_count_total_ += static_cast<double>(metrics.stall_count);
    near_ground_count_total_ += static_cast<double>(metrics.near_ground_count);
}

SummaryStatistics SummaryAggregator::statistics(double total_duration_seconds) const noexcept
{
    SummaryStatistics summary{};
    summary.total_duration_seconds = total_duration_seconds;
    summary.sample_count = sample_count_;
    if (sample_count_ == 0) {
        return summary;
    }

    const auto denominator = static_cast<double>(sample_count_);
    summary.mean_polarization = polarization_total_ / denominator;
    summary.max_polarization = polarization_max_;
    summary.mean_cohesion = cohesion_total_ / denominator;
    summary.max_dispersion = dispersion_max_;
    summary.mean_average_speed = average_speed_total_ / denominator;
    summary.mean_average_neighbors = average_neighbors_total_ / denominator;
    summary.mean_nearest_neighbor_distance = nearest_neighbor_distance_total_ / denominator;
    summary.mean_altitude = altitude_total_ / denominator;
    summary.mean_altitude_variance = altitude_variance_total_ / denominator;
    summary.mean_stall_count = stall_count_total_ / denominator;
    summary.mean_near_ground_count = near_ground_count_total_ / denominator;
    return summary;
}

sim::SimulationMetrics SummaryAggregator::aggregate(double total_duration_seconds) const noexcept
{
    const auto summary = statistics(total_duration_seconds);
    sim::SimulationMetrics metrics{};
    metrics.polarization = static_cast<float>(summary.mean_polarization);
    metrics.cohesion = static_cast<float>(summary.mean_cohesion);
    metrics.dispersion = static_cast<float>(summary.max_dispersion);
    metrics.average_speed = static_cast<float>(summary.mean_average_speed);
    metrics.average_neighbors_per_boid = static_cast<float>(summary.mean_average_neighbors);
    metrics.nearest_neighbor_average_distance = static_cast<float>(summary.mean_nearest_neighbor_distance);
    metrics.mean_altitude = static_cast<float>(summary.mean_altitude);
    metrics.altitude_variance = static_cast<float>(summary.mean_altitude_variance);
    metrics.stall_count = static_cast<std::size_t>(summary.mean_stall_count);
    metrics.near_ground_count = static_cast<std::size_t>(summary.mean_near_ground_count);
    metrics.simulation_update_ms = summary.total_duration_seconds;
    metrics.neighbor_queries = static_cast<std::uint64_t>(summary.sample_count);
    metrics.neighbor_candidates = static_cast<std::uint64_t>(summary.max_polarization * 1'000'000.0);
    return metrics;
}

CsvMetricsWriter::CsvMetricsWriter(std::filesystem::path output_path)
{
    open(output_path);
}

bool CsvMetricsWriter::open(const std::filesystem::path& output_path)
{
    close();
    output_path_ = output_path;
    if (output_path_.has_parent_path()) {
        std::filesystem::create_directories(output_path_.parent_path());
    }
    stream_.open(output_path_);
    if (stream_.is_open()) {
        stream_ << header() << '\n';
    }
    return stream_.is_open();
}

void CsvMetricsWriter::close()
{
    if (stream_.is_open()) {
        stream_.close();
    }
}

void CsvMetricsWriter::write_sample(const SampleMetadata& metadata, const sim::SimulationMetrics& metrics)
{
    if (!stream_.is_open()) {
        return;
    }

    stream_ << std::setprecision(10);
    write_csv_value(stream_, metadata.scenario);
    stream_ << ',' << metadata.seed << ',';
    write_csv_value(stream_, metadata.timestamp);
    stream_ << ',';
    write_csv_value(stream_, metadata.git_commit);
    stream_ << ',';
    write_csv_value(stream_, to_string(metadata.export_mode));
    stream_ << ',' << metadata.sample_rate_hz << ',' << metadata.sample_index << ',' << metadata.simulation_time << ','
            << metadata.boid_count << ',' << metrics.polarization << ',' << metrics.cohesion << ',' << metrics.dispersion
            << ',' << metrics.average_speed << ',' << metrics.average_neighbors_per_boid << ','
            << metrics.nearest_neighbor_average_distance << ',' << metrics.simulation_update_ms << ','
            << metrics.neighbor_queries << ',' << metrics.spatial_hash_cell_count << ',' << metrics.mean_altitude << ','
            << metrics.altitude_variance << ',' << metrics.stall_count << ',' << metrics.near_ground_count << ',';
    write_csv_value(stream_, metadata.sweep_parameter);
    stream_ << ',';
    write_csv_value(stream_, metadata.sweep_value);
    stream_ << '\n';
}

constexpr std::string_view CsvMetricsWriter::header()
{
    return csv_header;
}

void MetricsRecorder::start(
    const std::filesystem::path& output_path,
    sim::ScenarioType scenario,
    std::uint32_t seed,
    ExportMode mode,
    double sample_rate_hz)
{
    stop(0.0, 0, sim::SimulationMetrics{});
    scenario_ = scenario;
    seed_ = seed;
    export_mode_ = mode;
    scheduler_.set_sample_rate_hz(sample_rate_hz);
    scheduler_.reset();
    summary_ = SummaryAggregator{};
    timestamp_ = timestamp_for_filename();
    git_commit_ = current_git_commit();
    recording_ = writer_.open(output_path);
}

void MetricsRecorder::stop(double simulation_time, std::size_t boid_count, const sim::SimulationMetrics& latest_metrics)
{
    if (!recording_) {
        return;
    }
    if (export_mode_ == ExportMode::Summary) {
        write_summary(simulation_time, boid_count, latest_metrics);
    }
    writer_.close();
    recording_ = false;
}

void MetricsRecorder::record_step(double simulation_time, std::size_t boid_count, const sim::SimulationMetrics& metrics)
{
    if (!recording_) {
        return;
    }
    if (export_mode_ == ExportMode::FullTrajectory) {
        return;
    }
    if (!scheduler_.should_sample(simulation_time)) {
        return;
    }

    const auto sample_index = scheduler_.consume_sample();
    if (export_mode_ == ExportMode::Summary) {
        summary_.add_sample(metrics);
        return;
    }

    writer_.write_sample(
        SampleMetadata{
            scenario_display_name(scenario_),
            seed_,
            timestamp_,
            git_commit_,
            export_mode_,
            scheduler_.sample_rate_hz(),
            sample_index,
            simulation_time,
            boid_count,
            {},
            {},
        },
        metrics);
}

void MetricsRecorder::set_export_mode(ExportMode mode) noexcept
{
    if (!recording_) {
        export_mode_ = mode;
    }
}

void MetricsRecorder::set_sample_rate_hz(double sample_rate_hz) noexcept
{
    if (!recording_) {
        scheduler_.set_sample_rate_hz(sample_rate_hz);
    }
}

void MetricsRecorder::write_summary(double simulation_time, std::size_t boid_count, const sim::SimulationMetrics& fallback_metrics)
{
    auto aggregate = summary_.aggregate(simulation_time);
    if (summary_.sample_count() == 0) {
        aggregate = fallback_metrics;
        aggregate.simulation_update_ms = simulation_time;
    }
    aggregate.neighbor_candidates = std::max<std::uint64_t>(aggregate.neighbor_candidates, 0U);
    writer_.write_sample(
        SampleMetadata{
            scenario_display_name(scenario_),
            seed_,
            timestamp_,
            git_commit_,
            export_mode_,
            scheduler_.sample_rate_hz(),
            summary_.sample_count(),
            simulation_time,
            boid_count,
            {},
            {},
        },
        aggregate);
}

std::string_view to_string(ExportMode mode) noexcept
{
    switch (mode) {
    case ExportMode::Summary:
        return "Summary";
    case ExportMode::SampledTimeSeries:
        return "SampledTimeSeries";
    case ExportMode::FullTrajectory:
        return "FullTrajectory";
    }
    return "SampledTimeSeries";
}

std::optional<ExportMode> parse_export_mode(std::string_view text) noexcept
{
    if (text == "summary" || text == "Summary") {
        return ExportMode::Summary;
    }
    if (text == "sampled" || text == "SampledTimeSeries" || text == "sampled_time_series") {
        return ExportMode::SampledTimeSeries;
    }
    if (text == "full" || text == "FullTrajectory" || text == "full_trajectory") {
        return ExportMode::FullTrajectory;
    }
    return std::nullopt;
}

std::string timestamp_for_filename()
{
    const auto now = std::chrono::system_clock::now();
    const auto time = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
#if defined(_WIN32)
    gmtime_s(&tm, &time);
#else
    gmtime_r(&time, &tm);
#endif
    std::ostringstream stream;
    stream << std::put_time(&tm, "%Y%m%dT%H%M%SZ");
    return stream.str();
}

std::string current_git_commit()
{
#ifdef FLOCK3D_GIT_COMMIT
    return FLOCK3D_GIT_COMMIT;
#else
    return "unknown";
#endif
}

std::filesystem::path default_output_path(sim::ScenarioType scenario, ExportMode mode)
{
    std::string filename{"flock3d_"};
    filename += scenario_cli_name(scenario);
    filename += '_';
    filename += to_string(mode);
    filename += '_';
    filename += timestamp_for_filename();
    filename += ".csv";
    return std::filesystem::path{"outputs"} / filename;
}

} // namespace flock3d::experiment
