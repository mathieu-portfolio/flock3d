#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <vector>

namespace flock3d::bench::ticks
{

using Clock = std::chrono::steady_clock;

inline constexpr double default_dt = 1.0 / 60.0;
inline constexpr double default_warmup_seconds = 5.0;
inline constexpr double default_measured_seconds = 25.0;
inline constexpr std::uint32_t default_seed = 12'345U;

struct TickBenchmarkOptions
{
    std::vector<std::uint32_t> boid_counts{128U, 256U, 512U, 1024U};
    double warmup_seconds{default_warmup_seconds};
    double measured_seconds{default_measured_seconds};
    double dt{default_dt};
    std::uint32_t repetitions{1U};
    std::uint32_t seed{default_seed};
};

struct TickDurations
{
    std::size_t warmup_updates{};
    double measured_wall_seconds{};
    std::vector<double> measured_tick_ms{};
};

struct TickSummary
{
    std::size_t tick_count{};
    double simulated_seconds{};
    double total_wall_seconds{};
    double average_ms_per_tick{};
    double mean_ns_per_tick{};
    double p50_ms_per_tick{};
    double p95_ms_per_tick{};
    double p99_ms_per_tick{};
    double max_ms_per_tick{};
    double ticks_per_second{};
    double updates_per_second{};
    double real_time_factor{};
};

[[nodiscard]] inline std::size_t seconds_to_ticks(double seconds, double dt)
{
    if (dt <= 0.0 || !std::isfinite(dt))
    {
        throw std::invalid_argument{"benchmark dt must be finite and positive"};
    }
    if (seconds <= 0.0)
    {
        return 0U;
    }
    return static_cast<std::size_t>(std::ceil(seconds / dt));
}

[[nodiscard]] inline double percentile_nearest_rank(std::vector<double> values, double percentile)
{
    if (values.empty())
    {
        return 0.0;
    }
    std::sort(values.begin(), values.end());
    const double clamped = std::clamp(percentile, 0.0, 1.0);
    if (clamped <= 0.0)
    {
        return values.front();
    }
    const auto rank = static_cast<std::size_t>(std::ceil(clamped * static_cast<double>(values.size())));
    const std::size_t index = std::min(values.size() - 1U, rank > 0U ? rank - 1U : 0U);
    return values[index];
}

[[nodiscard]] inline TickSummary summarize_ticks(const std::vector<double> &measured_tick_ms, double dt,
                                                 double measured_wall_seconds = -1.0)
{
    if (dt <= 0.0 || !std::isfinite(dt))
    {
        throw std::invalid_argument{"benchmark dt must be finite and positive"};
    }

    TickSummary summary{};
    summary.tick_count = measured_tick_ms.size();
    summary.simulated_seconds = static_cast<double>(summary.tick_count) * dt;
    const double total_ms = std::accumulate(measured_tick_ms.begin(), measured_tick_ms.end(), 0.0);
    summary.total_wall_seconds = measured_wall_seconds >= 0.0 ? measured_wall_seconds : total_ms / 1000.0;

    if (summary.tick_count == 0U)
    {
        return summary;
    }

    summary.average_ms_per_tick = total_ms / static_cast<double>(summary.tick_count);
    summary.mean_ns_per_tick = summary.average_ms_per_tick * 1'000'000.0;
    summary.p50_ms_per_tick = percentile_nearest_rank(measured_tick_ms, 0.50);
    summary.p95_ms_per_tick = percentile_nearest_rank(measured_tick_ms, 0.95);
    summary.p99_ms_per_tick = percentile_nearest_rank(measured_tick_ms, 0.99);
    summary.max_ms_per_tick = *std::max_element(measured_tick_ms.begin(), measured_tick_ms.end());
    if (summary.total_wall_seconds > 0.0)
    {
        summary.ticks_per_second = static_cast<double>(summary.tick_count) / summary.total_wall_seconds;
        summary.updates_per_second = summary.ticks_per_second;
        summary.real_time_factor = summary.simulated_seconds / summary.total_wall_seconds;
    }
    else
    {
        summary.ticks_per_second = std::numeric_limits<double>::infinity();
        summary.updates_per_second = std::numeric_limits<double>::infinity();
        summary.real_time_factor = std::numeric_limits<double>::infinity();
    }
    return summary;
}

template <typename UpdateFn, typename MeasureFn>
TickDurations run_fixed_tick_benchmark(std::size_t warmup_ticks, std::size_t measured_ticks, double dt,
                                       UpdateFn &&update, MeasureFn &&measure_update_ms)
{
    TickDurations result{};
    result.measured_tick_ms.reserve(measured_ticks);

    for (std::size_t tick = 0; tick < warmup_ticks; ++tick)
    {
        update(dt);
        ++result.warmup_updates;
    }

    const auto measured_start = Clock::now();
    for (std::size_t tick = 0; tick < measured_ticks; ++tick)
    {
        result.measured_tick_ms.push_back(measure_update_ms(dt, update));
    }
    const auto measured_stop = Clock::now();
    result.measured_wall_seconds = std::chrono::duration<double>(measured_stop - measured_start).count();

    return result;
}

template <typename UpdateFn> [[nodiscard]] double time_update_ms(double dt, UpdateFn &&update)
{
    const auto start = Clock::now();
    update(dt);
    const auto stop = Clock::now();
    return std::chrono::duration<double, std::milli>(stop - start).count();
}

} // namespace flock3d::bench::ticks
