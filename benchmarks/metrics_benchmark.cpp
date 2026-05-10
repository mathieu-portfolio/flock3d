#include "benchmark_common.hpp"

#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <string_view>

#include <flock3d/sim/BoidSimulation.hpp>
#include <flock3d/sim/SimulationMetrics.hpp>

namespace {

using flock3d::bench::BenchmarkOptions;
using flock3d::bench::ProgressBar;
using flock3d::bench::UpdateStats;

void run_scenario(std::string_view metric_mode, bool metrics_enabled, std::uint32_t boid_count, const BenchmarkOptions& options, ProgressBar& progress)
{
    auto parameters = flock3d::bench::parameters_for_model(
        flock3d::sim::SimulationModel::ClassicBoids,
        boid_count,
        22'000U + boid_count + (metrics_enabled ? 1U : 0U));
    flock3d::sim::BoidSimulation simulation{parameters};
    flock3d::sim::SimulationMetrics metrics{};

    const std::size_t warmup_ticks = flock3d::bench::simulated_seconds_to_ticks(options.warmup_seconds);
    for (std::size_t tick = 0; tick < warmup_ticks; ++tick) {
        simulation.update(flock3d::bench::fixed_dt, metrics_enabled ? &metrics : nullptr);
    }

    const std::size_t total_ticks = flock3d::bench::simulated_seconds_to_ticks(options.duration_seconds);
    const std::size_t sample_ticks = std::max<std::size_t>(1U, flock3d::bench::simulated_seconds_to_ticks(options.sample_seconds));
    std::size_t completed_ticks = 0U;
    int sample_index = 0;

    while (completed_ticks < total_ticks) {
        UpdateStats stats{};
        stats.samples_ms.reserve(sample_ticks);
        const std::size_t sample_start_tick = completed_ticks;
        while (completed_ticks < total_ticks && (stats.count == 0U || completed_ticks - sample_start_tick < sample_ticks)) {
            const double milliseconds = flock3d::bench::time_ms([&]() {
                simulation.update(flock3d::bench::fixed_dt, metrics_enabled ? &metrics : nullptr);
            });
            stats.record(milliseconds);
            ++completed_ticks;

            const double elapsed = flock3d::bench::ticks_to_simulated_seconds(completed_ticks);
            if ((stats.count % 64U) == 0U) {
                progress.update("metrics", metric_mode, boid_count, elapsed, options.duration_seconds);
            }
        }

        const double elapsed = flock3d::bench::ticks_to_simulated_seconds(completed_ticks);
        const std::size_t ticks_in_sample = stats.count;
        const std::size_t simulated_ticks = completed_ticks;
        std::cout << "baseline," << metric_mode << ',' << boid_count << ',' << std::fixed << std::setprecision(3)
                  << elapsed << ',' << sample_index << ',' << stats.count << ',' << stats.mean_ms() << ','
                  << stats.min_or_zero() << ',' << stats.max_ms << ',' << elapsed << ',' << simulated_ticks << ','
                  << ticks_in_sample << ',' << stats.wall_seconds() << ',' << stats.mean_ns() << ','
                  << stats.p50_ms() << ',' << stats.p95_ms() << ',' << stats.p99_ms() << ',' << stats.ticks_per_second() << ','
                  << stats.ticks_per_second() << ',' << stats.real_time_factor() << '\n';
        ++sample_index;
    }
    progress.finish();
}

} // namespace

int main(int argc, char** argv)
{
    const BenchmarkOptions options = flock3d::bench::parse_options(argc, argv);
    ProgressBar progress{flock3d::bench::progress_enabled()};

    std::cout << "scenario,metric_mode,boid_count,elapsed_seconds,sample_index,iterations_in_sample,mean_update_ms,min_update_ms,max_update_ms,simulated_seconds,simulated_ticks,ticks_in_sample,sample_wall_seconds,mean_ns_per_tick,p50_update_ms,p95_update_ms,p99_update_ms,ticks_per_second,updates_per_second,real_time_factor\n";
    for (const std::uint32_t boid_count : flock3d::bench::benchmark_boid_counts()) {
        run_scenario("no_metrics", false, boid_count, options, progress);
        run_scenario("metrics_pointer", true, boid_count, options, progress);
    }

    return 0;
}
