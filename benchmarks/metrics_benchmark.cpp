#include "benchmark_common.hpp"

#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <string_view>

#include <flock3d/sim/BoidSimulation.hpp>
#include <flock3d/sim/SimulationMetrics.hpp>

namespace {

using flock3d::bench::BenchmarkOptions;
using flock3d::bench::Clock;
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

    const auto warmup_start = Clock::now();
    while (std::chrono::duration<double>(Clock::now() - warmup_start).count() < options.warmup_seconds) {
        simulation.update(flock3d::bench::fixed_dt, metrics_enabled ? &metrics : nullptr);
    }

    const auto benchmark_start = Clock::now();
    auto sample_start = benchmark_start;
    int sample_index = 0;

    while (std::chrono::duration<double>(Clock::now() - benchmark_start).count() < options.duration_seconds) {
        UpdateStats stats{};
        while (true) {
            const auto now = Clock::now();
            const double elapsed = std::chrono::duration<double>(now - benchmark_start).count();
            const double sample_elapsed = std::chrono::duration<double>(now - sample_start).count();
            if (elapsed >= options.duration_seconds || (stats.count > 0 && sample_elapsed >= options.sample_seconds)) {
                break;
            }

            const double milliseconds = flock3d::bench::time_ms([&simulation, &metrics, metrics_enabled]() {
                simulation.update(flock3d::bench::fixed_dt, metrics_enabled ? &metrics : nullptr);
            });
            stats.record(milliseconds);

            if ((stats.count % 64U) == 0U) {
                progress.update("metrics", metric_mode, boid_count, elapsed, options.duration_seconds);
            }
        }

        const double elapsed = std::chrono::duration<double>(Clock::now() - benchmark_start).count();
        std::cout << "baseline," << metric_mode << ',' << boid_count << ',' << std::fixed << std::setprecision(3)
                  << elapsed << ',' << sample_index << ',' << stats.count << ',' << stats.mean_ms() << ','
                  << stats.min_or_zero() << ',' << stats.max_ms << '\n';
        ++sample_index;
        sample_start = Clock::now();
    }
    progress.finish();
}

} // namespace

int main(int argc, char** argv)
{
    const BenchmarkOptions options = flock3d::bench::parse_options(argc, argv);
    ProgressBar progress{flock3d::bench::progress_enabled()};

    std::cout << "scenario,metric_mode,boid_count,elapsed_seconds,sample_index,iterations_in_sample,mean_update_ms,min_update_ms,max_update_ms\n";
    for (const std::uint32_t boid_count : flock3d::bench::benchmark_boid_counts()) {
        run_scenario("no_metrics", false, boid_count, options, progress);
        run_scenario("metrics_pointer", true, boid_count, options, progress);
    }

    return 0;
}
