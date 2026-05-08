#include "benchmark_common.hpp"

#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>

#include <flock3d/sim/BoidSimulation.hpp>
#include <flock3d/sim/SimulationParameters.hpp>

namespace {

using flock3d::bench::BenchmarkOptions;
using flock3d::bench::Clock;
using flock3d::bench::ProgressBar;
using flock3d::bench::UpdateStats;

void run_scenario(flock3d::sim::SimulationModel model, std::uint32_t boid_count, const BenchmarkOptions& options, ProgressBar& progress)
{
    auto parameters = flock3d::bench::parameters_for_model(model, boid_count, 12'345U + boid_count);
    flock3d::sim::BoidSimulation simulation{parameters};

    const auto warmup_start = Clock::now();
    while (std::chrono::duration<double>(Clock::now() - warmup_start).count() < options.warmup_seconds) {
        simulation.update(flock3d::bench::fixed_dt, nullptr);
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

            const double milliseconds = flock3d::bench::time_ms([&simulation]() {
                simulation.update(flock3d::bench::fixed_dt, nullptr);
            });
            stats.record(milliseconds);

            if ((stats.count % 64U) == 0U) {
                progress.update(
                    "simulation_update",
                    flock3d::bench::model_name(model),
                    boid_count,
                    elapsed,
                    options.duration_seconds);
            }
        }

        const double elapsed = std::chrono::duration<double>(Clock::now() - benchmark_start).count();
        std::cout << "baseline," << flock3d::bench::model_name(model) << ',' << boid_count << ',' << std::fixed
                  << std::setprecision(3) << elapsed << ',' << sample_index << ',' << stats.count << ','
                  << stats.mean_ms() << ',' << stats.min_or_zero() << ',' << stats.max_ms << '\n';
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

    std::cout << "scenario,model,boid_count,elapsed_seconds,sample_index,iterations_in_sample,mean_update_ms,min_update_ms,max_update_ms\n";
    for (const auto model : {
             flock3d::sim::SimulationModel::ClassicBoids,
             flock3d::sim::SimulationModel::BirdFlight,
             flock3d::sim::SimulationModel::FishSchool,
             flock3d::sim::SimulationModel::NoiseExperiment,
         }) {
        for (const std::uint32_t boid_count : flock3d::bench::benchmark_boid_counts()) {
            run_scenario(model, boid_count, options, progress);
        }
    }

    return 0;
}
