#include "simulation_ticks_benchmark.hpp"

#include <algorithm>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>

#include <flock3d/sim/BoidSimulation.hpp>
#include <flock3d/sim/Scenario.hpp>
#include <flock3d/sim/SimulationParameters.hpp>

namespace
{

using flock3d::bench::ticks::TickBenchmarkOptions;

void print_usage(std::string_view executable)
{
    std::cerr << "Usage: " << executable
              << " [--counts 128,256,512,1024] [--warmup seconds] [--measured seconds]"
                 " [--dt seconds | --tick-rate hz] [--repetitions count] [--seed "
                 "value]\n"
              << "CSV is printed to stdout. The benchmark advances fixed simulation "
                 "ticks as fast as possible;"
                 " it does not wait for wall-clock benchmark duration.\n";
}

std::vector<std::uint32_t> parse_counts(std::string_view text)
{
    std::vector<std::uint32_t> counts{};
    std::stringstream stream{std::string{text}};
    std::string item{};
    while (std::getline(stream, item, ','))
    {
        const auto value = static_cast<unsigned long>(std::strtoul(item.c_str(), nullptr, 10));
        if (value > 0UL)
        {
            counts.push_back(static_cast<std::uint32_t>(value));
        }
    }
    if (counts.empty())
    {
        throw std::invalid_argument{"--counts must include at least one positive boid count"};
    }
    return counts;
}

TickBenchmarkOptions parse_options(int argc, char **argv)
{
    TickBenchmarkOptions options{};
    for (int index = 1; index < argc; ++index)
    {
        const std::string_view argument{argv[index]};
        if (argument == "--help" || argument == "-h")
        {
            print_usage(argv[0]);
            std::exit(EXIT_SUCCESS);
        }
        if (index + 1 >= argc)
        {
            print_usage(argv[0]);
            std::exit(EXIT_FAILURE);
        }

        const std::string_view value{argv[++index]};
        if (argument == "--counts")
        {
            options.boid_counts = parse_counts(value);
        }
        else if (argument == "--warmup")
        {
            options.warmup_seconds = std::max(0.0, std::atof(value.data()));
        }
        else if (argument == "--measured" || argument == "--duration")
        {
            options.measured_seconds = std::max(0.0, std::atof(value.data()));
        }
        else if (argument == "--sample")
        {
            // Accepted for compatibility with scripts/run_benchmark.sh; fixed-tick
            // benchmarks emit one summary row per repetition.
        }
        else if (argument == "--dt")
        {
            options.dt = std::atof(value.data());
        }
        else if (argument == "--tick-rate")
        {
            const double tick_rate = std::atof(value.data());
            options.dt = tick_rate > 0.0 ? 1.0 / tick_rate : 0.0;
        }
        else if (argument == "--repetitions")
        {
            options.repetitions = static_cast<std::uint32_t>(std::max(1UL, std::strtoul(value.data(), nullptr, 10)));
        }
        else if (argument == "--seed")
        {
            options.seed = static_cast<std::uint32_t>(std::strtoul(value.data(), nullptr, 10));
        }
        else
        {
            print_usage(argv[0]);
            std::exit(EXIT_FAILURE);
        }
    }

    if (options.dt <= 0.0)
    {
        throw std::invalid_argument{"--dt must be positive"};
    }
    return options;
}

flock3d::sim::SimulationParameters benchmark_parameters(std::uint32_t boid_count, std::uint32_t seed)
{
    auto parameters = flock3d::sim::build_scenario(flock3d::sim::ScenarioType::ClassicBoids).simulation_parameters;
    parameters.boid_count = boid_count;
    parameters.random_seed = seed;
    flock3d::sim::sync_spatial_cell_size_to_query_radius(parameters);
    return parameters;
}

void print_header()
{
    std::cout << "scenario,boid_count,repetition,seed,dt,warmup_seconds,warmup_"
                 "ticks,simulated_seconds,"
                 "measured_ticks,total_wall_seconds,average_ms_per_tick,p50_ms_"
                 "per_tick,p95_ms_per_tick,"
                 "max_ms_per_tick,ticks_per_second,real_time_factor,ticks_in_sample,simulated_ticks,wall_seconds,mean_ns_per_tick,updates_per_second\n";
}

void run_count(std::uint32_t boid_count, const TickBenchmarkOptions &options)
{
    const std::size_t warmup_ticks = flock3d::bench::ticks::seconds_to_ticks(options.warmup_seconds, options.dt);
    const std::size_t measured_ticks = flock3d::bench::ticks::seconds_to_ticks(options.measured_seconds, options.dt);

    for (std::uint32_t repetition = 0U; repetition < options.repetitions; ++repetition)
    {
        const auto parameters = benchmark_parameters(boid_count, options.seed + boid_count);
        flock3d::sim::BoidSimulation simulation{parameters};

        auto update = [&simulation](double dt) { simulation.update(static_cast<float>(dt), nullptr); };
        auto measure = [](double dt, auto &update_fn) { return flock3d::bench::ticks::time_update_ms(dt, update_fn); };

        const auto durations =
            flock3d::bench::ticks::run_fixed_tick_benchmark(warmup_ticks, measured_ticks, options.dt, update, measure);
        const auto summary = flock3d::bench::ticks::summarize_ticks(durations.measured_tick_ms, options.dt,
                                                                    durations.measured_wall_seconds);

        std::cout << "fixed_tick_simulation," << boid_count << ',' << repetition << ',' << parameters.random_seed << ','
                  << std::fixed << std::setprecision(9) << options.dt << ',' << std::setprecision(3)
                  << options.warmup_seconds << ',' << warmup_ticks << ',' << summary.simulated_seconds << ','
                  << summary.tick_count << ',' << std::setprecision(6) << summary.total_wall_seconds << ','
                  << summary.average_ms_per_tick << ',' << summary.p50_ms_per_tick << ',' << summary.p95_ms_per_tick
                  << ',' << summary.max_ms_per_tick << ',' << summary.ticks_per_second << ','
                  << summary.real_time_factor << ',' << summary.tick_count << ',' << summary.tick_count << ','
                  << summary.total_wall_seconds << ',' << summary.mean_ns_per_tick << ','
                  << summary.updates_per_second << '\n';
    }
}

} // namespace

int main(int argc, char **argv)
{
    try
    {
        const TickBenchmarkOptions options = parse_options(argc, argv);
        print_header();
        for (const std::uint32_t boid_count : options.boid_counts)
        {
            run_count(boid_count, options);
        }
    }
    catch (const std::exception &error)
    {
        std::cerr << "benchmark error: " << error.what() << '\n';
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
