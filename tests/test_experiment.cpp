#include <flock3d/experiment/ExperimentRunner.hpp>
#include <flock3d/experiment/MetricsExport.hpp>
#include <flock3d/sim/BoidSimulation.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

TEST_CASE("CSV metrics writer exposes required header", "[experiment][csv]")
{
    CHECK(flock3d::experiment::CsvMetricsWriter::header()
        == "scenario,seed,timestamp,git_commit,export_mode,sample_rate_hz,sample_index,simulation_time,boid_count,polarization,cohesion,dispersion,average_speed,average_neighbors,nearest_neighbor_distance,simulation_update_ms,neighbor_queries,spatial_cell_count,sweep_parameter,sweep_value");
}

TEST_CASE("SampleScheduler samples independently from fixed dt", "[experiment][sampling]")
{
    flock3d::experiment::SampleScheduler scheduler{5.0};
    double simulation_time = 0.0;
    int samples = 0;
    for (int step = 0; step < 120; ++step) {
        simulation_time += 1.0 / 120.0;
        if (scheduler.should_sample(simulation_time)) {
            CHECK(scheduler.consume_sample() == static_cast<std::size_t>(samples));
            ++samples;
        }
    }

    CHECK(samples == 5);
}

TEST_CASE("SummaryAggregator computes macroscopic aggregate fields", "[experiment][summary]")
{
    flock3d::experiment::SummaryAggregator aggregator{};
    flock3d::sim::SimulationMetrics first{};
    first.polarization = 0.25F;
    first.cohesion = 10.0F;
    first.dispersion = 11.0F;
    first.average_speed = 3.0F;
    first.average_neighbors_per_boid = 4.0F;
    flock3d::sim::SimulationMetrics second{};
    second.polarization = 0.75F;
    second.cohesion = 14.0F;
    second.dispersion = 20.0F;
    second.average_speed = 5.0F;
    second.average_neighbors_per_boid = 8.0F;

    aggregator.add_sample(first);
    aggregator.add_sample(second);

    const auto summary = aggregator.statistics(30.0);
    CHECK(summary.mean_polarization == Catch::Approx(0.5));
    CHECK(summary.max_polarization == Catch::Approx(0.75));
    CHECK(summary.mean_cohesion == Catch::Approx(12.0));
    CHECK(summary.max_dispersion == Catch::Approx(20.0));
    CHECK(summary.mean_average_speed == Catch::Approx(4.0));
    CHECK(summary.mean_average_neighbors == Catch::Approx(6.0));
    CHECK(summary.total_duration_seconds == Catch::Approx(30.0));
}

TEST_CASE("First sampled metrics are deterministic for seed and dt", "[experiment][determinism]")
{
    flock3d::sim::SimulationParameters parameters{};
    parameters.random_seed = 123U;
    parameters.boid_count = 64U;

    auto first = parameters;
    auto second = parameters;
    flock3d::sim::BoidSimulation first_sim{first};
    flock3d::sim::BoidSimulation second_sim{second};
    flock3d::sim::SimulationMetrics first_metrics{};
    flock3d::sim::SimulationMetrics second_metrics{};
    constexpr double dt = 1.0 / 120.0;
    flock3d::experiment::SampleScheduler first_scheduler{5.0};
    flock3d::experiment::SampleScheduler second_scheduler{5.0};

    double time = 0.0;
    while (!first_scheduler.should_sample(time)) {
        first_sim.update(static_cast<float>(dt), &first_metrics);
        second_sim.update(static_cast<float>(dt), &second_metrics);
        time += dt;
    }

    CHECK(first_metrics.polarization == second_metrics.polarization);
    CHECK(first_metrics.cohesion == second_metrics.cohesion);
    CHECK(first_metrics.dispersion == second_metrics.dispersion);
    CHECK(first_metrics.average_speed == second_metrics.average_speed);
    CHECK(first_scheduler.consume_sample() == second_scheduler.consume_sample());
}

TEST_CASE("Scenario lookup accepts CLI and display names", "[experiment][scenario]")
{
    CHECK(flock3d::sim::scenario_type_from_name("ClassicBoids") == flock3d::sim::ScenarioType::ClassicBoids);
    CHECK(flock3d::sim::scenario_type_from_name("classic boids") == flock3d::sim::ScenarioType::ClassicBoids);
    CHECK(flock3d::sim::scenario_type_from_name("Predator-Prey") == flock3d::sim::ScenarioType::PredatorPrey);
    CHECK_FALSE(flock3d::sim::scenario_type_from_name("unknown").has_value());
}

TEST_CASE("Sweep parser expands inclusive numeric ranges", "[experiment][sweep]")
{
    const auto sweep = flock3d::experiment::parse_sweep("alignment_weight=0:1:0.5");
    REQUIRE(sweep.has_value());
    CHECK(sweep->parameter == "alignment_weight");
    const auto values = flock3d::experiment::sweep_values(*sweep);
    REQUIRE(values.size() == 3);
    CHECK(values[0] == Catch::Approx(0.0));
    CHECK(values[1] == Catch::Approx(0.5));
    CHECK(values[2] == Catch::Approx(1.0));
    CHECK_FALSE(flock3d::experiment::parse_sweep("alignment_weight=0:1:0").has_value());
}
