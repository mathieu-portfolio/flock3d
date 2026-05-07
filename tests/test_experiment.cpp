#include <flock3d/experiment/ExperimentRunner.hpp>
#include <flock3d/experiment/MetricsExport.hpp>
#include <flock3d/sim/BoidSimulation.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

TEST_CASE("CSV metrics writer exposes required header", "[experiment][csv]")
{
    CHECK(flock3d::experiment::CsvMetricsWriter::header()
        == "scenario,seed,timestamp,git_commit,export_mode,sample_rate_hz,sample_index,simulation_time,boid_count,polarization,cohesion,dispersion,average_speed,average_neighbors,nearest_neighbor_distance,simulation_update_ms,neighbor_queries,spatial_cell_count,mean_depth,depth_variance,mean_altitude,altitude_variance,stall_count,near_ground_count,noise_strength,order_loss,sweep_parameter,sweep_value");
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
    first.mean_depth = -8.0F;
    first.depth_variance = 2.0F;
    flock3d::sim::SimulationMetrics second{};
    second.polarization = 0.75F;
    second.cohesion = 14.0F;
    second.dispersion = 20.0F;
    second.average_speed = 5.0F;
    second.average_neighbors_per_boid = 8.0F;
    second.mean_depth = -12.0F;
    second.depth_variance = 4.0F;

    aggregator.add_sample(first);
    aggregator.add_sample(second);

    const auto summary = aggregator.statistics(30.0);
    CHECK(summary.mean_polarization == Catch::Approx(0.5));
    CHECK(summary.max_polarization == Catch::Approx(0.75));
    CHECK(summary.mean_cohesion == Catch::Approx(12.0));
    CHECK(summary.max_dispersion == Catch::Approx(20.0));
    CHECK(summary.mean_average_speed == Catch::Approx(4.0));
    CHECK(summary.mean_average_neighbors == Catch::Approx(6.0));
    CHECK(summary.mean_depth == Catch::Approx(-10.0));
    CHECK(summary.mean_depth_variance == Catch::Approx(3.0));
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


TEST_CASE("Experiment radius sweeps keep spatial cells aligned with query radius", "[experiment][sweep]")
{
    flock3d::sim::SimulationParameters parameters{};

    CHECK(flock3d::experiment::apply_sweep_value(parameters, "neighbor_radius", 6.0));
    CHECK(parameters.neighbor_radius == Catch::Approx(6.0F));
    CHECK(parameters.spatial_cell_size == Catch::Approx(6.0F));

    CHECK(flock3d::experiment::apply_sweep_value(parameters, "separation_radius", 8.0));
    CHECK(parameters.separation_radius == Catch::Approx(8.0F));
    CHECK(parameters.spatial_cell_size == Catch::Approx(8.0F));
}

TEST_CASE("Experiment sweeps BirdFlight constraint parameters", "[experiment][sweep][birdflight]")
{
    flock3d::sim::SimulationParameters parameters{};
    CHECK(flock3d::experiment::apply_sweep_value(parameters, "gravity", 8.5));
    CHECK(parameters.gravity == Catch::Approx(8.5F));
    CHECK(flock3d::experiment::apply_sweep_value(parameters, "max_turn_rate", 95.0));
    CHECK(parameters.max_turn_rate == Catch::Approx(95.0F));
    CHECK(flock3d::experiment::apply_sweep_value(parameters, "field_of_view_degrees", 210.0));
    CHECK(parameters.field_of_view_degrees == Catch::Approx(210.0F));
    CHECK(flock3d::experiment::apply_sweep_value(parameters, "altitude_correction_strength", 2.25));
    CHECK(parameters.altitude_correction_strength == Catch::Approx(2.25F));
}


TEST_CASE("BirdFlight experiment presets are discoverable", "[experiment][preset][birdflight]")
{
    const auto names = flock3d::experiment::experiment_preset_names();
    CHECK(names.size() == 13);
    CHECK(flock3d::experiment::experiment_preset("bird_baseline").has_value());
    CHECK(flock3d::experiment::experiment_preset("bird_low_lift").has_value());
    CHECK(flock3d::experiment::experiment_preset("bird_high_gravity").has_value());
    CHECK(flock3d::experiment::experiment_preset("bird_narrow_fov").has_value());
    CHECK(flock3d::experiment::experiment_preset("bird_low_turn_rate").has_value());
    CHECK_FALSE(flock3d::experiment::experiment_preset("unknown").has_value());
}

TEST_CASE("BirdFlight preset initialization is deterministic", "[experiment][preset][birdflight]")
{
    flock3d::experiment::ExperimentConfig first{};
    flock3d::experiment::ExperimentConfig second{};

    REQUIRE(flock3d::experiment::apply_experiment_preset(first, "bird_high_gravity"));
    REQUIRE(flock3d::experiment::apply_experiment_preset(second, "bird_high_gravity"));

    const auto first_parameters = flock3d::experiment::experiment_parameters(first);
    const auto second_parameters = flock3d::experiment::experiment_parameters(second);
    CHECK(first.scenario == flock3d::sim::ScenarioType::BirdFlight);
    CHECK(first_parameters.model == flock3d::sim::SimulationModel::BirdFlight);
    CHECK(first_parameters.random_seed == second_parameters.random_seed);
    CHECK(first_parameters.gravity == second_parameters.gravity);
    CHECK(first_parameters.lift_strength == second_parameters.lift_strength);
    CHECK(first_parameters.field_of_view_degrees == second_parameters.field_of_view_degrees);
    CHECK(first_parameters.max_turn_rate == second_parameters.max_turn_rate);
}

TEST_CASE("CLI overrides take precedence after preset defaults", "[experiment][preset][cli]")
{
    std::string error{};
    char program[] = "flock3d_experiment_runner";
    char preset_flag[] = "--preset";
    char preset_name[] = "bird_high_gravity";
    char seed_flag[] = "--seed";
    char seed_value[] = "77";
    char boids_flag[] = "--boids";
    char boids_value[] = "96";
    char sweep_flag[] = "--sweep";
    char sweep_value[] = "gravity=7:7:1";
    char* argv[] = {
        program,
        preset_flag,
        preset_name,
        seed_flag,
        seed_value,
        boids_flag,
        boids_value,
        sweep_flag,
        sweep_value,
    };

    auto config = flock3d::experiment::parse_cli(static_cast<int>(std::size(argv)), argv, error);
    REQUIRE(config.has_value());
    CHECK(config->scenario == flock3d::sim::ScenarioType::BirdFlight);
    CHECK(config->seed == 77U);
    CHECK(config->boids == 96U);
    REQUIRE(config->sweep.has_value());
    CHECK(config->sweep->parameter == "gravity");

    auto parameters = flock3d::experiment::experiment_parameters(*config);
    CHECK(parameters.gravity == Catch::Approx(12.5F));
    REQUIRE(flock3d::experiment::apply_sweep_value(parameters, config->sweep->parameter, config->sweep->start));
    CHECK(parameters.gravity == Catch::Approx(7.0F));
}

TEST_CASE("BirdFlight sampled and summary exports include stability metrics", "[experiment][csv][birdflight]")
{
    const auto output_dir = std::filesystem::temp_directory_path() / "flock3d_test_exports";
    std::filesystem::create_directories(output_dir);

    flock3d::experiment::ExperimentConfig sampled{};
    REQUIRE(flock3d::experiment::apply_experiment_preset(sampled, "bird_baseline"));
    sampled.seed = 11U;
    sampled.boids = 16U;
    sampled.duration_seconds = 0.25;
    sampled.sample_rate_hz = 4.0;
    sampled.output_path = output_dir / "bird_sampled.csv";
    sampled.export_mode = flock3d::experiment::ExportMode::SampledTimeSeries;

    const auto sampled_result = flock3d::experiment::run_experiment(sampled);
    CHECK(sampled_result.rows_written == 1U);

    std::ifstream sampled_stream{sampled.output_path};
    REQUIRE(sampled_stream.is_open());
    std::string sampled_header{};
    std::string sampled_row{};
    std::getline(sampled_stream, sampled_header);
    std::getline(sampled_stream, sampled_row);
    CHECK(sampled_header.find("mean_altitude,altitude_variance,stall_count") != std::string::npos);
    CHECK(sampled_row.find("Bird Flight") != std::string::npos);

    auto summary = sampled;
    summary.output_path = output_dir / "bird_summary.csv";
    summary.export_mode = flock3d::experiment::ExportMode::Summary;
    const auto summary_result = flock3d::experiment::run_experiment(summary);
    CHECK(summary_result.rows_written == 1U);

    std::ifstream summary_stream{summary.output_path};
    REQUIRE(summary_stream.is_open());
    std::string summary_header{};
    std::string summary_row{};
    std::getline(summary_stream, summary_header);
    std::getline(summary_stream, summary_row);
    CHECK(summary_header.find("mean_altitude,altitude_variance,stall_count") != std::string::npos);
    CHECK(summary_row.find("Summary") != std::string::npos);
}

TEST_CASE("FishSchool experiment presets are discoverable and overridable", "[experiment][preset][fishschool]")
{
    const auto names = flock3d::experiment::experiment_preset_names();
    CHECK(names.size() == 13);
    CHECK(flock3d::experiment::experiment_preset("fish_baseline").has_value());
    CHECK(flock3d::experiment::experiment_preset("fish_high_drag").has_value());
    CHECK(flock3d::experiment::experiment_preset("fish_strong_current").has_value());
    CHECK(flock3d::experiment::experiment_preset("fish_low_visibility").has_value());

    flock3d::experiment::ExperimentConfig config{};
    REQUIRE(flock3d::experiment::apply_experiment_preset(config, "fish_high_drag"));
    CHECK(config.scenario == flock3d::sim::ScenarioType::FishSchool);

    auto parameters = flock3d::experiment::experiment_parameters(config);
    CHECK(parameters.model == flock3d::sim::SimulationModel::FishSchool);
    CHECK(parameters.drag_coefficient == Catch::Approx(0.75F));
    REQUIRE(flock3d::experiment::apply_sweep_value(parameters, "drag_coefficient", 0.2));
    CHECK(parameters.drag_coefficient == Catch::Approx(0.2F));
}

TEST_CASE("NoiseExperiment presets and sweeps are discoverable", "[experiment][preset][noise]")
{
    const auto names = flock3d::experiment::experiment_preset_names();
    CHECK(names.size() == 13);
    CHECK(flock3d::experiment::experiment_preset("noise_baseline").has_value());
    CHECK(flock3d::experiment::experiment_preset("noise_low").has_value());
    CHECK(flock3d::experiment::experiment_preset("noise_medium").has_value());
    CHECK(flock3d::experiment::experiment_preset("noise_high").has_value());

    flock3d::experiment::ExperimentConfig config{};
    REQUIRE(flock3d::experiment::apply_experiment_preset(config, "noise_medium"));
    CHECK(config.scenario == flock3d::sim::ScenarioType::NoiseExperiment);

    auto parameters = flock3d::experiment::experiment_parameters(config);
    CHECK(parameters.model == flock3d::sim::SimulationModel::NoiseExperiment);
    CHECK(parameters.noise_enabled);
    REQUIRE(flock3d::experiment::apply_sweep_value(parameters, "perception_noise_strength", 0.25));
    REQUIRE(flock3d::experiment::apply_sweep_value(parameters, "steering_noise_strength", 0.5));
    CHECK(parameters.perception_noise_strength == Catch::Approx(0.25F));
    CHECK(parameters.steering_noise_strength == Catch::Approx(0.5F));
}

TEST_CASE("NoiseExperiment sampled exports include robustness metrics", "[experiment][csv][noise]")
{
    const auto output_dir = std::filesystem::temp_directory_path() / "flock3d_test_exports";
    std::filesystem::create_directories(output_dir);

    flock3d::experiment::ExperimentConfig config{};
    REQUIRE(flock3d::experiment::apply_experiment_preset(config, "noise_low"));
    config.seed = 21U;
    config.boids = 16U;
    config.duration_seconds = 0.25;
    config.sample_rate_hz = 4.0;
    config.output_path = output_dir / "noise_sampled.csv";
    config.export_mode = flock3d::experiment::ExportMode::SampledTimeSeries;

    const auto result = flock3d::experiment::run_experiment(config);
    CHECK(result.rows_written == 1U);

    std::ifstream stream{config.output_path};
    REQUIRE(stream.is_open());
    std::string header{};
    std::string row{};
    std::getline(stream, header);
    std::getline(stream, row);
    CHECK(header.find("noise_strength,order_loss") != std::string::npos);
    CHECK(row.find("Noise Experiment") != std::string::npos);
}

TEST_CASE("FishSchool sampled exports include depth metrics", "[experiment][csv][fishschool]")
{
    const auto output_dir = std::filesystem::temp_directory_path() / "flock3d_test_exports";
    std::filesystem::create_directories(output_dir);

    flock3d::experiment::ExperimentConfig config{};
    REQUIRE(flock3d::experiment::apply_experiment_preset(config, "fish_baseline"));
    config.seed = 12U;
    config.boids = 16U;
    config.duration_seconds = 0.25;
    config.sample_rate_hz = 4.0;
    config.output_path = output_dir / "fish_sampled.csv";
    config.export_mode = flock3d::experiment::ExportMode::SampledTimeSeries;

    const auto result = flock3d::experiment::run_experiment(config);
    CHECK(result.rows_written == 1U);

    std::ifstream stream{config.output_path};
    REQUIRE(stream.is_open());
    std::string header{};
    std::string row{};
    std::getline(stream, header);
    std::getline(stream, row);
    CHECK(header.find("mean_depth,depth_variance") != std::string::npos);
    CHECK(row.find("Fish School") != std::string::npos);
}
