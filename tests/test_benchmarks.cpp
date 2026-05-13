#include "../benchmarks/benchmark_common.hpp"
#include "../benchmarks/simulation_ticks_benchmark.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cstddef>
#include <stdexcept>
#include <vector>

TEST_CASE("Benchmark progress only requires an interactive stderr by default", "[benchmark][progress]")
{
    CHECK(flock3d::bench::progress_enabled_for_streams(true, true));
    CHECK(flock3d::bench::progress_enabled_for_streams(false, true));
    CHECK_FALSE(flock3d::bench::progress_enabled_for_streams(true, false));
    CHECK_FALSE(flock3d::bench::progress_enabled_for_streams(false, false));
}

TEST_CASE("Benchmark progress environment mode can override terminal detection", "[benchmark][progress]")
{
    CHECK(flock3d::bench::progress_enabled_for_mode("always", false));
    CHECK(flock3d::bench::progress_enabled_for_mode("1", false));
    CHECK(flock3d::bench::progress_enabled_for_mode("true", false));
    CHECK_FALSE(flock3d::bench::progress_enabled_for_mode("never", true));
    CHECK_FALSE(flock3d::bench::progress_enabled_for_mode("0", true));
    CHECK_FALSE(flock3d::bench::progress_enabled_for_mode("false", true));
    CHECK(flock3d::bench::progress_enabled_for_mode("auto", true));
    CHECK_FALSE(flock3d::bench::progress_enabled_for_mode("auto", false));
    CHECK(flock3d::bench::progress_enabled_for_mode("unexpected", true));
    CHECK_FALSE(flock3d::bench::progress_enabled_for_mode("unexpected", false));
}

TEST_CASE("Benchmark progress counter formats completed scenarios", "[benchmark][progress]")
{
    CHECK(flock3d::bench::format_progress_counter(1U, 16U) == "1/16");
    CHECK(flock3d::bench::format_progress_counter(16U, 16U) == "16/16");
    CHECK(flock3d::bench::format_progress_counter(17U, 16U) == "16/16");
    CHECK(flock3d::bench::format_progress_counter(1U, 0U).empty());
}

TEST_CASE("Common benchmark windows convert simulated seconds to fixed ticks", "[benchmark][common]")
{
    CHECK(flock3d::bench::simulated_seconds_to_ticks(0.0) == 0U);
    CHECK(flock3d::bench::simulated_seconds_to_ticks(1.0 / 120.0) == 1U);
    CHECK(flock3d::bench::simulated_seconds_to_ticks(0.001) == 1U);
    CHECK(flock3d::bench::simulated_seconds_to_ticks(1.0) == 120U);
    CHECK(flock3d::bench::ticks_to_simulated_seconds(120U) == Catch::Approx(1.0));
}

TEST_CASE("Fixed tick benchmark converts simulated seconds to ticks", "[benchmark][ticks]")
{
    using flock3d::bench::ticks::seconds_to_ticks;

    CHECK(seconds_to_ticks(5.0, 1.0 / 60.0) == 300U);
    CHECK(seconds_to_ticks(25.0, 1.0 / 60.0) == 1500U);
    CHECK(seconds_to_ticks(0.0, 1.0 / 60.0) == 0U);
    CHECK(seconds_to_ticks(0.001, 1.0 / 60.0) == 1U);
    CHECK_THROWS_AS(seconds_to_ticks(1.0, 0.0), std::invalid_argument);
}

TEST_CASE("Fixed tick benchmark runs expected update calls and excludes warmup "
          "timings",
          "[benchmark][ticks]")
{
    std::size_t update_count = 0U;
    std::vector<double> update_dt_values{};

    auto update = [&update_count, &update_dt_values](double dt) {
        ++update_count;
        update_dt_values.push_back(dt);
    };
    auto measure = [](double dt, auto &update_fn) {
        update_fn(dt);
        return 2.5;
    };

    const auto result = flock3d::bench::ticks::run_fixed_tick_benchmark(3U, 4U, 1.0 / 60.0, update, measure);

    CHECK(result.warmup_updates == 3U);
    CHECK(result.measured_tick_ms.size() == 4U);
    CHECK(update_count == 7U);
    CHECK(update_dt_values.size() == 7U);
    for (const double dt : update_dt_values)
    {
        CHECK(dt == Catch::Approx(1.0 / 60.0));
    }
    for (const double measured_ms : result.measured_tick_ms)
    {
        CHECK(measured_ms == Catch::Approx(2.5));
    }
}

TEST_CASE("Fixed tick benchmark summary statistics are computed from measured ticks", "[benchmark][ticks]")
{
    const std::vector<double> measured_tick_ms{1.0, 2.0, 3.0, 4.0, 100.0};
    const auto summary = flock3d::bench::ticks::summarize_ticks(measured_tick_ms, 0.5);

    CHECK(summary.tick_count == 5U);
    CHECK(summary.simulated_seconds == Catch::Approx(2.5));
    CHECK(summary.total_wall_seconds == Catch::Approx(0.110));
    CHECK(summary.average_ms_per_tick == Catch::Approx(22.0));
    CHECK(summary.mean_ns_per_tick == Catch::Approx(22'000'000.0));
    CHECK(summary.p50_ms_per_tick == Catch::Approx(3.0));
    CHECK(summary.p95_ms_per_tick == Catch::Approx(100.0));
    CHECK(summary.p99_ms_per_tick == Catch::Approx(100.0));
    CHECK(summary.max_ms_per_tick == Catch::Approx(100.0));
    CHECK(summary.ticks_per_second == Catch::Approx(5.0 / 0.110));
    CHECK(summary.updates_per_second == Catch::Approx(summary.ticks_per_second));
    CHECK(summary.real_time_factor == Catch::Approx(2.5 / 0.110));
}


TEST_CASE("Common benchmark update stats report latency and throughput metrics", "[benchmark][common]")
{
    flock3d::bench::UpdateStats stats{};
    for (const double milliseconds : {1.0, 2.0, 3.0, 4.0, 100.0}) {
        stats.record(milliseconds);
    }

    CHECK(stats.count == 5U);
    CHECK(stats.mean_ms() == Catch::Approx(22.0));
    CHECK(stats.mean_ns() == Catch::Approx(22'000'000.0));
    CHECK(stats.p50_ms() == Catch::Approx(3.0));
    CHECK(stats.p95_ms() == Catch::Approx(100.0));
    CHECK(stats.p99_ms() == Catch::Approx(100.0));
    CHECK(stats.wall_seconds() == Catch::Approx(0.110));
    CHECK(stats.ticks_per_second() == Catch::Approx(5.0 / 0.110));
    CHECK(stats.real_time_factor(0.5) == Catch::Approx(2.5 / 0.110));
}


TEST_CASE("Common benchmark thread count parser deduplicates requested workers", "[benchmark][common][threads]")
{
    const auto counts = flock3d::bench::parse_thread_counts("0,1,2,4,4,8");
    REQUIRE(counts.size() == 5U);
    CHECK(counts[0] == 0U);
    CHECK(counts[1] == 1U);
    CHECK(counts[2] == 2U);
    CHECK(counts[3] == 4U);
    CHECK(counts[4] == 8U);
}

TEST_CASE("Common benchmark list parsers validate comma-separated values", "[benchmark][common][options]")
{
    const auto counts = flock3d::bench::parse_positive_integer_list("512,1024,1024");
    REQUIRE(counts.size() == 2U);
    CHECK(counts[0] == 512U);
    CHECK(counts[1] == 1024U);

    CHECK(flock3d::bench::parse_positive_integer_list("512,zero").empty());

    const auto names = flock3d::bench::parse_string_list("BirdFlight, FishSchool,BirdFlight");
    REQUIRE(names.size() == 2U);
    CHECK(names[0] == "BirdFlight");
    CHECK(names[1] == "FishSchool");
}

TEST_CASE("Common benchmark boolean parser accepts explicit CLI forms", "[benchmark][common][options]")
{
    REQUIRE(flock3d::bench::parse_bool("on").has_value());
    CHECK(*flock3d::bench::parse_bool("on"));
    CHECK(*flock3d::bench::parse_bool("true"));
    CHECK(*flock3d::bench::parse_bool("1"));
    CHECK_FALSE(*flock3d::bench::parse_bool("off"));
    CHECK_FALSE(*flock3d::bench::parse_bool("false"));
    CHECK_FALSE(*flock3d::bench::parse_bool("0"));
    CHECK_FALSE(flock3d::bench::parse_bool("maybe").has_value());
}

TEST_CASE("Common benchmark diagnostics parser accepts compact and opt-in levels", "[benchmark][common][options]")
{
    REQUIRE(flock3d::bench::parse_diagnostics_level("none").has_value());
    CHECK(*flock3d::bench::parse_diagnostics_level("none") == flock3d::bench::DiagnosticsLevel::None);
    CHECK(*flock3d::bench::parse_diagnostics_level("basic") == flock3d::bench::DiagnosticsLevel::None);
    CHECK(*flock3d::bench::parse_diagnostics_level("phases") == flock3d::bench::DiagnosticsLevel::Phases);
    CHECK(*flock3d::bench::parse_diagnostics_level("workers") == flock3d::bench::DiagnosticsLevel::Workers);
    CHECK(*flock3d::bench::parse_diagnostics_level("full") == flock3d::bench::DiagnosticsLevel::Full);
    CHECK_FALSE(flock3d::bench::parse_diagnostics_level("verbose").has_value());

    CHECK(flock3d::bench::includes_phase_diagnostics(flock3d::bench::DiagnosticsLevel::Phases));
    CHECK_FALSE(flock3d::bench::includes_phase_diagnostics(flock3d::bench::DiagnosticsLevel::Workers));
    CHECK(flock3d::bench::includes_worker_diagnostics(flock3d::bench::DiagnosticsLevel::Workers));
    CHECK_FALSE(flock3d::bench::includes_worker_diagnostics(flock3d::bench::DiagnosticsLevel::Phases));
    CHECK(flock3d::bench::includes_internal_diagnostics(flock3d::bench::DiagnosticsLevel::Full));
}

TEST_CASE("Common benchmark model parser validates known model names", "[benchmark][common][options]")
{
    const auto models = flock3d::bench::parse_model_list("BirdFlight,fish_school");
    REQUIRE(models.size() == 2U);
    CHECK(models[0] == flock3d::sim::SimulationModel::BirdFlight);
    CHECK(models[1] == flock3d::sim::SimulationModel::FishSchool);
    CHECK(flock3d::bench::parse_model_list("UnknownModel").empty());
}
