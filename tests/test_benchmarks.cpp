#include "../benchmarks/benchmark_common.hpp"

#include <catch2/catch_test_macros.hpp>

TEST_CASE("Benchmark progress only requires an interactive stderr", "[benchmark][progress]")
{
    CHECK(flock3d::bench::progress_enabled_for_streams(true, true));
    CHECK(flock3d::bench::progress_enabled_for_streams(false, true));
    CHECK_FALSE(flock3d::bench::progress_enabled_for_streams(true, false));
    CHECK_FALSE(flock3d::bench::progress_enabled_for_streams(false, false));
}
