#include <algorithm>
#include <cstddef>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <raylib.h>

#include <flock3d/sim/SpatialGrid3D.hpp>
#include <flock3d/sim/SpatialHash3D.hpp>

namespace {

struct FixtureBoid {
    Vector3 position{};
    Vector3 velocity{};
};

[[nodiscard]] std::vector<FixtureBoid> deterministic_fixture()
{
    return {
        {Vector3{0.0F, 0.0F, 0.0F}, Vector3{1.0F, 0.0F, 0.0F}},
        {Vector3{0.8F, 0.2F, 0.1F}, Vector3{0.0F, 1.0F, 0.0F}},
        {Vector3{2.1F, 0.1F, 0.0F}, Vector3{0.0F, 0.0F, 1.0F}},
        {Vector3{-0.4F, -0.6F, 0.2F}, Vector3{1.0F, 1.0F, 0.0F}},
        {Vector3{1.9F, 2.2F, -0.2F}, Vector3{0.5F, 0.0F, 0.5F}},
        {Vector3{-2.2F, 0.0F, 0.0F}, Vector3{-1.0F, 0.0F, 0.0F}},
        {Vector3{4.3F, 0.1F, 0.1F}, Vector3{0.0F, -1.0F, 0.0F}},
        {Vector3{0.1F, -2.1F, 1.9F}, Vector3{0.0F, 0.0F, -1.0F}},
        {Vector3{2.1F, 0.8F, 0.0F}, Vector3{2.0F, 0.0F, 1.0F}},
    };
}

struct BuiltSpatialIndexes {
    flock3d::sim::SpatialHash3D hash{2.0F};
    flock3d::sim::SpatialGrid3D grid{2.0F};
};

[[nodiscard]] BuiltSpatialIndexes
build_indexes(const std::vector<FixtureBoid>& boids)
{
    BuiltSpatialIndexes indexes{};
    std::vector<Vector3> positions;
    std::vector<Vector3> velocities;
    positions.reserve(boids.size());
    velocities.reserve(boids.size());
    for (std::size_t i = 0; i < boids.size(); ++i) {
        indexes.hash.insert(i, boids[i].position, boids[i].velocity);
        positions.push_back(boids[i].position);
        velocities.push_back(boids[i].velocity);
    }
    indexes.grid.rebuild(positions, velocities);
    return indexes;
}

void check_vector_near(Vector3 actual, Vector3 expected)
{
    CHECK(actual.x == Catch::Approx(expected.x));
    CHECK(actual.y == Catch::Approx(expected.y));
    CHECK(actual.z == Catch::Approx(expected.z));
}

void check_aggregate_near(const flock3d::sim::CellAggregate& actual,
                          const flock3d::sim::CellAggregate& expected)
{
    CHECK((actual.coord == expected.coord));
    CHECK(actual.count == expected.count);
    check_vector_near(actual.sum_position, expected.sum_position);
    check_vector_near(actual.sum_velocity, expected.sum_velocity);
    check_vector_near(actual.centroid, expected.centroid);
    check_vector_near(actual.average_velocity, expected.average_velocity);
}

void check_diagnostics_equal(
    const flock3d::sim::NeighborQueryDiagnostics& actual,
    const flock3d::sim::NeighborQueryDiagnostics& expected)
{
    CHECK(actual.visited_cells == expected.visited_cells);
    CHECK(actual.candidates_tested == expected.candidates_tested);
}

} // namespace

TEST_CASE("SpatialGrid3D maps cells like SpatialHash3D", "[spatial][grid]")
{
    const flock3d::sim::SpatialHash3D hash{2.0F};
    const flock3d::sim::SpatialGrid3D grid{2.0F};

    const std::vector<Vector3> positions{
        Vector3{0.0F, 0.0F, 0.0F},
        Vector3{2.1F, -0.1F, -4.0F},
        Vector3{-0.001F, -2.0F, 3.999F},
        Vector3{4.0F, 4.0F, 4.0F},
    };

    for (const Vector3 position : positions) {
        CHECK((grid.cell_for(position) == hash.cell_for(position)));
    }
}

TEST_CASE("SpatialGrid3D insert path keeps deterministic in-cell ordering",
          "[spatial][grid]")
{
    flock3d::sim::SpatialGrid3D grid{2.0F};
    grid.insert(4, Vector3{0.5F, 0.0F, 0.0F});
    grid.insert(2, Vector3{0.25F, 0.0F, 0.0F});
    grid.insert(3, Vector3{2.2F, 0.0F, 0.0F});

    const auto neighbors =
        grid.query_neighbors(Vector3{0.0F, 0.0F, 0.0F}, 1.0F);

    REQUIRE(neighbors.size() == 2U);
    CHECK(neighbors[0] == 2U);
    CHECK(neighbors[1] == 4U);
}

TEST_CASE("SpatialGrid3D returns equivalent deterministic neighbor results",
          "[spatial][grid]")
{
    const auto boids = deterministic_fixture();
    auto indexes = build_indexes(boids);

    const std::vector<Vector3> queries{
        Vector3{0.0F, 0.0F, 0.0F},
        Vector3{2.0F, 0.5F, 0.0F},
        Vector3{-1.0F, -1.0F, 0.5F},
    };

    for (const Vector3 query : queries) {
        std::vector<std::size_t> hash_neighbors;
        std::vector<std::size_t> grid_neighbors;
        flock3d::sim::NeighborQueryDiagnostics hash_diagnostics{};
        flock3d::sim::NeighborQueryDiagnostics grid_diagnostics{};

        indexes.hash.query_neighbors(query, 2.35F, hash_neighbors,
                                     hash_diagnostics);
        indexes.grid.query_neighbors(query, 2.35F, grid_neighbors,
                                     grid_diagnostics);

        CHECK(grid_neighbors == hash_neighbors);
        check_diagnostics_equal(grid_diagnostics, hash_diagnostics);
    }
}

TEST_CASE("SpatialGrid3D exposes equivalent occupancy diagnostics",
          "[spatial][grid][metrics]")
{
    const auto boids = deterministic_fixture();
    auto indexes = build_indexes(boids);

    CHECK(indexes.grid.cell_count() == indexes.hash.cell_count());
    CHECK(indexes.grid.total_entries() == indexes.hash.total_entries());
    CHECK(indexes.grid.max_cell_occupancy() ==
          indexes.hash.max_cell_occupancy());
    CHECK(indexes.grid.average_cell_occupancy() ==
          Catch::Approx(indexes.hash.average_cell_occupancy()));
}

TEST_CASE("SpatialGrid3D produces equivalent aggregates",
          "[spatial][grid][aggregates]")
{
    const auto boids = deterministic_fixture();
    auto indexes = build_indexes(boids);

    for (const FixtureBoid& boid : boids) {
        const flock3d::sim::CellCoord coord =
            indexes.hash.cell_for(boid.position);
        const auto* hash_aggregate = indexes.hash.aggregate_for(coord);
        const auto* grid_aggregate = indexes.grid.aggregate_for(coord);
        REQUIRE(hash_aggregate != nullptr);
        REQUIRE(grid_aggregate != nullptr);
        check_aggregate_near(*grid_aggregate, *hash_aggregate);
    }

    CHECK(indexes.grid.aggregate_for(flock3d::sim::CellCoord{10, 10, 10}) ==
          nullptr);
}

TEST_CASE("SpatialGrid3D queries aggregate cells like SpatialHash3D",
          "[spatial][grid][aggregates]")
{
    const auto boids = deterministic_fixture();
    auto indexes = build_indexes(boids);

    std::vector<flock3d::sim::CellAggregate> hash_aggregates;
    std::vector<flock3d::sim::CellAggregate> grid_aggregates;
    flock3d::sim::NeighborQueryDiagnostics hash_diagnostics{};
    flock3d::sim::NeighborQueryDiagnostics grid_diagnostics{};

    indexes.hash.query_cell_aggregates(Vector3{0.0F, 0.0F, 0.0F}, 3.25F,
                                       hash_aggregates, hash_diagnostics);
    indexes.grid.query_cell_aggregates(Vector3{0.0F, 0.0F, 0.0F}, 3.25F,
                                       grid_aggregates, grid_diagnostics);

    REQUIRE(grid_aggregates.size() == hash_aggregates.size());
    for (std::size_t i = 0; i < hash_aggregates.size(); ++i) {
        check_aggregate_near(grid_aggregates[i], hash_aggregates[i]);
    }
    check_diagnostics_equal(grid_diagnostics, hash_diagnostics);
}

TEST_CASE("SpatialGrid3D query diagnostics count sparse row-span lookups",
          "[spatial][grid][metrics]")
{
    const auto boids = deterministic_fixture();
    auto indexes = build_indexes(boids);

    std::vector<std::size_t> neighbors;
    flock3d::sim::NeighborQueryDiagnostics diagnostics{};

    indexes.grid.query_neighbors(Vector3{0.0F, 0.0F, 0.0F}, 1.0F,
                                 neighbors, diagnostics);

    CHECK(neighbors == indexes.hash.query_neighbors(Vector3{0.0F, 0.0F, 0.0F},
                                                     1.0F));
    CHECK(diagnostics.visited_cells == 27U);
    CHECK(diagnostics.cell_lookups == 3U);
    CHECK(diagnostics.occupied_cells == 4U);
}

TEST_CASE("SpatialGrid3D visible aggregate queries match SpatialHash3D",
          "[spatial][grid][aggregates]")
{
    const auto boids = deterministic_fixture();
    auto indexes = build_indexes(boids);

    std::vector<flock3d::sim::CellAggregate> hash_aggregates;
    std::vector<flock3d::sim::CellAggregate> grid_aggregates;
    flock3d::sim::NeighborQueryDiagnostics hash_diagnostics{};
    flock3d::sim::NeighborQueryDiagnostics grid_diagnostics{};

    indexes.hash.query_visible_cell_aggregates(
        Vector3{0.0F, 0.0F, 0.0F}, 4.0F, Vector3{1.0F, 0.0F, 0.0F}, 120.0F,
        hash_aggregates, hash_diagnostics);
    indexes.grid.query_visible_cell_aggregates(
        Vector3{0.0F, 0.0F, 0.0F}, 4.0F, Vector3{1.0F, 0.0F, 0.0F}, 120.0F,
        grid_aggregates, grid_diagnostics);

    REQUIRE(grid_aggregates.size() == hash_aggregates.size());
    for (std::size_t i = 0; i < hash_aggregates.size(); ++i) {
        check_aggregate_near(grid_aggregates[i], hash_aggregates[i]);
    }
    check_diagnostics_equal(grid_diagnostics, hash_diagnostics);
}
