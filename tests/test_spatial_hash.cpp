#include <algorithm>
#include <cstddef>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <raylib.h>

#include <flock3d/sim/SpatialHash3D.hpp>

namespace {

[[nodiscard]] bool contains(const std::vector<std::size_t>& values, std::size_t expected)
{
    return std::find(values.begin(), values.end(), expected) != values.end();
}

} // namespace

TEST_CASE("SpatialHash3D maps positions to stable grid cells", "[spatial]")
{
    const flock3d::sim::SpatialHash3D hash{2.0F};

    CHECK((hash.cell_for(Vector3{0.0F, 0.0F, 0.0F}) == flock3d::sim::CellCoord{0, 0, 0}));
    CHECK((hash.cell_for(Vector3{2.1F, -0.1F, -4.0F}) == flock3d::sim::CellCoord{1, -1, -2}));
}

TEST_CASE("SpatialHash3D returns nearby boids and excludes distant ones", "[spatial]")
{
    flock3d::sim::SpatialHash3D hash{2.0F};
    hash.insert(1, Vector3{0.0F, 0.0F, 0.0F});
    hash.insert(2, Vector3{1.0F, 0.0F, 0.0F});
    hash.insert(3, Vector3{7.0F, 0.0F, 0.0F});

    const auto neighbors = hash.query_neighbors(Vector3{0.0F, 0.0F, 0.0F}, 1.5F);

    CHECK(contains(neighbors, 1));
    CHECK(contains(neighbors, 2));
    CHECK_FALSE(contains(neighbors, 3));
}


TEST_CASE("SpatialHash3D exposes cell occupancy diagnostics", "[spatial][metrics]")
{
    flock3d::sim::SpatialHash3D hash{2.0F};
    hash.insert(1, Vector3{0.0F, 0.0F, 0.0F});
    hash.insert(2, Vector3{1.0F, 0.0F, 0.0F});
    hash.insert(3, Vector3{4.0F, 0.0F, 0.0F});

    CHECK(hash.cell_count() == 2);
    CHECK(hash.total_entries() == 3);
    CHECK(hash.max_cell_occupancy() == 2);
    CHECK(hash.average_cell_occupancy() == Catch::Approx(1.5));
}

TEST_CASE("SpatialHash3D total entries tracks inserted boids after rebuild", "[spatial][metrics]")
{
    flock3d::sim::SpatialHash3D hash{4.0F};
    const std::vector<Vector3> positions{
        Vector3{0.0F, 0.0F, 0.0F},
        Vector3{4.5F, 0.0F, 0.0F},
        Vector3{-4.5F, 0.0F, 0.0F},
        Vector3{0.0F, 4.5F, 0.0F},
    };

    hash.clear();
    for (std::size_t i = 0; i < positions.size(); ++i) {
        hash.insert(i, positions[i]);
    }

    CHECK(hash.total_entries() == positions.size());
}
