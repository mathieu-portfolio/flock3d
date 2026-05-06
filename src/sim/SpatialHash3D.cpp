#include "sim/SpatialHash3D.hpp"

#include <cmath>
#include <stdexcept>

#include "math/Vec3.hpp"

namespace flock3d::sim {
namespace {

[[nodiscard]] int fast_floor_to_int(float value) noexcept
{
    return static_cast<int>(std::floor(value));
}

} // namespace

std::size_t CellCoordHash::operator()(CellCoord coord) const noexcept
{
    const auto x = static_cast<std::size_t>(coord.x) * 73'856'093U;
    const auto y = static_cast<std::size_t>(coord.y) * 19'349'663U;
    const auto z = static_cast<std::size_t>(coord.z) * 83'492'791U;
    return x ^ y ^ z;
}

SpatialHash3D::SpatialHash3D(float cell_size)
    : cell_size_{cell_size}
{
    if (cell_size_ <= 0.0F) {
        throw std::invalid_argument{"SpatialHash3D cell size must be positive"};
    }
}

void SpatialHash3D::clear()
{
    cells_.clear();
}

void SpatialHash3D::insert(std::size_t boid_index, Vector3 position)
{
    cells_[cell_for(position)].push_back(Entry{boid_index, position});
}

std::vector<std::size_t> SpatialHash3D::query_neighbors(Vector3 position, float radius) const
{
    std::vector<std::size_t> result;
    if (radius < 0.0F) {
        return result;
    }

    const auto center = cell_for(position);
    const int cell_radius = static_cast<int>(std::ceil(radius / cell_size_));
    const float radius_squared = radius * radius;

    for (int z = center.z - cell_radius; z <= center.z + cell_radius; ++z) {
        for (int y = center.y - cell_radius; y <= center.y + cell_radius; ++y) {
            for (int x = center.x - cell_radius; x <= center.x + cell_radius; ++x) {
                const auto found = cells_.find(CellCoord{x, y, z});
                if (found == cells_.end()) {
                    continue;
                }

                for (const Entry& entry : found->second) {
                    const auto offset = math::subtract(entry.position, position);
                    if (math::length_squared(offset) <= radius_squared) {
                        result.push_back(entry.boid_index);
                    }
                }
            }
        }
    }

    return result;
}

CellCoord SpatialHash3D::cell_for(Vector3 position) const noexcept
{
    return CellCoord{
        fast_floor_to_int(position.x / cell_size_),
        fast_floor_to_int(position.y / cell_size_),
        fast_floor_to_int(position.z / cell_size_),
    };
}

} // namespace flock3d::sim
