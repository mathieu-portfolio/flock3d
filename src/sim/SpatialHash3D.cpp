#include <flock3d/sim/SpatialHash3D.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

#include <flock3d/math/Vec3.hpp>

namespace flock3d::sim {
namespace {

[[nodiscard]] int fast_floor_to_int(float value) noexcept
{
    return static_cast<int>(std::floor(value));
}

[[nodiscard]] Vector3 cell_center(CellCoord coord, float cell_size) noexcept
{
    return Vector3{
        (static_cast<float>(coord.x) + 0.5F) * cell_size,
        (static_cast<float>(coord.y) + 0.5F) * cell_size,
        (static_cast<float>(coord.z) + 0.5F) * cell_size,
    };
}

[[nodiscard]] bool cell_may_intersect_field_of_view(
    Vector3 position,
    CellCoord coord,
    float cell_size,
    Vector3 forward,
    float field_of_view_degrees) noexcept
{
    if (field_of_view_degrees >= 359.999F) {
        return true;
    }
    if (field_of_view_degrees <= 0.0F) {
        return false;
    }

    const Vector3 forward_direction = math::normalize_safe(forward);
    if (math::length_squared(forward_direction) <= 0.000001F) {
        return true;
    }

    const Vector3 offset = math::subtract(cell_center(coord, cell_size), position);
    const float distance = math::length(offset);
    constexpr float half_diagonal_scale = 0.8660254037844386F;
    const float cell_bounding_radius = cell_size * half_diagonal_scale;
    if (distance <= cell_bounding_radius || distance <= 0.000001F) {
        return true;
    }

    const float half_angle = field_of_view_degrees * (std::numbers::pi_v<float> / 180.0F) * 0.5F;
    const float angular_margin = std::asin(std::clamp(cell_bounding_radius / distance, 0.0F, 1.0F));
    const float conservative_half_angle = std::min(std::numbers::pi_v<float>, half_angle + angular_margin);
    const float minimum_dot = std::cos(conservative_half_angle);
    const Vector3 direction = math::scale(offset, 1.0F / distance);
    return math::dot(forward_direction, direction) >= minimum_dot;
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

void SpatialHash3D::insert(std::size_t boid_index, Vector3 position, Vector3 velocity)
{
    const CellCoord coord = cell_for(position);
    Cell& cell = cells_[coord];
    cell.entries.push_back(Entry{boid_index, position});
    cell.aggregate.coord = coord;
    ++cell.aggregate.count;
    cell.aggregate.sum_position = math::add(cell.aggregate.sum_position, position);
    cell.aggregate.sum_velocity = math::add(cell.aggregate.sum_velocity, velocity);
    const float inverse_count = 1.0F / static_cast<float>(cell.aggregate.count);
    cell.aggregate.centroid = math::scale(cell.aggregate.sum_position, inverse_count);
    cell.aggregate.average_velocity = math::scale(cell.aggregate.sum_velocity, inverse_count);
}

std::vector<std::size_t> SpatialHash3D::query_neighbors(Vector3 position, float radius) const
{
    std::vector<std::size_t> result;
    query_neighbors(position, radius, result);
    return result;
}

void SpatialHash3D::query_neighbors(Vector3 position, float radius, std::vector<std::size_t>& result) const
{
    NeighborQueryDiagnostics diagnostics{};
    query_neighbors(position, radius, result, diagnostics);
}

void SpatialHash3D::query_neighbors(
    Vector3 position,
    float radius,
    std::vector<std::size_t>& result,
    NeighborQueryDiagnostics& diagnostics) const
{
    result.clear();
    diagnostics = NeighborQueryDiagnostics{};
    if (radius < 0.0F) {
        return;
    }

    const auto center = cell_for(position);
    const int cell_radius = static_cast<int>(std::ceil(radius / cell_size_));
    const float radius_squared = radius * radius;

    for (int z = center.z - cell_radius; z <= center.z + cell_radius; ++z) {
        for (int y = center.y - cell_radius; y <= center.y + cell_radius; ++y) {
            for (int x = center.x - cell_radius; x <= center.x + cell_radius; ++x) {
                ++diagnostics.visited_cells;
                const auto found = cells_.find(CellCoord{x, y, z});
                if (found == cells_.end()) {
                    continue;
                }

                diagnostics.candidates_tested += found->second.entries.size();
                for (const Entry& entry : found->second.entries) {
                    const auto offset = math::subtract(entry.position, position);
                    if (math::length_squared(offset) <= radius_squared) {
                        result.push_back(entry.boid_index);
                    }
                }
            }
        }
    }
}

std::vector<CellAggregate> SpatialHash3D::query_cell_aggregates(Vector3 position, float radius) const
{
    std::vector<CellAggregate> result;
    query_cell_aggregates(position, radius, result);
    return result;
}

void SpatialHash3D::query_cell_aggregates(Vector3 position, float radius, std::vector<CellAggregate>& result) const
{
    NeighborQueryDiagnostics diagnostics{};
    query_cell_aggregates(position, radius, result, diagnostics);
}

void SpatialHash3D::query_cell_aggregates(
    Vector3 position,
    float radius,
    std::vector<CellAggregate>& result,
    NeighborQueryDiagnostics& diagnostics) const
{
    result.clear();
    diagnostics = NeighborQueryDiagnostics{};
    if (radius < 0.0F) {
        return;
    }

    const auto center = cell_for(position);
    const int cell_radius = static_cast<int>(std::ceil(radius / cell_size_));
    const float radius_squared = radius * radius;

    for (int z = center.z - cell_radius; z <= center.z + cell_radius; ++z) {
        for (int y = center.y - cell_radius; y <= center.y + cell_radius; ++y) {
            for (int x = center.x - cell_radius; x <= center.x + cell_radius; ++x) {
                ++diagnostics.visited_cells;
                const auto found = cells_.find(CellCoord{x, y, z});
                if (found == cells_.end()) {
                    continue;
                }

                ++diagnostics.candidates_tested;
                const CellAggregate& aggregate = found->second.aggregate;
                const auto offset = math::subtract(aggregate.centroid, position);
                if (math::length_squared(offset) <= radius_squared) {
                    result.push_back(aggregate);
                }
            }
        }
    }
}

void SpatialHash3D::query_visible_cell_aggregates(
    Vector3 position,
    float radius,
    Vector3 forward,
    float field_of_view_degrees,
    std::vector<CellAggregate>& result,
    NeighborQueryDiagnostics& diagnostics) const
{
    result.clear();
    diagnostics = NeighborQueryDiagnostics{};
    if (radius < 0.0F) {
        return;
    }

    const auto center = cell_for(position);
    const int cell_radius = static_cast<int>(std::ceil(radius / cell_size_));
    const float radius_squared = radius * radius;

    for (int z = center.z - cell_radius; z <= center.z + cell_radius; ++z) {
        for (int y = center.y - cell_radius; y <= center.y + cell_radius; ++y) {
            for (int x = center.x - cell_radius; x <= center.x + cell_radius; ++x) {
                const CellCoord coord{x, y, z};
                if (!cell_may_intersect_field_of_view(
                        position, coord, cell_size_, forward, field_of_view_degrees)) {
                    continue;
                }

                ++diagnostics.visited_cells;
                const auto found = cells_.find(coord);
                if (found == cells_.end()) {
                    continue;
                }

                ++diagnostics.candidates_tested;
                const CellAggregate& aggregate = found->second.aggregate;
                const auto offset = math::subtract(aggregate.centroid, position);
                if (math::length_squared(offset) <= radius_squared) {
                    result.push_back(aggregate);
                }
            }
        }
    }
}

const CellAggregate* SpatialHash3D::aggregate_for(CellCoord coord) const noexcept
{
    const auto found = cells_.find(coord);
    if (found == cells_.end()) {
        return nullptr;
    }
    return &found->second.aggregate;
}

std::size_t SpatialHash3D::max_cell_occupancy() const noexcept
{
    std::size_t maximum = 0;
    for (const auto& cell : cells_) {
        maximum = std::max(maximum, cell.second.entries.size());
    }
    return maximum;
}

double SpatialHash3D::average_cell_occupancy() const noexcept
{
    return cells_.empty() ? 0.0 : static_cast<double>(total_entries()) / static_cast<double>(cells_.size());
}

std::size_t SpatialHash3D::total_entries() const noexcept
{
    std::size_t total = 0;
    for (const auto& cell : cells_) {
        total += cell.second.entries.size();
    }
    return total;
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
