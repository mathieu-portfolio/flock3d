#include <flock3d/sim/SpatialGrid3D.hpp>

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

[[nodiscard]] bool coord_less(CellCoord left, CellCoord right) noexcept
{
    if (left.z != right.z) {
        return left.z < right.z;
    }
    if (left.y != right.y) {
        return left.y < right.y;
    }
    return left.x < right.x;
}

[[nodiscard]] bool coord_equal(CellCoord left, CellCoord right) noexcept
{
    return left.x == right.x && left.y == right.y && left.z == right.z;
}

[[nodiscard]] bool row_less(int left_z, int left_y, int right_z,
                            int right_y) noexcept
{
    if (left_z != right_z) {
        return left_z < right_z;
    }
    return left_y < right_y;
}

[[nodiscard]] Vector3 cell_center(CellCoord coord, float cell_size) noexcept
{
    return Vector3{
        (static_cast<float>(coord.x) + 0.5F) * cell_size,
        (static_cast<float>(coord.y) + 0.5F) * cell_size,
        (static_cast<float>(coord.z) + 0.5F) * cell_size,
    };
}

[[nodiscard]] std::size_t query_cube_cell_count(int cell_radius) noexcept
{
    const auto side = static_cast<std::size_t>((cell_radius * 2) + 1);
    return side * side * side;
}

[[nodiscard]] bool cell_may_intersect_field_of_view(Vector3 position,
                                                    CellCoord coord,
                                                    float cell_size,
                                                    Vector3 forward_direction,
                                                    float half_angle) noexcept
{
    const Vector3 offset =
        math::subtract(cell_center(coord, cell_size), position);
    const float distance_squared = math::length_squared(offset);
    constexpr float half_diagonal_scale = 0.8660254037844386F;
    const float cell_bounding_radius = cell_size * half_diagonal_scale;
    const float cell_bounding_radius_squared =
        cell_bounding_radius * cell_bounding_radius;
    if (distance_squared <= cell_bounding_radius_squared ||
        distance_squared <= 0.000001F) {
        return true;
    }

    const float distance = std::sqrt(distance_squared);
    const float angular_margin =
        std::asin(std::clamp(cell_bounding_radius / distance, 0.0F, 1.0F));
    const float conservative_half_angle =
        std::min(std::numbers::pi_v<float>, half_angle + angular_margin);
    const float minimum_dot = std::cos(conservative_half_angle);
    const Vector3 direction = math::scale(offset, 1.0F / distance);
    return math::dot(forward_direction, direction) >= minimum_dot;
}

} // namespace

SpatialGrid3D::SpatialGrid3D(float cell_size) : cell_size_{cell_size}
{
    if (cell_size_ <= 0.0F) {
        throw std::invalid_argument{"SpatialGrid3D cell size must be positive"};
    }
}

void SpatialGrid3D::clear()
{
    entries_.clear();
    cell_ranges_.clear();
    row_spans_.clear();
    aggregates_.clear();
    max_cell_occupancy_ = 0U;
}

void SpatialGrid3D::insert(std::size_t boid_index, Vector3 position,
                           Vector3 velocity)
{
    entries_.push_back(
        Entry{cell_for(position), boid_index, position, velocity});
    build_ranges_from_entries();
}

void SpatialGrid3D::rebuild(const std::vector<Vector3>& positions,
                            const std::vector<Vector3>& velocities)
{
    entries_.clear();
    entries_.reserve(positions.size());
    for (std::size_t i = 0; i < positions.size(); ++i) {
        const Vector3 velocity =
            i < velocities.size() ? velocities[i] : Vector3{};
        entries_.push_back(
            Entry{cell_for(positions[i]), i, positions[i], velocity});
    }
    build_ranges_from_entries();
}

std::vector<std::size_t> SpatialGrid3D::query_neighbors(Vector3 position,
                                                        float radius) const
{
    std::vector<std::size_t> result;
    query_neighbors(position, radius, result);
    return result;
}

void SpatialGrid3D::query_neighbors(Vector3 position, float radius,
                                    std::vector<std::size_t>& result) const
{
    NeighborQueryDiagnostics diagnostics{};
    query_neighbors(position, radius, result, diagnostics);
}

void SpatialGrid3D::query_neighbors(Vector3 position, float radius,
                                    std::vector<std::size_t>& result,
                                    NeighborQueryDiagnostics& diagnostics) const
{
    result.clear();
    diagnostics = NeighborQueryDiagnostics{};
    if (radius < 0.0F) {
        return;
    }

    const CellCoord center = cell_for(position);
    const int cell_radius = static_cast<int>(std::ceil(radius / cell_size_));
    const float radius_squared = radius * radius;
    const int min_x = center.x - cell_radius;
    const int max_x = center.x + cell_radius;

    diagnostics.visited_cells = query_cube_cell_count(cell_radius);
    if (row_spans_.empty()) {
        return;
    }

    for (int z = center.z - cell_radius; z <= center.z + cell_radius; ++z) {
        ++diagnostics.cell_lookups;
        const int min_y = center.y - cell_radius;
        const int max_y = center.y + cell_radius;
        for (auto row = lower_bound_row(z, min_y);
             row != row_spans_.end() && row->z == z && row->y <= max_y;
             ++row) {
            auto range = std::lower_bound(
                cell_ranges_.begin() +
                    static_cast<std::ptrdiff_t>(row->begin_range),
                cell_ranges_.begin() +
                    static_cast<std::ptrdiff_t>(row->end_range),
                min_x,
                [](const CellRange& candidate, int x) noexcept {
                    return candidate.coord.x < x;
                });
            const auto row_end = cell_ranges_.begin() +
                                 static_cast<std::ptrdiff_t>(row->end_range);
            for (; range != row_end; ++range) {
                if (range->coord.x > max_x) {
                    break;
                }
                ++diagnostics.occupied_cells;
                diagnostics.candidates_tested += range->end - range->begin;
                for (std::size_t entry_index = range->begin;
                     entry_index < range->end; ++entry_index) {
                    const Entry& entry = entries_[entry_index];
                    const Vector3 offset =
                        math::subtract(entry.position, position);
                    if (math::length_squared(offset) <= radius_squared) {
                        result.push_back(entry.boid_index);
                    }
                }
            }
        }
    }
}

std::vector<CellAggregate>
SpatialGrid3D::query_cell_aggregates(Vector3 position, float radius) const
{
    std::vector<CellAggregate> result;
    query_cell_aggregates(position, radius, result);
    return result;
}

void SpatialGrid3D::query_cell_aggregates(
    Vector3 position, float radius, std::vector<CellAggregate>& result) const
{
    NeighborQueryDiagnostics diagnostics{};
    query_cell_aggregates(position, radius, result, diagnostics);
}

void SpatialGrid3D::query_cell_aggregates(
    Vector3 position, float radius, std::vector<CellAggregate>& result,
    NeighborQueryDiagnostics& diagnostics) const
{
    result.clear();
    diagnostics = NeighborQueryDiagnostics{};
    if (radius < 0.0F) {
        return;
    }

    const CellCoord center = cell_for(position);
    const int cell_radius = static_cast<int>(std::ceil(radius / cell_size_));
    const float radius_squared = radius * radius;
    const int min_x = center.x - cell_radius;
    const int max_x = center.x + cell_radius;

    diagnostics.visited_cells = query_cube_cell_count(cell_radius);
    if (row_spans_.empty()) {
        return;
    }

    for (int z = center.z - cell_radius; z <= center.z + cell_radius; ++z) {
        ++diagnostics.cell_lookups;
        const int min_y = center.y - cell_radius;
        const int max_y = center.y + cell_radius;
        for (auto row = lower_bound_row(z, min_y);
             row != row_spans_.end() && row->z == z && row->y <= max_y;
             ++row) {
            auto range = std::lower_bound(
                cell_ranges_.begin() +
                    static_cast<std::ptrdiff_t>(row->begin_range),
                cell_ranges_.begin() +
                    static_cast<std::ptrdiff_t>(row->end_range),
                min_x,
                [](const CellRange& candidate, int x) noexcept {
                    return candidate.coord.x < x;
                });
            const auto row_end = cell_ranges_.begin() +
                                 static_cast<std::ptrdiff_t>(row->end_range);
            for (; range != row_end; ++range) {
                if (range->coord.x > max_x) {
                    break;
                }
                ++diagnostics.occupied_cells;
                ++diagnostics.candidates_tested;
                const CellAggregate& aggregate =
                    aggregates_[range->aggregate_index];
                const Vector3 offset =
                    math::subtract(aggregate.centroid, position);
                if (math::length_squared(offset) <= radius_squared) {
                    result.push_back(aggregate);
                }
            }
        }
    }
}

void SpatialGrid3D::query_visible_cell_aggregates(
    Vector3 position, float radius, Vector3 forward,
    float field_of_view_degrees, std::vector<CellAggregate>& result,
    NeighborQueryDiagnostics& diagnostics) const
{
    result.clear();
    diagnostics = NeighborQueryDiagnostics{};
    if (radius < 0.0F || field_of_view_degrees <= 0.0F) {
        return;
    }

    const Vector3 forward_direction = math::normalize_safe(forward);
    const bool cull_by_field_of_view =
        field_of_view_degrees < 359.999F &&
        math::length_squared(forward_direction) > 0.000001F;
    const float half_angle =
        field_of_view_degrees * (std::numbers::pi_v<float> / 180.0F) * 0.5F;

    const CellCoord center = cell_for(position);
    const int cell_radius = static_cast<int>(std::ceil(radius / cell_size_));
    const float radius_squared = radius * radius;
    const int min_x = center.x - cell_radius;
    const int max_x = center.x + cell_radius;
    const int min_y = center.y - cell_radius;
    const int max_y = center.y + cell_radius;

    for (int z = center.z - cell_radius; z <= center.z + cell_radius; ++z) {
        RowSpanIterator row = row_spans_.end();
        RowSpanIterator row_end = row_spans_.end();
        if (!row_spans_.empty()) {
            ++diagnostics.cell_lookups;
            row = lower_bound_row(z, min_y);
            row_end = row_spans_.end();
        }
        for (int y = min_y; y <= max_y; ++y) {
            while (row != row_end && row->z == z && row->y < y) {
                ++row;
            }

            std::size_t range_index = 0U;
            std::size_t range_end = 0U;
            if (row != row_end && row->z == z && row->y == y) {
                range_index = row->begin_range;
                range_end = row->end_range;
            }
            for (int x = min_x; x <= max_x; ++x) {
                const CellCoord coord{x, y, z};
                if (cull_by_field_of_view &&
                    !cell_may_intersect_field_of_view(
                        position, coord, cell_size_, forward_direction,
                        half_angle)) {
                    continue;
                }

                ++diagnostics.visited_cells;
                while (range_index < range_end &&
                       cell_ranges_[range_index].coord.x < x) {
                    ++range_index;
                }
                if (range_index >= range_end ||
                    cell_ranges_[range_index].coord.x != x) {
                    continue;
                }

                ++diagnostics.occupied_cells;
                ++diagnostics.candidates_tested;
                const CellRange& range = cell_ranges_[range_index];
                const CellAggregate& aggregate =
                    aggregates_[range.aggregate_index];
                const Vector3 offset =
                    math::subtract(aggregate.centroid, position);
                if (math::length_squared(offset) <= radius_squared) {
                    result.push_back(aggregate);
                }
                ++range_index;
            }
        }
    }
}

const CellAggregate*
SpatialGrid3D::aggregate_for(CellCoord coord) const noexcept
{
    const CellRange* range = range_for(coord);
    if (range == nullptr) {
        return nullptr;
    }
    return &aggregates_[range->aggregate_index];
}

CellCoord SpatialGrid3D::cell_for(Vector3 position) const noexcept
{
    return CellCoord{
        fast_floor_to_int(position.x / cell_size_),
        fast_floor_to_int(position.y / cell_size_),
        fast_floor_to_int(position.z / cell_size_),
    };
}

std::size_t SpatialGrid3D::cell_count() const noexcept
{
    return cell_ranges_.size();
}

std::size_t SpatialGrid3D::max_cell_occupancy() const noexcept
{
    return max_cell_occupancy_;
}

double SpatialGrid3D::average_cell_occupancy() const noexcept
{
    return cell_ranges_.empty() ? 0.0
                                : static_cast<double>(entries_.size()) /
                                      static_cast<double>(cell_ranges_.size());
}

std::size_t SpatialGrid3D::total_entries() const noexcept
{
    return entries_.size();
}

void SpatialGrid3D::build_ranges_from_entries()
{
    std::stable_sort(entries_.begin(), entries_.end(),
                     [](const Entry& left, const Entry& right) noexcept {
                         if (!coord_equal(left.coord, right.coord)) {
                             return coord_less(left.coord, right.coord);
                         }
                         return left.boid_index < right.boid_index;
                     });

    cell_ranges_.clear();
    row_spans_.clear();
    aggregates_.clear();
    max_cell_occupancy_ = 0U;
    cell_ranges_.reserve(entries_.size());
    aggregates_.reserve(entries_.size());

    std::size_t begin = 0U;
    while (begin < entries_.size()) {
        const CellCoord coord = entries_[begin].coord;
        CellAggregate aggregate{};
        aggregate.coord = coord;

        std::size_t end = begin;
        while (end < entries_.size() &&
               coord_equal(entries_[end].coord, coord)) {
            const Entry& entry = entries_[end];
            ++aggregate.count;
            aggregate.sum_position =
                math::add(aggregate.sum_position, entry.position);
            aggregate.sum_velocity =
                math::add(aggregate.sum_velocity, entry.velocity);
            ++end;
        }

        const float inverse_count = 1.0F / static_cast<float>(aggregate.count);
        aggregate.centroid = math::scale(aggregate.sum_position, inverse_count);
        aggregate.average_velocity =
            math::scale(aggregate.sum_velocity, inverse_count);

        const std::size_t aggregate_index = aggregates_.size();
        aggregates_.push_back(aggregate);
        cell_ranges_.push_back(CellRange{coord, begin, end, aggregate_index});
        max_cell_occupancy_ = std::max(max_cell_occupancy_, end - begin);
        begin = end;
    }

    build_row_spans_from_ranges();
}

void SpatialGrid3D::build_row_spans_from_ranges()
{
    row_spans_.clear();
    row_spans_.reserve(cell_ranges_.size());

    std::size_t begin = 0U;
    while (begin < cell_ranges_.size()) {
        const CellCoord coord = cell_ranges_[begin].coord;
        std::size_t end = begin + 1U;
        while (end < cell_ranges_.size() &&
               cell_ranges_[end].coord.z == coord.z &&
               cell_ranges_[end].coord.y == coord.y) {
            ++end;
        }

        row_spans_.push_back(RowSpan{coord.z, coord.y, begin, end});
        begin = end;
    }
}

SpatialGrid3D::RowSpanIterator
SpatialGrid3D::lower_bound_row(int z, int y) const noexcept
{
    return std::lower_bound(
        row_spans_.begin(), row_spans_.end(), RowSpan{z, y, 0U, 0U},
        [](const RowSpan& left, const RowSpan& right) noexcept {
            return row_less(left.z, left.y, right.z, right.y);
        });
}

SpatialGrid3D::CellRangeIterator
SpatialGrid3D::lower_bound_range(CellCoord coord) const noexcept
{
    return std::lower_bound(
        cell_ranges_.begin(), cell_ranges_.end(), coord,
        [](const CellRange& range, CellCoord value) noexcept {
            return coord_less(range.coord, value);
        });
}

const SpatialGrid3D::CellRange*
SpatialGrid3D::range_for(CellCoord coord) const noexcept
{
    const auto row = lower_bound_row(coord.z, coord.y);
    if (row == row_spans_.end() || row->z != coord.z || row->y != coord.y) {
        return nullptr;
    }

    const auto begin = cell_ranges_.begin() +
                       static_cast<std::ptrdiff_t>(row->begin_range);
    const auto end = cell_ranges_.begin() +
                     static_cast<std::ptrdiff_t>(row->end_range);
    const auto found = std::lower_bound(
        begin, end, coord.x,
        [](const CellRange& range, int x) noexcept {
            return range.coord.x < x;
        });
    if (found == end || found->coord.x != coord.x) {
        return nullptr;
    }
    return &(*found);
}

} // namespace flock3d::sim
