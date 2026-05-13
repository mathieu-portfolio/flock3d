#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

#include <raylib.h>

#include <flock3d/math/Vec3.hpp>
#include <flock3d/sim/SpatialHash3D.hpp>

namespace flock3d::sim {

class SpatialGrid3D {
public:
    explicit SpatialGrid3D(float cell_size);

    void clear();
    void insert(std::size_t boid_index, Vector3 position,
                Vector3 velocity = Vector3{});
    void rebuild(const std::vector<Vector3>& positions,
                 const std::vector<Vector3>& velocities = {});

    [[nodiscard]] std::vector<std::size_t> query_neighbors(Vector3 position,
                                                           float radius) const;
    void query_neighbors(Vector3 position, float radius,
                         std::vector<std::size_t>& result) const;
    void query_neighbors(Vector3 position, float radius,
                         std::vector<std::size_t>& result,
                         NeighborQueryDiagnostics& diagnostics) const;

    template <typename Callback>
    void for_each_neighbor(Vector3 position, float radius,
                           NeighborQueryDiagnostics& diagnostics,
                           Callback&& callback) const;

    [[nodiscard]] std::vector<CellAggregate>
    query_cell_aggregates(Vector3 position, float radius) const;
    void query_cell_aggregates(Vector3 position, float radius,
                               std::vector<CellAggregate>& result) const;
    void query_cell_aggregates(Vector3 position, float radius,
                               std::vector<CellAggregate>& result,
                               NeighborQueryDiagnostics& diagnostics) const;
    void
    query_visible_cell_aggregates(Vector3 position, float radius,
                                  Vector3 forward, float field_of_view_degrees,
                                  std::vector<CellAggregate>& result,
                                  NeighborQueryDiagnostics& diagnostics) const;

    [[nodiscard]] const CellAggregate*
    aggregate_for(CellCoord coord) const noexcept;
    [[nodiscard]] CellCoord cell_for(Vector3 position) const noexcept;
    [[nodiscard]] float cell_size() const noexcept { return cell_size_; }
    [[nodiscard]] std::size_t cell_count() const noexcept;
    [[nodiscard]] std::size_t max_cell_occupancy() const noexcept;
    [[nodiscard]] double average_cell_occupancy() const noexcept;
    [[nodiscard]] std::size_t total_entries() const noexcept;

private:
    struct Entry {
        CellCoord coord{};
        std::size_t boid_index{};
        Vector3 position{};
        Vector3 velocity{};
    };

    struct QueryEntry {
        std::size_t boid_index{};
        Vector3 position{};
    };

    struct CellRange {
        CellCoord coord{};
        std::size_t begin{};
        std::size_t end{};
        std::size_t aggregate_index{};
    };

    struct RowSpan {
        int z{};
        int y{};
        std::size_t begin_range{};
        std::size_t end_range{};
    };

    using CellRangeIterator = std::vector<CellRange>::const_iterator;
    using RowSpanIterator = std::vector<RowSpan>::const_iterator;

    void build_ranges_from_entries();
    void build_row_spans_from_ranges();
    [[nodiscard]] RowSpanIterator lower_bound_row(int z, int y) const noexcept;
    [[nodiscard]] CellRangeIterator
    lower_bound_range(CellCoord coord) const noexcept;
    [[nodiscard]] const CellRange* range_for(CellCoord coord) const noexcept;

    template <typename Callback>
    void for_each_neighbor_impl(Vector3 position, float radius,
                                NeighborQueryDiagnostics& diagnostics,
                                Callback&& callback) const;

    float cell_size_{};
    std::vector<Entry> entries_{};
    std::vector<QueryEntry> query_entries_{};
    std::vector<CellRange> cell_ranges_{};
    std::vector<RowSpan> row_spans_{};
    std::vector<CellAggregate> aggregates_{};
    std::size_t max_cell_occupancy_{};
};


template <typename Callback>
void SpatialGrid3D::for_each_neighbor(Vector3 position, float radius,
                                      NeighborQueryDiagnostics& diagnostics,
                                      Callback&& callback) const
{
    for_each_neighbor_impl(position, radius, diagnostics,
                           std::forward<Callback>(callback));
}

template <typename Callback>
void SpatialGrid3D::for_each_neighbor_impl(Vector3 position, float radius,
                                           NeighborQueryDiagnostics& diagnostics,
                                           Callback&& callback) const
{
    diagnostics = NeighborQueryDiagnostics{};
    if (radius < 0.0F) {
        return;
    }

    const CellCoord center = cell_for(position);
    const int cell_radius = static_cast<int>(std::ceil(radius / cell_size_));
    const float radius_squared = radius * radius;
    const int min_x = center.x - cell_radius;
    const int max_x = center.x + cell_radius;
    const auto side = static_cast<std::size_t>((cell_radius * 2) + 1);

    diagnostics.visited_cells = side * side * side;
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
                    const QueryEntry& entry = query_entries_[entry_index];
                    const Vector3 offset =
                        math::subtract(entry.position, position);
                    if (math::length_squared(offset) <= radius_squared) {
                        callback(entry.boid_index);
                    }
                }
            }
        }
    }
}

} // namespace flock3d::sim
