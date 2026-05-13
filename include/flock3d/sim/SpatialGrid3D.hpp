#pragma once

#include <cstddef>
#include <vector>

#include <raylib.h>

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

    struct CellRange {
        CellCoord coord{};
        std::size_t begin{};
        std::size_t end{};
        std::size_t aggregate_index{};
    };

    using CellRangeIterator = std::vector<CellRange>::const_iterator;

    void build_ranges_from_entries();
    [[nodiscard]] CellRangeIterator
    lower_bound_range(CellCoord coord) const noexcept;
    [[nodiscard]] const CellRange* range_for(CellCoord coord) const noexcept;

    float cell_size_{};
    std::vector<Entry> entries_{};
    std::vector<CellRange> cell_ranges_{};
    std::vector<CellAggregate> aggregates_{};
    std::size_t max_cell_occupancy_{};
};

} // namespace flock3d::sim
