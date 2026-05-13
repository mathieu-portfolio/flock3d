#pragma once

#include <cstddef>
#include <functional>
#include <unordered_map>
#include <vector>

#include <raylib.h>

namespace flock3d::sim {

struct CellCoord {
    int x{};
    int y{};
    int z{};

    [[nodiscard]] constexpr bool operator==(const CellCoord&) const noexcept = default;
};

struct CellCoordHash {
    [[nodiscard]] std::size_t operator()(CellCoord coord) const noexcept;
};

struct NeighborQueryDiagnostics {
    std::size_t visited_cells{};
    std::size_t candidates_tested{};
    std::size_t cell_lookups{};
    std::size_t occupied_cells{};
};

struct CellAggregate {
    CellCoord coord{};
    std::size_t count{};
    Vector3 sum_position{};
    Vector3 sum_velocity{};
    Vector3 centroid{};
    Vector3 average_velocity{};
};

class SpatialHash3D {
public:
    explicit SpatialHash3D(float cell_size);

    void clear();
    void insert(std::size_t boid_index, Vector3 position, Vector3 velocity = Vector3{});

    [[nodiscard]] std::vector<std::size_t> query_neighbors(Vector3 position, float radius) const;
    [[nodiscard]] std::vector<CellAggregate> query_cell_aggregates(Vector3 position, float radius) const;
    void query_cell_aggregates(Vector3 position, float radius, std::vector<CellAggregate>& result) const;
    void query_cell_aggregates(
        Vector3 position,
        float radius,
        std::vector<CellAggregate>& result,
        NeighborQueryDiagnostics& diagnostics) const;
    void query_visible_cell_aggregates(
        Vector3 position,
        float radius,
        Vector3 forward,
        float field_of_view_degrees,
        std::vector<CellAggregate>& result,
        NeighborQueryDiagnostics& diagnostics) const;
    [[nodiscard]] const CellAggregate* aggregate_for(CellCoord coord) const noexcept;
    void query_neighbors(Vector3 position, float radius, std::vector<std::size_t>& result) const;
    void query_neighbors(
        Vector3 position,
        float radius,
        std::vector<std::size_t>& result,
        NeighborQueryDiagnostics& diagnostics) const;
    [[nodiscard]] CellCoord cell_for(Vector3 position) const noexcept;
    [[nodiscard]] float cell_size() const noexcept { return cell_size_; }
    [[nodiscard]] std::size_t cell_count() const noexcept { return cells_.size(); }
    [[nodiscard]] std::size_t max_cell_occupancy() const noexcept;
    [[nodiscard]] double average_cell_occupancy() const noexcept;
    [[nodiscard]] std::size_t total_entries() const noexcept;

private:
    struct Entry {
        std::size_t boid_index{};
        Vector3 position{};
    };

    struct Cell {
        std::vector<Entry> entries{};
        CellAggregate aggregate{};
    };

    float cell_size_{};
    std::unordered_map<CellCoord, Cell, CellCoordHash> cells_;
};

} // namespace flock3d::sim
