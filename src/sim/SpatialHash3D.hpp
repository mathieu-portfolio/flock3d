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

class SpatialHash3D {
public:
    explicit SpatialHash3D(float cell_size);

    void clear();
    void insert(std::size_t boid_index, Vector3 position);

    [[nodiscard]] std::vector<std::size_t> query_neighbors(Vector3 position, float radius) const;
    [[nodiscard]] CellCoord cell_for(Vector3 position) const noexcept;
    [[nodiscard]] float cell_size() const noexcept { return cell_size_; }

private:
    struct Entry {
        std::size_t boid_index{};
        Vector3 position{};
    };

    float cell_size_{};
    std::unordered_map<CellCoord, std::vector<Entry>, CellCoordHash> cells_;
};

} // namespace flock3d::sim
