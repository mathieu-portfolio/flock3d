#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

#include <raylib.h>

#include <flock3d/math/Vec3.hpp>
#include <flock3d/sim/SpatialHash3D.hpp>

namespace {

using Clock = std::chrono::steady_clock;

struct TimedCount {
    std::size_t count{};
    double milliseconds{};
};

[[nodiscard]] std::vector<Vector3> generate_positions(std::size_t count)
{
    std::mt19937 rng{1337U};
    std::uniform_real_distribution<float> distribution{-40.0F, 40.0F};

    std::vector<Vector3> positions;
    positions.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        positions.push_back(Vector3{distribution(rng), distribution(rng), distribution(rng)});
    }
    return positions;
}

[[nodiscard]] TimedCount count_neighbors_naive(const std::vector<Vector3>& positions, float radius)
{
    const float radius_squared = radius * radius;
    const auto start = Clock::now();

    std::size_t total = 0;
    for (std::size_t i = 0; i < positions.size(); ++i) {
        for (std::size_t j = 0; j < positions.size(); ++j) {
            if (i == j) {
                continue;
            }

            if (flock3d::math::length_squared(flock3d::math::subtract(positions[j], positions[i])) <= radius_squared) {
                ++total;
            }
        }
    }

    const auto stop = Clock::now();
    return TimedCount{total, std::chrono::duration<double, std::milli>(stop - start).count()};
}

[[nodiscard]] TimedCount count_neighbors_spatial_hash(const std::vector<Vector3>& positions, float radius, float cell_size)
{
    const auto start = Clock::now();

    flock3d::sim::SpatialHash3D hash{cell_size};
    for (std::size_t i = 0; i < positions.size(); ++i) {
        hash.insert(i, positions[i]);
    }

    std::size_t total = 0;
    for (std::size_t i = 0; i < positions.size(); ++i) {
        const auto neighbors = hash.query_neighbors(positions[i], radius);
        for (const std::size_t neighbor : neighbors) {
            if (neighbor != i) {
                ++total;
            }
        }
    }

    const auto stop = Clock::now();
    return TimedCount{total, std::chrono::duration<double, std::milli>(stop - start).count()};
}

} // namespace

int main()
{
    constexpr float neighbor_radius = 4.0F;
    constexpr float cell_size = 4.0F;
    constexpr std::size_t sizes[] = {512, 2'048, 8'192};

    std::cout << "boids,naive_ms,spatial_hash_ms,neighbor_counts_match\n";
    for (const std::size_t size : sizes) {
        const auto positions = generate_positions(size);
        const TimedCount naive = count_neighbors_naive(positions, neighbor_radius);
        const TimedCount spatial = count_neighbors_spatial_hash(positions, neighbor_radius, cell_size);

        std::cout << size << ',' << std::fixed << std::setprecision(3) << naive.milliseconds << ','
                  << spatial.milliseconds << ',' << std::boolalpha << (naive.count == spatial.count) << '\n';
    }

    return 0;
}
