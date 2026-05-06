#include "render/BoidRenderer.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

#include <raylib.h>

#include <flock3d/math/Vec3.hpp>

namespace flock3d::render {
namespace {

struct OrientationBasis {
    Vector3 forward{};
    Vector3 right{};
    Vector3 up{};
};

[[nodiscard]] OrientationBasis make_velocity_basis(Vector3 velocity) noexcept
{
    constexpr Vector3 default_forward{0.0F, 0.0F, 1.0F};
    constexpr Vector3 world_up{0.0F, 1.0F, 0.0F};
    constexpr Vector3 fallback_up{1.0F, 0.0F, 0.0F};
    constexpr float parallel_threshold = 0.96F;

    const Vector3 forward = math::length_squared(velocity) > 0.000001F
        ? math::normalize_safe(velocity)
        : default_forward;
    const Vector3 reference_up = std::fabs(math::dot(forward, world_up)) > parallel_threshold
        ? fallback_up
        : world_up;
    const Vector3 right = math::normalize_safe(math::cross(reference_up, forward));
    const Vector3 up = math::normalize_safe(math::cross(forward, right));

    return OrientationBasis{forward, right, up};
}

void draw_boid_tetrahedron(Vector3 position, Vector3 velocity, float scale)
{
    const OrientationBasis basis = make_velocity_basis(velocity);
    const float clamped_scale = std::max(scale, 0.01F);

    const Vector3 nose = math::add(position, math::scale(basis.forward, clamped_scale * 1.65F));
    const Vector3 tail = math::subtract(position, math::scale(basis.forward, clamped_scale * 0.75F));
    const Vector3 left = math::subtract(tail, math::scale(basis.right, clamped_scale * 0.65F));
    const Vector3 right = math::add(tail, math::scale(basis.right, clamped_scale * 0.65F));
    const Vector3 top = math::add(tail, math::scale(basis.up, clamped_scale * 0.8F));

    DrawTriangle3D(nose, left, top, SKYBLUE);
    DrawTriangle3D(nose, top, right, BLUE);
    DrawTriangle3D(nose, right, left, Color{60, 180, 255, 255});
    DrawTriangle3D(left, right, top, Fade(SKYBLUE, 0.75F));
}

} // namespace

void BoidRenderer::draw(const sim::BoidSimulation& simulation) const
{
    const auto& positions = simulation.positions();
    const auto& velocities = simulation.velocities();
    const float boid_scale = simulation.parameters().boid_scale;

    for (std::size_t i = 0; i < positions.size(); ++i) {
        draw_boid_tetrahedron(positions[i], velocities[i], boid_scale);
    }

    const float extent = simulation.parameters().world_half_extent;
    DrawCubeWires(Vector3{0.0F, 0.0F, 0.0F}, extent * 2.0F, extent * 2.0F, extent * 2.0F, Fade(LIGHTGRAY, 0.65F));
}

} // namespace flock3d::render
