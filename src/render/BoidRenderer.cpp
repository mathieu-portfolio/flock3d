#include "render/BoidRenderer.hpp"

#include <raylib.h>

namespace flock3d::render {

void BoidRenderer::draw(const sim::BoidSimulation& simulation) const
{
    constexpr float boid_radius = 0.16F;

    for (const Vector3 position : simulation.positions()) {
        DrawSphere(position, boid_radius, SKYBLUE);
    }

    const float extent = simulation.parameters().world_half_extent;
    DrawCubeWires(Vector3{0.0F, 0.0F, 0.0F}, extent * 2.0F, extent * 2.0F, extent * 2.0F, Fade(LIGHTGRAY, 0.65F));
}

} // namespace flock3d::render
