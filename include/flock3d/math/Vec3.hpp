#pragma once

#include <cmath>

#include <raylib.h>

namespace flock3d::math {

[[nodiscard]] constexpr Vector3 make_vec3(float x, float y, float z) noexcept
{
    return Vector3{x, y, z};
}

[[nodiscard]] constexpr Vector3 add(Vector3 lhs, Vector3 rhs) noexcept
{
    return Vector3{lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

[[nodiscard]] constexpr Vector3 scale(Vector3 value, float scalar) noexcept
{
    return Vector3{value.x * scalar, value.y * scalar, value.z * scalar};
}

[[nodiscard]] constexpr Vector3 subtract(Vector3 lhs, Vector3 rhs) noexcept
{
    return Vector3{lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

[[nodiscard]] constexpr float length_squared(Vector3 value) noexcept
{
    return (value.x * value.x) + (value.y * value.y) + (value.z * value.z);
}

[[nodiscard]] inline float length(Vector3 value) noexcept
{
    return std::sqrt(length_squared(value));
}

[[nodiscard]] inline Vector3 clamp_length(Vector3 value, float max_length) noexcept
{
    const float squared = length_squared(value);
    const float max_squared = max_length * max_length;
    if (squared <= max_squared || squared <= 0.0F) {
        return value;
    }

    return scale(value, max_length / std::sqrt(squared));
}

} // namespace flock3d::math
