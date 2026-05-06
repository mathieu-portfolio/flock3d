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

[[nodiscard]] constexpr float dot(Vector3 lhs, Vector3 rhs) noexcept
{
    return (lhs.x * rhs.x) + (lhs.y * rhs.y) + (lhs.z * rhs.z);
}

[[nodiscard]] constexpr Vector3 cross(Vector3 lhs, Vector3 rhs) noexcept
{
    return Vector3{
        (lhs.y * rhs.z) - (lhs.z * rhs.y),
        (lhs.z * rhs.x) - (lhs.x * rhs.z),
        (lhs.x * rhs.y) - (lhs.y * rhs.x),
    };
}

[[nodiscard]] constexpr float length_squared(Vector3 value) noexcept
{
    return dot(value, value);
}

[[nodiscard]] inline float length(Vector3 value) noexcept
{
    return std::sqrt(length_squared(value));
}

[[nodiscard]] inline Vector3 normalize_safe(Vector3 value) noexcept
{
    const float squared = length_squared(value);
    if (squared <= 0.000001F) {
        return Vector3{};
    }

    return scale(value, 1.0F / std::sqrt(squared));
}

[[nodiscard]] inline Vector3 clamp_length(Vector3 value, float max_length) noexcept
{
    if (max_length <= 0.0F) {
        return Vector3{};
    }

    const float squared = length_squared(value);
    const float max_squared = max_length * max_length;
    if (squared <= max_squared || squared <= 0.0F) {
        return value;
    }

    return scale(value, max_length / std::sqrt(squared));
}

} // namespace flock3d::math
