#include "app/CameraController.hpp"

#include <algorithm>
#include <cmath>

namespace flock3d::app {
namespace {

constexpr float pitch_limit = 1.50F;
constexpr float wheel_speed_step = 4.0F;

[[nodiscard]] Vector3 add(Vector3 lhs, Vector3 rhs) noexcept
{
    return Vector3{lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

[[nodiscard]] Vector3 subtract(Vector3 lhs, Vector3 rhs) noexcept
{
    return Vector3{lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

[[nodiscard]] Vector3 scale(Vector3 value, float amount) noexcept
{
    return Vector3{value.x * amount, value.y * amount, value.z * amount};
}

[[nodiscard]] float length(Vector3 value) noexcept
{
    return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

[[nodiscard]] Vector3 normalize(Vector3 value) noexcept
{
    const float magnitude = length(value);
    if (magnitude <= 0.0001F) {
        return Vector3{};
    }
    return scale(value, 1.0F / magnitude);
}

[[nodiscard]] Vector3 cross(Vector3 lhs, Vector3 rhs) noexcept
{
    return Vector3{
        lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.z * rhs.x - lhs.x * rhs.z,
        lhs.x * rhs.y - lhs.y * rhs.x,
    };
}

[[nodiscard]] Vector3 forward_from_angles(float yaw, float pitch) noexcept
{
    const float cos_pitch = std::cos(pitch);
    return normalize(Vector3{std::sin(yaw) * cos_pitch, std::sin(pitch), std::cos(yaw) * cos_pitch});
}

[[nodiscard]] bool ctrl_down() noexcept
{
    return IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL);
}

[[nodiscard]] bool shift_down() noexcept
{
    return IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);
}

} // namespace

CameraController::CameraController(CameraSettings settings)
    : settings_{settings}
    , move_speed_{clamped_camera_speed(settings.base_move_speed, settings)}
{
}

CameraController::~CameraController()
{
    if (cursor_captured_ && IsWindowReady()) {
        EnableCursor();
    }
}

void CameraController::initialize_from_camera(const Camera3D& camera) noexcept
{
    const Vector3 forward = normalize(subtract(camera.target, camera.position));
    yaw_ = std::atan2(forward.x, forward.z);
    pitch_ = std::asin(std::clamp(forward.y, -1.0F, 1.0F));
}

bool CameraController::update(Camera3D& camera, float delta_seconds)
{
    bool changed = false;

    const float wheel = GetMouseWheelMove();
    if (wheel != 0.0F) {
        move_speed_ = clamped_camera_speed(move_speed_ + wheel * wheel_speed_step, settings_);
        changed = true;
    }

    if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
        if (!cursor_captured_) {
            DisableCursor();
            cursor_captured_ = true;
        }
        const Vector2 delta = GetMouseDelta();
        yaw_ -= delta.x * settings_.mouse_sensitivity;
        pitch_ = std::clamp(pitch_ - delta.y * settings_.mouse_sensitivity, -pitch_limit, pitch_limit);
        changed = changed || delta.x != 0.0F || delta.y != 0.0F;
    } else if (cursor_captured_) {
        EnableCursor();
        cursor_captured_ = false;
    }

    const Vector3 world_up{0.0F, 1.0F, 0.0F};
    const Vector3 forward = forward_from_angles(yaw_, pitch_);
    const Vector3 right = normalize(cross(forward, world_up));
    Vector3 movement{};

    if (IsKeyDown(KEY_W)) {
        movement = add(movement, forward);
    }
    if (IsKeyDown(KEY_S)) {
        movement = subtract(movement, forward);
    }
    if (IsKeyDown(KEY_D)) {
        movement = add(movement, right);
    }
    if (IsKeyDown(KEY_A)) {
        movement = subtract(movement, right);
    }
    if (IsKeyDown(KEY_SPACE)) {
        movement = add(movement, world_up);
    }
    if (ctrl_down() || IsKeyDown(KEY_C)) {
        movement = subtract(movement, world_up);
    }

    const float movement_length = length(movement);
    if (movement_length > 0.0001F && delta_seconds > 0.0F) {
        const float speed = move_speed_ * (shift_down() ? settings_.fast_move_multiplier : 1.0F);
        camera.position = add(camera.position, scale(movement, speed * delta_seconds / movement_length));
        changed = true;
    }

    camera.up = world_up;
    camera.target = add(camera.position, forward);
    return changed;
}

} // namespace flock3d::app
