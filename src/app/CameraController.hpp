#pragma once

#include <raylib.h>

#include <flock3d/app/ControlHelpers.hpp>

namespace flock3d::app {

class CameraController {
public:
    explicit CameraController(CameraSettings settings = {});
    ~CameraController();

    CameraController(const CameraController&) = delete;
    CameraController& operator=(const CameraController&) = delete;
    CameraController(CameraController&&) = delete;
    CameraController& operator=(CameraController&&) = delete;

    void initialize_from_camera(const Camera3D& camera) noexcept;
    bool update(Camera3D& camera, float delta_seconds);

    [[nodiscard]] float move_speed() const noexcept { return move_speed_; }
    [[nodiscard]] const CameraSettings& settings() const noexcept { return settings_; }

private:
    CameraSettings settings_{};
    float move_speed_{};
    float yaw_{};
    float pitch_{};
    bool cursor_captured_{};
};

} // namespace flock3d::app
