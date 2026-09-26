#pragma once

#include "voxel_lab/math.hpp"

namespace voxel_lab {

enum class CameraMovement {
    Forward,
    Backward,
    Left,
    Right,
    Up,
    Down
};

class Camera {
public:
    Camera(const Vec3& position = Vec3(48.0f, 48.0f, 64.0f),
           float yaw = -120.0f,
           float pitch = -25.0f);

    void process_keyboard(CameraMovement direction, float delta_time) noexcept;
    void process_mouse_movement(float x_offset, float y_offset, bool constrain_pitch = true) noexcept;

    void reset(const Vec3& position, float yaw, float pitch) noexcept;

    Mat4 get_view_matrix() const noexcept;
    Mat4 get_projection_matrix(float aspect) const noexcept;

    const Vec3& get_position() const noexcept { return position; }
    const Vec3& get_front() const noexcept { return front; }
    float get_yaw() const noexcept { return yaw; }
    float get_pitch() const noexcept { return pitch; }

    float fov{60.0f}; // In degrees
    float near_clip{0.1f};
    float far_clip{1000.0f};
    float speed{30.0f};
    float sensitivity{0.1f};

private:
    void update_camera_vectors() noexcept;

    Vec3 position;
    Vec3 front{0.0f, 0.0f, -1.0f};
    Vec3 up{0.0f, 1.0f, 0.0f};
    Vec3 right{1.0f, 0.0f, 0.0f};
    Vec3 world_up{0.0f, 1.0f, 0.0f};

    float yaw{-120.0f};
    float pitch{-25.0f};
};

} // namespace voxel_lab
