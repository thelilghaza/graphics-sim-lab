#include "voxel_lab/camera.hpp"
#include <algorithm>
#include <cmath>

namespace voxel_lab {

namespace {
constexpr float PI = 3.14159265358979323846f;
inline float to_radians(float degrees) noexcept {
    return degrees * (PI / 180.0f);
}
} // namespace

Camera::Camera(const Vec3& pos, float initial_yaw, float initial_pitch)
    : position(pos), yaw(initial_yaw), pitch(initial_pitch) {
    update_camera_vectors();
}

void Camera::reset(const Vec3& pos, float new_yaw, float new_pitch) noexcept {
    position = pos;
    yaw = new_yaw;
    pitch = new_pitch;
    update_camera_vectors();
}

void Camera::process_keyboard(CameraMovement direction, float delta_time) noexcept {
    float velocity = speed * delta_time;
    switch (direction) {
        case CameraMovement::Forward:
            position += front * velocity;
            break;
        case CameraMovement::Backward:
            position -= front * velocity;
            break;
        case CameraMovement::Left:
            position -= right * velocity;
            break;
        case CameraMovement::Right:
            position += right * velocity;
            break;
        case CameraMovement::Up:
            position += world_up * velocity;
            break;
        case CameraMovement::Down:
            position -= world_up * velocity;
            break;
    }
}

void Camera::process_mouse_movement(float x_offset, float y_offset, bool constrain_pitch) noexcept {
    x_offset *= sensitivity;
    y_offset *= sensitivity;

    yaw += x_offset;
    pitch += y_offset;

    if (constrain_pitch) {
        pitch = std::clamp(pitch, -89.0f, 89.0f);
    }

    update_camera_vectors();
}

Mat4 Camera::get_view_matrix() const noexcept {
    return Mat4::look_at(position, position + front, up);
}

Mat4 Camera::get_projection_matrix(float aspect) const noexcept {
    return Mat4::perspective(to_radians(fov), aspect, near_clip, far_clip);
}

void Camera::update_camera_vectors() noexcept {
    float y_rad = to_radians(yaw);
    float p_rad = to_radians(pitch);

    Vec3 new_front;
    new_front.x = std::cos(y_rad) * std::cos(p_rad);
    new_front.y = std::sin(p_rad);
    new_front.z = std::sin(y_rad) * std::cos(p_rad);
    front = normalize(new_front);

    right = normalize(cross(front, world_up));
    up = normalize(cross(right, front));
}

} // namespace voxel_lab
