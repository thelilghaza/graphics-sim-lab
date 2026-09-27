#ifndef DESTRUCTION_CAMERA_HPP
#define DESTRUCTION_CAMERA_HPP

#include "destruction/math/vec3.hpp"
#include <array>
#include <cmath>
#include <algorithm>

namespace destruction::render {

using namespace destruction::math;

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
    Vec3 position{0.0f, 4.0f, 10.0f};
    Vec3 front{0.0f, -0.2f, -1.0f};
    Vec3 up{0.0f, 1.0f, 0.0f};
    Vec3 right{1.0f, 0.0f, 0.0f};
    Vec3 world_up{0.0f, 1.0f, 0.0f};

    float yaw{-90.0f};
    float pitch{-10.0f};

    float fov{60.0f};
    float near_clip{0.1f};
    float far_clip{1000.0f};
    float speed{8.0f};
    float sensitivity{0.1f};

    Camera() {
        update_camera_vectors();
    }

    Camera(const Vec3& pos, float initial_yaw = -90.0f, float initial_pitch = -10.0f)
        : position(pos), yaw(initial_yaw), pitch(initial_pitch) {
        update_camera_vectors();
    }

    void reset(const Vec3& pos, float initial_yaw = -90.0f, float initial_pitch = -10.0f) {
        position = pos;
        yaw = initial_yaw;
        pitch = initial_pitch;
        update_camera_vectors();
    }

    void process_keyboard(CameraMovement direction, float dt, bool fast_mode = false) {
        float velocity = speed * dt * (fast_mode ? 2.5f : 1.0f);
        if (direction == CameraMovement::Forward)
            position += front * velocity;
        if (direction == CameraMovement::Backward)
            position -= front * velocity;
        if (direction == CameraMovement::Left)
            position -= right * velocity;
        if (direction == CameraMovement::Right)
            position += right * velocity;
        if (direction == CameraMovement::Up)
            position += world_up * velocity;
        if (direction == CameraMovement::Down)
            position -= world_up * velocity;
    }

    void process_mouse_movement(float x_offset, float y_offset, bool constrain_pitch = true) {
        x_offset *= sensitivity;
        y_offset *= sensitivity;

        yaw += x_offset;
        pitch += y_offset;

        if (constrain_pitch) {
            pitch = std::clamp(pitch, -89.0f, 89.0f);
        }

        update_camera_vectors();
    }

    std::array<float, 16> get_view_matrix() const {
        // LookAt matrix in column-major order for OpenGL
        Vec3 f = front.normalize();
        Vec3 s = f.cross(world_up).normalize();
        Vec3 u = s.cross(f);

        std::array<float, 16> m{};
        // Col 0
        m[0] = s.x;
        m[1] = u.x;
        m[2] = -f.x;
        m[3] = 0.0f;

        // Col 1
        m[4] = s.y;
        m[5] = u.y;
        m[6] = -f.y;
        m[7] = 0.0f;

        // Col 2
        m[8] = s.z;
        m[9] = u.z;
        m[10] = -f.z;
        m[11] = 0.0f;

        // Col 3
        m[12] = -s.dot(position);
        m[13] = -u.dot(position);
        m[14] = f.dot(position);
        m[15] = 1.0f;

        return m;
    }

    std::array<float, 16> get_projection_matrix(float aspect) const {
        std::array<float, 16> m{};
        float f = 1.0f / std::tan((fov * 3.141592653589793f / 180.0f) * 0.5f);
        float nf = 1.0f / (near_clip - far_clip);

        // Column-major perspective matrix
        m[0] = f / aspect;
        m[1] = 0.0f;
        m[2] = 0.0f;
        m[3] = 0.0f;

        m[4] = 0.0f;
        m[5] = f;
        m[6] = 0.0f;
        m[7] = 0.0f;

        m[8] = 0.0f;
        m[9] = 0.0f;
        m[10] = (far_clip + near_clip) * nf;
        m[11] = -1.0f;

        m[12] = 0.0f;
        m[13] = 0.0f;
        m[14] = (2.0f * far_clip * near_clip) * nf;
        m[15] = 0.0f;

        return m;
    }

private:
    void update_camera_vectors() {
        float rad_yaw = yaw * 3.141592653589793f / 180.0f;
        float rad_pitch = pitch * 3.141592653589793f / 180.0f;

        Vec3 new_front(
            std::cos(rad_yaw) * std::cos(rad_pitch),
            std::sin(rad_pitch),
            std::sin(rad_yaw) * std::cos(rad_pitch)
        );
        front = new_front.normalize();
        right = front.cross(world_up).normalize();
        up = right.cross(front).normalize();
    }
};

} // namespace destruction::render

#endif // DESTRUCTION_CAMERA_HPP
