#include "voxel_lab/camera.hpp"
#include "voxel_lab/math.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

using namespace voxel_lab;

void test_vec3_operations() {
    Vec3 a(1.0f, 2.0f, 3.0f);
    Vec3 b(4.0f, 5.0f, 6.0f);

    Vec3 add = a + b;
    assert(add.x == 5.0f && add.y == 7.0f && add.z == 9.0f);

    Vec3 sub = b - a;
    assert(sub.x == 3.0f && sub.y == 3.0f && sub.z == 3.0f);

    float d = dot(a, b);
    assert(d == (1.0f*4.0f + 2.0f*5.0f + 3.0f*6.0f));
    static_cast<void>(d);

    Vec3 c = cross(Vec3(1.0f, 0.0f, 0.0f), Vec3(0.0f, 1.0f, 0.0f));
    assert(c == Vec3(0.0f, 0.0f, 1.0f));

    Vec3 norm = normalize(Vec3(0.0f, 5.0f, 0.0f));
    assert(std::abs(norm.y - 1.0f) < 0.0001f);

    std::cout << "[PASS] test_vec3_operations\n";
}

void test_mat4_matrices() {
    Mat4 id = Mat4::identity();
    assert(id.m[0] == 1.0f && id.m[5] == 1.0f && id.m[10] == 1.0f && id.m[15] == 1.0f);
    assert(id.m[1] == 0.0f && id.m[4] == 0.0f);

    Mat4 look = Mat4::look_at(Vec3(0.0f, 0.0f, 5.0f), Vec3(0.0f, 0.0f, 0.0f), Vec3(0.0f, 1.0f, 0.0f));
    assert(look.data() != nullptr);

    Mat4 proj = Mat4::perspective(3.14159f / 3.0f, 16.0f / 9.0f, 0.1f, 100.0f);
    assert(proj.data() != nullptr);

    std::cout << "[PASS] test_mat4_matrices\n";
}

void test_camera_navigation() {
    Camera camera(Vec3(0.0f, 0.0f, 10.0f), -90.0f, 0.0f);

    Vec3 initial_pos = camera.get_position();

    // Moving forward with yaw = -90 moves along -Z
    camera.process_keyboard(CameraMovement::Forward, 0.1f);
    assert(camera.get_position().z < initial_pos.z);

    // Moving backward moves along +Z
    camera.process_keyboard(CameraMovement::Backward, 0.1f);
    assert(std::abs(camera.get_position().z - initial_pos.z) < 0.001f);

    // Up / down
    camera.process_keyboard(CameraMovement::Up, 0.1f);
    assert(camera.get_position().y > initial_pos.y);

    camera.process_keyboard(CameraMovement::Down, 0.1f);
    assert(std::abs(camera.get_position().y - initial_pos.y) < 0.001f);

    // Mouse movement updates pitch and yaw
    float old_yaw = camera.get_yaw();
    camera.process_mouse_movement(10.0f, 5.0f);
    assert(camera.get_yaw() > old_yaw);
    static_cast<void>(old_yaw);

    // View matrix generation
    Mat4 view = camera.get_view_matrix();
    assert(view.data() != nullptr);

    std::cout << "[PASS] test_camera_navigation\n";
}

int main() {
    test_vec3_operations();
    test_mat4_matrices();
    test_camera_navigation();
    std::cout << "All voxel camera unit tests passed successfully!\n";
    return 0;
}
