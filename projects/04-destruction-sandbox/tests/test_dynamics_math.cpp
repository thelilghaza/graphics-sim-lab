#include "destruction/math/math_utils.hpp"
#include "destruction/math/vec2.hpp"
#include "destruction/math/vec3.hpp"
#include "destruction/math/vec4.hpp"
#include "destruction/math/mat3.hpp"
#include "destruction/math/quat.hpp"
#include "destruction/math/transform.hpp"
#include "destruction/dynamics/inertia.hpp"
#include "destruction/dynamics/rigid_body.hpp"
#include "destruction/dynamics/integrator.hpp"
#include "destruction/dynamics/physics_world.hpp"

#include <iostream>
#include <cmath>
#include <cassert>

using namespace destruction::math;
using namespace destruction::dynamics;

void test_vec3() {
    Vec3 a(1.0f, 2.0f, 3.0f);
    Vec3 b(4.0f, 5.0f, 6.0f);

    Vec3 add = a + b;
    assert(is_nearly_equal(add.x, 5.0f));
    assert(is_nearly_equal(add.y, 7.0f));
    assert(is_nearly_equal(add.z, 9.0f));

    Vec3 sub = b - a;
    assert(is_nearly_equal(sub.x, 3.0f));
    assert(is_nearly_equal(sub.y, 3.0f));
    assert(is_nearly_equal(sub.z, 3.0f));

    float dot = a.dot(b);
    assert(is_nearly_equal(dot, 32.0f));
    (void)dot;

    Vec3 cross = Vec3::unit_x().cross(Vec3::unit_y());
    assert(cross == Vec3::unit_z());

    Vec3 zero = Vec3::zero();
    Vec3 norm_zero = zero.normalize();
    assert(norm_zero == Vec3::zero());
    assert(norm_zero.is_valid());

    Vec3 unit = a.normalize();
    assert(is_nearly_equal(unit.length(), 1.0f));
}

void test_mat3() {
    Mat3 identity = Mat3::identity();
    Vec3 v(3.0f, -2.0f, 5.0f);
    assert(identity * v == v);

    Mat3 rot_z = Mat3::rotation_z(HALF_PI);
    Vec3 rot_v = rot_z * Vec3::unit_x();
    assert(is_nearly_equal(rot_v.x, 0.0f));
    assert(is_nearly_equal(rot_v.y, 1.0f));
    assert(is_nearly_equal(rot_v.z, 0.0f));

    Mat3 m(
        1.0f, 2.0f, 3.0f,
        0.0f, 1.0f, 4.0f,
        5.0f, 6.0f, 0.0f
    );

    float det = m.determinant();
    assert(is_nearly_equal(det, 1.0f));
    (void)det;

    bool success = false;
    Mat3 inv = m.inverse(&success);
    assert(success);

    Mat3 prod = m * inv;
    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 3; ++j) {
            float expected = (i == j) ? 1.0f : 0.0f;
            assert(is_nearly_equal(prod(i, j), expected, 1e-4f));
            (void)expected;
        }
    }

    Mat3 singular = Mat3::zero();
    Mat3 inv_singular = singular.inverse(&success);
    assert(!success);
    assert(inv_singular == Mat3::zero());
}

void test_quat() {
    Quat id = Quat::identity();
    Vec3 v(1.0f, 2.0f, 3.0f);
    assert(id.rotate(v) == v);

    Quat rot_z = Quat::axis_angle(Vec3::unit_z(), HALF_PI);
    Vec3 rot_v = rot_z.rotate(Vec3::unit_x());
    assert(is_nearly_equal(rot_v.x, 0.0f));
    assert(is_nearly_equal(rot_v.y, 1.0f));
    assert(is_nearly_equal(rot_v.z, 0.0f));

    Mat3 mat_z = rot_z.to_mat3();
    Vec3 mat_v = mat_z * Vec3::unit_x();
    assert(is_nearly_equal(mat_v.x, rot_v.x));
    assert(is_nearly_equal(mat_v.y, rot_v.y));
    assert(is_nearly_equal(mat_v.z, rot_v.z));

    Quat inv_z = rot_z.inverse();
    Vec3 back_v = inv_z.rotate(rot_v);
    assert(is_nearly_equal(back_v.x, 1.0f));
    assert(is_nearly_equal(back_v.y, 0.0f));
    assert(is_nearly_equal(back_v.z, 0.0f));
}

void test_transform() {
    Vec3 pos(10.0f, -5.0f, 2.0f);
    Quat rot = Quat::axis_angle(Vec3::unit_y(), HALF_PI);
    Transform t(pos, rot);

    Vec3 local_p(1.0f, 0.0f, 0.0f);
    Vec3 world_p = t.transform_point(local_p);
    assert(is_nearly_equal(world_p.x, 10.0f));
    assert(is_nearly_equal(world_p.y, -5.0f));
    assert(is_nearly_equal(world_p.z, 1.0f));

    Vec3 back_p = t.inverse_transform_point(world_p);
    assert(is_nearly_equal(back_p.x, local_p.x));
    assert(is_nearly_equal(back_p.y, local_p.y));
    assert(is_nearly_equal(back_p.z, local_p.z));

    Transform inv_t = t.inverse();
    Vec3 inv_world_p = inv_t.transform_point(world_p);
    assert(is_nearly_equal(inv_world_p.x, local_p.x));
    assert(is_nearly_equal(inv_world_p.y, local_p.y));
    assert(is_nearly_equal(inv_world_p.z, local_p.z));
}

void test_inertia() {
    float mass = 12.0f;
    float w = 2.0f, h = 4.0f, d = 6.0f;
    InertiaTensor box_i = InertiaTensor::box(mass, w, h, d);

    float expected_ixx = (1.0f / 12.0f) * mass * (h * h + d * d); // 1/12 * 12 * (16+36) = 52
    float expected_iyy = (1.0f / 12.0f) * mass * (w * w + d * d); // 1/12 * 12 * (4+36) = 40
    float expected_izz = (1.0f / 12.0f) * mass * (w * w + h * h); // 1/12 * 12 * (4+16) = 20

    assert(is_nearly_equal(box_i.body_inertia.x, expected_ixx));
    assert(is_nearly_equal(box_i.body_inertia.y, expected_iyy));
    assert(is_nearly_equal(box_i.body_inertia.z, expected_izz));
    (void)expected_ixx;
    (void)expected_iyy;
    (void)expected_izz;

    Quat id_rot = Quat::identity();
    Mat3 world_inv = box_i.get_world_inv_inertia(id_rot);
    assert(is_nearly_equal(world_inv(0, 0), 1.0f / expected_ixx));
    assert(is_nearly_equal(world_inv(1, 1), 1.0f / expected_iyy));
    assert(is_nearly_equal(world_inv(2, 2), 1.0f / expected_izz));
}

void test_free_fall() {
    PhysicsWorld world;
    world.set_gravity(Vec3(0.0f, -9.81f, 0.0f));

    InertiaTensor sphere_i = InertiaTensor::sphere(1.0f, 0.5f);
    RigidBody body = RigidBody::create_dynamic(1, 1.0f, sphere_i, Vec3(0.0f, 10.0f, 0.0f));
    world.add_body(body);

    float dt = 1.0f / 60.0f;
    int steps = 60;

    for (int i = 0; i < steps; ++i) {
        world.step(dt);
    }

    const RigidBody* updated_body = world.get_body(1);
    assert(updated_body != nullptr);

    float expected_v_y = steps * (-9.81f) * dt; // -9.81 m/s
    float expected_y = 10.0f + ((steps * (steps + 1)) / 2.0f) * (-9.81f) * (dt * dt);

    assert(is_nearly_equal(updated_body->linear_velocity.y, expected_v_y, 1e-3f));
    assert(is_nearly_equal(updated_body->position.y, expected_y, 1e-3f));
    (void)expected_v_y;
    (void)expected_y;
    (void)updated_body;
}

void test_constant_force_and_torque() {
    PhysicsWorld world;
    world.set_gravity(Vec3::zero());

    InertiaTensor box_i = InertiaTensor::box(2.0f, 1.0f, 1.0f, 1.0f);
    RigidBody body = RigidBody::create_dynamic(10, 2.0f, box_i, Vec3::zero());
    world.add_body(body);

    RigidBody* b = world.get_body(10);
    b->apply_force(Vec3(10.0f, 0.0f, 0.0f));
    b->apply_torque(Vec3(0.0f, 10.0f, 0.0f));

    float dt = 1.0f / 60.0f;
    world.step(dt);

    assert(is_nearly_equal(b->linear_velocity.x, 5.0f * dt, 1e-4f));
    assert(is_nearly_equal(b->angular_velocity.y, (10.0f / box_i.body_inertia.y) * dt, 1e-4f));
}

void test_determinism() {
    auto run_sim = [](Vec3* final_pos, Quat* final_rot) {
        PhysicsWorld world;
        world.set_gravity(Vec3(0.0f, -9.81f, 0.0f));
        InertiaTensor box_i = InertiaTensor::box(5.0f, 2.0f, 3.0f, 1.0f);
        RigidBody body = RigidBody::create_dynamic(1, 5.0f, box_i, Vec3(1.0f, 2.0f, 3.0f), Quat::axis_angle(Vec3(1,1,0), 0.5f));
        body.angular_velocity = Vec3(0.1f, 0.5f, -0.2f);
        body.linear_velocity = Vec3(2.0f, 0.0f, -1.0f);
        world.add_body(body);

        float dt = 1.0f / 60.0f;
        for (int i = 0; i < 100; ++i) {
            world.step(dt);
        }
        const RigidBody* result = world.get_body(1);
        *final_pos = result->position;
        *final_rot = result->orientation;
    };

    Vec3 pos1, pos2;
    Quat rot1, rot2;
    run_sim(&pos1, &rot1);
    run_sim(&pos2, &rot2);

    assert(pos1.x == pos2.x && pos1.y == pos2.y && pos1.z == pos2.z);
    assert(rot1.w == rot2.w && rot1.x == rot2.x && rot1.y == rot2.y && rot1.z == rot2.z);
}

int main() {
    std::cout << "Running Project 04 Milestone 1 Math & Dynamics Unit Tests...\n";

    test_vec3();
    test_mat3();
    test_quat();
    test_transform();
    test_inertia();
    test_free_fall();
    test_constant_force_and_torque();
    test_determinism();

    std::cout << "All Project 04 Milestone 1 Unit Tests Passed Successfully!\n";
    return 0;
}
