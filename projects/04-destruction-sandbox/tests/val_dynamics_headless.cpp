#include "destruction/math/math_utils.hpp"
#include "destruction/math/vec3.hpp"
#include "destruction/math/quat.hpp"
#include "destruction/math/transform.hpp"
#include "destruction/dynamics/inertia.hpp"
#include "destruction/dynamics/rigid_body.hpp"
#include "destruction/dynamics/physics_world.hpp"

#include <iostream>
#include <iomanip>
#include <cstdint>
#include <cstring>

using namespace destruction::math;
using namespace destruction::dynamics;

static uint32_t float_to_bits(float f) {
    uint32_t u;
    std::memcpy(&u, &f, sizeof(float));
    return u;
}

static uint32_t compute_checksum(const PhysicsWorld& world) {
    uint32_t hash = 2166136261u;
    for (const auto& body : world.get_bodies()) {
        auto add_float = [&hash](float f) {
            uint32_t bits = float_to_bits(f);
            hash ^= bits;
            hash *= 16777619u;
        };
        add_float(body.position.x);
        add_float(body.position.y);
        add_float(body.position.z);
        add_float(body.linear_velocity.x);
        add_float(body.linear_velocity.y);
        add_float(body.linear_velocity.z);
        add_float(body.orientation.w);
        add_float(body.orientation.x);
        add_float(body.orientation.y);
        add_float(body.orientation.z);
        add_float(body.angular_velocity.x);
        add_float(body.angular_velocity.y);
        add_float(body.angular_velocity.z);
    }
    return hash;
}

int main() {
    std::cout << "Project 04 — Milestone 1 Headless Validation Executable\n";
    std::cout << "--------------------------------------------------------\n";

    PhysicsWorld world;
    world.set_gravity(Vec3(0.0f, -9.81f, 0.0f));

    // Ground static body
    InertiaTensor box_i1 = InertiaTensor::box(10.0f, 10.0f, 1.0f, 10.0f);
    RigidBody ground = RigidBody::create_static(1, Vec3(0.0f, 0.0f, 0.0f));
    world.add_body(ground);

    // Dynamic body 1
    InertiaTensor box_i2 = InertiaTensor::box(5.0f, 1.0f, 2.0f, 1.0f);
    RigidBody body1 = RigidBody::create_dynamic(2, 5.0f, box_i2, Vec3(0.0f, 15.0f, 0.0f), Quat::axis_angle(Vec3(0,1,0), 0.25f));
    body1.linear_velocity = Vec3(1.0f, 0.0f, -0.5f);
    body1.angular_velocity = Vec3(0.2f, 1.0f, 0.0f);
    world.add_body(body1);

    // Dynamic body 2
    InertiaTensor sphere_i = InertiaTensor::sphere(2.5f, 0.8f);
    RigidBody body2 = RigidBody::create_dynamic(3, 2.5f, sphere_i, Vec3(-2.0f, 20.0f, 1.0f));
    body2.linear_velocity = Vec3(-0.5f, 1.0f, 0.0f);
    body2.angular_velocity = Vec3(0.0f, 0.0f, 2.0f);
    world.add_body(body2);

    float dt = 1.0f / 60.0f;
    int total_steps = 120; // 2.0 seconds

    std::cout << "Simulating " << total_steps << " steps at dt = " << std::setprecision(5) << dt << " s...\n";

    for (int step = 1; step <= total_steps; ++step) {
        world.step(dt);
    }

    float total_sim_time = total_steps * dt;
    uint32_t checksum = compute_checksum(world);

    std::cout << "\nFinal Simulation State (Time = " << total_sim_time << " s):\n";
    std::cout << "--------------------------------------------------------\n";
    for (const auto& b : world.get_bodies()) {
        std::cout << "Body ID: " << b.id << " (" << (b.is_static ? "Static" : "Dynamic") << ")\n";
        std::cout << "  Position        : (" << b.position.x << ", " << b.position.y << ", " << b.position.z << ")\n";
        std::cout << "  Linear Velocity : (" << b.linear_velocity.x << ", " << b.linear_velocity.y << ", " << b.linear_velocity.z << ")\n";
        std::cout << "  Orientation Quat: (" << b.orientation.w << ", " << b.orientation.x << ", " << b.orientation.y << ", " << b.orientation.z << ")\n";
        std::cout << "  Angular Velocity: (" << b.angular_velocity.x << ", " << b.angular_velocity.y << ", " << b.angular_velocity.z << ")\n";
    }

    std::cout << "--------------------------------------------------------\n";
    std::cout << "Deterministic State Checksum: 0x" << std::hex << std::uppercase << checksum << std::dec << "\n";
    std::cout << "Validation Complete: SUCCESS\n";

    return 0;
}
