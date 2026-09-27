#include "destruction/dynamics/physics_world.hpp"
#include "destruction/solver/solver_settings.hpp"
#include "destruction/collision/narrowphase.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace destruction::math;
using namespace destruction::dynamics;
using namespace destruction::collision;
using namespace destruction::solver;

void test_resting_single_box() {
    std::cout << "[Test] Single Box Resting Stability on Static Ground..." << std::endl;

    PhysicsWorld world;
    world.set_gravity(Vec3(0.0f, -9.81f, 0.0f));

    SolverSettings settings;
    settings.velocity_iterations = 10;
    settings.position_iterations = 5;
    settings.default_friction = 0.4f;
    settings.default_restitution = 0.15f;
    settings.penetration_slop = 0.005f;
    settings.baumgarte_beta = 0.25f;
    world.set_solver_settings(settings);

    // Ground: pos=(0,-0.5,0), extents=(10,0.5,10) -> top at y=0.0
    RigidBody ground = RigidBody::create_static(1, Vec3(0.0f, -0.5f, 0.0f));
    Collider ground_col = Collider::create_box(1, 1, Vec3(10.0f, 0.5f, 10.0f), ground.get_transform());
    world.add_body(ground);
    world.add_collider(ground_col);

    // Dynamic Box: pos=(0, 0.5, 0), extents=(0.5,0.5,0.5) -> bottom at y=0.0
    InertiaTensor inertia = InertiaTensor::box(10.0f, 1.0f, 1.0f, 1.0f);
    RigidBody box = RigidBody::create_dynamic(2, 10.0f, inertia, Vec3(0.0f, 0.5f, 0.0f));
    Collider box_col = Collider::create_box(2, 2, Vec3(0.5f, 0.5f, 0.5f), box.get_transform());
    world.add_body(box);
    world.add_collider(box_col);

    float dt = 1.0f / 60.0f;
    for (int step = 1; step <= 120; ++step) {
        world.step_full(dt);
    }

    const RigidBody* b = world.get_body(2);
    assert(b != nullptr);

    // Box position y must remain strictly bounded near 0.500m (penetration < 1.0mm)
    float pen_depth = 0.5f - b->position.y;
    assert(std::abs(pen_depth) < 0.001f && "Single resting box y position drift must be under 1.0mm");

    // Vertical velocity must remain near zero
    assert(std::abs(b->linear_velocity.y) < 1e-3f && "Single resting box vertical velocity must be near zero");

    // Center position X and Z must remain stable
    assert(std::abs(b->position.x) < 1e-4f && "Single resting box X position drift must be near zero");
    assert(std::abs(b->position.z) < 1e-4f && "Single resting box Z position drift must be near zero");

    // Static ground must remain completely unchanged
    const RigidBody* g = world.get_body(1);
    assert(g->position == Vec3(0.0f, -0.5f, 0.0f));
    assert(g->linear_velocity == Vec3::zero());
    assert(g->angular_velocity == Vec3::zero());

    std::cout << "  PASSED." << std::endl;
}

void test_two_box_stack() {
    std::cout << "[Test] Two-Box Vertical Stack Resting Stability..." << std::endl;

    PhysicsWorld world;
    world.set_gravity(Vec3(0.0f, -9.81f, 0.0f));

    SolverSettings settings;
    settings.velocity_iterations = 10;
    settings.position_iterations = 5;
    settings.default_friction = 0.4f;
    settings.default_restitution = 0.15f;
    settings.penetration_slop = 0.005f;
    settings.baumgarte_beta = 0.25f;
    world.set_solver_settings(settings);

    RigidBody ground = RigidBody::create_static(1, Vec3(0.0f, -0.5f, 0.0f));
    Collider ground_col = Collider::create_box(1, 1, Vec3(10.0f, 0.5f, 10.0f), ground.get_transform());
    world.add_body(ground);
    world.add_collider(ground_col);

    InertiaTensor inertia = InertiaTensor::box(10.0f, 1.0f, 1.0f, 1.0f);
    RigidBody b1 = RigidBody::create_dynamic(2, 10.0f, inertia, Vec3(0.0f, 0.5f, 0.0f));
    Collider col1 = Collider::create_box(2, 2, Vec3(0.5f, 0.5f, 0.5f), b1.get_transform());
    world.add_body(b1);
    world.add_collider(col1);

    RigidBody b2 = RigidBody::create_dynamic(3, 10.0f, inertia, Vec3(0.0f, 1.5f, 0.0f));
    Collider col2 = Collider::create_box(3, 3, Vec3(0.5f, 0.5f, 0.5f), b2.get_transform());
    world.add_body(b2);
    world.add_collider(col2);

    float dt = 1.0f / 60.0f;
    for (int step = 1; step <= 120; ++step) {
        world.step_full(dt);
    }

    const RigidBody* p1 = world.get_body(2);
    const RigidBody* p2 = world.get_body(3);

    // Box 1 y must be bounded near 0.500m
    float pen1 = 0.5f - p1->position.y;
    assert(std::abs(pen1) < 0.002f && "Lower box y position drift must be under 2.0mm");

    // Box 2 y must be bounded near 1.500m
    float pen2 = 1.5f - p2->position.y;
    assert(std::abs(pen2) < 0.002f && "Upper box y position drift must be under 2.0mm");

    // Velocities near zero
    assert(std::abs(p1->linear_velocity.y) < 1e-3f);
    assert(std::abs(p2->linear_velocity.y) < 1e-3f);

    std::cout << "  PASSED." << std::endl;
}

void test_three_box_stack() {
    std::cout << "[Test] Three-Box Vertical Stack Resting Stability..." << std::endl;

    PhysicsWorld world;
    world.set_gravity(Vec3(0.0f, -9.81f, 0.0f));

    SolverSettings settings;
    settings.velocity_iterations = 15;
    settings.position_iterations = 8;
    settings.default_friction = 0.4f;
    settings.default_restitution = 0.15f;
    settings.penetration_slop = 0.005f;
    settings.baumgarte_beta = 0.25f;
    world.set_solver_settings(settings);

    RigidBody ground = RigidBody::create_static(1, Vec3(0.0f, -0.5f, 0.0f));
    Collider ground_col = Collider::create_box(1, 1, Vec3(10.0f, 0.5f, 10.0f), ground.get_transform());
    world.add_body(ground);
    world.add_collider(ground_col);

    InertiaTensor inertia = InertiaTensor::box(10.0f, 1.0f, 1.0f, 1.0f);
    RigidBody b1 = RigidBody::create_dynamic(2, 10.0f, inertia, Vec3(0.0f, 0.5f, 0.0f));
    Collider col1 = Collider::create_box(2, 2, Vec3(0.5f, 0.5f, 0.5f), b1.get_transform());
    world.add_body(b1);
    world.add_collider(col1);

    RigidBody b2 = RigidBody::create_dynamic(3, 10.0f, inertia, Vec3(0.0f, 1.5f, 0.0f));
    Collider col2 = Collider::create_box(3, 3, Vec3(0.5f, 0.5f, 0.5f), b2.get_transform());
    world.add_body(b2);
    world.add_collider(col2);

    RigidBody b3 = RigidBody::create_dynamic(4, 10.0f, inertia, Vec3(0.0f, 2.5f, 0.0f));
    Collider col3 = Collider::create_box(4, 4, Vec3(0.5f, 0.5f, 0.5f), b3.get_transform());
    world.add_body(b3);
    world.add_collider(col3);

    float dt = 1.0f / 60.0f;
    for (int step = 1; step <= 120; ++step) {
        world.step_full(dt);
    }

    const RigidBody* p1 = world.get_body(2);
    const RigidBody* p2 = world.get_body(3);
    const RigidBody* p3 = world.get_body(4);

    // Box 1 y bounded (under 10mm drift)
    assert(std::abs(0.5f - p1->position.y) < 0.01f);
    // Box 2 y bounded (under 80mm drift)
    assert(std::abs(1.5f - p2->position.y) < 0.08f);
    // Box 3 y bounded (under 80mm drift)
    assert(std::abs(2.5f - p3->position.y) < 0.08f);

    std::cout << "  PASSED." << std::endl;
}

void test_face_face_contact_generation() {
    std::cout << "[Test] Box Face-Face Contact Normal & Corner Point Verification..." << std::endl;

    Transform t_a(Vec3(0.0f, 0.5f, 0.0f), Quat::identity());
    Transform t_b(Vec3(0.0f, 1.5f, 0.0f), Quat::identity());

    Collider col_a = Collider::create_box(1, 1, Vec3(0.5f, 0.5f, 0.5f), t_a);
    Collider col_b = Collider::create_box(2, 2, Vec3(0.5f, 0.5f, 0.5f), t_b);

    ContactManifold manifold = Narrowphase::collide(col_a, col_b);
    assert(!manifold.points.empty());
    assert(manifold.points.size() == 4 && "Box-box face contact must emit 4 support points");

    // Contact normal pointing from A to B must be (0, 1, 0)
    assert((manifold.normal - Vec3(0.0f, 1.0f, 0.0f)).length() < 1e-4f);

    // Verify all 4 feature IDs are unique 1, 2, 3, 4
    std::vector<uint32_t> feat_ids;
    for (const auto& pt : manifold.points) {
        feat_ids.push_back(pt.feature_id);
    }
    std::sort(feat_ids.begin(), feat_ids.end());
    assert(feat_ids == std::vector<uint32_t>({1, 2, 3, 4}) && "Feature IDs must be stable 1, 2, 3, 4");

    std::cout << "  PASSED." << std::endl;
}

void test_restitution_thresholding() {
    std::cout << "[Test] Restitution Thresholding (Zero Repeated Bounce)..." << std::endl;

    PhysicsWorld world;
    world.set_gravity(Vec3(0.0f, -9.81f, 0.0f));

    SolverSettings settings;
    settings.default_restitution = 0.8f; // High restitution setting
    settings.restitution_threshold = 0.5f; // Threshold at 0.5 m/s
    world.set_solver_settings(settings);

    RigidBody ground = RigidBody::create_static(1, Vec3(0.0f, -0.5f, 0.0f));
    Collider ground_col = Collider::create_box(1, 1, Vec3(10.0f, 0.5f, 10.0f), ground.get_transform());
    world.add_body(ground);
    world.add_collider(ground_col);

    InertiaTensor inertia = InertiaTensor::box(10.0f, 1.0f, 1.0f, 1.0f);
    // Box placed slightly touching ground with small downward velocity below threshold (0.1 m/s < 0.5 m/s)
    RigidBody box = RigidBody::create_dynamic(2, 10.0f, inertia, Vec3(0.0f, 0.501f, 0.0f));
    box.linear_velocity = Vec3(0.0f, -0.1f, 0.0f);
    Collider box_col = Collider::create_box(2, 2, Vec3(0.5f, 0.5f, 0.5f), box.get_transform());
    world.add_body(box);
    world.add_collider(box_col);

    world.step_full(1.0f / 60.0f);

    const RigidBody* b = world.get_body(2);
    // Velocity should NOT bounce upward with 0.8 * 0.1 = 0.08 m/s! It should be clamped to zero
    assert(b->linear_velocity.y <= 0.01f && "Resting contact below threshold must not bounce");

    std::cout << "  PASSED." << std::endl;
}

void test_deterministic_repetition() {
    std::cout << "[Test] Deterministic Repetition Bit-Exact State Match..." << std::endl;

    auto run_sim = []() -> Vec3 {
        PhysicsWorld world;
        world.set_gravity(Vec3(0.0f, -9.81f, 0.0f));
        SolverSettings settings;
        settings.velocity_iterations = 10;
        settings.position_iterations = 5;
        world.set_solver_settings(settings);

        RigidBody ground = RigidBody::create_static(1, Vec3(0.0f, -0.5f, 0.0f));
        Collider ground_col = Collider::create_box(1, 1, Vec3(10.0f, 0.5f, 10.0f), ground.get_transform());
        world.add_body(ground);
        world.add_collider(ground_col);

        InertiaTensor inertia = InertiaTensor::box(10.0f, 1.0f, 1.0f, 1.0f);
        RigidBody b1 = RigidBody::create_dynamic(2, 10.0f, inertia, Vec3(0.0f, 0.5f, 0.0f));
        Collider col1 = Collider::create_box(2, 2, Vec3(0.5f, 0.5f, 0.5f), b1.get_transform());
        world.add_body(b1);
        world.add_collider(col1);

        RigidBody b2 = RigidBody::create_dynamic(3, 10.0f, inertia, Vec3(0.0f, 1.5f, 0.0f));
        Collider col2 = Collider::create_box(3, 3, Vec3(0.5f, 0.5f, 0.5f), b2.get_transform());
        world.add_body(b2);
        world.add_collider(col2);

        float dt = 1.0f / 60.0f;
        for (int step = 0; step < 120; ++step) {
            world.step_full(dt);
        }
        return world.get_body(3)->position;
    };

    Vec3 pos_run1 = run_sim();
    Vec3 pos_run2 = run_sim();

    assert(pos_run1 == pos_run2 && "Repeated simulations must produce bit-exact identical final positions");
    std::cout << "  PASSED." << std::endl;
}

int main() {
    std::cout << "========================================================\n";
    std::cout << "Running Project 04 Contact Stability Acceptance Tests\n";
    std::cout << "========================================================\n";

    test_resting_single_box();
    test_two_box_stack();
    test_three_box_stack();
    test_face_face_contact_generation();
    test_restitution_thresholding();
    test_deterministic_repetition();

    std::cout << "========================================================\n";
    std::cout << "All Contact Stability Acceptance Tests Passed Successfully.\n";
    std::cout << "========================================================\n";
    return 0;
}
