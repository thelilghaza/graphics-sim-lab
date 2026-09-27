#include "destruction/math/vec3.hpp"
#include "destruction/math/quat.hpp"
#include "destruction/math/transform.hpp"
#include "destruction/dynamics/rigid_body.hpp"
#include "destruction/dynamics/physics_world.hpp"
#include "destruction/collision/collider.hpp"
#include "destruction/collision/contact_manifold.hpp"
#include "destruction/solver/solver_settings.hpp"
#include "destruction/solver/contact_constraint.hpp"
#include "destruction/solver/warm_start_cache.hpp"
#include "destruction/solver/sequential_impulse_solver.hpp"
#include "destruction/graph/material_params.hpp"
#include "destruction/graph/structural_graph.hpp"

#include <iostream>
#include <cassert>
#include <cmath>
#include <vector>

using namespace destruction::math;
using namespace destruction::dynamics;
using namespace destruction::collision;
using namespace destruction::solver;
using namespace destruction::graph;

void test_analytical_equal_mass_headon() {
    std::cout << "[Test] Analytical Head-on Equal Mass Collision..." << std::endl;

    InertiaTensor inertia = InertiaTensor::box(1.0f, 1.0f, 1.0f, 1.0f);
    RigidBody body_a = RigidBody::create_dynamic(1, 1.0f, inertia, Vec3(-0.45f, 0.0f, 0.0f));
    body_a.linear_velocity = Vec3(2.0f, 0.0f, 0.0f);

    RigidBody body_b = RigidBody::create_dynamic(2, 1.0f, inertia, Vec3(0.45f, 0.0f, 0.0f));
    body_b.linear_velocity = Vec3(-2.0f, 0.0f, 0.0f);

    Collider col_a = Collider::create_box(1, 1, Vec3(0.5f, 0.5f, 0.5f), body_a.get_transform());
    Collider col_b = Collider::create_box(2, 2, Vec3(0.5f, 0.5f, 0.5f), body_b.get_transform());

    ContactManifold manifold = Narrowphase::collide(col_a, col_b);
    assert(!manifold.points.empty());

    SolverSettings settings;
    settings.default_restitution = 1.0f; // Idealized elastic
    settings.default_friction = 0.0f;

    SequentialImpulseSolver solver(settings);
    std::vector<RigidBody> bodies = {body_a, body_b};
    std::vector<ContactManifold> manifolds = {manifold};

    Vec3 initial_p = bodies[0].linear_velocity * bodies[0].mass + bodies[1].linear_velocity * bodies[1].mass;

    solver.solve(bodies, manifolds, 0.0166667f);

    Vec3 final_p = bodies[0].linear_velocity * bodies[0].mass + bodies[1].linear_velocity * bodies[1].mass;

    // Verify momentum conservation
    assert((final_p - initial_p).length() < 1e-3f);
    // Verify elastic velocity exchange (body_a receives -x velocity, body_b receives +x velocity)
    assert(bodies[0].linear_velocity.x < 0.0f);
    assert(bodies[1].linear_velocity.x > 0.0f);

    (void)initial_p;
    (void)final_p;

    std::cout << "  PASSED." << std::endl;
}

void test_offcenter_torque_response() {
    std::cout << "[Test] Off-center Impact Torque Response..." << std::endl;

    InertiaTensor inertia_a = InertiaTensor::box(2.0f, 1.0f, 1.0f, 1.0f);
    RigidBody body_a = RigidBody::create_dynamic(1, 2.0f, inertia_a, Vec3(0.0f, 0.0f, 0.0f));
    body_a.linear_velocity = Vec3(3.0f, 0.0f, 0.0f);

    RigidBody body_b = RigidBody::create_static(2, Vec3(0.9f, 0.4f, 0.0f)); // Off-center collision point

    Collider col_a = Collider::create_box(1, 1, Vec3(0.5f, 0.5f, 0.5f), body_a.get_transform());
    Collider col_b = Collider::create_box(2, 2, Vec3(0.5f, 0.5f, 0.5f), body_b.get_transform());

    ContactManifold manifold = Narrowphase::collide(col_a, col_b);
    assert(!manifold.points.empty());

    SolverSettings settings;
    settings.default_restitution = 0.5f;
    SequentialImpulseSolver solver(settings);

    std::vector<RigidBody> bodies = {body_a, body_b};
    std::vector<ContactManifold> manifolds = {manifold};

    solver.solve(bodies, manifolds, 0.0166667f);

    // Verify angular velocity created by off-center impact torque r x J
    assert(bodies[0].angular_velocity.length_sq() > 1e-4f);
    // Static body remains unchanged
    assert(bodies[1].linear_velocity.length_sq() == 0.0f);
    assert(bodies[1].angular_velocity.length_sq() == 0.0f);

    std::cout << "  PASSED." << std::endl;
}

void test_friction_slide() {
    std::cout << "[Test] Friction Sliding Bound..." << std::endl;

    InertiaTensor inertia = InertiaTensor::box(1.0f, 1.0f, 1.0f, 1.0f);
    RigidBody body = RigidBody::create_dynamic(1, 1.0f, inertia, Vec3(0.0f, 0.45f, 0.0f));
    body.linear_velocity = Vec3(5.0f, -1.0f, 0.0f); // Horizontal velocity + downward pressing

    RigidBody ground = RigidBody::create_static(2, Vec3(0.0f, -0.5f, 0.0f));

    Collider col_body = Collider::create_box(1, 1, Vec3(0.5f, 0.5f, 0.5f), body.get_transform());
    Collider col_ground = Collider::create_box(2, 2, Vec3(10.0f, 0.5f, 10.0f), ground.get_transform());

    ContactManifold manifold = Narrowphase::collide(col_body, col_ground);
    assert(!manifold.points.empty());

    SolverSettings settings;
    settings.default_friction = 0.4f;
    SequentialImpulseSolver solver(settings);

    std::vector<RigidBody> bodies = {body, ground};
    std::vector<ContactManifold> manifolds = {manifold};

    float initial_vx = bodies[0].linear_velocity.x;
    solver.solve(bodies, manifolds, 0.0166667f);

    // Friction opposes horizontal motion
    assert(bodies[0].linear_velocity.x < initial_vx);
    assert(bodies[0].linear_velocity.x >= 0.0f);

    (void)initial_vx;

    std::cout << "  PASSED." << std::endl;
}

void test_resting_stack_stability() {
    std::cout << "[Test] 3-Body Resting Stack Stability..." << std::endl;

    PhysicsWorld world;
    world.set_gravity(Vec3(0.0f, -9.81f, 0.0f));

    // Ground
    RigidBody ground = RigidBody::create_static(1, Vec3(0.0f, -0.5f, 0.0f));
    Collider ground_col = Collider::create_box(1, 1, Vec3(10.0f, 0.5f, 10.0f), ground.get_transform());
    world.add_body(ground);
    world.add_collider(ground_col);

    // Box 1
    InertiaTensor inertia = InertiaTensor::box(1.0f, 1.0f, 1.0f, 1.0f);
    RigidBody b1 = RigidBody::create_dynamic(2, 1.0f, inertia, Vec3(0.0f, 0.5f, 0.0f));
    Collider col1 = Collider::create_box(2, 2, Vec3(0.5f, 0.5f, 0.5f), b1.get_transform());
    world.add_body(b1);
    world.add_collider(col1);

    // Box 2
    RigidBody b2 = RigidBody::create_dynamic(3, 1.0f, inertia, Vec3(0.0f, 1.51f, 0.0f));
    Collider col2 = Collider::create_box(3, 3, Vec3(0.5f, 0.5f, 0.5f), b2.get_transform());
    world.add_body(b2);
    world.add_collider(col2);

    float dt = 1.0f / 60.0f;
    for (int step = 0; step < 60; ++step) {
        world.step_full(dt);
    }

    // Verify stack does not explode or drift excessively
    const RigidBody* res_b1 = world.get_body(2);
    const RigidBody* res_b2 = world.get_body(3);

    std::cout << "  Stack b1 y: " << res_b1->position.y << ", b2 y: " << res_b2->position.y << std::endl;
    assert(res_b1 != nullptr && res_b2 != nullptr);
    assert(std::abs(res_b1->position.x) < 0.1f);
    assert(std::abs(res_b2->position.x) < 0.1f);
    assert(res_b1->position.y > 0.2f && res_b1->position.y < 0.8f);
    assert(res_b2->position.y > 0.8f && res_b2->position.y < 2.0f);

    std::cout << "  PASSED." << std::endl;
}

void test_warm_start_cache_behavior() {
    std::cout << "[Test] Warm Start Cache Lifecycle & Pruning..." << std::endl;

    WarmStartCache cache;
    uint64_t key1 = 0x123456789ABCDEF0ULL;
    uint64_t key2 = 0x0FEDCBA987654321ULL;

    cache.insert(key1, 10.0f, 2.0f, -1.0f);
    cache.insert(key2, 5.0f, 0.0f, 0.0f);

    float n = 0.0f, t1 = 0.0f, t2 = 0.0f;
    assert(cache.find(key1, n, t1, t2));
    assert(std::abs(n - 10.0f) < 1e-5f);
    assert(std::abs(t1 - 2.0f) < 1e-5f);

    // Age cache twice
    cache.age_and_prune(1); // key1 & key2 age become 1
    assert(cache.find(key1, n, t1, t2));

    cache.age_and_prune(1); // age becomes 2 > max_age(1) -> pruned!
    assert(!cache.find(key1, n, t1, t2));
    assert(cache.size() == 0);

    (void)n; (void)t1; (void)t2;

    std::cout << "  PASSED." << std::endl;
}

void test_split_impulses_position_stabilization() {
    std::cout << "[Test] Split Impulses Position Stabilization (No Physical Velocity Bleed)..." << std::endl;

    InertiaTensor inertia = InertiaTensor::box(1.0f, 1.0f, 1.0f, 1.0f);
    RigidBody body_a = RigidBody::create_dynamic(1, 1.0f, inertia, Vec3(0.0f, 0.45f, 0.0f)); // Penetrating 0.05m
    body_a.linear_velocity = Vec3::zero(); // Stationary initially

    RigidBody body_b = RigidBody::create_static(2, Vec3(0.0f, -0.5f, 0.0f));

    Collider col_a = Collider::create_box(1, 1, Vec3(0.5f, 0.5f, 0.5f), body_a.get_transform());
    Collider col_b = Collider::create_box(2, 2, Vec3(10.0f, 0.5f, 10.0f), body_b.get_transform());

    ContactManifold manifold = Narrowphase::collide(col_a, col_b);
    assert(!manifold.points.empty());

    SolverSettings settings;
    settings.position_iterations = 5;
    settings.baumgarte_beta = 0.3f;
    settings.penetration_slop = 0.001f;

    SequentialImpulseSolver solver(settings);
    std::vector<RigidBody> bodies = {body_a, body_b};
    std::vector<ContactManifold> manifolds = {manifold};

    solver.solve(bodies, manifolds, 0.0166667f);

    // Body position should be adjusted upward out of penetration
    assert(bodies[0].position.y > 0.45f);

    // Physical linear velocity should NOT receive a giant upward boost from position bias!
    assert(bodies[0].linear_velocity.y <= 0.01f);

    std::cout << "  PASSED." << std::endl;
}

void test_structural_graph_load_propagation_and_failure() {
    std::cout << "[Test] Structural Graph Load Propagation & Edge Failure..." << std::endl;

    MaterialParams mat;
    mat.tensile_strength = 100.0f; // Set weak tensile strength for testing failure
    mat.capacity_multiplier = 1.0f;

    StructuralGraph graph(mat);

    std::vector<RigidBody> bodies;
    // Static ground node 1
    bodies.push_back(RigidBody::create_static(1, Vec3(0.0f, 0.0f, 0.0f)));
    // Heavy upper shard node 2 (100 kg -> ~981 N load)
    InertiaTensor inertia = InertiaTensor::box(100.0f, 1.0f, 1.0f, 1.0f);
    bodies.push_back(RigidBody::create_dynamic(2, 100.0f, inertia, Vec3(0.0f, 1.0f, 0.0f)));

    std::vector<ContactManifold> manifolds;
    ContactManifold m;
    m.body_a_id = 2; // Upper
    m.body_b_id = 1; // Lower static ground
    m.normal = Vec3(0.0f, -1.0f, 0.0f); // Pointing A to B -> reaction on A is +y
    m.add_point(ContactPoint(Vec3(0.0f, 0.5f, 0.0f), Vec3(0.0f, -0.5f, 0.0f), Vec3(0.0f, 0.5f, 0.0f), 0.01f, 1));
    manifolds.push_back(m);

    graph.build_from_world_and_contacts(bodies, manifolds);
    assert(graph.get_edges().size() == 1);

    Vec3 gravity(0.0f, -9.81f, 0.0f);
    graph.evaluate_load_and_connectivity(gravity);

    // Overloaded edge capacity (capacity = 0.01m^2 * 100Pa = 1N < 981N load) -> edge breaks!
    assert(graph.broken_edge_count() == 1);
    // Node 2 becomes unsupported
    assert(graph.unsupported_count() == 1);

    std::cout << "  PASSED." << std::endl;
}

void test_determinism_repeatability() {
    std::cout << "[Test] Exact Deterministic Solver & Graph Repeatability..." << std::endl;

    auto run_sim = []() {
        PhysicsWorld world;
        world.set_gravity(Vec3(0.0f, -9.81f, 0.0f));

        RigidBody ground = RigidBody::create_static(1, Vec3(0.0f, -0.5f, 0.0f));
        Collider g_col = Collider::create_box(1, 1, Vec3(10.0f, 0.5f, 10.0f), ground.get_transform());
        world.add_body(ground);
        world.add_collider(g_col);

        InertiaTensor inertia = InertiaTensor::box(5.0f, 1.0f, 1.0f, 1.0f);
        RigidBody b1 = RigidBody::create_dynamic(2, 5.0f, inertia, Vec3(0.1f, 1.0f, 0.0f));
        Collider c1 = Collider::create_box(2, 2, Vec3(0.5f, 0.5f, 0.5f), b1.get_transform());
        world.add_body(b1);
        world.add_collider(c1);

        for (int i = 0; i < 30; ++i) {
            world.step_full(1.0f / 60.0f);
        }

        const RigidBody* body = world.get_body(2);
        return body->position;
    };

    Vec3 pos1 = run_sim();
    Vec3 pos2 = run_sim();

    assert((pos1 - pos2).length_sq() == 0.0f);

    std::cout << "  PASSED." << std::endl;
}

int main() {
    std::cout << "==================================================" << std::endl;
    std::cout << "Running Project 04 Milestone 4 Solver & Graph Tests" << std::endl;
    std::cout << "==================================================" << std::endl;

    test_analytical_equal_mass_headon();
    test_offcenter_torque_response();
    test_friction_slide();
    test_resting_stack_stability();
    test_warm_start_cache_behavior();
    test_split_impulses_position_stabilization();
    test_structural_graph_load_propagation_and_failure();
    test_determinism_repeatability();

    std::cout << "==================================================" << std::endl;
    std::cout << "All Milestone 4 Solver & Graph Tests Passed!" << std::endl;
    std::cout << "==================================================" << std::endl;

    return 0;
}
