#include "destruction/math/vec3.hpp"
#include "destruction/math/quat.hpp"
#include "destruction/math/transform.hpp"
#include "destruction/dynamics/rigid_body.hpp"
#include "destruction/dynamics/physics_world.hpp"
#include "destruction/collision/collider.hpp"
#include "destruction/fracture/voronoi3d.hpp"
#include "destruction/fracture/site_generator.hpp"
#include "destruction/fracture/fracture_volume.hpp"
#include "destruction/solver/solver_settings.hpp"
#include "destruction/graph/material_params.hpp"
#include "destruction/graph/structural_graph.hpp"

#include <iostream>
#include <cassert>
#include <cmath>
#include <vector>
#include <set>

using namespace destruction::math;
using namespace destruction::dynamics;
using namespace destruction::collision;
using namespace destruction::fracture;
using namespace destruction::solver;
using namespace destruction::graph;

void test_scene_initialization_and_uniqueness() {
    std::cout << "[Test] Scene Initialization & ID Uniqueness..." << std::endl;

    PhysicsWorld world;
    world.set_gravity(Vec3(0.0f, -9.81f, 0.0f));

    RigidBody ground = RigidBody::create_static(1, Vec3(0.0f, -0.5f, 0.0f));
    Collider g_col = Collider::create_box(1, 1, Vec3(10.0f, 0.5f, 10.0f), ground.get_transform());
    world.add_body(ground);
    world.add_collider(g_col);

    for (uint32_t i = 2; i <= 6; ++i) {
        InertiaTensor inertia = InertiaTensor::box(10.0f, 1.0f, 1.0f, 1.0f);
        RigidBody b = RigidBody::create_dynamic(i, 10.0f, inertia, Vec3(0.0f, static_cast<float>(i - 1), 0.0f));
        Collider col = Collider::create_box(i, i, Vec3(0.5f, 0.5f, 0.5f), b.get_transform());
        world.add_body(b);
        world.add_collider(col);
    }

    assert(world.body_count() == 6);
    assert(world.collider_count() == 6);

    std::set<uint32_t> b_ids, c_ids;
    for (const auto& b : world.get_bodies()) {
        assert(b_ids.find(b.id) == b_ids.end());
        b_ids.insert(b.id);
    }
    for (const auto& c : world.get_colliders()) {
        assert(c_ids.find(c.id) == c_ids.end());
        c_ids.insert(c.id);
    }

    std::cout << "  PASSED." << std::endl;
}

void test_fracture_lifecycle_and_shard_replacement() {
    std::cout << "[Test] Fracture Lifecycle & Shard Replacement..." << std::endl;

    PhysicsWorld world;
    world.set_gravity(Vec3(0.0f, -9.81f, 0.0f));

    // Ground
    RigidBody ground = RigidBody::create_static(1, Vec3(0.0f, -0.5f, 0.0f));
    Collider g_col = Collider::create_box(1, 1, Vec3(10.0f, 0.5f, 10.0f), ground.get_transform());
    world.add_body(ground);
    world.add_collider(g_col);

    // Intact block (ID 2)
    InertiaTensor inertia = InertiaTensor::box(24.0f, 1.0f, 1.0f, 1.0f);
    RigidBody intact = RigidBody::create_dynamic(2, 24.0f, inertia, Vec3(0.0f, 0.5f, 0.0f));
    Collider intact_col = Collider::create_box(2, 2, Vec3(0.5f, 0.5f, 0.5f), intact.get_transform());
    world.add_body(intact);
    world.add_collider(intact_col);

    // Perform fracture of intact body
    FractureVolume vol(Vec3(-0.5f, -0.5f, -0.5f), Vec3(0.5f, 0.5f, 0.5f), 24.0f);
    std::vector<Vec3> sites = SiteGenerator::generate_3d(vol, 6, 0.15f, 42);
    std::vector<Shard> shards = Voronoi3D::compute_partition(vol, sites);
    assert(!shards.empty());

    // Remove intact body
    auto& bodies = world.get_bodies();
    bodies.erase(std::remove_if(bodies.begin(), bodies.end(),
        [](const RigidBody& b) { return b.id == 2; }), bodies.end());

    auto& colliders = world.get_colliders();
    colliders.erase(std::remove_if(colliders.begin(), colliders.end(),
        [](const Collider& c) { return c.id == 2; }), colliders.end());

    assert(world.get_body(2) == nullptr);
    assert(world.get_collider(2) == nullptr);

    // Register shards
    uint32_t next_id = 3;
    for (const auto& s : shards) {
        if (!s.is_valid) continue;
        InertiaTensor s_inertia = InertiaTensor::box(s.mass, 0.3f, 0.3f, 0.3f);
        RigidBody sb = RigidBody::create_dynamic(next_id, s.mass, s_inertia, s.centroid + Vec3(0, 0.5f, 0));
        Collider sc = Collider::create_polyhedron(next_id, next_id, s.mesh, sb.get_transform());
        world.add_body(sb);
        world.add_collider(sc);
        next_id++;
    }

    assert(world.body_count() > 2);
    assert(world.collider_count() > 2);

    // Verify simulation step with fractured shards
    world.step_full(1.0f / 60.0f);

    for (const auto& b : world.get_bodies()) {
        assert(!std::isnan(b.position.x) && !std::isinf(b.position.x));
        assert(!std::isnan(b.position.y) && !std::isinf(b.position.y));
        assert(!std::isnan(b.position.z) && !std::isinf(b.position.z));
    }

    std::cout << "  PASSED." << std::endl;
}

void test_cascading_collapse_propagation() {
    std::cout << "[Test] Cascading Collapse & Structural Failure..." << std::endl;

    PhysicsWorld world;
    world.set_gravity(Vec3(0.0f, -9.81f, 0.0f));

    MaterialParams mat;
    mat.tensile_strength = 200.0f; // Weak strength to induce failure
    mat.capacity_multiplier = 1.0f;
    world.get_graph().set_material_params(mat);

    // Ground
    RigidBody ground = RigidBody::create_static(1, Vec3(0.0f, -0.5f, 0.0f));
    Collider g_col = Collider::create_box(1, 1, Vec3(10.0f, 0.5f, 10.0f), ground.get_transform());
    world.add_body(ground);
    world.add_collider(g_col);

    // Base pillar (body 2)
    InertiaTensor inertia = InertiaTensor::box(10.0f, 1.0f, 1.0f, 1.0f);
    RigidBody b2 = RigidBody::create_dynamic(2, 10.0f, inertia, Vec3(0.0f, 0.5f, 0.0f));
    Collider c2 = Collider::create_box(2, 2, Vec3(0.5f, 0.5f, 0.5f), b2.get_transform());
    world.add_body(b2);
    world.add_collider(c2);

    // Extremely heavy top load (body 3, 5000 kg) -> exceeds edge capacity
    InertiaTensor heavy_inertia = InertiaTensor::box(5000.0f, 1.0f, 1.0f, 1.0f);
    RigidBody b3 = RigidBody::create_dynamic(3, 5000.0f, heavy_inertia, Vec3(0.0f, 1.5f, 0.0f));
    Collider c3 = Collider::create_box(3, 3, Vec3(0.5f, 0.5f, 0.5f), b3.get_transform());
    world.add_body(b3);
    world.add_collider(c3);

    world.step_full(1.0f / 60.0f);

    // Structural graph should detect failure and broken edges
    assert(world.get_graph().broken_edge_count() > 0);
    assert(world.get_graph().unsupported_count() > 0);

    std::cout << "  PASSED." << std::endl;
}

void test_deterministic_reset_equality() {
    std::cout << "[Test] Deterministic Scene Reset State Match..." << std::endl;

    auto build_and_run = []() {
        PhysicsWorld world;
        world.set_gravity(Vec3(0.0f, -9.81f, 0.0f));

        RigidBody ground = RigidBody::create_static(1, Vec3(0.0f, -0.5f, 0.0f));
        Collider g_col = Collider::create_box(1, 1, Vec3(10.0f, 0.5f, 10.0f), ground.get_transform());
        world.add_body(ground);
        world.add_collider(g_col);

        for (uint32_t i = 2; i <= 4; ++i) {
            InertiaTensor inertia = InertiaTensor::box(5.0f, 1.0f, 1.0f, 1.0f);
            RigidBody b = RigidBody::create_dynamic(i, 5.0f, inertia, Vec3(0.02f * static_cast<float>(i), 0.5f + static_cast<float>(i - 2) * 1.01f, 0.0f));
            Collider col = Collider::create_box(i, i, Vec3(0.5f, 0.5f, 0.5f), b.get_transform());
            world.add_body(b);
            world.add_collider(col);
        }

        for (int step = 0; step < 30; ++step) {
            world.step_full(1.0f / 60.0f);
        }

        std::vector<Vec3> positions;
        for (const auto& b : world.get_bodies()) {
            positions.push_back(b.position);
        }
        return positions;
    };

    auto run1 = build_and_run();
    auto run2 = build_and_run();

    assert(run1.size() == run2.size());
    for (size_t i = 0; i < run1.size(); ++i) {
        assert((run1[i] - run2[i]).length_sq() == 0.0f);
    }

    std::cout << "  PASSED." << std::endl;
}

int main() {
    std::cout << "========================================================\n";
    std::cout << "Running Project 04 Milestone 5 Sandbox Integration Tests\n";
    std::cout << "========================================================\n";

    test_scene_initialization_and_uniqueness();
    test_fracture_lifecycle_and_shard_replacement();
    test_cascading_collapse_propagation();
    test_deterministic_reset_equality();

    std::cout << "========================================================\n";
    std::cout << "All Milestone 5 Integration Tests Passed Successfully.\n";
    std::cout << "========================================================\n";

    return 0;
}
