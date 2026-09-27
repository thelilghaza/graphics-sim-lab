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
#include <iomanip>
#include <vector>
#include <cmath>
#include <cstdint>
#include <cassert>

using namespace destruction::math;
using namespace destruction::dynamics;
using namespace destruction::collision;
using namespace destruction::fracture;
using namespace destruction::solver;
using namespace destruction::graph;

static uint32_t compute_checksum(const PhysicsWorld& world) {
    uint32_t hash = 0x811C9DC5u;
    auto fnv1a = [&hash](uint32_t val) {
        hash ^= val;
        hash *= 0x01000193u;
    };

    for (const auto& body : world.get_bodies()) {
        fnv1a(body.id);
        int px = static_cast<int>(std::floor(body.position.x * 1000.0f));
        int py = static_cast<int>(std::floor(body.position.y * 1000.0f));
        int pz = static_cast<int>(std::floor(body.position.z * 1000.0f));
        fnv1a(static_cast<uint32_t>(px));
        fnv1a(static_cast<uint32_t>(py));
        fnv1a(static_cast<uint32_t>(pz));
    }

    fnv1a(static_cast<uint32_t>(world.get_active_manifolds().size()));
    fnv1a(static_cast<uint32_t>(world.get_graph().supported_count()));
    fnv1a(static_cast<uint32_t>(world.get_graph().unsupported_count()));
    fnv1a(static_cast<uint32_t>(world.get_graph().broken_edge_count()));

    return hash;
}

int main() {
    std::cout << "========================================================\n";
    std::cout << "Project 04 — Headless Destruction Scenario Validation\n";
    std::cout << "========================================================\n";

    PhysicsWorld world;
    world.set_gravity(Vec3(0.0f, -9.81f, 0.0f));

    SolverSettings settings;
    settings.velocity_iterations = 10;
    settings.position_iterations = 5;
    settings.default_friction = 0.4f;
    settings.default_restitution = 0.2f;
    settings.penetration_slop = 0.005f;
    settings.baumgarte_beta = 0.25f;
    world.set_solver_settings(settings);

    MaterialParams mat;
    mat.density = 2400.0f;
    mat.tensile_strength = 4.0e4f;
    mat.capacity_multiplier = 4.0f;
    world.get_graph().set_material_params(mat);

    // 1. Create Structure (Ground + 3 Tower Blocks)
    RigidBody ground = RigidBody::create_static(1, Vec3(0.0f, -0.5f, 0.0f));
    Collider ground_col = Collider::create_box(1, 1, Vec3(15.0f, 0.5f, 15.0f), ground.get_transform());
    world.add_body(ground);
    world.add_collider(ground_col);

    // Block 1 (Base, y = 0.5)
    InertiaTensor i1 = InertiaTensor::box(2400.0f, 1.0f, 1.0f, 1.0f);
    RigidBody b1 = RigidBody::create_dynamic(2, 2400.0f, i1, Vec3(0.0f, 0.5f, 0.0f));
    Collider c1 = Collider::create_box(2, 2, Vec3(0.5f, 0.5f, 0.5f), b1.get_transform());
    world.add_body(b1);
    world.add_collider(c1);

    // Block 2 (Middle, y = 1.5 - Target of fracture)
    RigidBody b2 = RigidBody::create_dynamic(3, 2400.0f, i1, Vec3(0.0f, 1.5f, 0.0f));
    Collider c2 = Collider::create_box(3, 3, Vec3(0.5f, 0.5f, 0.5f), b2.get_transform());
    world.add_body(b2);
    world.add_collider(c2);

    // Block 3 (Top, y = 2.5)
    RigidBody b3 = RigidBody::create_dynamic(4, 2400.0f, i1, Vec3(0.0f, 2.5f, 0.0f));
    Collider c3 = Collider::create_box(4, 4, Vec3(0.5f, 0.5f, 0.5f), b3.get_transform());
    world.add_body(b3);
    world.add_collider(c3);

    size_t initial_body_count = world.body_count();

    // 2. Create Projectile (ID 1000)
    InertiaTensor proj_i = InertiaTensor::box(15.0f, 0.4f, 0.4f, 0.4f);
    RigidBody proj = RigidBody::create_dynamic(1000, 15.0f, proj_i, Vec3(-3.0f, 1.5f, 0.0f));
    proj.linear_velocity = Vec3(25.0f, 0.0f, 0.0f); // High-speed impact
    Collider proj_col = Collider::create_box(1000, 1000, Vec3(0.2f, 0.2f, 0.2f), proj.get_transform());
    world.add_body(proj);
    world.add_collider(proj_col);

    const float dt = 1.0f / 60.0f;
    size_t fracture_site_count = 6;
    size_t final_shard_count = 0;
    bool fractured = false;

    // Simulate 120 steps
    for (int step = 0; step < 120; ++step) {
        // Impact detection and fracture trigger
        if (!fractured) {
            for (const auto& m : world.get_active_manifolds()) {
                if ((m.body_a_id == 1000 && m.body_b_id == 3) || (m.body_a_id == 3 && m.body_b_id == 1000)) {
                    // Middle block hit! Shatter block 3
                    FractureVolume vol(Vec3(-0.5f, -0.5f, -0.5f), Vec3(0.5f, 0.5f, 0.5f), 2400.0f);
                    std::vector<Vec3> sites = SiteGenerator::generate_3d(vol, fracture_site_count, 0.15f, 42);
                    std::vector<Shard> shards = Voronoi3D::compute_partition(vol, sites);

                    // Remove block 3
                    auto& bodies = world.get_bodies();
                    bodies.erase(std::remove_if(bodies.begin(), bodies.end(),
                        [](const RigidBody& b) { return b.id == 3; }), bodies.end());

                    auto& colliders = world.get_colliders();
                    colliders.erase(std::remove_if(colliders.begin(), colliders.end(),
                        [](const Collider& c) { return c.id == 3; }), colliders.end());

                    uint32_t shard_id = 10;
                    for (const auto& s : shards) {
                        if (!s.is_valid) continue;
                        InertiaTensor s_inertia = InertiaTensor::box(s.mass, 0.3f, 0.3f, 0.3f);
                        RigidBody sb = RigidBody::create_dynamic(shard_id, s.mass, s_inertia, s.centroid + Vec3(0, 1.5f, 0));
                        Collider sc = Collider::create_polyhedron(shard_id, shard_id, s.mesh, sb.get_transform());
                        world.add_body(sb);
                        world.add_collider(sc);
                        shard_id++;
                        final_shard_count++;
                    }
                    fractured = true;
                    break;
                }
            }
        }

        world.step_full(dt);
    }

    // Diagnostics
    float max_pen = 0.0f;
    size_t collision_count = 0;
    for (const auto& m : world.get_active_manifolds()) {
        collision_count += m.points.size();
        max_pen = std::max(max_pen, m.max_penetration_depth);
    }

    float total_ke = 0.0f;
    for (const auto& b : world.get_bodies()) {
        if (b.is_static) continue;
        total_ke += 0.5f * b.mass * b.linear_velocity.length_sq();
    }

    uint32_t checksum = compute_checksum(world);

    // Assert finite states
    for (const auto& b : world.get_bodies()) {
        assert(!std::isnan(b.position.x) && !std::isinf(b.position.x));
        assert(!std::isnan(b.position.y) && !std::isinf(b.position.y));
        assert(!std::isnan(b.position.z) && !std::isinf(b.position.z));
    }

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Initial Body Count    : " << initial_body_count << "\n";
    std::cout << "Fracture Site Count   : " << fracture_site_count << "\n";
    std::cout << "Final Shard Count     : " << final_shard_count << "\n";
    std::cout << "Collision Count       : " << collision_count << "\n";
    std::cout << "Maximum Penetration   : " << max_pen << " m\n";
    std::cout << "Broken Support Edges  : " << world.get_graph().broken_edge_count() << "\n";
    std::cout << "Unsupported Shards    : " << world.get_graph().unsupported_count() << "\n";
    std::cout << "Final Kinetic Energy  : " << total_ke << " J\n";
    std::cout << "Deterministic Checksum: 0x" << std::hex << std::uppercase << checksum << std::dec << "\n";
    std::cout << "========================================================\n";

    return 0;
}
