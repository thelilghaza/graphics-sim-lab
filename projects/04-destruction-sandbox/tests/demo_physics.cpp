#include "destruction/math/vec3.hpp"
#include "destruction/math/quat.hpp"
#include "destruction/math/transform.hpp"
#include "destruction/dynamics/rigid_body.hpp"
#include "destruction/dynamics/physics_world.hpp"
#include "destruction/collision/collider.hpp"
#include "destruction/collision/contact_manifold.hpp"
#include "destruction/fracture/fracture_volume.hpp"
#include "destruction/fracture/site_generator.hpp"
#include "destruction/fracture/voronoi3d.hpp"
#include "destruction/fracture/obj_exporter.hpp"
#include "destruction/solver/solver_settings.hpp"
#include "destruction/solver/sequential_impulse_solver.hpp"
#include "destruction/graph/material_params.hpp"
#include "destruction/graph/structural_graph.hpp"

#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <cstdint>

using namespace destruction::math;
using namespace destruction::dynamics;
using namespace destruction::collision;
using namespace destruction::fracture;
using namespace destruction::solver;
using namespace destruction::graph;

static uint32_t compute_scene_checksum(const PhysicsWorld& world) {
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

static void export_scene_obj(const std::string& filepath, const PhysicsWorld& world) {
    std::ofstream out(filepath);
    if (!out.is_open()) return;

    out << "# Project 04 Milestone 4 Physics Scene Diagnostic Export\n";

    uint32_t v_offset = 1;
    for (const auto& col : world.get_colliders()) {
        if (col.type == ColliderType::Box) {
            Vec3 h = col.box_half_extents;
            Vec3 corners[8] = {
                Vec3(-h.x, -h.y, -h.z), Vec3( h.x, -h.y, -h.z),
                Vec3( h.x,  h.y, -h.z), Vec3(-h.x,  h.y, -h.z),
                Vec3(-h.x, -h.y,  h.z), Vec3( h.x, -h.y,  h.z),
                Vec3( h.x,  h.y,  h.z), Vec3(-h.x,  h.y,  h.z)
            };
            out << "o Collider_" << col.id << "\n";
            for (int i = 0; i < 8; ++i) {
                Vec3 w = col.world_transform.transform_point(corners[i]);
                out << "v " << w.x << " " << w.y << " " << w.z << "\n";
            }
            int faces[6][4] = {
                {0, 3, 2, 1}, {4, 5, 6, 7},
                {0, 1, 5, 4}, {2, 3, 7, 6},
                {0, 4, 7, 3}, {1, 2, 6, 5}
            };
            for (int f = 0; f < 6; ++f) {
                out << "f " << (v_offset + faces[f][0]) << " " << (v_offset + faces[f][1]) << " "
                    << (v_offset + faces[f][2]) << " " << (v_offset + faces[f][3]) << "\n";
            }
            v_offset += 8;
        } else if (col.type == ColliderType::ConvexPolyhedron) {
            out << "o ShardCollider_" << col.id << "\n";
            for (const auto& v : col.mesh.vertices) {
                Vec3 w = col.world_transform.transform_point(v);
                out << "v " << w.x << " " << w.y << " " << w.z << "\n";
            }
            for (const auto& f : col.mesh.faces) {
                out << "f";
                for (int idx : f.vertex_indices) {
                    out << " " << (v_offset + idx);
                }
                out << "\n";
            }
            v_offset += static_cast<uint32_t>(col.mesh.vertices.size());
        }
    }

    out.close();
}

int main() {
    std::cout << "==================================================" << std::endl;
    std::cout << "Project 04 — Milestone 4 Physics Demo" << std::endl;
    std::cout << "==================================================" << std::endl;

    // 1. Generate deterministic fracture shards using M2 Voronoi3D
    FractureVolume vol(Vec3(-0.5f, -0.5f, -0.5f), Vec3(0.5f, 0.5f, 0.5f), 2400.0f);
    std::vector<Vec3> sites = SiteGenerator::generate_3d(vol, 4, 0.2f, 42);
    std::vector<Shard> shards = Voronoi3D::compute_partition(vol, sites);

    // 2. Initialize Physics World
    PhysicsWorld world;
    world.set_gravity(Vec3(0.0f, -9.81f, 0.0f));

    SolverSettings solver_settings;
    solver_settings.velocity_iterations = 10;
    solver_settings.position_iterations = 5;
    solver_settings.default_friction = 0.4f;
    solver_settings.default_restitution = 0.2f;
    world.set_solver_settings(solver_settings);

    MaterialParams mat_params;
    mat_params.density = 2400.0f;
    mat_params.tensile_strength = 5.0e4f;
    mat_params.capacity_multiplier = 5.0f;
    world.get_graph().set_material_params(mat_params);

    // Add static ground
    RigidBody ground_body = RigidBody::create_static(1, Vec3(0.0f, -0.5f, 0.0f));
    Collider ground_col = Collider::create_box(1, 1, Vec3(10.0f, 0.5f, 10.0f), ground_body.get_transform());
    world.add_body(ground_body);
    world.add_collider(ground_col);

    // Add fracture shard colliders stacked vertically
    uint32_t current_id = 2;
    for (size_t i = 0; i < shards.size(); ++i) {
        const Shard& s = shards[i];
        if (!s.is_valid) continue;

        Vec3 pos(0.0f, 0.5f + static_cast<float>(i) * 0.6f, 0.0f);
        InertiaTensor inertia = InertiaTensor::box(s.mass, 0.5f, 0.5f, 0.5f);

        RigidBody shard_body = RigidBody::create_dynamic(current_id, s.mass, inertia, pos);
        Collider shard_col = Collider::create_polyhedron(current_id, current_id, s.mesh, shard_body.get_transform());

        world.add_body(shard_body);
        world.add_collider(shard_col);
        current_id++;
    }

    // Add lateral projectile block
    InertiaTensor proj_inertia = InertiaTensor::box(5.0f, 0.5f, 0.5f, 0.5f);
    RigidBody proj_body = RigidBody::create_dynamic(current_id, 5.0f, proj_inertia, Vec3(-3.0f, 1.0f, 0.0f));
    proj_body.linear_velocity = Vec3(8.0f, 0.0f, 0.0f); // Lateral impact
    Collider proj_col = Collider::create_box(current_id, current_id, Vec3(0.5f, 0.5f, 0.5f), proj_body.get_transform());
    world.add_body(proj_body);
    world.add_collider(proj_col);

    // 3. Run Simulation Loop
    const float dt = 1.0f / 60.0f;
    const int total_steps = 60;

    for (int step = 0; step < total_steps; ++step) {
        world.step_full(dt);
    }

    // 4. Gather Diagnostics & Statistics
    float max_penetration = 0.0f;
    size_t total_contacts = 0;
    for (const auto& manifold : world.get_active_manifolds()) {
        total_contacts += manifold.points.size();
        max_penetration = std::max(max_penetration, manifold.max_penetration_depth);
    }

    float total_ke = 0.0f;
    for (const auto& b : world.get_bodies()) {
        if (b.is_static) continue;
        float ke_lin = 0.5f * b.mass * b.linear_velocity.length_sq();
        Mat3 world_I_inv = b.get_world_inv_inertia();
        // Check for non-zero angular velocity
        float ke_ang = 0.0f;
        if (b.angular_velocity.length_sq() > 1e-10f) {
            Mat3 world_I = b.inertia.get_world_inv_inertia(b.orientation); // Inertia tensor calculation
            (void)world_I_inv;
            ke_ang = 0.5f * b.angular_velocity.dot(b.angular_velocity);
        }
        total_ke += (ke_lin + ke_ang);
    }

    uint32_t checksum = compute_scene_checksum(world);

    // Export scene OBJ
    export_scene_obj("physics_scene.obj", world);

    // 5. Output Report
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Simulation Steps        : " << total_steps << "\n";
    std::cout << "Rigid Body Count        : " << world.body_count() << "\n";
    std::cout << "Active Contact Manifolds: " << world.get_active_manifolds().size() << "\n";
    std::cout << "Total Contact Points    : " << total_contacts << "\n";
    std::cout << "Velocity Iterations     : " << solver_settings.velocity_iterations << "\n";
    std::cout << "Position Iterations     : " << solver_settings.position_iterations << "\n";
    std::cout << "Maximum Penetration     : " << max_penetration << " m\n";
    std::cout << "Total Kinetic Energy    : " << total_ke << " J\n";
    std::cout << "Supported Graph Nodes   : " << world.get_graph().supported_count() << "\n";
    std::cout << "Unsupported Graph Nodes : " << world.get_graph().unsupported_count() << "\n";
    std::cout << "Broken Support Edges    : " << world.get_graph().broken_edge_count() << "\n";
    std::cout << "Scene Checksum          : 0x" << std::hex << std::uppercase << checksum << std::dec << "\n";
    std::cout << "Exported Diagnostic OBJ : physics_scene.obj\n";
    std::cout << "==================================================" << std::endl;

    return 0;
}
