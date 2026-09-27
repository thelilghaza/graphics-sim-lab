#include "performance_lab/bench_runner.hpp"
#include "destruction/math/vec3.hpp"
#include "destruction/dynamics/physics_world.hpp"
#include "destruction/collision/collider.hpp"
#include "destruction/collision/dynamic_aabb_tree.hpp"
#include "destruction/collision/narrowphase.hpp"
#include "destruction/fracture/fracture_volume.hpp"
#include "destruction/fracture/site_generator.hpp"
#include "destruction/fracture/voronoi3d.hpp"
#include "destruction/solver/sequential_impulse_solver.hpp"
#include "destruction/graph/structural_graph.hpp"

#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <string>
#include <cassert>

using namespace performance_lab;
using namespace destruction::math;
using namespace destruction::dynamics;
using namespace destruction::collision;
using namespace destruction::fracture;
using namespace destruction::solver;
using namespace destruction::graph;

void export_all_csv(const std::vector<BenchResult>& results, const std::string& path) {
    std::ofstream file(path.c_str(), std::ios::out | std::ios::trunc);
    if (!file.is_open()) {
        std::cerr << "[CSV Error] Failed to open CSV output path: " << path << "\n";
        return;
    }

    file << "benchmark_name,workload,build_config,compiler,architecture,os,warmups,iterations,"
         << "mean_us,median_us,stddev_us,min_us,max_us,ops_per_sec,mb_per_sec\n";

    for (const auto& r : results) {
        file << "\"" << r.config.name << "\","
             << "\"" << r.config.workload_name << "\","
             << "\"" << r.build_config << "\","
             << "\"" << r.compiler_info << "\","
             << "\"" << r.arch_info << "\","
             << "\"" << r.os_info << "\","
             << r.config.warmups << ","
             << r.config.iterations << ","
             << std::fixed << std::setprecision(3)
             << r.mean_us << ","
             << r.median_us << ","
             << r.stddev_us << ","
             << r.min_us << ","
             << r.max_us << ","
             << std::setprecision(0)
             << r.ops_per_sec << ","
             << std::setprecision(2)
             << r.mb_per_sec << "\n";
    }

    file.close();
}

int main(int argc, char** argv) {
    std::string csv_path = "projects/04-destruction-sandbox/benchmarks/reports/destruction_release.csv";

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--csv" && i + 1 < argc) {
            csv_path = argv[++i];
        }
    }

    std::cout << "========================================================\n";
    std::cout << "Project 04 — Procedural Destruction Benchmark Suite\n";
    std::cout << "Target CSV: " << csv_path << "\n";
    std::cout << "========================================================\n";

    std::vector<BenchResult> all_results;

    // 1. Voronoi Fracture Scaling Benchmark
    std::cout << "\n--- 1. Voronoi Fracture Scaling Benchmark ---\n";
    std::vector<size_t> site_counts = {4, 8, 16, 32};
    for (size_t count : site_counts) {
        FractureVolume vol(Vec3(-0.5f, -0.5f, -0.5f), Vec3(0.5f, 0.5f, 0.5f), 2400.0f);
        std::vector<Vec3> sites = SiteGenerator::generate_3d(vol, count, 0.1f, 42);

        BenchConfig config;
        config.name = "voronoi_fracture";
        config.workload_name = std::to_string(count) + "_sites";
        config.warmups = 3;
        config.iterations = 20;
        config.total_operations = count;

        BenchResult res = BenchRunner::run(config, [&]() {
            auto shards = Voronoi3D::compute_partition(vol, sites);
            clobber_memory();
        });
        all_results.push_back(res);
    }

    // 2. Dynamic AABB Tree Broadphase Benchmark
    std::cout << "\n--- 2. Dynamic AABB Tree Broadphase Benchmark ---\n";
    std::vector<size_t> collider_counts = {16, 64, 128, 256};
    for (size_t count : collider_counts) {
        std::vector<Collider> colliders;
        colliders.reserve(count);

        for (size_t i = 0; i < count; ++i) {
            float x = static_cast<float>(i % 16) * 1.2f;
            float y = static_cast<float>((i / 16) % 16) * 1.2f;
            float z = static_cast<float>(i / 256) * 1.2f;
            Transform t(Vec3(x, y, z), Quat::identity());
            Collider col = Collider::create_box(static_cast<uint32_t>(i + 1), static_cast<uint32_t>(i + 1), Vec3(0.5f, 0.5f, 0.5f), t);
            colliders.push_back(col);
        }

        BenchConfig config;
        config.name = "broadphase_tree";
        config.workload_name = std::to_string(count) + "_colliders";
        config.warmups = 5;
        config.iterations = 50;
        config.total_operations = count;

        BenchResult res = BenchRunner::run(config, [&]() {
            DynamicAabbTree tree;
            for (auto& c : colliders) {
                c.update_world_aabb();
                tree.insert_leaf(c.id, c.world_aabb);
            }
            std::vector<BroadphasePair> pairs;
            tree.generate_candidate_pairs(pairs);
            clobber_memory();
        });
        all_results.push_back(res);
    }

    // 3. Narrowphase GJK/EPA Benchmark
    std::cout << "\n--- 3. Narrowphase GJK/EPA Benchmark ---\n";
    std::vector<std::pair<Collider, Collider>> pairs;
    pairs.reserve(100);
    for (size_t i = 0; i < 100; ++i) {
        float offset = (static_cast<float>(i) / 100.0f) * 1.5f;
        Transform t1(Vec3(0.0f, 0.0f, 0.0f), Quat::identity());
        Transform t2(Vec3(offset, 0.0f, 0.0f), Quat::identity());
        Collider c1 = Collider::create_box(1, 1, Vec3(0.5f, 0.5f, 0.5f), t1);
        Collider c2 = Collider::create_box(2, 2, Vec3(0.5f, 0.5f, 0.5f), t2);
        pairs.emplace_back(c1, c2);
    }

    BenchConfig np_config;
    np_config.name = "narrowphase_gjk_epa";
    np_config.workload_name = "100_pairs_batch";
    np_config.warmups = 5;
    np_config.iterations = 50;
    np_config.total_operations = 100;

    BenchResult np_res = BenchRunner::run(np_config, [&]() {
        for (const auto& p : pairs) {
            ContactManifold m = Narrowphase::collide(p.first, p.second);
            clobber_memory();
        }
    });
    all_results.push_back(np_res);

    // 4. Sequential Impulse Solver Benchmark
    std::cout << "\n--- 4. Sequential Impulse Solver Benchmark ---\n";
    PhysicsWorld stack_world;
    stack_world.set_gravity(Vec3(0.0f, -9.81f, 0.0f));
    RigidBody ground = RigidBody::create_static(1, Vec3(0.0f, -0.5f, 0.0f));
    Collider g_col = Collider::create_box(1, 1, Vec3(20.0f, 0.5f, 20.0f), ground.get_transform());
    stack_world.add_body(ground);
    stack_world.add_collider(g_col);

    for (size_t i = 0; i < 10; ++i) {
        uint32_t id = static_cast<uint32_t>(i + 2);
        InertiaTensor inertia = InertiaTensor::box(10.0f, 1.0f, 1.0f, 1.0f);
        RigidBody b = RigidBody::create_dynamic(id, 10.0f, inertia, Vec3(0.0f, 0.5f + static_cast<float>(i) * 0.95f, 0.0f));
        Collider col = Collider::create_box(id, id, Vec3(0.5f, 0.5f, 0.5f), b.get_transform());
        stack_world.add_body(b);
        stack_world.add_collider(col);
    }

    BenchConfig solver_config;
    solver_config.name = "impulse_solver";
    solver_config.workload_name = "10_body_stack_step";
    solver_config.warmups = 5;
    solver_config.iterations = 50;
    solver_config.total_operations = 10;

    BenchResult solver_res = BenchRunner::run(solver_config, [&]() {
        stack_world.step_full(1.0f / 60.0f);
        clobber_memory();
    });
    all_results.push_back(solver_res);

    // 5. Structural Graph Benchmark
    std::cout << "\n--- 5. Structural Graph Load Propagation Benchmark ---\n";
    MaterialParams mat;
    StructuralGraph graph(mat);
    std::vector<RigidBody> bodies;
    std::vector<ContactManifold> manifolds;

    bodies.push_back(RigidBody::create_static(1, Vec3(0.0f, 0.0f, 0.0f)));
    for (uint32_t i = 2; i <= 20; ++i) {
        InertiaTensor inertia = InertiaTensor::box(10.0f, 1.0f, 1.0f, 1.0f);
        bodies.push_back(RigidBody::create_dynamic(i, 10.0f, inertia, Vec3(0.0f, static_cast<float>(i), 0.0f)));

        ContactManifold m;
        m.body_a_id = i;
        m.body_b_id = i - 1;
        m.normal = Vec3(0.0f, -1.0f, 0.0f);
        m.add_point(ContactPoint(Vec3(0.0f, static_cast<float>(i) - 0.5f, 0.0f), Vec3(0, -0.5f, 0), Vec3(0, 0.5f, 0), 0.005f, 1));
        manifolds.push_back(m);
    }

    BenchConfig graph_config;
    graph_config.name = "structural_graph";
    graph_config.workload_name = "20_node_load_sweep";
    graph_config.warmups = 5;
    graph_config.iterations = 50;
    graph_config.total_operations = 20;

    BenchResult graph_res = BenchRunner::run(graph_config, [&]() {
        graph.build_from_world_and_contacts(bodies, manifolds);
        graph.evaluate_load_and_connectivity(Vec3(0.0f, -9.81f, 0.0f));
        clobber_memory();
    });
    all_results.push_back(graph_res);

    // 6. Integrated Physics Benchmark
    std::cout << "\n--- 6. Integrated Physics Simulation Step Benchmark ---\n";
    PhysicsWorld world_integ;
    world_integ.set_gravity(Vec3(0.0f, -9.81f, 0.0f));

    RigidBody g_body = RigidBody::create_static(1, Vec3(0.0f, -0.5f, 0.0f));
    Collider g_c = Collider::create_box(1, 1, Vec3(20.0f, 0.5f, 20.0f), g_body.get_transform());
    world_integ.add_body(g_body);
    world_integ.add_collider(g_c);

    for (uint32_t i = 0; i < 16; ++i) {
        float x = (static_cast<float>(i % 4) - 1.5f) * 1.1f;
        float y = 0.5f + static_cast<float>(i / 4) * 1.05f;
        InertiaTensor inertia = InertiaTensor::box(5.0f, 1.0f, 1.0f, 1.0f);
        RigidBody b = RigidBody::create_dynamic(i + 2, 5.0f, inertia, Vec3(x, y, 0.0f));
        Collider col = Collider::create_box(i + 2, i + 2, Vec3(0.5f, 0.5f, 0.5f), b.get_transform());
        world_integ.add_body(b);
        world_integ.add_collider(col);
    }

    BenchConfig integ_config;
    integ_config.name = "integrated_physics";
    integ_config.workload_name = "16_block_scene_step";
    integ_config.warmups = 5;
    integ_config.iterations = 50;
    integ_config.total_operations = 16;

    BenchResult integ_res = BenchRunner::run(integ_config, [&]() {
        world_integ.step_full(1.0f / 60.0f);
        clobber_memory();
    });
    all_results.push_back(integ_res);

    // 7. Full Fracture-to-Collapse Scenario Benchmark
    std::cout << "\n--- 7. Full Fracture-to-Collapse Scenario Benchmark ---\n";
    BenchConfig f2c_config;
    f2c_config.name = "fracture_to_collapse";
    f2c_config.workload_name = "fracture_shards_and_solve";
    f2c_config.warmups = 3;
    f2c_config.iterations = 20;
    f2c_config.total_operations = 1;

    BenchResult f2c_res = BenchRunner::run(f2c_config, [&]() {
        FractureVolume vol(Vec3(-0.5f, -0.5f, -0.5f), Vec3(0.5f, 0.5f, 0.5f), 2400.0f);
        std::vector<Vec3> sites = SiteGenerator::generate_3d(vol, 8, 0.15f, 42);
        std::vector<Shard> shards = Voronoi3D::compute_partition(vol, sites);

        PhysicsWorld world;
        world.set_gravity(Vec3(0.0f, -9.81f, 0.0f));

        RigidBody g = RigidBody::create_static(1, Vec3(0.0f, -0.5f, 0.0f));
        Collider gcol = Collider::create_box(1, 1, Vec3(10.0f, 0.5f, 10.0f), g.get_transform());
        world.add_body(g);
        world.add_collider(gcol);

        uint32_t id = 2;
        for (const auto& s : shards) {
            if (!s.is_valid) continue;
            InertiaTensor inertia = InertiaTensor::box(s.mass, 0.4f, 0.4f, 0.4f);
            RigidBody b = RigidBody::create_dynamic(id, s.mass, inertia, s.centroid + Vec3(0, 1.0f, 0));
            Collider col = Collider::create_polyhedron(id, id, s.mesh, b.get_transform());
            world.add_body(b);
            world.add_collider(col);
            id++;
        }

        world.step_full(1.0f / 60.0f);
        clobber_memory();
    });
    all_results.push_back(f2c_res);

    // Export to CSV
    export_all_csv(all_results, csv_path);

    std::cout << "\n========================================================\n";
    std::cout << "All Benchmarks Completed Successfully.\n";
    std::cout << "Report exported to: " << csv_path << "\n";
    std::cout << "========================================================\n";

    return 0;
}
