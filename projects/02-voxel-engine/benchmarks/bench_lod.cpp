#include "voxel_lab/chunk_manager.hpp"
#include "voxel_lab/greedy_mesher.hpp"
#include "voxel_lab/lod.hpp"
#include "voxel_lab/naive_mesher.hpp"
#include "voxel_lab/test_worlds.hpp"
#include "voxel_lab/world_grid.hpp"

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace voxel_lab;

struct LodBenchmarkResult {
    std::string name;
    bool lod_enabled{false};
    size_t worker_count{0};
    size_t resident_chunks{0};
    size_t lod0_chunks{0};
    size_t lod1_chunks{0};
    size_t lod2_chunks{0};
    size_t total_quads{0};
    size_t total_vertices{0};
    size_t total_indices{0};
    size_t logical_mesh_bytes{0};
    double wall_time_ms{0.0};
    double cpu_gen_time_ms{0.0};
    double cpu_mesh_time_ms{0.0};
    size_t jobs_completed{0};
    size_t lod_changes{0};
    size_t stale_jobs{0};
};

LodBenchmarkResult run_workload_a_single_chunk(LODLevel level, MesherType mesher) {
    WorldGrid world;
    generate_sphere_world(world, WorldCoord(16, 16, 16), 14, Voxel(1, 0));
    ChunkCoord c(0, 0, 0);

    const int iterations = 100;
    // Warmup
    MeshData m_warmup = mesh_chunk_lod(world, c, level, mesher);

    auto t0 = std::chrono::high_resolution_clock::now();
    MeshData m;
    for (int i = 0; i < iterations; ++i) {
        m = mesh_chunk_lod(world, c, level, mesher);
    }
    auto t1 = std::chrono::high_resolution_clock::now();
    double total_us = std::chrono::duration<double, std::micro>(t1 - t0).count();
    double avg_ms = (total_us / iterations) / 1000.0;

    LodBenchmarkResult res;
    std::stringstream ss;
    ss << "Workload A: Single Chunk " << lod_level_name(level) << " [" << mesher_type_name(mesher) << "]";
    res.name = ss.str();
    res.lod_enabled = (level != LODLevel::LOD0);
    res.worker_count = 0;
    res.resident_chunks = 1;
    res.lod0_chunks = (level == LODLevel::LOD0 ? 1 : 0);
    res.lod1_chunks = (level == LODLevel::LOD1 ? 1 : 0);
    res.lod2_chunks = (level == LODLevel::LOD2 ? 1 : 0);
    res.total_quads = m.quad_count();
    res.total_vertices = m.vertex_count();
    res.total_indices = m.index_count();
    res.logical_mesh_bytes = m.total_logical_bytes();
    res.wall_time_ms = avg_ms;
    res.cpu_mesh_time_ms = avg_ms;
    res.jobs_completed = 1;
    return res;
}

LodBenchmarkResult run_region_benchmark(const std::string& name, int radius, bool enable_lod, size_t worker_count) {
    StreamingConfig cfg;
    cfg.load_radius = radius;
    cfg.unload_radius = radius + 1;
    cfg.worker_count = worker_count;
    cfg.enable_lod = enable_lod;
    cfg.lod0_radius = 1;
    cfg.lod1_radius = 2;

    ChunkManager mgr(cfg, MesherType::Greedy);

    auto t0 = std::chrono::high_resolution_clock::now();
    mgr.update_streaming(Vec3(0.0f, 0.0f, 0.0f));
    mgr.wait_all_pending();
    auto t1 = std::chrono::high_resolution_clock::now();

    double wall_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    const auto& m = mgr.get_metrics();

    LodBenchmarkResult res;
    res.name = name;
    res.lod_enabled = enable_lod;
    res.worker_count = worker_count;
    res.resident_chunks = mgr.loaded_chunk_count();
    res.lod0_chunks = m.lod0_chunk_count;
    res.lod1_chunks = m.lod1_chunk_count;
    res.lod2_chunks = m.lod2_chunk_count;
    res.total_quads = m.total_faces_or_quads;
    res.total_vertices = m.total_vertices;
    res.total_indices = m.total_indices;
    res.logical_mesh_bytes = mgr.get_memory_stats().total_mesh_logical_bytes;
    res.wall_time_ms = wall_ms;
    res.cpu_gen_time_ms = m.total_generation_time_us / 1000.0;
    res.cpu_mesh_time_ms = m.total_mesh_time_us / 1000.0;
    res.jobs_completed = m.jobs_completed;
    res.lod_changes = m.lod_changes;
    res.stale_jobs = m.jobs_discarded_stale;
    return res;
}

LodBenchmarkResult run_streaming_movement_benchmark(const std::string& name, bool enable_lod, size_t worker_count) {
    StreamingConfig cfg;
    cfg.load_radius = 2;
    cfg.unload_radius = 3;
    cfg.worker_count = worker_count;
    cfg.enable_lod = enable_lod;
    cfg.lod0_radius = 1;
    cfg.lod1_radius = 2;

    ChunkManager mgr(cfg, MesherType::Greedy);
    mgr.update_streaming(Vec3(0.0f, 0.0f, 0.0f));
    mgr.wait_all_pending();
    mgr.reset_cumulative_metrics();

    auto t0 = std::chrono::high_resolution_clock::now();
    // Simulate camera moving across 5 chunk positions
    for (int step = 1; step <= 5; ++step) {
        Vec3 pos(step * 32.0f, 0.0f, step * 32.0f);
        mgr.update_streaming(pos);
        mgr.wait_all_pending();
    }
    auto t1 = std::chrono::high_resolution_clock::now();

    double wall_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    const auto& m = mgr.get_metrics();

    LodBenchmarkResult res;
    res.name = name;
    res.lod_enabled = enable_lod;
    res.worker_count = worker_count;
    res.resident_chunks = mgr.loaded_chunk_count();
    res.lod0_chunks = m.lod0_chunk_count;
    res.lod1_chunks = m.lod1_chunk_count;
    res.lod2_chunks = m.lod2_chunk_count;
    res.total_quads = m.total_faces_or_quads;
    res.total_vertices = m.total_vertices;
    res.total_indices = m.total_indices;
    res.logical_mesh_bytes = mgr.get_memory_stats().total_mesh_logical_bytes;
    res.wall_time_ms = wall_ms;
    res.cpu_gen_time_ms = m.total_generation_time_us / 1000.0;
    res.cpu_mesh_time_ms = m.total_mesh_time_us / 1000.0;
    res.jobs_completed = m.jobs_completed;
    res.lod_changes = m.lod_changes;
    res.stale_jobs = m.jobs_discarded_stale;
    return res;
}

LodBenchmarkResult run_large_world_experiment(bool enable_lod) {
    // Large World Experiment: 10x10x1 chunk grid = 100 chunks
    WorldGrid world;
    for (int cz = -5; cz < 5; ++cz) {
        for (int cx = -5; cx < 5; ++cx) {
            generate_default_terrain_chunk(world, ChunkCoord(cx, 0, cz));
        }
    }

    ChunkCoord cam_chunk(0, 0, 0);
    auto t0 = std::chrono::high_resolution_clock::now();

    size_t total_quads = 0;
    size_t total_verts = 0;
    size_t total_indices = 0;
    size_t logical_bytes = 0;
    size_t l0_cnt = 0, l1_cnt = 0, l2_cnt = 0;

    for (int cz = -5; cz < 5; ++cz) {
        for (int cx = -5; cx < 5; ++cx) {
            ChunkCoord c(cx, 0, cz);
            LODLevel lod = select_lod_level(c, cam_chunk, enable_lod, 1, 3);
            if (lod == LODLevel::LOD0) l0_cnt++;
            else if (lod == LODLevel::LOD1) l1_cnt++;
            else l2_cnt++;

            MeshData m = mesh_chunk_lod(world, c, lod, MesherType::Greedy);
            total_quads += m.quad_count();
            total_verts += m.vertex_count();
            total_indices += m.index_count();
            logical_bytes += m.total_logical_bytes();
        }
    }
    auto t1 = std::chrono::high_resolution_clock::now();
    double wall_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    LodBenchmarkResult res;
    res.name = std::string("Large-World (100 Chunks): ") + (enable_lod ? "Mixed-LOD" : "All-LOD0");
    res.lod_enabled = enable_lod;
    res.worker_count = 0;
    res.resident_chunks = 100;
    res.lod0_chunks = l0_cnt;
    res.lod1_chunks = l1_cnt;
    res.lod2_chunks = l2_cnt;
    res.total_quads = total_quads;
    res.total_vertices = total_verts;
    res.total_indices = total_indices;
    res.logical_mesh_bytes = logical_bytes;
    res.wall_time_ms = wall_ms;
    res.cpu_mesh_time_ms = wall_ms;
    res.jobs_completed = 100;
    return res;
}

int main() {
    std::cout << "========================================================\n";
    std::cout << "   RUNNING BENCHMARK: bench_lod\n";
    std::cout << "========================================================\n";

    std::vector<LodBenchmarkResult> results;

    // Workload A: Single Chunk LOD 0, LOD 1, LOD 2
    results.push_back(run_workload_a_single_chunk(LODLevel::LOD0, MesherType::Greedy));
    results.push_back(run_workload_a_single_chunk(LODLevel::LOD1, MesherType::Greedy));
    results.push_back(run_workload_a_single_chunk(LODLevel::LOD2, MesherType::Greedy));

    // Workload B: 27 Chunks Region (Load radius 1)
    results.push_back(run_region_benchmark("Workload B: 27 Chunks (LOD OFF)", 1, false, 4));
    results.push_back(run_region_benchmark("Workload B: 27 Chunks (LOD ON)", 1, true, 4));

    // Workload C: 125 Chunks Region (Load radius 2)
    results.push_back(run_region_benchmark("Workload C: 125 Chunks (LOD OFF)", 2, false, 4));
    results.push_back(run_region_benchmark("Workload C: 125 Chunks (LOD ON)", 2, true, 4));

    // Workload D: Streaming Region (Load radius 2 around origin)
    results.push_back(run_region_benchmark("Workload D: Streaming 150 Chunks (LOD OFF)", 2, false, 4));
    results.push_back(run_region_benchmark("Workload D: Streaming 150 Chunks (LOD ON)", 2, true, 4));

    // Workload E: Repeated Streaming Movement (5 crossings)
    results.push_back(run_streaming_movement_benchmark("Workload E: 5 Boundary Crossings (LOD OFF)", false, 4));
    results.push_back(run_streaming_movement_benchmark("Workload E: 5 Boundary Crossings (LOD ON)", true, 4));

    // Large World Experiment (10x10 = 100 chunks grid)
    LodBenchmarkResult lw_lod0 = run_large_world_experiment(false);
    LodBenchmarkResult lw_lod_mixed = run_large_world_experiment(true);
    results.push_back(lw_lod0);
    results.push_back(lw_lod_mixed);

    // Write text report
    std::ofstream out("benchmarks/milestone10_benchmark.txt");
    std::stringstream report;

    report << "=====================================================================================================\n";
    report << "   PROJECT 02 — MILESTONE 10 BENCHMARK REPORT: LEVEL OF DETAIL & LARGE-WORLD SCALE EXPERIMENTS\n";
    report << "=====================================================================================================\n\n";

    std::string compiler_info;
#if defined(_MSC_VER)
    compiler_info = "MSVC " + std::to_string(_MSC_VER) + " (x64 Release build)";
#elif defined(__clang__)
    compiler_info = "Clang " + std::to_string(__clang_major__) + "." + std::to_string(__clang_minor__) + " (Release build)";
#elif defined(__GNUC__)
    compiler_info = "GCC " + std::to_string(__GNUC__) + "." + std::to_string(__GNUC_MINOR__) + " (Release build)";
#else
    compiler_info = "Unknown Compiler (Release build)";
#endif

    report << "CONFIGURATION:\n";
    report << "  Compiler: " << compiler_info << "\n";
    report << "  Timing Source: std::chrono::high_resolution_clock\n";
    report << "  Worker Count: 4 threads\n";
    report << "  LOD Scheme: LOD 0 (1x1x1), LOD 1 (2x2x2 step), LOD 2 (4x4x4 step)\n\n";

    report << "SUMMARY COMPARISON TABLE:\n";
    report << std::left
           << std::setw(45) << "Workload"
           << std::setw(10) << "LOD Mode"
           << std::setw(12) << "Resident"
           << std::setw(14) << "Quads"
           << std::setw(14) << "Vertices"
           << std::setw(16) << "Logical Bytes"
           << std::setw(12) << "Wall (ms)"
           << "\n";
    report << std::string(123, '-') << "\n";

    for (const auto& r : results) {
        report << std::left
               << std::setw(45) << r.name
               << std::setw(10) << (r.lod_enabled ? "ON" : "OFF")
               << std::setw(12) << r.resident_chunks
               << std::setw(14) << r.total_quads
               << std::setw(14) << r.total_vertices
               << std::setw(16) << r.logical_mesh_bytes
               << std::setw(12) << std::fixed << std::setprecision(2) << r.wall_time_ms
               << "\n";
    }

    report << "\n" << std::string(100, '=') << "\n";
    report << "LARGE-WORLD SCALE EXPERIMENT ANALYSIS (100 Chunks 10x10 Grid):\n";
    report << std::string(100, '=') << "\n";

    double quad_red = 100.0 * (1.0 - static_cast<double>(lw_lod_mixed.total_quads) / static_cast<double>(lw_lod0.total_quads));
    double vert_red = 100.0 * (1.0 - static_cast<double>(lw_lod_mixed.total_vertices) / static_cast<double>(lw_lod0.total_vertices));
    double mem_red = 100.0 * (1.0 - static_cast<double>(lw_lod_mixed.logical_mesh_bytes) / static_cast<double>(lw_lod0.logical_mesh_bytes));

    report << "  All-LOD0 Baseline Geometry:  " << lw_lod0.total_quads << " quads, " << lw_lod0.total_vertices << " verts (" << (lw_lod0.logical_mesh_bytes / 1024.0 / 1024.0) << " MB)\n";
    report << "  Mixed-LOD Optimized Geometry:" << lw_lod_mixed.total_quads << " quads, " << lw_lod_mixed.total_vertices << " verts (" << (lw_lod_mixed.logical_mesh_bytes / 1024.0 / 1024.0) << " MB)\n";
    report << "  LOD Chunk Distribution:      LOD0: " << lw_lod_mixed.lod0_chunks << ", LOD1: " << lw_lod_mixed.lod1_chunks << ", LOD2: " << lw_lod_mixed.lod2_chunks << "\n";
    report << "  Geometry Quad Reduction:     " << std::fixed << std::setprecision(2) << quad_red << "%\n";
    report << "  Vertex Reduction:            " << std::fixed << std::setprecision(2) << vert_red << "%\n";
    report << "  Logical Mesh Memory Savings: " << std::fixed << std::setprecision(2) << mem_red << "%\n";
    report << "  Build Cost Comparison:       All-LOD0: " << lw_lod0.wall_time_ms << " ms vs Mixed-LOD: " << lw_lod_mixed.wall_time_ms << " ms\n";

    out << report.str();
    out.close();

    std::cout << report.str();
    std::cout << "\nBenchmark results saved to benchmarks/milestone10_benchmark.txt\n";
    return 0;
}
