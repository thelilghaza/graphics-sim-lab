#include "voxel_lab/naive_mesher.hpp"
#include "voxel_lab/test_worlds.hpp"
#include "voxel_lab/world_grid.hpp"
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

using namespace voxel_lab;

struct BenchmarkResult {
    std::string name;
    std::string chunk_desc;
    size_t face_count;
    size_t vertex_count;
    size_t index_count;
    double avg_time_us;
    double ops_per_sec;
};

BenchmarkResult run_benchmark(const std::string& name,
                              const std::string& chunk_desc,
                              const WorldAccessor& world,
                              const ChunkCoord& chunk_coord,
                              int warmup_runs = 20,
                              int bench_runs = 200) {
    MeshData mesh;
    // Warmup
    for (int i = 0; i < warmup_runs; ++i) {
        mesh.clear();
        mesh_chunk(world, chunk_coord, mesh);
    }

    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < bench_runs; ++i) {
        mesh.clear();
        mesh_chunk(world, chunk_coord, mesh);
    }
    auto end = std::chrono::high_resolution_clock::now();

    double total_us = std::chrono::duration<double, std::micro>(end - start).count();
    double avg_us = total_us / static_cast<double>(bench_runs);
    double ops = (avg_us > 0.0) ? (1000000.0 / avg_us) : 0.0;

    return BenchmarkResult{
        name,
        chunk_desc,
        mesh.face_count(),
        mesh.vertex_count(),
        mesh.index_count(),
        avg_us,
        ops
    };
}

int main(int argc, char** argv) {
    std::cout << "=== Voxel Engine Naive Mesher Benchmark Suite ===\n";

    // A. Empty Chunk
    WorldGrid empty_world;
    BenchmarkResult res_empty = run_benchmark("Empty Chunk", "(0,0,0)", empty_world, ChunkCoord(0,0,0));

    // B. Single Solid Voxel
    WorldGrid single_world;
    single_world.set_voxel(15, 15, 15, Voxel(1, 0));
    BenchmarkResult res_single = run_benchmark("Single Solid Voxel", "(0,0,0)", single_world, ChunkCoord(0,0,0));

    // C. Full Solid 32^3 Chunk
    WorldGrid solid_world;
    generate_solid_world(solid_world, WorldCoord(0, 0, 0), WorldCoord(31, 31, 31), Voxel(1, 0));
    BenchmarkResult res_solid = run_benchmark("Full Solid 32^3 Chunk", "(0,0,0)", solid_world, ChunkCoord(0,0,0));

    // D. Deterministic Plane World (y = 15)
    WorldGrid plane_world;
    generate_plane_world(plane_world, WorldCoord(0, 0, 0), WorldCoord(31, 31, 31), 15, PlaneAxis::Y, Voxel(1, 0));
    BenchmarkResult res_plane = run_benchmark("Deterministic Plane World (y=15)", "(0,0,0)", plane_world, ChunkCoord(0,0,0));

    // E. Deterministic Sphere World (radius = 12 at center (15,15,15))
    WorldGrid sphere_world;
    generate_sphere_world(sphere_world, WorldCoord(15, 15, 15), 12, Voxel(1, 0));
    BenchmarkResult res_sphere = run_benchmark("Deterministic Sphere World (r=12)", "(0,0,0)", sphere_world, ChunkCoord(0,0,0));

    const BenchmarkResult results[] = {res_empty, res_single, res_solid, res_plane, res_sphere};

    std::stringstream report;
    report << "========================================================================\n";
    report << " PROJECT 02 — VOXEL ENGINE: MILESTONE 4 NAIVE MESHER BENCHMARK REPORT\n";
    report << "========================================================================\n\n";
    report << std::left << std::setw(32) << "Benchmark Workload"
           << std::setw(12) << "Faces"
           << std::setw(12) << "Vertices"
           << std::setw(12) << "Indices"
           << std::setw(16) << "Time (us)"
           << std::setw(16) << "Meshes/sec" << "\n";
    report << "----------------------------------------------------------------------------------------------------\n";

    for (const auto& r : results) {
        report << std::left << std::setw(32) << r.name
               << std::setw(12) << r.face_count
               << std::setw(12) << r.vertex_count
               << std::setw(12) << r.index_count
               << std::fixed << std::setprecision(2)
               << std::setw(16) << r.avg_time_us
               << std::setw(16) << r.ops_per_sec << "\n";
    }
    report << "========================================================================\n";

    std::cout << report.str();

    // Optionally write to benchmark output file if requested or write to milestone4_benchmark.txt
    if (argc > 1) {
        std::ofstream out(argv[1]);
        if (out.is_open()) {
            out << report.str();
            std::cout << "[INFO] Benchmark results saved to: " << argv[1] << "\n";
        }
    }

    return 0;
}
