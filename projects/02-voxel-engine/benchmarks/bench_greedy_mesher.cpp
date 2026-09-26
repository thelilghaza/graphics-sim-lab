#include "voxel_lab/greedy_mesher.hpp"
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

struct MesherStats {
    size_t face_count{0};
    size_t vertex_count{0};
    size_t index_count{0};
    double avg_time_us{0.0};
};

struct BenchmarkComparison {
    std::string name;
    std::string chunk_coord;
    MesherStats naive;
    MesherStats greedy;

    double face_reduction_pct() const {
        if (naive.face_count == 0) return 0.0;
        return (1.0 - static_cast<double>(greedy.face_count) / static_cast<double>(naive.face_count)) * 100.0;
    }

    double vertex_reduction_pct() const {
        if (naive.vertex_count == 0) return 0.0;
        return (1.0 - static_cast<double>(greedy.vertex_count) / static_cast<double>(naive.vertex_count)) * 100.0;
    }

    double index_reduction_pct() const {
        if (naive.index_count == 0) return 0.0;
        return (1.0 - static_cast<double>(greedy.index_count) / static_cast<double>(naive.index_count)) * 100.0;
    }

    double speedup_ratio() const {
        if (greedy.avg_time_us <= 0.0) return 1.0;
        return naive.avg_time_us / greedy.avg_time_us;
    }

    double abs_time_diff_us() const {
        return greedy.avg_time_us - naive.avg_time_us;
    }
};

MesherStats benchmark_naive(const WorldAccessor& world, const ChunkCoord& coord, int warmup, int iters) {
    MeshData mesh;
    for (int i = 0; i < warmup; ++i) {
        mesh.clear();
        mesh_chunk(world, coord, mesh);
    }

    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < iters; ++i) {
        mesh.clear();
        mesh_chunk(world, coord, mesh);
    }
    auto end = std::chrono::high_resolution_clock::now();

    double total_us = std::chrono::duration<double, std::micro>(end - start).count();
    return MesherStats{
        mesh.face_count(),
        mesh.vertex_count(),
        mesh.index_count(),
        total_us / static_cast<double>(iters)
    };
}

MesherStats benchmark_greedy(const WorldAccessor& world, const ChunkCoord& coord, int warmup, int iters) {
    MeshData mesh;
    for (int i = 0; i < warmup; ++i) {
        mesh.clear();
        greedy_mesh_chunk(world, coord, mesh);
    }

    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < iters; ++i) {
        mesh.clear();
        greedy_mesh_chunk(world, coord, mesh);
    }
    auto end = std::chrono::high_resolution_clock::now();

    double total_us = std::chrono::duration<double, std::micro>(end - start).count();
    return MesherStats{
        mesh.face_count(),
        mesh.vertex_count(),
        mesh.index_count(),
        total_us / static_cast<double>(iters)
    };
}

BenchmarkComparison run_case(const std::string& name,
                             const std::string& coord_str,
                             const WorldAccessor& world,
                             const ChunkCoord& coord,
                             int warmup = 20,
                             int iters = 200) {
    MesherStats naive = benchmark_naive(world, coord, warmup, iters);
    MesherStats greedy = benchmark_greedy(world, coord, warmup, iters);

    return BenchmarkComparison{name, coord_str, naive, greedy};
}

int main(int argc, char** argv) {
    const int warmup = 20;
    const int iters = 200;

    std::cout << "=== Milestone 6: Naive vs Greedy Mesher Benchmark ===\n";

    std::vector<BenchmarkComparison> comparisons;

    // A. Empty Chunk
    {
        WorldGrid world;
        comparisons.push_back(run_case("Empty Chunk", "(0,0,0)", world, ChunkCoord(0,0,0), warmup, iters));
    }

    // B. Single Solid Voxel
    {
        WorldGrid world;
        world.set_voxel(15, 15, 15, Voxel(1, 0));
        comparisons.push_back(run_case("Single Solid Voxel", "(0,0,0)", world, ChunkCoord(0,0,0), warmup, iters));
    }

    // C. Full Solid 32^3 Chunk
    {
        WorldGrid world;
        generate_solid_world(world, WorldCoord(0,0,0), WorldCoord(31,31,31), Voxel(1, 0));
        comparisons.push_back(run_case("Full Solid 32^3 Chunk", "(0,0,0)", world, ChunkCoord(0,0,0), warmup, iters));
    }

    // D. Deterministic Plane World (y=15)
    {
        WorldGrid world;
        generate_plane_world(world, WorldCoord(0,0,0), WorldCoord(31,31,31), 15, PlaneAxis::Y, Voxel(1, 0));
        comparisons.push_back(run_case("Deterministic Plane (y=15)", "(0,0,0)", world, ChunkCoord(0,0,0), warmup, iters));
    }

    // E. Deterministic Sphere World (r=12 at (15,15,15))
    {
        WorldGrid world;
        generate_sphere_world(world, WorldCoord(15,15,15), 12, Voxel(1, 0));
        comparisons.push_back(run_case("Deterministic Sphere (r=12)", "(0,0,0)", world, ChunkCoord(0,0,0), warmup, iters));
    }

    // F. Cross-Chunk Sphere (r=12 at (31,31,31), meshing chunk (0,0,0))
    {
        WorldGrid world;
        generate_sphere_world(world, WorldCoord(31,31,31), 12, Voxel(1, 0));
        comparisons.push_back(run_case("Cross-Chunk Sphere (chunk 0)", "(0,0,0)", world, ChunkCoord(0,0,0), warmup, iters));
    }

    // G. Mixed-Voxel Compatibility Case (Planar slab y in [0,15] with alternating 4-voxel stripes of type 1 and type 2)
    {
        WorldGrid world;
        for (int z = 0; z < 32; ++z) {
            for (int y = 0; y <= 15; ++y) {
                for (int x = 0; x < 32; ++x) {
                    uint8_t type = ((x / 4) % 2 == 0) ? 1 : 2;
                    world.set_voxel(x, y, z, Voxel(type, 0));
                }
            }
        }
        comparisons.push_back(run_case("Mixed Voxel Stripes (types 1&2)", "(0,0,0)", world, ChunkCoord(0,0,0), warmup, iters));
    }

    std::stringstream report;
    report << "========================================================================================================\n";
    report << " PROJECT 02 — VOXEL ENGINE: MILESTONE 6 NAIVE vs GREEDY MESHER BENCHMARK REPORT\n";
    report << "========================================================================================================\n\n";

#if defined(_MSC_VER)
    std::string compiler_info = "MSVC " + std::to_string(_MSC_VER);
#elif defined(__clang__)
    std::string compiler_info = "Clang " + std::to_string(__clang_major__) + "." + std::to_string(__clang_minor__);
#elif defined(__GNUC__)
    std::string compiler_info = "GCC " + std::to_string(__GNUC__) + "." + std::to_string(__GNUC_MINOR__);
#else
    std::string compiler_info = "Unknown Compiler";
#endif

#if defined(NDEBUG)
    std::string build_config = "Release";
#else
    std::string build_config = "Debug";
#endif

    report << "Environment & Configuration:\n";
    report << "  Build Configuration: " << build_config << "\n";
    report << "  Compiler:            " << compiler_info << "\n";
    report << "  Warmup Iterations:   " << warmup << "\n";
    report << "  Benchmark Runs:      " << iters << "\n";
    report << "  Timing Source:       std::chrono::high_resolution_clock\n\n";

    report << "Geometry Reduction & Generation Timing Summary:\n";
    report << "--------------------------------------------------------------------------------------------------------------------------------\n";
    report << std::left << std::setw(32) << "Workload Case"
           << std::right
           << std::setw(12) << "Naive Faces"
           << std::setw(13) << "Greedy Quads"
           << std::setw(14) << "Face Red. %"
           << std::setw(14) << "Naive (us)"
           << std::setw(14) << "Greedy (us)"
           << std::setw(13) << "Speedup"
           << std::setw(14) << "Diff (us)"
           << "\n";
    report << "--------------------------------------------------------------------------------------------------------------------------------\n";

    for (const auto& c : comparisons) {
        report << std::left << std::setw(32) << c.name
               << std::right
               << std::setw(12) << c.naive.face_count
               << std::setw(13) << c.greedy.face_count
               << std::fixed << std::setprecision(2)
               << std::setw(13) << c.face_reduction_pct() << "%"
               << std::setw(14) << c.naive.avg_time_us
               << std::setw(14) << c.greedy.avg_time_us
               << std::setw(12) << c.speedup_ratio() << "x"
               << std::setw(14) << c.abs_time_diff_us()
               << "\n";
    }
    report << "--------------------------------------------------------------------------------------------------------------------------------\n\n";

    report << "Vertex & Index Detailed Counts:\n";
    report << "--------------------------------------------------------------------------------------------------------------------------------\n";
    report << std::left << std::setw(32) << "Workload Case"
           << std::right
           << std::setw(14) << "Naive Verts"
           << std::setw(14) << "Greedy Verts"
           << std::setw(13) << "Vert Red. %"
           << std::setw(14) << "Naive Idxs"
           << std::setw(14) << "Greedy Idxs"
           << std::setw(13) << "Idx Red. %"
           << "\n";
    report << "--------------------------------------------------------------------------------------------------------------------------------\n";

    for (const auto& c : comparisons) {
        report << std::left << std::setw(32) << c.name
               << std::right
               << std::setw(14) << c.naive.vertex_count
               << std::setw(14) << c.greedy.vertex_count
               << std::fixed << std::setprecision(2)
               << std::setw(12) << c.vertex_reduction_pct() << "%"
               << std::setw(14) << c.naive.index_count
               << std::setw(14) << c.greedy.index_count
               << std::setw(12) << c.index_reduction_pct() << "%"
               << "\n";
    }
    report << "========================================================================================================\n";

    std::cout << report.str();

    std::string out_path = (argc > 1) ? argv[1] : "projects/02-voxel-engine/benchmarks/milestone6_benchmark.txt";
    std::ofstream out(out_path);
    if (out.is_open()) {
        out << report.str();
        std::cout << "\n[INFO] Benchmark results successfully written to: " << out_path << "\n";
    }

    return 0;
}
