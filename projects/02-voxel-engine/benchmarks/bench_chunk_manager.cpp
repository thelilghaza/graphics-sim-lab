#include "voxel_lab/chunk_manager.hpp"

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace voxel_lab;

struct BenchmarkRecord {
    std::string workload;
    std::string mesher;
    double avg_time_us{0.0};
    size_t chunks_loaded{0};
    size_t chunks_unloaded{0};
    size_t resident_chunks{0};
    size_t total_faces_or_quads{0};
};

BenchmarkRecord bench_initial_population(MesherType mesher, int iters = 20) {
    StreamingConfig config;
    config.load_radius = 2;   // 5x5x5 = 125 chunks
    config.unload_radius = 3;

    double total_us = 0.0;
    size_t loaded = 0;
    size_t resident = 0;
    size_t faces = 0;

    for (int i = 0; i < iters; ++i) {
        ChunkManager mgr(config, mesher);
        auto start = std::chrono::high_resolution_clock::now();
        mgr.update_streaming(Vec3(0.0f, 0.0f, 0.0f));
        auto end = std::chrono::high_resolution_clock::now();

        total_us += std::chrono::duration<double, std::micro>(end - start).count();
        loaded = mgr.get_metrics().chunks_loaded_this_update;
        resident = mgr.loaded_chunk_count();
        faces = mgr.get_metrics().total_faces_or_quads;
    }

    return BenchmarkRecord{
        "Initial Population (r=2, 125 chunks)",
        (mesher == MesherType::Naive ? "Naive" : "Greedy"),
        total_us / iters,
        loaded,
        0,
        resident,
        faces
    };
}

BenchmarkRecord bench_single_chunk_move(MesherType mesher, int iters = 50) {
    StreamingConfig config;
    config.load_radius = 2;
    config.unload_radius = 3;

    double total_us = 0.0;
    size_t loaded = 0;
    size_t unloaded = 0;
    size_t resident = 0;
    size_t faces = 0;

    for (int i = 0; i < iters; ++i) {
        ChunkManager mgr(config, mesher);
        mgr.update_streaming(Vec3(0.0f, 0.0f, 0.0f)); // Setup initial

        auto start = std::chrono::high_resolution_clock::now();
        mgr.update_streaming(Vec3(32.0f, 0.0f, 0.0f)); // Move by 1 chunk (+X)
        auto end = std::chrono::high_resolution_clock::now();

        total_us += std::chrono::duration<double, std::micro>(end - start).count();
        loaded = mgr.get_metrics().chunks_loaded_this_update;
        unloaded = mgr.get_metrics().chunks_unloaded_this_update;
        resident = mgr.loaded_chunk_count();
        faces = mgr.get_metrics().total_faces_or_quads;
    }

    return BenchmarkRecord{
        "Move Camera by 1 Chunk (+X)",
        (mesher == MesherType::Naive ? "Naive" : "Greedy"),
        total_us / iters,
        loaded,
        unloaded,
        resident,
        faces
    };
}

BenchmarkRecord bench_repeated_crossings(MesherType mesher, int steps = 10, int iters = 10) {
    StreamingConfig config;
    config.load_radius = 2;
    config.unload_radius = 3;

    double total_us = 0.0;
    size_t total_loaded = 0;
    size_t total_unloaded = 0;
    size_t resident = 0;
    size_t faces = 0;

    for (int it = 0; it < iters; ++it) {
        ChunkManager mgr(config, mesher);
        mgr.update_streaming(Vec3(0.0f, 0.0f, 0.0f));

        auto start = std::chrono::high_resolution_clock::now();
        for (int s = 1; s <= steps; ++s) {
            float x = static_cast<float>(s * 32);
            mgr.update_streaming(Vec3(x, 0.0f, 0.0f));
        }
        auto end = std::chrono::high_resolution_clock::now();

        total_us += std::chrono::duration<double, std::micro>(end - start).count();
        total_loaded = mgr.get_metrics().total_chunks_loaded;
        total_unloaded = mgr.get_metrics().total_chunks_unloaded;
        resident = mgr.loaded_chunk_count();
        faces = mgr.get_metrics().total_faces_or_quads;
    }

    return BenchmarkRecord{
        "Repeated Boundary Crossings (10 steps)",
        (mesher == MesherType::Naive ? "Naive" : "Greedy"),
        total_us / iters,
        total_loaded,
        total_unloaded,
        resident,
        faces
    };
}

BenchmarkRecord bench_load_unload_sequence(MesherType mesher, int count = 30, int iters = 10) {
    double total_us = 0.0;
    size_t loaded = 0;
    size_t unloaded = 0;

    for (int it = 0; it < iters; ++it) {
        ChunkManager mgr(StreamingConfig{}, mesher);

        auto start = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < count; ++i) {
            mgr.load_chunk(ChunkCoord(i, 0, 0));
        }
        for (int i = 0; i < count; ++i) {
            mgr.unload_chunk(ChunkCoord(i, 0, 0));
        }
        auto end = std::chrono::high_resolution_clock::now();

        total_us += std::chrono::duration<double, std::micro>(end - start).count();
        loaded = count;
        unloaded = count;
    }

    return BenchmarkRecord{
        "Manual Load/Unload Sequence (30 chunks)",
        (mesher == MesherType::Naive ? "Naive" : "Greedy"),
        total_us / iters,
        loaded,
        unloaded,
        0,
        0
    };
}

int main(int argc, char** argv) {
    std::cout << "=== Milestone 7: Dynamic Chunk Manager Streaming Benchmark ===\n";

    std::vector<BenchmarkRecord> records;

    // Workload A: Initial population
    records.push_back(bench_initial_population(MesherType::Naive));
    records.push_back(bench_initial_population(MesherType::Greedy));

    // Workload B: Moving by 1 chunk
    records.push_back(bench_single_chunk_move(MesherType::Naive));
    records.push_back(bench_single_chunk_move(MesherType::Greedy));

    // Workload C: Repeated boundary crossings
    records.push_back(bench_repeated_crossings(MesherType::Naive));
    records.push_back(bench_repeated_crossings(MesherType::Greedy));

    // Workload D: Load/unload sequence
    records.push_back(bench_load_unload_sequence(MesherType::Naive));
    records.push_back(bench_load_unload_sequence(MesherType::Greedy));

    std::stringstream report;
    report << "========================================================================================================\n";
    report << " PROJECT 02 — VOXEL ENGINE: MILESTONE 7 CHUNK MANAGER STREAMING BENCHMARK REPORT\n";
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
    report << "  Timing Source:       std::chrono::high_resolution_clock\n\n";

    report << "Dynamic Streaming Benchmark Summary:\n";
    report << "--------------------------------------------------------------------------------------------------------------------------------\n";
    report << std::left << std::setw(42) << "Workload Description"
           << std::setw(10) << "Mesher"
           << std::right
           << std::setw(16) << "Avg Time (us)"
           << std::setw(14) << "Time (ms)"
           << std::setw(14) << "Chunks Loaded"
           << std::setw(14) << "Chunks Unload"
           << std::setw(14) << "Resident"
           << std::setw(16) << "Total Faces/Quads"
           << "\n";
    report << "--------------------------------------------------------------------------------------------------------------------------------\n";

    for (const auto& r : records) {
        report << std::left << std::setw(42) << r.workload
               << std::setw(10) << r.mesher
               << std::right << std::fixed << std::setprecision(2)
               << std::setw(16) << r.avg_time_us
               << std::setw(14) << (r.avg_time_us / 1000.0)
               << std::setw(14) << r.chunks_loaded
               << std::setw(14) << r.chunks_unloaded
               << std::setw(14) << r.resident_chunks
               << std::setw(16) << r.total_faces_or_quads
               << "\n";
    }
    report << "========================================================================================================\n";

    std::cout << report.str();

    std::string out_path = (argc > 1) ? argv[1] : "projects/02-voxel-engine/benchmarks/milestone7_benchmark.txt";
    std::ofstream out(out_path);
    if (out.is_open()) {
        out << report.str();
        std::cout << "\n[INFO] Benchmark results successfully written to: " << out_path << "\n";
    }

    return 0;
}
