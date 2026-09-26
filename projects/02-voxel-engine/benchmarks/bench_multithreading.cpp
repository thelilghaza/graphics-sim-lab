#include "voxel_lab/chunk_manager.hpp"

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace voxel_lab;

struct ConcurrencyBenchResult {
    std::string workload;
    size_t workers{0};
    double total_wall_time_ms{0.0};
    double total_generation_time_ms{0.0};
    double total_mesh_time_ms{0.0};
    size_t chunks_generated{0};
    size_t chunks_meshed{0};
    size_t stale_jobs{0};
    size_t resident_chunks{0};
    size_t total_faces_or_quads{0};
    double speedup_vs_1{1.0};
    double parallel_efficiency{1.0};
};

// Workload A: Initial population (radius 2, 125 chunks)
ConcurrencyBenchResult bench_initial_population(size_t workers, MesherType mesher, int iters = 5) {
    StreamingConfig config;
    config.load_radius = 2;
    config.unload_radius = 3;
    config.worker_count = workers;

    double total_wall_ms = 0.0;
    double total_gen_ms = 0.0;
    double total_mesh_ms = 0.0;
    size_t gen_count = 0;
    size_t mesh_count = 0;
    size_t resident = 0;
    size_t faces = 0;

    for (int i = 0; i < iters; ++i) {
        ChunkManager mgr(config, mesher);
        auto start = std::chrono::high_resolution_clock::now();
        mgr.update_streaming(Vec3(0.0f, 0.0f, 0.0f));
        mgr.wait_all_pending();
        auto end = std::chrono::high_resolution_clock::now();

        total_wall_ms += std::chrono::duration<double, std::milli>(end - start).count();
        total_gen_ms += (mgr.get_metrics().total_generation_time_us / 1000.0);
        total_mesh_ms += (mgr.get_metrics().total_mesh_time_us / 1000.0);
        gen_count = mgr.get_metrics().chunks_generated;
        mesh_count = mgr.get_metrics().total_chunks_meshed;
        resident = mgr.loaded_chunk_count();
        faces = mgr.get_metrics().total_faces_or_quads;
    }

    return ConcurrencyBenchResult{
        "Workload A: Initial Population (r=2, 125 chunks)",
        workers,
        total_wall_ms / iters,
        total_gen_ms / iters,
        total_mesh_ms / iters,
        gen_count,
        mesh_count,
        0,
        resident,
        faces
    };
}

// Workload B: Move camera by one chunk
ConcurrencyBenchResult bench_single_chunk_move(size_t workers, MesherType mesher, int iters = 10) {
    StreamingConfig config;
    config.load_radius = 2;
    config.unload_radius = 3;
    config.worker_count = workers;

    double total_wall_ms = 0.0;
    double total_gen_ms = 0.0;
    double total_mesh_ms = 0.0;
    size_t gen_count = 0;
    size_t mesh_count = 0;
    size_t resident = 0;
    size_t faces = 0;

    for (int i = 0; i < iters; ++i) {
        ChunkManager mgr(config, mesher);
        mgr.update_streaming(Vec3(0.0f, 0.0f, 0.0f));
        mgr.wait_all_pending();

        mgr.reset_cumulative_metrics();

        auto start = std::chrono::high_resolution_clock::now();
        mgr.update_streaming(Vec3(32.0f, 0.0f, 0.0f));
        mgr.wait_all_pending();
        auto end = std::chrono::high_resolution_clock::now();

        total_wall_ms += std::chrono::duration<double, std::milli>(end - start).count();
        total_gen_ms += (mgr.get_metrics().total_generation_time_us / 1000.0);
        total_mesh_ms += (mgr.get_metrics().total_mesh_time_us / 1000.0);
        gen_count = mgr.get_metrics().chunks_generated;
        mesh_count = mgr.get_metrics().total_chunks_meshed;
        resident = mgr.loaded_chunk_count();
        faces = mgr.get_metrics().total_faces_or_quads;
    }

    return ConcurrencyBenchResult{
        "Workload B: Move Camera by 1 Chunk (+X, 25 chunks)",
        workers,
        total_wall_ms / iters,
        total_gen_ms / iters,
        total_mesh_ms / iters,
        gen_count,
        mesh_count,
        0,
        resident,
        faces
    };
}

// Workload C: Repeated boundary crossings (5 steps)
ConcurrencyBenchResult bench_repeated_crossings(size_t workers, MesherType mesher, int iters = 5) {
    StreamingConfig config;
    config.load_radius = 2;
    config.unload_radius = 3;
    config.worker_count = workers;

    double total_wall_ms = 0.0;
    double total_gen_ms = 0.0;
    double total_mesh_ms = 0.0;
    size_t gen_count = 0;
    size_t mesh_count = 0;
    size_t stale_count = 0;
    size_t resident = 0;
    size_t faces = 0;

    for (int i = 0; i < iters; ++i) {
        ChunkManager mgr(config, mesher);
        mgr.update_streaming(Vec3(0.0f, 0.0f, 0.0f));
        mgr.wait_all_pending();

        mgr.reset_cumulative_metrics();

        auto start = std::chrono::high_resolution_clock::now();
        for (int s = 1; s <= 5; ++s) {
            float x = static_cast<float>(s * 32);
            mgr.update_streaming(Vec3(x, 0.0f, 0.0f));
        }
        mgr.wait_all_pending();
        auto end = std::chrono::high_resolution_clock::now();

        total_wall_ms += std::chrono::duration<double, std::milli>(end - start).count();
        total_gen_ms += (mgr.get_metrics().total_generation_time_us / 1000.0);
        total_mesh_ms += (mgr.get_metrics().total_mesh_time_us / 1000.0);
        gen_count = mgr.get_metrics().chunks_generated;
        mesh_count = mgr.get_metrics().total_chunks_meshed;
        stale_count = mgr.get_metrics().jobs_discarded_stale;
        resident = mgr.loaded_chunk_count();
        faces = mgr.get_metrics().total_faces_or_quads;
    }

    return ConcurrencyBenchResult{
        "Workload C: Repeated Crossings (5 steps)",
        workers,
        total_wall_ms / iters,
        total_gen_ms / iters,
        total_mesh_ms / iters,
        gen_count,
        mesh_count,
        stale_count,
        resident,
        faces
    };
}

// Workload D: Manual load/unload sequence (30 chunks)
ConcurrencyBenchResult bench_load_unload_sequence(size_t workers, MesherType mesher, int iters = 5) {
    double total_wall_ms = 0.0;
    double total_gen_ms = 0.0;
    double total_mesh_ms = 0.0;
    size_t gen_count = 0;
    size_t mesh_count = 0;

    for (int it = 0; it < iters; ++it) {
        StreamingConfig config;
        config.worker_count = workers;
        ChunkManager mgr(config, mesher);

        auto start = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < 30; ++i) {
            mgr.load_chunk(ChunkCoord(i, 0, 0));
        }
        for (int i = 0; i < 30; ++i) {
            mgr.unload_chunk(ChunkCoord(i, 0, 0));
        }
        mgr.wait_all_pending();
        auto end = std::chrono::high_resolution_clock::now();

        total_wall_ms += std::chrono::duration<double, std::milli>(end - start).count();
        total_gen_ms += (mgr.get_metrics().total_generation_time_us / 1000.0);
        total_mesh_ms += (mgr.get_metrics().total_mesh_time_us / 1000.0);
        gen_count = mgr.get_metrics().chunks_generated;
        mesh_count = mgr.get_metrics().total_chunks_meshed;
    }

    return ConcurrencyBenchResult{
        "Workload D: Manual Load/Unload (30 chunks)",
        workers,
        total_wall_ms / iters,
        total_gen_ms / iters,
        total_mesh_ms / iters,
        gen_count,
        mesh_count,
        0,
        0,
        0
    };
}

// Workload E: Fixed multi-chunk generation + meshing workload (64 chunks)
ConcurrencyBenchResult bench_fixed_workload(size_t workers, MesherType mesher, int iters = 5) {
    double total_wall_ms = 0.0;
    double total_gen_ms = 0.0;
    double total_mesh_ms = 0.0;
    size_t gen_count = 0;
    size_t mesh_count = 0;
    size_t faces = 0;

    for (int it = 0; it < iters; ++it) {
        StreamingConfig config;
        config.worker_count = workers;
        ChunkManager mgr(config, mesher);

        auto start = std::chrono::high_resolution_clock::now();
        for (int z = 0; z < 4; ++z) {
            for (int y = 0; y < 4; ++y) {
                for (int x = 0; x < 4; ++x) {
                    mgr.load_chunk(ChunkCoord(x, y, z));
                }
            }
        }
        mgr.wait_all_pending();
        auto end = std::chrono::high_resolution_clock::now();

        total_wall_ms += std::chrono::duration<double, std::milli>(end - start).count();
        total_gen_ms += (mgr.get_metrics().total_generation_time_us / 1000.0);
        total_mesh_ms += (mgr.get_metrics().total_mesh_time_us / 1000.0);
        gen_count = mgr.get_metrics().chunks_generated;
        mesh_count = mgr.get_metrics().total_chunks_meshed;
        faces = mgr.get_metrics().total_faces_or_quads;
    }

    return ConcurrencyBenchResult{
        "Workload E: Fixed Multi-Chunk Build (64 chunks)",
        workers,
        total_wall_ms / iters,
        total_gen_ms / iters,
        total_mesh_ms / iters,
        gen_count,
        mesh_count,
        0,
        64,
        faces
    };
}

int main(int argc, char** argv) {
    std::cout << "=== Milestone 8: Multithreaded Chunk Streaming & Meshing Scaling Benchmark ===\n";

    const size_t worker_counts[] = {1, 2, 4, 8};
    std::vector<ConcurrencyBenchResult> all_results;

    auto run_suite_for_workload = [&](const std::string& name, auto bench_fn, MesherType mesher) {
        std::cout << "Running " << name << " [" << (mesher == MesherType::Naive ? "Naive" : "Greedy") << "]...\n";
        double base_time = 0.0;
        for (size_t w : worker_counts) {
            auto res = bench_fn(w, mesher);
            if (w == 1) {
                base_time = res.total_wall_time_ms;
                res.speedup_vs_1 = 1.0;
                res.parallel_efficiency = 1.0;
            } else {
                res.speedup_vs_1 = (res.total_wall_time_ms > 0.0) ? (base_time / res.total_wall_time_ms) : 1.0;
                res.parallel_efficiency = res.speedup_vs_1 / static_cast<double>(w);
            }
            all_results.push_back(res);
        }
    };

    // Benchmark workloads across 1, 2, 4, 8 workers
    run_suite_for_workload("Workload A", [](size_t w, MesherType m) { return bench_initial_population(w, m); }, MesherType::Naive);
    run_suite_for_workload("Workload A", [](size_t w, MesherType m) { return bench_initial_population(w, m); }, MesherType::Greedy);

    run_suite_for_workload("Workload B", [](size_t w, MesherType m) { return bench_single_chunk_move(w, m); }, MesherType::Naive);
    run_suite_for_workload("Workload B", [](size_t w, MesherType m) { return bench_single_chunk_move(w, m); }, MesherType::Greedy);

    run_suite_for_workload("Workload C", [](size_t w, MesherType m) { return bench_repeated_crossings(w, m); }, MesherType::Naive);
    run_suite_for_workload("Workload C", [](size_t w, MesherType m) { return bench_repeated_crossings(w, m); }, MesherType::Greedy);

    run_suite_for_workload("Workload D", [](size_t w, MesherType m) { return bench_load_unload_sequence(w, m); }, MesherType::Naive);
    run_suite_for_workload("Workload D", [](size_t w, MesherType m) { return bench_load_unload_sequence(w, m); }, MesherType::Greedy);

    run_suite_for_workload("Workload E", [](size_t w, MesherType m) { return bench_fixed_workload(w, m); }, MesherType::Naive);
    run_suite_for_workload("Workload E", [](size_t w, MesherType m) { return bench_fixed_workload(w, m); }, MesherType::Greedy);

    std::stringstream report;
    report << "========================================================================================================================\n";
    report << " PROJECT 02 — VOXEL ENGINE: MILESTONE 8 MULTITHREADED STREAMING & SCALING BENCHMARK REPORT\n";
    report << "========================================================================================================================\n\n";

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
    report << "  Timing Source:       std::chrono::high_resolution_clock\n";
    report << "  Tested Workers:      1, 2, 4, 8 threads\n\n";

    report << "Scaling Benchmark Results Summary:\n";
    report << "------------------------------------------------------------------------------------------------------------------------------------------------------\n";
    report << std::left << std::setw(50) << "Workload Description"
           << std::right
           << std::setw(9) << "Workers"
           << std::setw(14) << "Wall (ms)"
           << std::setw(14) << "Gen (ms)"
           << std::setw(14) << "Mesh (ms)"
           << std::setw(10) << "Built"
           << std::setw(10) << "Stale"
           << std::setw(10) << "Resident"
           << std::setw(12) << "Speedup"
           << std::setw(12) << "Efficiency"
           << "\n";
    report << "------------------------------------------------------------------------------------------------------------------------------------------------------\n";

    for (const auto& r : all_results) {
        report << std::left << std::setw(50) << r.workload
               << std::right
               << std::setw(9) << r.workers
               << std::fixed << std::setprecision(2)
               << std::setw(14) << r.total_wall_time_ms
               << std::setw(14) << r.total_generation_time_ms
               << std::setw(14) << r.total_mesh_time_ms
               << std::setw(10) << r.chunks_meshed
               << std::setw(10) << r.stale_jobs
               << std::setw(10) << r.resident_chunks
               << std::setw(11) << r.speedup_vs_1 << "x"
               << std::setw(11) << (r.parallel_efficiency * 100.0) << "%"
               << "\n";
    }
    report << "========================================================================================================================\n";

    std::cout << report.str();

    std::string out_path = (argc > 1) ? argv[1] : "projects/02-voxel-engine/benchmarks/milestone8_benchmark.txt";
    std::ofstream out(out_path);
    if (out.is_open()) {
        out << report.str();
        std::cout << "\n[INFO] Benchmark results written to: " << out_path << "\n";
    }

    return 0;
}
