#include "voxel_lab/chunk_manager.hpp"

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <psapi.h>
#endif

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace voxel_lab;

struct MemoryBenchmarkResult {
    std::string workload;
    std::string mesher;
    bool buffer_reuse{false};
    size_t resident_chunks{0};
    size_t raw_chunk_bytes{0};
    size_t mesh_logical_bytes{0};
    size_t mesh_capacity_bytes{0};
    size_t recycled_buffer_count{0};
    size_t recycled_capacity_bytes{0};
    size_t fresh_buffers_allocated{0};
    size_t buffers_reused{0};
    size_t working_set_bytes{0};
    size_t private_bytes{0};
    double elapsed_ms{0.0};
};

// Workload A: Single Loaded Chunk
MemoryBenchmarkResult run_workload_a(MesherType mesher, bool reuse, int iters = 20) {
    StreamingConfig cfg;
    cfg.load_radius = 0; // Exactly 1 chunk at (0,0,0)
    cfg.unload_radius = 0;
    cfg.worker_count = 0; // Synchronous for precise single-chunk profiling
    cfg.enable_mesh_buffer_reuse = reuse;

    ChunkManager mgr(cfg, mesher);

    // Warmup
    mgr.load_chunk(ChunkCoord(0, 0, 0));
    mgr.unload_chunk(ChunkCoord(0, 0, 0));
    mgr.reset_cumulative_metrics();

    auto t0 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < iters; ++i) {
        mgr.load_chunk(ChunkCoord(0, 0, 0));
        if (i + 1 < iters) {
            mgr.unload_chunk(ChunkCoord(0, 0, 0));
        }
    }
    auto t1 = std::chrono::high_resolution_clock::now();
    double elapsed = std::chrono::duration<double, std::milli>(t1 - t0).count() / iters;

    ChunkManagerMemoryStats stats = mgr.get_memory_stats();
    const auto& m = mgr.get_metrics();

    return MemoryBenchmarkResult{
        "Workload A: 1 Chunk",
        (mesher == MesherType::Naive ? "Naive" : "Greedy"),
        reuse,
        stats.resident_chunk_count,
        stats.raw_chunk_payload_bytes,
        stats.total_mesh_logical_bytes,
        stats.total_mesh_capacity_bytes,
        stats.recycled_mesh_buffer_count,
        stats.recycled_mesh_capacity_bytes,
        m.mesh_buffers_allocated_fresh,
        m.mesh_buffers_reused,
        stats.process_working_set_bytes,
        stats.process_private_bytes,
        elapsed
    };
}

// Workload B: 27 Loaded Chunks (Radius 1)
MemoryBenchmarkResult run_workload_b(MesherType mesher, bool reuse, int iters = 10) {
    StreamingConfig cfg;
    cfg.load_radius = 1; // 3x3x3 = 27 chunks
    cfg.unload_radius = 2;
    cfg.worker_count = 2;
    cfg.enable_mesh_buffer_reuse = reuse;

    double total_ms = 0.0;
    MemoryBenchmarkResult last_res;

    for (int i = 0; i < iters; ++i) {
        ChunkManager mgr(cfg, mesher);
        auto t0 = std::chrono::high_resolution_clock::now();
        mgr.update_streaming(Vec3(0.0f, 0.0f, 0.0f));
        mgr.wait_all_pending();
        auto t1 = std::chrono::high_resolution_clock::now();
        total_ms += std::chrono::duration<double, std::milli>(t1 - t0).count();

        if (i == iters - 1) {
            ChunkManagerMemoryStats stats = mgr.get_memory_stats();
            const auto& m = mgr.get_metrics();
            last_res = MemoryBenchmarkResult{
                "Workload B: 27 Chunks (r=1)",
                (mesher == MesherType::Naive ? "Naive" : "Greedy"),
                reuse,
                stats.resident_chunk_count,
                stats.raw_chunk_payload_bytes,
                stats.total_mesh_logical_bytes,
                stats.total_mesh_capacity_bytes,
                stats.recycled_mesh_buffer_count,
                stats.recycled_mesh_capacity_bytes,
                m.mesh_buffers_allocated_fresh,
                m.mesh_buffers_reused,
                stats.process_working_set_bytes,
                stats.process_private_bytes,
                total_ms / iters
            };
        }
    }
    return last_res;
}

// Workload C: 125 Loaded Chunks (Radius 2)
MemoryBenchmarkResult run_workload_c(MesherType mesher, bool reuse, int iters = 5) {
    StreamingConfig cfg;
    cfg.load_radius = 2; // 5x5x5 = 125 chunks
    cfg.unload_radius = 3;
    cfg.worker_count = 4;
    cfg.enable_mesh_buffer_reuse = reuse;

    double total_ms = 0.0;
    MemoryBenchmarkResult last_res;

    for (int i = 0; i < iters; ++i) {
        ChunkManager mgr(cfg, mesher);
        auto t0 = std::chrono::high_resolution_clock::now();
        mgr.update_streaming(Vec3(0.0f, 0.0f, 0.0f));
        mgr.wait_all_pending();
        auto t1 = std::chrono::high_resolution_clock::now();
        total_ms += std::chrono::duration<double, std::milli>(t1 - t0).count();

        if (i == iters - 1) {
            ChunkManagerMemoryStats stats = mgr.get_memory_stats();
            const auto& m = mgr.get_metrics();
            last_res = MemoryBenchmarkResult{
                "Workload C: 125 Chunks (r=2)",
                (mesher == MesherType::Naive ? "Naive" : "Greedy"),
                reuse,
                stats.resident_chunk_count,
                stats.raw_chunk_payload_bytes,
                stats.total_mesh_logical_bytes,
                stats.total_mesh_capacity_bytes,
                stats.recycled_mesh_buffer_count,
                stats.recycled_mesh_capacity_bytes,
                m.mesh_buffers_allocated_fresh,
                m.mesh_buffers_reused,
                stats.process_working_set_bytes,
                stats.process_private_bytes,
                total_ms / iters
            };
        }
    }
    return last_res;
}

// Workload D: Streaming Scene Around Radius 2 (150 Resident Chunks)
MemoryBenchmarkResult run_workload_d(MesherType mesher, bool reuse, int iters = 5) {
    StreamingConfig cfg;
    cfg.load_radius = 2;
    cfg.unload_radius = 3;
    cfg.worker_count = 4;
    cfg.enable_mesh_buffer_reuse = reuse;

    double total_ms = 0.0;
    MemoryBenchmarkResult last_res;

    for (int i = 0; i < iters; ++i) {
        ChunkManager mgr(cfg, mesher);
        mgr.update_streaming(Vec3(0.0f, 0.0f, 0.0f));
        mgr.wait_all_pending();

        auto t0 = std::chrono::high_resolution_clock::now();
        mgr.update_streaming(Vec3(32.0f, 0.0f, 0.0f)); // Move into hysteresis band -> 150 resident
        mgr.wait_all_pending();
        auto t1 = std::chrono::high_resolution_clock::now();
        total_ms += std::chrono::duration<double, std::milli>(t1 - t0).count();

        if (i == iters - 1) {
            ChunkManagerMemoryStats stats = mgr.get_memory_stats();
            const auto& m = mgr.get_metrics();
            last_res = MemoryBenchmarkResult{
                "Workload D: 150 Chunks (Hysteresis)",
                (mesher == MesherType::Naive ? "Naive" : "Greedy"),
                reuse,
                stats.resident_chunk_count,
                stats.raw_chunk_payload_bytes,
                stats.total_mesh_logical_bytes,
                stats.total_mesh_capacity_bytes,
                stats.recycled_mesh_buffer_count,
                stats.recycled_mesh_capacity_bytes,
                m.mesh_buffers_allocated_fresh,
                m.mesh_buffers_reused,
                stats.process_working_set_bytes,
                stats.process_private_bytes,
                total_ms / iters
            };
        }
    }
    return last_res;
}

// Workload E: Repeated Load/Unload Cycles (10 Boundary Crossings)
MemoryBenchmarkResult run_workload_e(MesherType mesher, bool reuse, int steps = 10, int iters = 3) {
    StreamingConfig cfg;
    cfg.load_radius = 2;
    cfg.unload_radius = 3;
    cfg.worker_count = 4;
    cfg.enable_mesh_buffer_reuse = reuse;

    double total_ms = 0.0;
    MemoryBenchmarkResult last_res;

    for (int it = 0; it < iters; ++it) {
        ChunkManager mgr(cfg, mesher);
        mgr.update_streaming(Vec3(0.0f, 0.0f, 0.0f));
        mgr.wait_all_pending();

        auto t0 = std::chrono::high_resolution_clock::now();
        for (int s = 1; s <= steps; ++s) {
            float x = static_cast<float>(s * 32);
            mgr.update_streaming(Vec3(x, 0.0f, 0.0f));
            mgr.wait_all_pending();
        }
        auto t1 = std::chrono::high_resolution_clock::now();
        total_ms += std::chrono::duration<double, std::milli>(t1 - t0).count();

        if (it == iters - 1) {
            ChunkManagerMemoryStats stats = mgr.get_memory_stats();
            const auto& m = mgr.get_metrics();
            last_res = MemoryBenchmarkResult{
                "Workload E: 10 Crossings (Churn)",
                (mesher == MesherType::Naive ? "Naive" : "Greedy"),
                reuse,
                stats.resident_chunk_count,
                stats.raw_chunk_payload_bytes,
                stats.total_mesh_logical_bytes,
                stats.total_mesh_capacity_bytes,
                stats.recycled_mesh_buffer_count,
                stats.recycled_mesh_capacity_bytes,
                m.mesh_buffers_allocated_fresh,
                m.mesh_buffers_reused,
                stats.process_working_set_bytes,
                stats.process_private_bytes,
                total_ms / iters
            };
        }
    }
    return last_res;
}

// Workload F: Repeated Remeshing of the Same Chunk (50 remesh cycles)
MemoryBenchmarkResult run_workload_f(MesherType mesher, bool reuse, int cycles = 50) {
    StreamingConfig cfg;
    cfg.load_radius = 0;
    cfg.unload_radius = 0;
    cfg.worker_count = 0;
    cfg.enable_mesh_buffer_reuse = reuse;

    ChunkManager mgr(cfg, mesher);
    mgr.load_chunk(ChunkCoord(0, 0, 0));
    mgr.reset_cumulative_metrics();

    auto t0 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < cycles; ++i) {
        // Toggle mesher or remesh to simulate dynamic editing / dirty remeshes
        mgr.set_mesher_type(mesher == MesherType::Naive ? MesherType::Greedy : MesherType::Naive);
        mgr.set_mesher_type(mesher);
    }
    auto t1 = std::chrono::high_resolution_clock::now();
    double total_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    ChunkManagerMemoryStats stats = mgr.get_memory_stats();
    const auto& m = mgr.get_metrics();

    return MemoryBenchmarkResult{
        "Workload F: 50 Remesh Cycles",
        (mesher == MesherType::Naive ? "Naive" : "Greedy"),
        reuse,
        stats.resident_chunk_count,
        stats.raw_chunk_payload_bytes,
        stats.total_mesh_logical_bytes,
        stats.total_mesh_capacity_bytes,
        stats.recycled_mesh_buffer_count,
        stats.recycled_mesh_capacity_bytes,
        m.mesh_buffers_allocated_fresh,
        m.mesh_buffers_reused,
        stats.process_working_set_bytes,
        stats.process_private_bytes,
        total_ms
    };
}

int main() {
    std::cout << "=== Milestone 9: Memory Footprint & Buffer Reuse Benchmark ===\n";

    std::vector<MemoryBenchmarkResult> results;

    std::cout << "Running Workload A (1 Chunk)...\n";
    results.push_back(run_workload_a(MesherType::Naive, false));
    results.push_back(run_workload_a(MesherType::Naive, true));
    results.push_back(run_workload_a(MesherType::Greedy, false));
    results.push_back(run_workload_a(MesherType::Greedy, true));

    std::cout << "Running Workload B (27 Chunks)...\n";
    results.push_back(run_workload_b(MesherType::Naive, false));
    results.push_back(run_workload_b(MesherType::Naive, true));
    results.push_back(run_workload_b(MesherType::Greedy, false));
    results.push_back(run_workload_b(MesherType::Greedy, true));

    std::cout << "Running Workload C (125 Chunks)...\n";
    results.push_back(run_workload_c(MesherType::Naive, false));
    results.push_back(run_workload_c(MesherType::Naive, true));
    results.push_back(run_workload_c(MesherType::Greedy, false));
    results.push_back(run_workload_c(MesherType::Greedy, true));

    std::cout << "Running Workload D (150 Chunks Hysteresis)...\n";
    results.push_back(run_workload_d(MesherType::Naive, false));
    results.push_back(run_workload_d(MesherType::Naive, true));
    results.push_back(run_workload_d(MesherType::Greedy, false));
    results.push_back(run_workload_d(MesherType::Greedy, true));

    std::cout << "Running Workload E (10 Boundary Crossings Churn)...\n";
    results.push_back(run_workload_e(MesherType::Naive, false));
    results.push_back(run_workload_e(MesherType::Naive, true));
    results.push_back(run_workload_e(MesherType::Greedy, false));
    results.push_back(run_workload_e(MesherType::Greedy, true));

    std::cout << "Running Workload F (50 Remesh Cycles)...\n";
    results.push_back(run_workload_f(MesherType::Naive, false));
    results.push_back(run_workload_f(MesherType::Naive, true));
    results.push_back(run_workload_f(MesherType::Greedy, false));
    results.push_back(run_workload_f(MesherType::Greedy, true));

    std::stringstream report;
    report << "========================================================================================================================\n";
    report << " PROJECT 02 — VOXEL ENGINE: MILESTONE 9 MEMORY FOOTPRINT & BUFFER REUSE BENCHMARK REPORT\n";
    report << "========================================================================================================================\n\n";

    report << "Environment & Configuration:\n";
    report << "  Build Configuration: Release (/O2, NDEBUG)\n";
    report << "  Compiler:            MSVC " << _MSC_VER << "\n";
    report << "  Timing Source:       std::chrono::high_resolution_clock\n";
    report << "  Memory Measurement:  GetProcessMemoryInfo (WorkingSetSize / PrivateUsage)\n\n";

    report << "Structural Sizes (Exact Compiler sizeof):\n";
    report << "  sizeof(Voxel):                       " << sizeof(Voxel) << " bytes\n";
    report << "  sizeof(Chunk):                       " << sizeof(Chunk) << " bytes (32^3 * 2 bytes = 65,536 bytes)\n";
    report << "  sizeof(MeshVertex):                  " << sizeof(MeshVertex) << " bytes (3 floats pos + 3 floats normal)\n";
    report << "  sizeof(uint32_t index):              " << sizeof(uint32_t) << " bytes\n";
    report << "  Quad Memory Footprint:               120 bytes (4 vertices * 24B = 96B + 6 indices * 4B = 24B)\n";
    report << "  sizeof(ChunkBuildTask):              " << sizeof(ChunkBuildTask) << " bytes\n";
    report << "  sizeof(ChunkBuildResult):            " << sizeof(ChunkBuildResult) << " bytes\n";
    report << "  sizeof(ChunkNeighborhoodSnapshot):   " << sizeof(ChunkNeighborhoodSnapshot) << " bytes\n";
    report << "  sizeof(WorldGrid):                   " << sizeof(WorldGrid) << " bytes\n\n";

    report << "Memory Benchmark Results Summary (Baseline vs Optimized):\n";
    report << "----------------------------------------------------------------------------------------------------------------------------------------------------------------------\n";
    report << std::left << std::setw(34) << "Workload"
           << std::setw(8) << "Mesher"
           << std::setw(11) << "Reuse"
           << std::setw(6) << "Res"
           << std::right << std::setw(11) << "Raw Chk KB"
           << std::setw(13) << "Mesh Log KB"
           << std::setw(13) << "Mesh Cap KB"
           << std::setw(11) << "Recycle KB"
           << std::setw(11) << "Alloc Fresh"
           << std::setw(11) << "Reused"
           << std::setw(13) << "Proc WS MB"
           << std::setw(12) << "Time ms"
           << "\n";
    report << "----------------------------------------------------------------------------------------------------------------------------------------------------------------------\n";

    for (const auto& r : results) {
        double raw_kb = r.raw_chunk_bytes / 1024.0;
        double mesh_log_kb = r.mesh_logical_bytes / 1024.0;
        double mesh_cap_kb = r.mesh_capacity_bytes / 1024.0;
        double recycle_kb = r.recycled_capacity_bytes / 1024.0;
        double ws_mb = r.working_set_bytes / (1024.0 * 1024.0);

        report << std::left << std::setw(34) << r.workload
               << std::setw(8) << r.mesher
               << std::setw(11) << (r.buffer_reuse ? "ENABLED" : "DISABLED")
               << std::setw(6) << r.resident_chunks
               << std::right << std::fixed << std::setprecision(1)
               << std::setw(11) << raw_kb
               << std::setw(13) << mesh_log_kb
               << std::setw(13) << mesh_cap_kb
               << std::setw(11) << recycle_kb
               << std::setw(11) << r.fresh_buffers_allocated
               << std::setw(11) << r.buffers_reused
               << std::setw(13) << ws_mb
               << std::setw(12) << std::setprecision(2) << r.elapsed_ms
               << "\n";
    }
    report << "----------------------------------------------------------------------------------------------------------------------------------------------------------------------\n\n";

    std::cout << report.str();

    std::ofstream out_file("projects/02-voxel-engine/benchmarks/milestone9_benchmark.txt");
    if (out_file.is_open()) {
        out_file << report.str();
        out_file.close();
        std::cout << "[INFO] Milestone 9 benchmark results saved to projects/02-voxel-engine/benchmarks/milestone9_benchmark.txt\n";
    } else {
        std::cerr << "[ERROR] Could not open milestone9_benchmark.txt for writing.\n";
    }

    return 0;
}
