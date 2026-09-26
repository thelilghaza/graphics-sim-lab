#include "performance_lab/bench_runner.hpp"
#include "performance_lab/compiler_barrier.hpp"
#include "performance_lab/data_layouts.hpp"
#include "performance_lab/stride_benchmark.hpp"
#include "performance_lab/workingset_benchmark.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

using namespace performance_lab;

int main(int argc, char** argv) {
    std::cout << "=== Graphics Sim Lab — Project 03 Cache Locality & Data Layout Benchmarks ===\n\n";

    BenchConfig base_config;
    base_config.name = "cache_locality";
    base_config.warmups = 5;
    base_config.iterations = 100;

    if (!BenchCLI::parse(argc, argv, base_config)) {
        return 0;
    }

    // =========================================================================
    // SECTION 1 — Sequential Traversal: AoS vs SoA vs AoSoA
    // =========================================================================
    std::cout << "--- 1. Sequential Traversal (AoS vs SoA vs AoSoA) ---\n";
    const size_t record_count = 100'000;
    const size_t logical_bytes = record_count * sizeof(RecordAoS); // 3.2 MB

    auto data_aos = generate_aos(record_count);
    auto data_soa = generate_soa(record_count);
    auto data_aosoa = generate_aosoa<DEFAULT_TILE_WIDTH>(record_count);

    // Validation Checksum
    double cs_aos = compute_checksum_aos(data_aos);
    double cs_soa = compute_checksum_soa(data_soa);
    double cs_aosoa = compute_checksum_aosoa(data_aosoa);

    std::cout << "[Validation Checksum] AoS: " << cs_aos
              << " | SoA: " << cs_soa
              << " | AoSoA: " << cs_aosoa << "\n";

    if (std::abs(cs_aos - cs_soa) > 1e-2 || std::abs(cs_aos - cs_aosoa) > 1e-2) {
        std::cerr << "[Validation Error] Layout checksums do not match!\n";
        return 1;
    }
    std::cout << "[Validation] All layouts verified equivalent.\n\n";

    // 1A. AoS Sequential
    {
        BenchConfig config = base_config;
        config.workload_name = "aos_sequential";
        config.total_operations = record_count;
        config.total_bytes = logical_bytes;

        double final_sum = 0.0;
        BenchRunner::run(config, [&]() {
            double sum = compute_checksum_aos(data_aos);
            do_not_optimize(sum);
            final_sum = sum;
        });
        do_not_optimize(final_sum);
    }

    // 1B. SoA Sequential
    {
        BenchConfig config = base_config;
        config.workload_name = "soa_sequential";
        config.total_operations = record_count;
        config.total_bytes = logical_bytes;

        double final_sum = 0.0;
        BenchRunner::run(config, [&]() {
            double sum = compute_checksum_soa(data_soa);
            do_not_optimize(sum);
            final_sum = sum;
        });
        do_not_optimize(final_sum);
    }

    // 1C. AoSoA Sequential (Tile Width = 16)
    {
        BenchConfig config = base_config;
        config.workload_name = "aosoa_sequential_tile16";
        config.total_operations = record_count;
        config.total_bytes = logical_bytes;

        double final_sum = 0.0;
        BenchRunner::run(config, [&]() {
            double sum = compute_checksum_aosoa(data_aosoa);
            do_not_optimize(sum);
            final_sum = sum;
        });
        do_not_optimize(final_sum);
    }

    // =========================================================================
    // SECTION 2 — Strided Access Benchmark (Constant Access Count)
    // =========================================================================
    std::cout << "\n--- 2. Strided Access Benchmark (Constant 100,000 Elements Accessed) ---\n";
    const size_t stride_access_count = 100'000;
    const std::vector<size_t> strides = {1, 2, 4, 8, 16, 32, 64, 128, 256};

    for (size_t stride : strides) {
        auto buffer = generate_stride_buffer(stride_access_count, stride);
        size_t stride_bytes = stride * sizeof(float);

        BenchConfig config = base_config;
        std::stringstream ss;
        ss << "stride_" << stride << "_elem_" << stride_bytes << "B";
        config.workload_name = ss.str();
        config.total_operations = stride_access_count;
        config.total_bytes = stride_access_count * sizeof(float); // Logical accessed bytes

        float final_sum = 0.0f;
        BenchRunner::run(config, [&]() {
            float sum = run_stride_access(buffer, stride_access_count, stride);
            do_not_optimize(sum);
            final_sum = sum;
        });
        do_not_optimize(final_sum);
    }

    // =========================================================================
    // SECTION 3 — Working-Set Scaling Benchmark
    // =========================================================================
    std::cout << "\n--- 3. Working-Set Scaling Benchmark ---\n";
    const std::vector<size_t> working_sets_bytes = {
        4 * 1024,      // 4 KiB
        8 * 1024,      // 8 KiB
        16 * 1024,     // 16 KiB
        32 * 1024,     // 32 KiB
        64 * 1024,     // 64 KiB
        128 * 1024,    // 128 KiB
        256 * 1024,    // 256 KiB
        512 * 1024,    // 512 KiB
        1 * 1024 * 1024,  // 1 MiB
        2 * 1024 * 1024,  // 2 MiB
        4 * 1024 * 1024,  // 4 MiB
        8 * 1024 * 1024,  // 8 MiB
        16 * 1024 * 1024, // 16 MiB
        32 * 1024 * 1024, // 32 MiB
        64 * 1024 * 1024  // 64 MiB
    };

    for (size_t ws_bytes : working_sets_bytes) {
        auto buffer = generate_workingset_buffer(ws_bytes);
        size_t element_count = buffer.size();

        BenchConfig config = base_config;
        std::stringstream ss;
        if (ws_bytes < 1024 * 1024) {
            ss << "workingset_" << (ws_bytes / 1024) << "KiB";
        } else {
            ss << "workingset_" << (ws_bytes / (1024 * 1024)) << "MiB";
        }
        config.workload_name = ss.str();
        config.total_operations = element_count;
        config.total_bytes = ws_bytes;

        float final_sum = 0.0f;
        BenchRunner::run(config, [&]() {
            float sum = run_workingset_pass(buffer);
            do_not_optimize(sum);
            final_sum = sum;
        });
        do_not_optimize(final_sum);
    }

    std::cout << "\n[Cache Locality Benchmarks Complete] All benchmark workloads finished successfully.\n";
    return 0;
}
