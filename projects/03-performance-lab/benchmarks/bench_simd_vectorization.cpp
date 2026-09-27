#include "performance_lab/bench_runner.hpp"
#include "performance_lab/compiler_barrier.hpp"
#include "performance_lab/simd_caps.hpp"
#include "performance_lab/simd_kernel_dot.hpp"
#include "performance_lab/simd_kernel_axpy.hpp"
#include "performance_lab/simd_kernel_ray_box.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace performance_lab;

int main(int argc, char** argv) {
    std::cout << "=== Graphics Sim Lab — Project 03 SIMD Vectorization & Intrinsic Benchmarks ===\n";
    std::cout << "Detected Active ISA: " << SimdCapabilities::active_isa_string() << "\n\n";

    BenchConfig base_config;
    base_config.name = "simd_vectorization";
    base_config.warmups = 5;
    base_config.iterations = 100;

    if (!BenchCLI::parse(argc, argv, base_config)) {
        return 0;
    }

    std::string csv_path = base_config.csv_path;
    bool overwrite_csv = base_config.overwrite_csv;
    base_config.csv_path.clear();

    std::vector<BenchResult> all_results;

    const std::vector<size_t> dataset_sizes = {16'384, 1'048'576, 16'777'216}; // 16K, 1M, 16M floats

    // =========================================================================
    // KERNEL A: Vector Fused Dot Arithmetic (sum += a[i] * b[i] + c[i])
    // =========================================================================
    std::cout << "--- Kernel A: Vector Fused Arithmetic (sum += a[i] * b[i] + c[i]) ---\n";
    for (size_t N : dataset_sizes) {
        std::vector<float> a(N, 1.001f);
        std::vector<float> b(N, 2.002f);
        std::vector<float> c(N, 0.500f);

        size_t bytes = N * 3 * sizeof(float);

        // Validation Checksum
        float res_scalar = kernel_dot_scalar_reference(a.data(), b.data(), c.data(), N);
        float res_opt    = kernel_dot_compiler_opt(a.data(), b.data(), c.data(), N);
        float res_sse    = kernel_dot_sse(a.data(), b.data(), c.data(), N);
        float res_avx2   = kernel_dot_avx2(a.data(), b.data(), c.data(), N);

        std::cout << "[Validation N=" << N << "] Scalar: " << res_scalar
                  << " | Opt: " << res_opt
                  << " | SSE: " << res_sse
                  << " | AVX2: " << res_avx2 << "\n";

        float tol = (N >= 10'000'000) ? 0.05f : 1e-3f;
        if (std::abs(res_scalar - res_opt) / res_scalar > tol ||
            std::abs(res_scalar - res_sse) / res_scalar > tol ||
            std::abs(res_scalar - res_avx2) / res_scalar > tol) {
            std::cerr << "[Validation Error] SIMD Dot checksum mismatch on N=" << N << "\n";
            return 1;
        }

        std::stringstream size_label;
        if (N >= 1'048'576) size_label << (N / 1'048'576) << "M";
        else size_label << (N / 1'024) << "K";

        // A1. Scalar Ref
        {
            BenchConfig config = base_config;
            config.workload_name = "dot_scalar_ref_" + size_label.str();
            config.total_operations = N * 2; // 1 mul, 1 add
            config.total_bytes = bytes;

            float final_res = 0.0f;
            auto res = BenchRunner::run(config, [&]() {
                float sum = kernel_dot_scalar_reference(a.data(), b.data(), c.data(), N);
                do_not_optimize(sum);
                final_res = sum;
            });
            do_not_optimize(final_res);
            all_results.push_back(res);
        }

        // A2. Compiler Opt
        {
            BenchConfig config = base_config;
            config.workload_name = "dot_compiler_opt_" + size_label.str();
            config.total_operations = N * 2;
            config.total_bytes = bytes;

            float final_res = 0.0f;
            auto res = BenchRunner::run(config, [&]() {
                float sum = kernel_dot_compiler_opt(a.data(), b.data(), c.data(), N);
                do_not_optimize(sum);
                final_res = sum;
            });
            do_not_optimize(final_res);
            all_results.push_back(res);
        }

        // A3. SSE Intrinsic
        {
            BenchConfig config = base_config;
            config.workload_name = "dot_sse_" + size_label.str();
            config.total_operations = N * 2;
            config.total_bytes = bytes;

            float final_res = 0.0f;
            auto res = BenchRunner::run(config, [&]() {
                float sum = kernel_dot_sse(a.data(), b.data(), c.data(), N);
                do_not_optimize(sum);
                final_res = sum;
            });
            do_not_optimize(final_res);
            all_results.push_back(res);
        }

        // A4. AVX2 Intrinsic
        if (SimdCapabilities::has_avx2()) {
            BenchConfig config = base_config;
            config.workload_name = "dot_avx2_" + size_label.str();
            config.total_operations = N * 2;
            config.total_bytes = bytes;

            float final_res = 0.0f;
            auto res = BenchRunner::run(config, [&]() {
                float sum = kernel_dot_avx2(a.data(), b.data(), c.data(), N);
                do_not_optimize(sum);
                final_res = sum;
            });
            do_not_optimize(final_res);
            all_results.push_back(res);
        }
    }

    // =========================================================================
    // KERNEL B: AXPY Transform (y[i] = a * x[i] + y[i])
    // =========================================================================
    std::cout << "\n--- Kernel B: AXPY Transform (y[i] = a * x[i] + y[i]) ---\n";
    for (size_t N : dataset_sizes) {
        float alpha = 2.5f;
        std::vector<float> x(N, 1.25f);
        std::vector<float> y_base(N, 0.75f);
        size_t bytes = N * 2 * sizeof(float); // 1 read x, 1 read/write y

        std::stringstream size_label;
        if (N >= 1'048'576) size_label << (N / 1'048'576) << "M";
        else size_label << (N / 1'024) << "K";

        // B1. Scalar Ref
        {
            BenchConfig config = base_config;
            config.workload_name = "axpy_scalar_ref_" + size_label.str();
            config.total_operations = N * 2;
            config.total_bytes = bytes;

            std::vector<float> y_work = y_base;
            auto res = BenchRunner::run_with_setup(
                config,
                [&]() { y_work = y_base; },
                [&]() {
                    kernel_axpy_scalar_reference(alpha, x.data(), y_work.data(), N);
                    do_not_optimize(y_work.data());
                },
                [&]() {}
            );
            all_results.push_back(res);
        }

        // B2. Compiler Opt
        {
            BenchConfig config = base_config;
            config.workload_name = "axpy_compiler_opt_" + size_label.str();
            config.total_operations = N * 2;
            config.total_bytes = bytes;

            std::vector<float> y_work = y_base;
            auto res = BenchRunner::run_with_setup(
                config,
                [&]() { y_work = y_base; },
                [&]() {
                    kernel_axpy_compiler_opt(alpha, x.data(), y_work.data(), N);
                    do_not_optimize(y_work.data());
                },
                [&]() {}
            );
            all_results.push_back(res);
        }

        // B3. SSE Intrinsic
        {
            BenchConfig config = base_config;
            config.workload_name = "axpy_sse_" + size_label.str();
            config.total_operations = N * 2;
            config.total_bytes = bytes;

            std::vector<float> y_work = y_base;
            auto res = BenchRunner::run_with_setup(
                config,
                [&]() { y_work = y_base; },
                [&]() {
                    kernel_axpy_sse(alpha, x.data(), y_work.data(), N);
                    do_not_optimize(y_work.data());
                },
                [&]() {}
            );
            all_results.push_back(res);
        }

        // B4. AVX2 Intrinsic
        if (SimdCapabilities::has_avx2()) {
            BenchConfig config = base_config;
            config.workload_name = "axpy_avx2_" + size_label.str();
            config.total_operations = N * 2;
            config.total_bytes = bytes;

            std::vector<float> y_work = y_base;
            auto res = BenchRunner::run_with_setup(
                config,
                [&]() { y_work = y_base; },
                [&]() {
                    kernel_axpy_avx2(alpha, x.data(), y_work.data(), N);
                    do_not_optimize(y_work.data());
                },
                [&]() {}
            );
            all_results.push_back(res);
        }
    }

    // =========================================================================
    // KERNEL C: Batch Ray-AABB Intersection
    // =========================================================================
    std::cout << "\n--- Kernel C: Batch Ray-AABB Intersection (4-Ray / 8-Ray Packets) ---\n";
    {
        const size_t num_packets = 50'000; // 50k packets of 4 rays = 200,000 rays
        AABB3D box{-1.0f, -1.0f, -1.0f, 1.0f, 1.0f, 1.0f};

        std::vector<RayPacket4> packets4(num_packets);
        std::vector<RayPacket8> packets8(num_packets);

        for (size_t i = 0; i < num_packets; ++i) {
            for (int k = 0; k < 4; ++k) {
                packets4[i].orig_x[k] = -2.0f + static_cast<float>(k) * 0.1f;
                packets4[i].orig_y[k] = 0.0f;
                packets4[i].orig_z[k] = 0.0f;
                packets4[i].inv_dir_x[k] = 1.0f;
                packets4[i].inv_dir_y[k] = 1.0f;
                packets4[i].inv_dir_z[k] = 1.0f;
            }
            for (int k = 0; k < 8; ++k) {
                packets8[i].orig_x[k] = -2.0f + static_cast<float>(k) * 0.1f;
                packets8[i].orig_y[k] = 0.0f;
                packets8[i].orig_z[k] = 0.0f;
                packets8[i].inv_dir_x[k] = 1.0f;
                packets8[i].inv_dir_y[k] = 1.0f;
                packets8[i].inv_dir_z[k] = 1.0f;
            }
        }

        // Validation Checksum
        uint32_t mask_scalar = kernel_ray_box_4_scalar_reference(packets4[0], box);
        uint32_t mask_opt    = kernel_ray_box_4_compiler_opt(packets4[0], box);
        uint32_t mask_sse    = kernel_ray_box_4_sse(packets4[0], box);
        uint32_t mask_avx2   = kernel_ray_box_8_avx2(packets8[0], box) & 0x0F;

        std::cout << "[Validation Ray-Box] Scalar: 0x" << std::hex << mask_scalar
                  << " | Opt: 0x" << mask_opt
                  << " | SSE: 0x" << mask_sse
                  << " | AVX2 (lower 4): 0x" << mask_avx2 << std::dec << "\n";

        if (mask_scalar != mask_opt || mask_scalar != mask_sse || mask_scalar != mask_avx2) {
            std::cerr << "[Validation Error] Ray-AABB hit mask mismatch!\n";
            return 1;
        }

        // C1. Scalar Ref (4-wide)
        {
            BenchConfig config = base_config;
            config.workload_name = "ray_box_4_scalar_ref";
            config.total_operations = num_packets * 4; // 200,000 ray-box tests

            uint32_t total_hits = 0;
            auto res = BenchRunner::run(config, [&]() {
                uint32_t hits = 0;
                for (size_t i = 0; i < num_packets; ++i) {
                    hits += kernel_ray_box_4_scalar_reference(packets4[i], box);
                }
                do_not_optimize(hits);
                total_hits = hits;
            });
            do_not_optimize(total_hits);
            all_results.push_back(res);
        }

        // C2. Compiler Opt (4-wide)
        {
            BenchConfig config = base_config;
            config.workload_name = "ray_box_4_compiler_opt";
            config.total_operations = num_packets * 4;

            uint32_t total_hits = 0;
            auto res = BenchRunner::run(config, [&]() {
                uint32_t hits = 0;
                for (size_t i = 0; i < num_packets; ++i) {
                    hits += kernel_ray_box_4_compiler_opt(packets4[i], box);
                }
                do_not_optimize(hits);
                total_hits = hits;
            });
            do_not_optimize(total_hits);
            all_results.push_back(res);
        }

        // C3. SSE 4-Wide
        {
            BenchConfig config = base_config;
            config.workload_name = "ray_box_4_sse";
            config.total_operations = num_packets * 4;

            uint32_t total_hits = 0;
            auto res = BenchRunner::run(config, [&]() {
                uint32_t hits = 0;
                for (size_t i = 0; i < num_packets; ++i) {
                    hits += kernel_ray_box_4_sse(packets4[i], box);
                }
                do_not_optimize(hits);
                total_hits = hits;
            });
            do_not_optimize(total_hits);
            all_results.push_back(res);
        }

        // C4. AVX2 8-Wide
        if (SimdCapabilities::has_avx2()) {
            BenchConfig config = base_config;
            config.workload_name = "ray_box_8_avx2";
            config.total_operations = num_packets * 8; // 400,000 ray-box tests

            uint32_t total_hits = 0;
            auto res = BenchRunner::run(config, [&]() {
                uint32_t hits = 0;
                for (size_t i = 0; i < num_packets; ++i) {
                    hits += kernel_ray_box_8_avx2(packets8[i], box);
                }
                do_not_optimize(hits);
                total_hits = hits;
            });
            do_not_optimize(total_hits);
            all_results.push_back(res);
        }
    }

    // CSV EXPORT
    if (!csv_path.empty()) {
        std::ifstream check_file(csv_path.c_str());
        bool file_exists = check_file.good();
        check_file.close();

        if (file_exists && !overwrite_csv) {
            std::cerr << "[CSV Error] Target CSV file already exists and --overwrite was not set: " << csv_path << "\n";
            return 1;
        }

        std::ofstream csv(csv_path.c_str(), std::ios::out | std::ios::trunc);
        if (!csv.is_open()) {
            std::cerr << "[CSV Error] Failed to open CSV file for writing: " << csv_path << "\n";
            return 1;
        }

        csv << "benchmark_name,workload,build_config,compiler,architecture,os,warmups,iterations,mean_us,median_us,stddev_us,min_us,max_us,ops_per_sec,mb_per_sec\n";
        for (const auto& r : all_results) {
            csv << "\"" << r.config.name << "\",\""
                << r.config.workload_name << "\",\""
                << r.build_config << "\",\""
                << r.compiler_info << "\",\""
                << r.arch_info << "\",\""
                << r.os_info << "\","
                << r.config.warmups << ","
                << r.config.iterations << ","
                << std::fixed << std::setprecision(4)
                << r.mean_us << ","
                << r.median_us << ","
                << r.stddev_us << ","
                << r.min_us << ","
                << r.max_us << ","
                << std::fixed << std::setprecision(2)
                << r.ops_per_sec << ","
                << (r.config.total_bytes > 0 ? std::to_string(r.mb_per_sec) : "")
                << "\n";
        }
        std::cout << "\n[CSV Export] All " << all_results.size() << " benchmark results successfully written to: " << csv_path << "\n";
    }

    std::cout << "\n[SIMD Benchmarks Complete] All SIMD workloads finished successfully.\n";
    return 0;
}
