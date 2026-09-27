#include "performance_lab/bench_runner.hpp"
#include "performance_lab/compiler_barrier.hpp"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

using namespace performance_lab;

int main(int argc, char** argv) {
    std::cout << "=== Graphics Sim Lab — Project 03 Performance Lab Harness Benchmark ===\n";

    BenchConfig base_config;
    base_config.name = "harness_validation";
    base_config.warmups = 5;
    base_config.iterations = 100;

    if (!BenchCLI::parse(argc, argv, base_config)) {
        return 0;
    }

    std::string csv_path = base_config.csv_path;
    bool overwrite_csv = base_config.overwrite_csv;
    base_config.csv_path.clear();

    std::vector<BenchResult> all_results;
    all_results.reserve(4);

    // 1. Workload A: Integer Accumulation Loop
    {
        BenchConfig config = base_config;
        config.workload_name = "integer_accumulation";
        config.total_operations = 1'000'000;

        uint64_t sum = 0;
        auto res = BenchRunner::run_with_setup(
            config,
            [&]() { sum = 0; },
            [&]() {
                uint64_t local_sum = 0;
                for (uint64_t i = 0; i < 1'000'000; ++i) {
                    local_sum += i;
                }
                do_not_optimize(local_sum);
                sum += local_sum;
            },
            [&]() { do_not_optimize(sum); }
        );
        all_results.push_back(res);
    }

    // 2. Workload B: Floating-Point Accumulation Loop
    {
        BenchConfig config = base_config;
        config.workload_name = "float_accumulation";
        config.total_operations = 1'000'000;

        double sum = 0.0;
        auto res = BenchRunner::run_with_setup(
            config,
            [&]() { sum = 0.0; },
            [&]() {
                double local_sum = 0.0;
                for (size_t i = 0; i < 1'000'000; ++i) {
                    local_sum += static_cast<double>(i) * 0.001;
                }
                do_not_optimize(local_sum);
                sum += local_sum;
            },
            [&]() { do_not_optimize(sum); }
        );
        all_results.push_back(res);
    }

    // 3. Workload C: Contiguous Array Traversal
    {
        const size_t element_count = 500'000; // 500k 64-bit ints = 4 MB data
        std::vector<uint64_t> data(element_count);
        std::iota(data.begin(), data.end(), 1ULL);

        BenchConfig config = base_config;
        config.workload_name = "array_traversal";
        config.total_operations = element_count;
        config.total_bytes = element_count * sizeof(uint64_t);

        uint64_t checksum = 0;
        auto res = BenchRunner::run_with_setup(
            config,
            [&]() { checksum = 0; },
            [&]() {
                uint64_t local_sum = 0;
                for (size_t i = 0; i < data.size(); ++i) {
                    local_sum += data[i];
                }
                do_not_optimize(local_sum);
                checksum += local_sum;
            },
            [&]() { do_not_optimize(checksum); }
        );
        all_results.push_back(res);
    }

    // 4. Workload D: Deterministic Scalar Transform over Array
    {
        const size_t element_count = 250'000;
        std::vector<float> data(element_count, 1.5f);

        BenchConfig config = base_config;
        config.workload_name = "scalar_transform";
        config.total_operations = element_count;
        config.total_bytes = element_count * sizeof(float);

        auto res = BenchRunner::run(
            config,
            [&]() {
                for (size_t i = 0; i < data.size(); ++i) {
                    data[i] = data[i] * 1.0001f + 0.5f;
                }
                do_not_optimize(data.data());
            }
        );
        all_results.push_back(res);
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

    std::cout << "[Harness Validation Complete] All baseline workloads executed cleanly.\n";
    return 0;
}
