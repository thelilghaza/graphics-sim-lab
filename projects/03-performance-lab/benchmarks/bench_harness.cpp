#include "performance_lab/bench_runner.hpp"
#include "performance_lab/compiler_barrier.hpp"

#include <iostream>
#include <vector>
#include <numeric>

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

    // 1. Workload A: Integer Accumulation Loop
    {
        BenchConfig config = base_config;
        config.workload_name = "integer_accumulation";
        config.total_operations = 1'000'000;

        uint64_t sum = 0;
        BenchRunner::run_with_setup(
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
    }

    // 2. Workload B: Floating-Point Accumulation Loop
    {
        BenchConfig config = base_config;
        config.workload_name = "float_accumulation";
        config.total_operations = 1'000'000;

        double sum = 0.0;
        BenchRunner::run_with_setup(
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
        BenchRunner::run_with_setup(
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
    }

    // 4. Workload D: Deterministic Scalar Transform over Array
    {
        const size_t element_count = 250'000;
        std::vector<float> data(element_count, 1.5f);

        BenchConfig config = base_config;
        config.workload_name = "scalar_transform";
        config.total_operations = element_count;
        config.total_bytes = element_count * sizeof(float);

        BenchRunner::run(
            config,
            [&]() {
                for (size_t i = 0; i < data.size(); ++i) {
                    data[i] = data[i] * 1.0001f + 0.5f;
                }
                do_not_optimize(data.data());
            }
        );
    }

    std::cout << "[Harness Validation Complete] All baseline workloads executed cleanly.\n";
    return 0;
}
