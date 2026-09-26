#ifndef PERFORMANCE_LAB_BENCH_RUNNER_HPP
#define PERFORMANCE_LAB_BENCH_RUNNER_HPP

#include "performance_lab/bench_types.hpp"
#include "performance_lab/compiler_barrier.hpp"
#include "performance_lab/statistics.hpp"

#include <chrono>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

namespace performance_lab {

/**
 * @brief Utility for gathering lightweight environment metadata.
 */
struct EnvironmentMetadata {
    static std::string compiler();
    static std::string build_config();
    static std::string architecture();
    static std::string os();
    static uint32_t hardware_threads();
};

/**
 * @brief CLI parser helper for benchmark executables.
 */
class BenchCLI {
public:
    static bool parse(int argc, char** argv, BenchConfig& config);
    static void print_help(const char* prog_name);
};

/**
 * @brief Core benchmarking harness managing execution protocols, timing, statistics, and reporting.
 */
class BenchRunner {
public:
    /**
     * @brief Executes a workload function according to BenchConfig protocol.
     */
    template <typename WorkloadFn>
    static BenchResult run(BenchConfig config, WorkloadFn&& workload) {
        auto noop = []() {};
        return run_with_setup(config, noop, std::forward<WorkloadFn>(workload), noop);
    }

    /**
     * @brief Executes a benchmark with setup and teardown hooks outside the timed loop.
     */
    template <typename SetupFn, typename WorkloadFn, typename TeardownFn>
    static BenchResult run_with_setup(BenchConfig config, SetupFn&& setup, WorkloadFn&& workload, TeardownFn&& teardown) {
        setup();

        // 1. Warmup Iterations (Un-timed)
        for (size_t i = 0; i < config.warmups; ++i) {
            workload();
            clobber_memory();
        }

        // 2. Timed Iterations
        std::vector<double> samples_us;
        samples_us.reserve(config.iterations);

        for (size_t i = 0; i < config.iterations; ++i) {
            auto start = std::chrono::steady_clock::now();
            workload();
            auto end = std::chrono::steady_clock::now();

            clobber_memory();

            double duration_us = std::chrono::duration<double, std::micro>(end - start).count();
            samples_us.push_back(duration_us);
        }

        teardown();

        // 3. Compute Statistics
        BenchResult result = compute_statistics(config, samples_us);

        // 4. Populate Metadata
        result.compiler_info = EnvironmentMetadata::compiler();
        result.build_config = EnvironmentMetadata::build_config();
        result.arch_info = EnvironmentMetadata::architecture();
        result.os_info = EnvironmentMetadata::os();
        result.hardware_threads = EnvironmentMetadata::hardware_threads();

        // 5. Terminal Reporting
        if (!config.quiet) {
            print_table(result);
        }

        // 6. CSV Export (if path provided)
        if (!config.csv_path.empty()) {
            export_csv(result, config.csv_path, config.overwrite_csv);
        }

        return result;
    }

    /**
     * @brief Formats and prints a human-readable plain-text summary table.
     */
    static void print_table(const BenchResult& result);

    /**
     * @brief Exports benchmark results to a structured CSV file.
     */
    static bool export_csv(const BenchResult& result, const std::string& path, bool overwrite);
};

} // namespace performance_lab

#endif // PERFORMANCE_LAB_BENCH_RUNNER_HPP
