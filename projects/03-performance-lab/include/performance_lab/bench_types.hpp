#ifndef PERFORMANCE_LAB_BENCH_TYPES_HPP
#define PERFORMANCE_LAB_BENCH_TYPES_HPP

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace performance_lab {

/**
 * @brief Configuration parameters for a benchmark run.
 */
struct BenchConfig {
    std::string name{"unnamed_benchmark"};
    std::string workload_name{"default_workload"};
    size_t warmups{5};
    size_t iterations{100};
    uint64_t total_operations{0}; // Operations performed per iteration
    uint64_t total_bytes{0};      // Bytes processed per iteration
    std::string csv_path{""};
    bool overwrite_csv{false};
    bool quiet{false};
};

/**
 * @brief Calculated statistical results and metadata for a benchmark run.
 */
struct BenchResult {
    BenchConfig config;
    std::vector<double> samples_us; // Iteration durations in microseconds
    double mean_us{0.0};
    double median_us{0.0};
    double stddev_us{0.0};
    double min_us{0.0};
    double max_us{0.0};
    double ops_per_sec{0.0};
    double mb_per_sec{0.0}; // 1 MB = 10^6 bytes
    bool warning_too_short{false};
    
    // Environment metadata
    std::string compiler_info;
    std::string build_config;
    std::string arch_info;
    std::string os_info;
    uint32_t hardware_threads{0};
};

} // namespace performance_lab

#endif // PERFORMANCE_LAB_BENCH_TYPES_HPP
