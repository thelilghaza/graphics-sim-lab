#include "performance_lab/bench_runner.hpp"
#include "performance_lab/compiler_barrier.hpp"
#include "performance_lab/statistics.hpp"

#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>

using namespace performance_lab;

void test_config_defaults() {
    std::cout << "[Test 1] BenchConfig Defaults...\n";
    BenchConfig config;
    assert(config.name == "unnamed_benchmark");
    assert(config.workload_name == "default_workload");
    assert(config.warmups == 5);
    assert(config.iterations == 100);
    assert(config.total_operations == 0);
    assert(config.total_bytes == 0);
    assert(config.csv_path.empty());
    assert(config.overwrite_csv == false);
    assert(config.quiet == false);
}

void test_user_overrides() {
    std::cout << "[Test 2] User Overrides...\n";
    BenchConfig config;
    config.name = "custom_test";
    config.warmups = 10;
    config.iterations = 250;
    config.total_operations = 1000;
    config.total_bytes = 2000;
    config.csv_path = "test.csv";
    config.overwrite_csv = true;
    config.quiet = true;

    assert(config.name == "custom_test");
    assert(config.warmups == 10);
    assert(config.iterations == 250);
    assert(config.total_operations == 1000);
    assert(config.total_bytes == 2000);
    assert(config.csv_path == "test.csv");
    assert(config.overwrite_csv == true);
    assert(config.quiet == true);
}

void test_warmup_and_iteration_counts() {
    std::cout << "[Test 3] Warmup and Iteration Counts...\n";
    size_t workload_counter = 0;

    BenchConfig config;
    config.warmups = 7;
    config.iterations = 15;
    config.quiet = true;

    BenchRunner::run_with_setup(
        config,
        [&]() {},
        [&]() { ++workload_counter; },
        [&]() {}
    );

    // Warmups (7) + Timed Iterations (15) = 22 total workload executions
    assert(workload_counter == 22);
}

void test_statistics_calculations() {
    std::cout << "[Test 4] Statistics Calculations (Mean, Median, StdDev, Min, Max)...\n";
    BenchConfig config;
    config.total_operations = 1'000'000;
    config.total_bytes = 10'000'000; // 10 MB

    // Sample timings in microseconds: {10, 20, 30, 40, 50}
    std::vector<double> samples = {10.0, 20.0, 30.0, 40.0, 50.0};
    BenchResult result = compute_statistics(config, samples);

    assert(result.min_us == 10.0);
    assert(result.max_us == 50.0);
    assert(std::abs(result.mean_us - 30.0) < 1e-6);
    assert(std::abs(result.median_us - 30.0) < 1e-6);

    // Sample stddev: sqrt( ((10-30)^2 + (20-30)^2 + (30-30)^2 + (40-30)^2 + (50-30)^2) / 4 )
    // = sqrt( (400 + 100 + 0 + 100 + 400) / 4 ) = sqrt(1000 / 4) = sqrt(250) approx 15.811388
    double expected_stddev = std::sqrt(250.0);
    assert(std::abs(result.stddev_us - expected_stddev) < 1e-4);
    (void)expected_stddev;

    // Throughput: ops_per_sec = 1,000,000 / (30 * 1e-6) = 33,333,333,333 ops/sec
    double expected_ops = 1'000'000.0 / 30e-6;
    assert(std::abs(result.ops_per_sec - expected_ops) / expected_ops < 1e-4);
    (void)expected_ops;

    // Bandwidth: mb_per_sec = (10 MB) / (30 * 1e-6 s) = 333,333.33 MB/s
    double expected_mb_sec = 10.0 / 30e-6;
    assert(std::abs(result.mb_per_sec - expected_mb_sec) / expected_mb_sec < 1e-4);
    (void)expected_mb_sec;
}

void test_median_even_samples() {
    std::cout << "[Test 5] Median Calculation with Even Samples...\n";
    BenchConfig config;
    std::vector<double> samples = {10.0, 20.0, 30.0, 40.0};
    BenchResult result = compute_statistics(config, samples);

    // Even samples: median = (20 + 30) / 2 = 25.0
    assert(std::abs(result.median_us - 25.0) < 1e-6);
}

void test_single_iteration_statistics() {
    std::cout << "[Test 6] Single Iteration Statistics...\n";
    BenchConfig config;
    std::vector<double> samples = {42.0};
    BenchResult result = compute_statistics(config, samples);

    assert(result.min_us == 42.0);
    assert(result.max_us == 42.0);
    assert(result.mean_us == 42.0);
    assert(result.median_us == 42.0);
    assert(result.stddev_us == 0.0); // Single sample stddev defined as 0
}

void test_empty_samples_handling() {
    std::cout << "[Test 7] Empty Samples Handling...\n";
    BenchConfig config;
    std::vector<double> samples;
    BenchResult result = compute_statistics(config, samples);

    assert(result.mean_us == 0.0);
    assert(result.median_us == 0.0);
    assert(result.stddev_us == 0.0);
}

void test_csv_export_and_schema() {
    std::cout << "[Test 8] CSV Export and Schema Verification...\n";
    const std::string temp_csv = "test_temp_report.csv";

    BenchConfig config;
    config.name = "unit_test_bench";
    config.workload_name = "unit_test_workload";
    config.warmups = 2;
    config.iterations = 5;
    config.total_operations = 500;
    config.total_bytes = 1000;
    config.quiet = true;

    BenchResult result = compute_statistics(config, {10.0, 12.0, 11.0, 13.0, 14.0});
    result.compiler_info = "TestCompiler";
    result.build_config = "TestConfig";
    result.arch_info = "TestArch";
    result.os_info = "TestOS";

    // Export CSV
    bool ok = BenchRunner::export_csv(result, temp_csv, true);
    assert(ok);
    (void)ok;

    // Read back and check header and data line
    std::ifstream file(temp_csv.c_str());
    assert(file.is_open());

    std::string header, data_line;
    std::getline(file, header);
    std::getline(file, data_line);
    file.close();

    assert(header.find("benchmark_name,workload,build_config,compiler") != std::string::npos);
    assert(data_line.find("\"unit_test_bench\"") != std::string::npos);
    assert(data_line.find("\"unit_test_workload\"") != std::string::npos);

    // Clean up temporary test file
    std::remove(temp_csv.c_str());
}

void test_compiler_barrier() {
    std::cout << "[Test 9] Compiler Barrier Invocations...\n";
    int val = 12345;
    do_not_optimize(val);
    clobber_memory();
    assert(val == 12345);
}

int main() {
    std::cout << "=== Running test_benchmark_harness ===\n";

    test_config_defaults();
    test_user_overrides();
    test_warmup_and_iteration_counts();
    test_statistics_calculations();
    test_median_even_samples();
    test_single_iteration_statistics();
    test_empty_samples_handling();
    test_csv_export_and_schema();
    test_compiler_barrier();

    std::cout << "=== All 9 benchmark harness unit tests PASSED cleanly ===\n";
    return 0;
}
