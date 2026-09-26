#include "performance_lab/bench_runner.hpp"

#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

namespace performance_lab {

std::string EnvironmentMetadata::compiler() {
#if defined(_MSC_VER)
    std::stringstream ss;
    ss << "MSVC " << _MSC_VER;
    return ss.str();
#elif defined(__clang__)
    std::stringstream ss;
    ss << "Clang " << __clang_major__ << "." << __clang_minor__ << "." << __clang_patchlevel__;
    return ss.str();
#elif defined(__GNUC__)
    std::stringstream ss;
    ss << "GCC " << __GNUC__ << "." << __GNUC_MINOR__ << "." << __GNUC_PATCHLEVEL__;
    return ss.str();
#else
    return "Unknown Compiler";
#endif
}

std::string EnvironmentMetadata::build_config() {
#if defined(NDEBUG)
    return "Release";
#else
    return "Debug";
#endif
}

std::string EnvironmentMetadata::architecture() {
#if defined(_M_X64) || defined(__x86_64__)
    return "x86_64";
#elif defined(_M_ARM64) || defined(__aarch64__)
    return "ARM64";
#elif defined(_M_IX86) || defined(__i386__)
    return "x86_32";
#else
    return "Unknown Arch";
#endif
}

std::string EnvironmentMetadata::os() {
#if defined(_WIN32)
    return "Windows";
#elif defined(__linux__)
    return "Linux";
#elif defined(__APPLE__)
    return "macOS";
#else
    return "Unknown OS";
#endif
}

uint32_t EnvironmentMetadata::hardware_threads() {
    uint32_t n = std::thread::hardware_concurrency();
    return (n > 0) ? n : 1;
}

void BenchCLI::print_help(const char* prog_name) {
    std::cout << "Usage: " << prog_name << " [options]\n\n"
              << "Options:\n"
              << "  -h, --help           Display this help message\n"
              << "  -i, --iterations N   Set number of timed iterations (default: 100)\n"
              << "  -w, --warmups N      Set number of un-timed warmup iterations (default: 5)\n"
              << "  --csv PATH           Export benchmark results to specified CSV path\n"
              << "  --overwrite          Allow overwriting existing CSV report file\n"
              << "  --quiet              Suppress terminal table output\n";
}

bool BenchCLI::parse(int argc, char** argv, BenchConfig& config) {
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            print_help(argv[0]);
            return false;
        } else if ((arg == "-i" || arg == "--iterations") && i + 1 < argc) {
            config.iterations = static_cast<size_t>(std::atol(argv[++i]));
        } else if ((arg == "-w" || arg == "--warmups") && i + 1 < argc) {
            config.warmups = static_cast<size_t>(std::atol(argv[++i]));
        } else if (arg == "--csv" && i + 1 < argc) {
            config.csv_path = argv[++i];
        } else if (arg == "--overwrite") {
            config.overwrite_csv = true;
        } else if (arg == "--quiet") {
            config.quiet = true;
        } else {
            std::cerr << "[CLI Warning] Unknown or malformed argument: " << arg << "\n";
            print_help(argv[0]);
            return false;
        }
    }
    return true;
}

void BenchRunner::print_table(const BenchResult& result) {
    std::cout << "\n=================================================================================================\n";
    std::cout << " BENCHMARK REPORT: " << result.config.name << " [" << result.config.workload_name << "]\n";
    std::cout << "=================================================================================================\n";
    std::cout << " Environment  : " << result.compiler_info << " | " << result.build_config
              << " | " << result.arch_info << " | " << result.os_info
              << " | " << result.hardware_threads << " threads\n";
    std::cout << " Protocol     : " << result.config.warmups << " warmups, "
              << result.config.iterations << " timed iterations\n";
    
    std::cout << std::fixed << std::setprecision(3);
    std::cout << " Timing (us)  : Mean: " << result.mean_us << " us"
              << " | Median: " << result.median_us << " us"
              << " | StdDev: " << result.stddev_us << " us\n";
    std::cout << "              : Min : " << result.min_us << " us"
              << " | Max   : " << result.max_us << " us\n";

    if (result.config.total_operations > 0 || result.config.total_bytes > 0) {
        std::cout << " Throughput   : ";
        if (result.config.total_operations > 0) {
            std::cout << "Ops/sec: " << std::fixed << std::setprecision(0) << result.ops_per_sec << " ";
        }
        if (result.config.total_bytes > 0) {
            std::cout << "Bandwidth: " << std::fixed << std::setprecision(2) << result.mb_per_sec << " MB/s";
        }
        std::cout << "\n";
    }

    if (result.warning_too_short) {
        std::cout << " [DIAGNOSTIC WARNING] Mean execution time (" << result.mean_us
                  << " us) is close to timer resolution overhead.\n";
    }

    std::cout << "=================================================================================================\n\n";
}

bool BenchRunner::export_csv(const BenchResult& result, const std::string& path, bool overwrite) {
    // Check if file exists
    std::ifstream check_file(path.c_str());
    bool file_exists = check_file.good();
    check_file.close();

    if (file_exists && !overwrite) {
        std::cerr << "[CSV Error] Target CSV file already exists and --overwrite was not set: " << path << "\n";
        return false;
    }

    std::ofstream file;
    if (overwrite) {
        file.open(path.c_str(), std::ios::out | std::ios::trunc);
    } else {
        file.open(path.c_str(), std::ios::out | std::ios::app);
    }

    if (!file.is_open()) {
        std::cerr << "[CSV Error] Failed to open CSV output path: " << path << "\n";
        return false;
    }

    // Write header if creating new file or truncating
    if (!file_exists || overwrite) {
        file << "benchmark_name,workload,build_config,compiler,architecture,os,warmups,iterations,"
             << "mean_us,median_us,stddev_us,min_us,max_us,ops_per_sec,mb_per_sec\n";
    }

    file << "\"" << result.config.name << "\","
         << "\"" << result.config.workload_name << "\","
         << "\"" << result.build_config << "\","
         << "\"" << result.compiler_info << "\","
         << "\"" << result.arch_info << "\","
         << "\"" << result.os_info << "\","
         << result.config.warmups << ","
         << result.config.iterations << ","
         << std::fixed << std::setprecision(4)
         << result.mean_us << ","
         << result.median_us << ","
         << result.stddev_us << ","
         << result.min_us << ","
         << result.max_us << ",";

    if (result.config.total_operations > 0) {
        file << std::fixed << std::setprecision(2) << result.ops_per_sec;
    }
    file << ",";

    if (result.config.total_bytes > 0) {
        file << std::fixed << std::setprecision(2) << result.mb_per_sec;
    }
    file << "\n";

    file.close();
    std::cout << "[CSV Export] Report successfully written to: " << path << "\n";
    return true;
}

} // namespace performance_lab
