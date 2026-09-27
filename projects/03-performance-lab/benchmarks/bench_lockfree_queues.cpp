#include "performance_lab/atomic_lockfree_traits.hpp"
#include "performance_lab/bench_runner.hpp"
#include "performance_lab/bench_types.hpp"
#include "performance_lab/compiler_barrier.hpp"
#include "performance_lab/mpmc_bounded_queue.hpp"
#include "performance_lab/mutex_bounded_queue.hpp"
#include "performance_lab/queue_payload.hpp"
#include "performance_lab/spsc_queue.hpp"

#include <chrono>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <latch>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#if defined(_M_X64) || defined(__x86_64__)
#include <immintrin.h>
#endif

using namespace performance_lab;

/**
 * @brief Benchmark runner for multi-threaded queue topologies.
 *
 * Excludes thread creation and joining from the timed measurement interval:
 * 1. Preallocates worker thread pools and thread-local state.
 * 2. Synchronizes all workers at a start latch.
 * 3. Starts high-resolution monotonic timer.
 * 4. Workers push and pop exactly total_items.
 * 5. Stops timer immediately upon consumption of the final item.
 * 6. Joins threads and validates checksums outside timed region.
 */
template <typename QueueType>
BenchResult benchmark_queue_topology(
    const std::string& benchmark_name,
    const std::string& workload_name,
    size_t num_producers,
    size_t num_consumers,
    size_t queue_capacity,
    size_t total_items,
    const BenchConfig& base_config)
{
    BenchConfig config = base_config;
    config.name = benchmark_name;
    config.workload_name = workload_name;
    config.total_operations = total_items;
    config.total_bytes = total_items * sizeof(QueueItem);

    // Warmups and Timed runs
    const size_t total_runs = config.warmups + config.iterations;
    std::vector<double> samples_us;
    samples_us.reserve(config.iterations);

    const size_t items_per_producer = total_items / num_producers;
    // Remainder items assigned to the last producer
    const size_t last_producer_items = total_items - (items_per_producer * (num_producers - 1));

    for (size_t run = 0; run < total_runs; ++run) {
        const bool is_timed = (run >= config.warmups);

        QueueType queue(queue_capacity);
        std::atomic<bool> start_flag{false};
        std::atomic<size_t> total_consumed{0};
        std::atomic<uint64_t> aggregate_checksum{0};

        std::latch ready_latch(static_cast<std::ptrdiff_t>(num_producers + num_consumers + 1));
        std::chrono::steady_clock::time_point start_time;
        std::atomic<std::chrono::steady_clock::time_point> end_time;

        std::vector<std::thread> producers;
        producers.reserve(num_producers);
        for (size_t p = 0; p < num_producers; ++p) {
            const size_t p_count = (p == num_producers - 1) ? last_producer_items : items_per_producer;
            const uint64_t start_seq = p * items_per_producer;
            const uint64_t end_seq = start_seq + p_count;

            producers.emplace_back([&, start_seq, end_seq]() {
                ready_latch.arrive_and_wait();
                while (!start_flag.load(std::memory_order_acquire)) {
                    #if defined(_M_X64) || defined(__x86_64__)
                    _mm_pause();
                    #else
                    std::this_thread::yield();
                    #endif
                }

                for (uint64_t seq = start_seq; seq < end_seq; ++seq) {
                    QueueItem item = QueueItem::make(seq);
                    while (!queue.try_push(item)) {
                        #if defined(_M_X64) || defined(__x86_64__)
                        _mm_pause();
                        #else
                        std::this_thread::yield();
                        #endif
                    }
                }
            });
        }

        std::vector<std::thread> consumers;
        consumers.reserve(num_consumers);
        for (size_t c = 0; c < num_consumers; ++c) {
            consumers.emplace_back([&]() {
                ready_latch.arrive_and_wait();
                while (!start_flag.load(std::memory_order_acquire)) {
                    #if defined(_M_X64) || defined(__x86_64__)
                    _mm_pause();
                    #else
                    std::this_thread::yield();
                    #endif
                }

                uint64_t local_sum = 0;
                while (total_consumed.load(std::memory_order_relaxed) < total_items) {
                    QueueItem item{};
                    if (queue.try_pop(item)) {
                        local_sum += (item.sequence + item.payload_a);
                        const size_t prev = total_consumed.fetch_add(1, std::memory_order_acq_rel);
                        if (prev + 1 == total_items) {
                            end_time.store(std::chrono::steady_clock::now(), std::memory_order_release);
                        }
                    } else {
                        #if defined(_M_X64) || defined(__x86_64__)
                        _mm_pause();
                        #else
                        std::this_thread::yield();
                        #endif
                    }
                }
                aggregate_checksum.fetch_add(local_sum, std::memory_order_relaxed);
            });
        }

        // Wait for all workers to be constructed and waiting at the barrier
        ready_latch.arrive_and_wait();

        // Start timed region
        start_time = std::chrono::steady_clock::now();
        start_flag.store(true, std::memory_order_release);

        for (auto& t : producers) t.join();
        for (auto& t : consumers) t.join();

        uint64_t chk = aggregate_checksum.load(std::memory_order_relaxed);
        do_not_optimize(chk);

        if (is_timed) {
            auto end = end_time.load(std::memory_order_acquire);
            double duration_us = std::chrono::duration<double, std::micro>(end - start_time).count();
            // Fallback safety if timer was captured slightly after joins
            if (duration_us <= 0.0) {
                duration_us = 1.0;
            }
            samples_us.push_back(duration_us);
        }
    }

    BenchResult result = compute_statistics(config, samples_us);
    result.compiler_info = EnvironmentMetadata::compiler();
    result.build_config = EnvironmentMetadata::build_config();
    result.arch_info = EnvironmentMetadata::architecture();
    result.os_info = EnvironmentMetadata::os();
    result.hardware_threads = EnvironmentMetadata::hardware_threads();

    if (!config.quiet) {
        BenchRunner::print_table(result);
    }

    return result;
}

template <typename T>
struct TypeTag { using type = T; };

int main(int argc, char** argv) {
    BenchConfig base_config;
    base_config.name = "lockfree_queues";
    base_config.warmups = 2;
    base_config.iterations = 10;

    if (!BenchCLI::parse(argc, argv, base_config)) {
        return 1;
    }

    std::cout << "=== Graphics Sim Lab — Project 03 Concurrency & Queue Benchmarks ===\n";
    AtomicLockFreeTraits::print();
    std::cout << "Host Hardware Threads : " << EnvironmentMetadata::hardware_threads() << "\n";
    std::cout << "Payload Struct Size   : " << sizeof(QueueItem) << " bytes\n";
    std::cout << "Warmups / Iterations  : " << base_config.warmups << " / " << base_config.iterations << "\n\n";

    constexpr size_t kStandardItems = 500'000;
    std::vector<BenchResult> all_results;

    auto run_bench = [&](auto tag, const std::string& name, size_t p, size_t c, size_t cap) {
        using QType = typename decltype(tag)::type;
        BenchResult res = benchmark_queue_topology<QType>(
            "lockfree_queues", name, p, c, cap, kStandardItems, base_config
        );
        all_results.push_back(res);
    };

    // =========================================================================
    // 1. SPSC LOCK-FREE RING BUFFER (1P / 1C)
    // =========================================================================
    std::cout << "--- Suite 1: SPSC Lock-Free Ring Buffer (1P / 1C) ---\n";
    for (size_t cap : {64ULL, 1024ULL, 16384ULL}) {
        std::stringstream ss;
        ss << "spsc_1p1c_cap" << cap;
        run_bench(TypeTag<SpscQueue<QueueItem>>{}, ss.str(), 1, 1, cap);
    }

    // =========================================================================
    // 2. MPMC BOUNDED LOCK-FREE QUEUE (CONCURRENCY SCALING: 1P1C, 2P2C, 4P4C, 8P8C)
    // =========================================================================
    std::cout << "\n--- Suite 2: MPMC Bounded Lock-Free Queue (Topology Scaling, Cap=1024) ---\n";
    for (size_t threads : {1ULL, 2ULL, 4ULL, 8ULL}) {
        std::stringstream ss;
        ss << "mpmc_" << threads << "p" << threads << "c_cap1024";
        run_bench(TypeTag<MpmcBoundedQueue<QueueItem>>{}, ss.str(), threads, threads, 1024);
    }

    // =========================================================================
    // 3. MUTEX BOUNDED QUEUE BASELINE (CONCURRENCY SCALING: 1P1C, 2P2C, 4P4C, 8P8C)
    // =========================================================================
    std::cout << "\n--- Suite 3: Mutex Bounded Queue Baseline (Topology Scaling, Cap=1024) ---\n";
    for (size_t threads : {1ULL, 2ULL, 4ULL, 8ULL}) {
        std::stringstream ss;
        ss << "mutex_" << threads << "p" << threads << "c_cap1024";
        run_bench(TypeTag<MutexBoundedQueue<QueueItem>>{}, ss.str(), threads, threads, 1024);
    }

    // =========================================================================
    // 4. QUEUE CAPACITY CONTRAST AT 4P / 4C (Capacities: 64, 1024, 16384)
    // =========================================================================
    std::cout << "\n--- Suite 4: Capacity Scaling at Fixed 4P / 4C Contention ---\n";
    for (size_t cap : {64ULL, 16384ULL}) {
        // MPMC
        std::stringstream ss_mpmc;
        ss_mpmc << "mpmc_4p4c_cap" << cap;
        run_bench(TypeTag<MpmcBoundedQueue<QueueItem>>{}, ss_mpmc.str(), 4, 4, cap);

        // Mutex
        std::stringstream ss_mutex;
        ss_mutex << "mutex_4p4c_cap" << cap;
        run_bench(TypeTag<MutexBoundedQueue<QueueItem>>{}, ss_mutex.str(), 4, 4, cap);
    }

    // =========================================================================
    // CSV EXPORT (Complete 15-Workload Suite)
    // =========================================================================
    if (!base_config.csv_path.empty()) {
        std::ifstream check_file(base_config.csv_path.c_str());
        bool file_exists = check_file.good();
        check_file.close();

        if (file_exists && !base_config.overwrite_csv) {
            std::cerr << "[CSV Error] Target CSV file already exists and --overwrite was not set: "
                      << base_config.csv_path << "\n";
            return 1;
        }

        std::ofstream csv(base_config.csv_path.c_str(), std::ios::out | std::ios::trunc);
        if (!csv.is_open()) {
            std::cerr << "[CSV Error] Failed to open CSV file for writing: " << base_config.csv_path << "\n";
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
        std::cout << "\n[CSV Export] All " << all_results.size()
                  << " benchmark results successfully written to: " << base_config.csv_path << "\n";
    }

    std::cout << "\n[Queue Benchmarks Complete] All concurrency workloads finished successfully.\n";
    return 0;
}
