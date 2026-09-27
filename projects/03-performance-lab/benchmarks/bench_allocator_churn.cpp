#include "performance_lab/allocator_payload.hpp"
#include "performance_lab/bench_runner.hpp"
#include "performance_lab/bench_types.hpp"
#include "performance_lab/compiler_barrier.hpp"
#include "performance_lab/fixed_block_pool.hpp"
#include "performance_lab/linear_arena.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

using namespace performance_lab;

namespace {

/**
 * @brief Helper to populate EnvironmentMetadata and compute BenchResult.
 */
BenchResult make_bench_result(const BenchConfig& config, const std::vector<double>& samples_us) {
    BenchResult result = compute_statistics(config, samples_us);
    result.compiler_info = EnvironmentMetadata::compiler();
    result.build_config = EnvironmentMetadata::build_config();
    result.arch_info = EnvironmentMetadata::architecture();
    result.os_info = EnvironmentMetadata::os();
    result.hardware_threads = EnvironmentMetadata::hardware_threads();
    return result;
}

// =============================================================================
// Benchmark A: Fixed-Size Allocation Churn (64 Bytes)
// =============================================================================

BenchResult bench_fixed_malloc(const BenchConfig& base_config, std::size_t num_allocs) {
    BenchConfig config = base_config;
    config.name = "allocator_churn";
    config.workload_name = "malloc_fixed64";
    config.total_operations = num_allocs;
    config.total_bytes = num_allocs * AllocatorWorkload::kFixedSize;

    std::vector<void*> ptrs(num_allocs, nullptr);
    std::vector<double> samples_us;
    samples_us.reserve(config.iterations);

    const std::size_t total_runs = config.warmups + config.iterations;
    for (std::size_t run = 0; run < total_runs; ++run) {
        std::uint64_t checksum = 0;

        auto start = std::chrono::steady_clock::now();
        for (std::size_t i = 0; i < num_allocs; ++i) {
            void* p = std::malloc(AllocatorWorkload::kFixedSize);
            AllocatorPayload::write_marker(p, AllocatorWorkload::kFixedSize, i);
            checksum += AllocatorPayload::read_marker(p, AllocatorWorkload::kFixedSize);
            ptrs[i] = p;
        }
        for (std::size_t i = 0; i < num_allocs; ++i) {
            std::free(ptrs[i]);
        }
        auto end = std::chrono::steady_clock::now();

        do_not_optimize(checksum);
        clobber_memory();

        if (run >= config.warmups) {
            double duration_us = std::chrono::duration<double, std::micro>(end - start).count();
            samples_us.push_back(duration_us);
        }
    }

    return make_bench_result(config, samples_us);
}

BenchResult bench_fixed_std_allocator(const BenchConfig& base_config, std::size_t num_allocs) {
    BenchConfig config = base_config;
    config.name = "allocator_churn";
    config.workload_name = "stdalloc_fixed64";
    config.total_operations = num_allocs;
    config.total_bytes = num_allocs * AllocatorWorkload::kFixedSize;

    std::allocator<std::byte> alloc;
    std::vector<std::byte*> ptrs(num_allocs, nullptr);
    std::vector<double> samples_us;
    samples_us.reserve(config.iterations);

    const std::size_t total_runs = config.warmups + config.iterations;
    for (std::size_t run = 0; run < total_runs; ++run) {
        std::uint64_t checksum = 0;

        auto start = std::chrono::steady_clock::now();
        for (std::size_t i = 0; i < num_allocs; ++i) {
            std::byte* p = alloc.allocate(AllocatorWorkload::kFixedSize);
            AllocatorPayload::write_marker(p, AllocatorWorkload::kFixedSize, i);
            checksum += AllocatorPayload::read_marker(p, AllocatorWorkload::kFixedSize);
            ptrs[i] = p;
        }
        for (std::size_t i = 0; i < num_allocs; ++i) {
            alloc.deallocate(ptrs[i], AllocatorWorkload::kFixedSize);
        }
        auto end = std::chrono::steady_clock::now();

        do_not_optimize(checksum);
        clobber_memory();

        if (run >= config.warmups) {
            double duration_us = std::chrono::duration<double, std::micro>(end - start).count();
            samples_us.push_back(duration_us);
        }
    }

    return make_bench_result(config, samples_us);
}

BenchResult bench_fixed_pool(const BenchConfig& base_config, std::size_t num_allocs) {
    BenchConfig config = base_config;
    config.name = "allocator_churn";
    config.workload_name = "pool_fixed64";
    config.total_operations = num_allocs;
    config.total_bytes = num_allocs * AllocatorWorkload::kFixedSize;

    FixedBlockPool pool(AllocatorWorkload::kFixedSize, num_allocs, 8);
    std::vector<void*> ptrs(num_allocs, nullptr);
    std::vector<double> samples_us;
    samples_us.reserve(config.iterations);

    const std::size_t total_runs = config.warmups + config.iterations;
    for (std::size_t run = 0; run < total_runs; ++run) {
        pool.reset();
        std::uint64_t checksum = 0;

        auto start = std::chrono::steady_clock::now();
        for (std::size_t i = 0; i < num_allocs; ++i) {
            void* p = pool.allocate();
            AllocatorPayload::write_marker(p, AllocatorWorkload::kFixedSize, i);
            checksum += AllocatorPayload::read_marker(p, AllocatorWorkload::kFixedSize);
            ptrs[i] = p;
        }
        for (std::size_t i = 0; i < num_allocs; ++i) {
            pool.deallocate(ptrs[i]);
        }
        auto end = std::chrono::steady_clock::now();

        do_not_optimize(checksum);
        clobber_memory();

        if (run >= config.warmups) {
            double duration_us = std::chrono::duration<double, std::micro>(end - start).count();
            samples_us.push_back(duration_us);
        }
    }

    return make_bench_result(config, samples_us);
}

BenchResult bench_fixed_arena(const BenchConfig& base_config, std::size_t num_allocs) {
    BenchConfig config = base_config;
    config.name = "allocator_churn";
    config.workload_name = "arena_batch_fixed64";
    config.total_operations = num_allocs;
    config.total_bytes = num_allocs * AllocatorWorkload::kFixedSize;

    // Preallocate arena sized for exactly num_allocs of 64 bytes with 8-byte alignment
    LinearArena arena(num_allocs * (AllocatorWorkload::kFixedSize + 8), 64);
    std::vector<double> samples_us;
    samples_us.reserve(config.iterations);

    const std::size_t total_runs = config.warmups + config.iterations;
    for (std::size_t run = 0; run < total_runs; ++run) {
        arena.reset();
        std::uint64_t checksum = 0;

        auto start = std::chrono::steady_clock::now();
        for (std::size_t i = 0; i < num_allocs; ++i) {
            void* p = arena.allocate(AllocatorWorkload::kFixedSize, 8);
            AllocatorPayload::write_marker(p, AllocatorWorkload::kFixedSize, i);
            checksum += AllocatorPayload::read_marker(p, AllocatorWorkload::kFixedSize);
        }
        arena.reset();
        auto end = std::chrono::steady_clock::now();

        do_not_optimize(checksum);
        clobber_memory();

        if (run >= config.warmups) {
            double duration_us = std::chrono::duration<double, std::micro>(end - start).count();
            samples_us.push_back(duration_us);
        }
    }

    return make_bench_result(config, samples_us);
}

// =============================================================================
// Benchmark B: Variable-Size Allocation Churn
// =============================================================================

BenchResult bench_variable_malloc(const BenchConfig& base_config, std::size_t num_allocs) {
    BenchConfig config = base_config;
    config.name = "allocator_churn";
    config.workload_name = "malloc_variable";
    config.total_operations = num_allocs;

    std::size_t total_bytes = 0;
    std::vector<std::size_t> sizes(num_allocs);
    for (std::size_t i = 0; i < num_allocs; ++i) {
        sizes[i] = AllocatorWorkload::variable_size_at(i);
        total_bytes += sizes[i];
    }
    config.total_bytes = total_bytes;

    std::vector<void*> ptrs(num_allocs, nullptr);
    std::vector<double> samples_us;
    samples_us.reserve(config.iterations);

    const std::size_t total_runs = config.warmups + config.iterations;
    for (std::size_t run = 0; run < total_runs; ++run) {
        std::uint64_t checksum = 0;

        auto start = std::chrono::steady_clock::now();
        for (std::size_t i = 0; i < num_allocs; ++i) {
            void* p = std::malloc(sizes[i]);
            AllocatorPayload::write_marker(p, sizes[i], i);
            checksum += AllocatorPayload::read_marker(p, sizes[i]);
            ptrs[i] = p;
        }
        for (std::size_t i = 0; i < num_allocs; ++i) {
            std::free(ptrs[i]);
        }
        auto end = std::chrono::steady_clock::now();

        do_not_optimize(checksum);
        clobber_memory();

        if (run >= config.warmups) {
            double duration_us = std::chrono::duration<double, std::micro>(end - start).count();
            samples_us.push_back(duration_us);
        }
    }

    return make_bench_result(config, samples_us);
}

BenchResult bench_variable_std_allocator(const BenchConfig& base_config, std::size_t num_allocs) {
    BenchConfig config = base_config;
    config.name = "allocator_churn";
    config.workload_name = "stdalloc_variable";
    config.total_operations = num_allocs;

    std::size_t total_bytes = 0;
    std::vector<std::size_t> sizes(num_allocs);
    for (std::size_t i = 0; i < num_allocs; ++i) {
        sizes[i] = AllocatorWorkload::variable_size_at(i);
        total_bytes += sizes[i];
    }
    config.total_bytes = total_bytes;

    std::allocator<std::byte> alloc;
    std::vector<std::byte*> ptrs(num_allocs, nullptr);
    std::vector<double> samples_us;
    samples_us.reserve(config.iterations);

    const std::size_t total_runs = config.warmups + config.iterations;
    for (std::size_t run = 0; run < total_runs; ++run) {
        std::uint64_t checksum = 0;

        auto start = std::chrono::steady_clock::now();
        for (std::size_t i = 0; i < num_allocs; ++i) {
            std::byte* p = alloc.allocate(sizes[i]);
            AllocatorPayload::write_marker(p, sizes[i], i);
            checksum += AllocatorPayload::read_marker(p, sizes[i]);
            ptrs[i] = p;
        }
        for (std::size_t i = 0; i < num_allocs; ++i) {
            alloc.deallocate(ptrs[i], sizes[i]);
        }
        auto end = std::chrono::steady_clock::now();

        do_not_optimize(checksum);
        clobber_memory();

        if (run >= config.warmups) {
            double duration_us = std::chrono::duration<double, std::micro>(end - start).count();
            samples_us.push_back(duration_us);
        }
    }

    return make_bench_result(config, samples_us);
}

BenchResult bench_variable_arena(const BenchConfig& base_config, std::size_t num_allocs) {
    BenchConfig config = base_config;
    config.name = "allocator_churn";
    config.workload_name = "arena_variable";
    config.total_operations = num_allocs;

    std::size_t total_bytes = 0;
    std::vector<std::size_t> sizes(num_allocs);
    for (std::size_t i = 0; i < num_allocs; ++i) {
        sizes[i] = AllocatorWorkload::variable_size_at(i);
        total_bytes += sizes[i];
    }
    config.total_bytes = total_bytes;

    LinearArena arena(total_bytes + (num_allocs * 16), 64);
    std::vector<double> samples_us;
    samples_us.reserve(config.iterations);

    const std::size_t total_runs = config.warmups + config.iterations;
    for (std::size_t run = 0; run < total_runs; ++run) {
        arena.reset();
        std::uint64_t checksum = 0;

        auto start = std::chrono::steady_clock::now();
        for (std::size_t i = 0; i < num_allocs; ++i) {
            void* p = arena.allocate(sizes[i], 8);
            AllocatorPayload::write_marker(p, sizes[i], i);
            checksum += AllocatorPayload::read_marker(p, sizes[i]);
        }
        arena.reset();
        auto end = std::chrono::steady_clock::now();

        do_not_optimize(checksum);
        clobber_memory();

        if (run >= config.warmups) {
            double duration_us = std::chrono::duration<double, std::micro>(end - start).count();
            samples_us.push_back(duration_us);
        }
    }

    return make_bench_result(config, samples_us);
}

// =============================================================================
// Benchmark C: Frame / Batch Temporary Allocation
// =============================================================================

BenchResult bench_frames_malloc(const BenchConfig& base_config, std::size_t num_frames, std::size_t allocs_per_frame) {
    BenchConfig config = base_config;
    config.name = "allocator_churn";
    config.workload_name = "malloc_frames";
    config.total_operations = num_frames * allocs_per_frame;

    std::size_t bytes_per_frame = 0;
    std::vector<std::size_t> frame_sizes(allocs_per_frame);
    for (std::size_t i = 0; i < allocs_per_frame; ++i) {
        frame_sizes[i] = AllocatorWorkload::variable_size_at(i);
        bytes_per_frame += frame_sizes[i];
    }
    config.total_bytes = num_frames * bytes_per_frame;

    std::vector<void*> frame_ptrs(allocs_per_frame, nullptr);
    std::vector<double> samples_us;
    samples_us.reserve(config.iterations);

    const std::size_t total_runs = config.warmups + config.iterations;
    for (std::size_t run = 0; run < total_runs; ++run) {
        std::uint64_t checksum = 0;

        auto start = std::chrono::steady_clock::now();
        for (std::size_t f = 0; f < num_frames; ++f) {
            for (std::size_t i = 0; i < allocs_per_frame; ++i) {
                void* p = std::malloc(frame_sizes[i]);
                AllocatorPayload::write_marker(p, frame_sizes[i], i);
                checksum += AllocatorPayload::read_marker(p, frame_sizes[i]);
                frame_ptrs[i] = p;
            }
            for (std::size_t i = 0; i < allocs_per_frame; ++i) {
                std::free(frame_ptrs[i]);
            }
        }
        auto end = std::chrono::steady_clock::now();

        do_not_optimize(checksum);
        clobber_memory();

        if (run >= config.warmups) {
            double duration_us = std::chrono::duration<double, std::micro>(end - start).count();
            samples_us.push_back(duration_us);
        }
    }

    return make_bench_result(config, samples_us);
}

BenchResult bench_frames_std_allocator(const BenchConfig& base_config, std::size_t num_frames, std::size_t allocs_per_frame) {
    BenchConfig config = base_config;
    config.name = "allocator_churn";
    config.workload_name = "stdalloc_frames";
    config.total_operations = num_frames * allocs_per_frame;

    std::size_t bytes_per_frame = 0;
    std::vector<std::size_t> frame_sizes(allocs_per_frame);
    for (std::size_t i = 0; i < allocs_per_frame; ++i) {
        frame_sizes[i] = AllocatorWorkload::variable_size_at(i);
        bytes_per_frame += frame_sizes[i];
    }
    config.total_bytes = num_frames * bytes_per_frame;

    std::allocator<std::byte> alloc;
    std::vector<std::byte*> frame_ptrs(allocs_per_frame, nullptr);
    std::vector<double> samples_us;
    samples_us.reserve(config.iterations);

    const std::size_t total_runs = config.warmups + config.iterations;
    for (std::size_t run = 0; run < total_runs; ++run) {
        std::uint64_t checksum = 0;

        auto start = std::chrono::steady_clock::now();
        for (std::size_t f = 0; f < num_frames; ++f) {
            for (std::size_t i = 0; i < allocs_per_frame; ++i) {
                std::byte* p = alloc.allocate(frame_sizes[i]);
                AllocatorPayload::write_marker(p, frame_sizes[i], i);
                checksum += AllocatorPayload::read_marker(p, frame_sizes[i]);
                frame_ptrs[i] = p;
            }
            for (std::size_t i = 0; i < allocs_per_frame; ++i) {
                alloc.deallocate(frame_ptrs[i], frame_sizes[i]);
            }
        }
        auto end = std::chrono::steady_clock::now();

        do_not_optimize(checksum);
        clobber_memory();

        if (run >= config.warmups) {
            double duration_us = std::chrono::duration<double, std::micro>(end - start).count();
            samples_us.push_back(duration_us);
        }
    }

    return make_bench_result(config, samples_us);
}

BenchResult bench_frames_arena(const BenchConfig& base_config, std::size_t num_frames, std::size_t allocs_per_frame) {
    BenchConfig config = base_config;
    config.name = "allocator_churn";
    config.workload_name = "arena_frames";
    config.total_operations = num_frames * allocs_per_frame;

    std::size_t bytes_per_frame = 0;
    std::vector<std::size_t> frame_sizes(allocs_per_frame);
    for (std::size_t i = 0; i < allocs_per_frame; ++i) {
        frame_sizes[i] = AllocatorWorkload::variable_size_at(i);
        bytes_per_frame += frame_sizes[i];
    }
    config.total_bytes = num_frames * bytes_per_frame;

    LinearArena frame_arena(bytes_per_frame + (allocs_per_frame * 16), 64);
    std::vector<double> samples_us;
    samples_us.reserve(config.iterations);

    const std::size_t total_runs = config.warmups + config.iterations;
    for (std::size_t run = 0; run < total_runs; ++run) {
        std::uint64_t checksum = 0;

        auto start = std::chrono::steady_clock::now();
        for (std::size_t f = 0; f < num_frames; ++f) {
            frame_arena.reset();
            for (std::size_t i = 0; i < allocs_per_frame; ++i) {
                void* p = frame_arena.allocate(frame_sizes[i], 8);
                AllocatorPayload::write_marker(p, frame_sizes[i], i);
                checksum += AllocatorPayload::read_marker(p, frame_sizes[i]);
            }
        }
        auto end = std::chrono::steady_clock::now();

        do_not_optimize(checksum);
        clobber_memory();

        if (run >= config.warmups) {
            double duration_us = std::chrono::duration<double, std::micro>(end - start).count();
            samples_us.push_back(duration_us);
        }
    }

    return make_bench_result(config, samples_us);
}

// =============================================================================
// Benchmark D: Pool Reuse Cycles
// =============================================================================

BenchResult bench_reuse_malloc(const BenchConfig& base_config, std::size_t working_set, std::size_t cycles) {
    BenchConfig config = base_config;
    config.name = "allocator_churn";
    config.workload_name = "malloc_reuse";
    config.total_operations = working_set * cycles;
    config.total_bytes = config.total_operations * AllocatorWorkload::kFixedSize;

    std::vector<void*> ptrs(working_set, nullptr);
    std::vector<double> samples_us;
    samples_us.reserve(config.iterations);

    const std::size_t total_runs = config.warmups + config.iterations;
    for (std::size_t run = 0; run < total_runs; ++run) {
        std::uint64_t checksum = 0;

        auto start = std::chrono::steady_clock::now();
        for (std::size_t c = 0; c < cycles; ++c) {
            for (std::size_t i = 0; i < working_set; ++i) {
                void* p = std::malloc(AllocatorWorkload::kFixedSize);
                AllocatorPayload::write_marker(p, AllocatorWorkload::kFixedSize, i);
                checksum += AllocatorPayload::read_marker(p, AllocatorWorkload::kFixedSize);
                ptrs[i] = p;
            }
            for (std::size_t i = 0; i < working_set; ++i) {
                std::free(ptrs[i]);
            }
        }
        auto end = std::chrono::steady_clock::now();

        do_not_optimize(checksum);
        clobber_memory();

        if (run >= config.warmups) {
            double duration_us = std::chrono::duration<double, std::micro>(end - start).count();
            samples_us.push_back(duration_us);
        }
    }

    return make_bench_result(config, samples_us);
}

BenchResult bench_reuse_std_allocator(const BenchConfig& base_config, std::size_t working_set, std::size_t cycles) {
    BenchConfig config = base_config;
    config.name = "allocator_churn";
    config.workload_name = "stdalloc_reuse";
    config.total_operations = working_set * cycles;
    config.total_bytes = config.total_operations * AllocatorWorkload::kFixedSize;

    std::allocator<std::byte> alloc;
    std::vector<std::byte*> ptrs(working_set, nullptr);
    std::vector<double> samples_us;
    samples_us.reserve(config.iterations);

    const std::size_t total_runs = config.warmups + config.iterations;
    for (std::size_t run = 0; run < total_runs; ++run) {
        std::uint64_t checksum = 0;

        auto start = std::chrono::steady_clock::now();
        for (std::size_t c = 0; c < cycles; ++c) {
            for (std::size_t i = 0; i < working_set; ++i) {
                std::byte* p = alloc.allocate(AllocatorWorkload::kFixedSize);
                AllocatorPayload::write_marker(p, AllocatorWorkload::kFixedSize, i);
                checksum += AllocatorPayload::read_marker(p, AllocatorWorkload::kFixedSize);
                ptrs[i] = p;
            }
            for (std::size_t i = 0; i < working_set; ++i) {
                alloc.deallocate(ptrs[i], AllocatorWorkload::kFixedSize);
            }
        }
        auto end = std::chrono::steady_clock::now();

        do_not_optimize(checksum);
        clobber_memory();

        if (run >= config.warmups) {
            double duration_us = std::chrono::duration<double, std::micro>(end - start).count();
            samples_us.push_back(duration_us);
        }
    }

    return make_bench_result(config, samples_us);
}

BenchResult bench_reuse_pool(const BenchConfig& base_config, std::size_t working_set, std::size_t cycles) {
    BenchConfig config = base_config;
    config.name = "allocator_churn";
    config.workload_name = "pool_reuse";
    config.total_operations = working_set * cycles;
    config.total_bytes = config.total_operations * AllocatorWorkload::kFixedSize;

    FixedBlockPool pool(AllocatorWorkload::kFixedSize, working_set, 8);
    std::vector<void*> ptrs(working_set, nullptr);
    std::vector<double> samples_us;
    samples_us.reserve(config.iterations);

    const std::size_t total_runs = config.warmups + config.iterations;
    for (std::size_t run = 0; run < total_runs; ++run) {
        pool.reset();
        std::uint64_t checksum = 0;

        auto start = std::chrono::steady_clock::now();
        for (std::size_t c = 0; c < cycles; ++c) {
            for (std::size_t i = 0; i < working_set; ++i) {
                void* p = pool.allocate();
                AllocatorPayload::write_marker(p, AllocatorWorkload::kFixedSize, i);
                checksum += AllocatorPayload::read_marker(p, AllocatorWorkload::kFixedSize);
                ptrs[i] = p;
            }
            for (std::size_t i = 0; i < working_set; ++i) {
                pool.deallocate(ptrs[i]);
            }
        }
        auto end = std::chrono::steady_clock::now();

        do_not_optimize(checksum);
        clobber_memory();

        if (run >= config.warmups) {
            double duration_us = std::chrono::duration<double, std::micro>(end - start).count();
            samples_us.push_back(duration_us);
        }
    }

    return make_bench_result(config, samples_us);
}

// =============================================================================
// Benchmark E: Arena Capacity / Scaling
// =============================================================================

BenchResult bench_arena_scale(const BenchConfig& base_config, const std::string& name, std::size_t capacity_bytes) {
    BenchConfig config = base_config;
    config.name = "allocator_churn";
    config.workload_name = name;

    const std::size_t alloc_size = AllocatorWorkload::kFixedSize;
    const std::size_t num_allocs = capacity_bytes / (alloc_size + 8);
    config.total_operations = num_allocs;
    config.total_bytes = num_allocs * alloc_size;

    LinearArena arena(capacity_bytes, 64);
    std::vector<double> samples_us;
    samples_us.reserve(config.iterations);

    const std::size_t total_runs = config.warmups + config.iterations;
    for (std::size_t run = 0; run < total_runs; ++run) {
        arena.reset();
        std::uint64_t checksum = 0;

        auto start = std::chrono::steady_clock::now();
        for (std::size_t i = 0; i < num_allocs; ++i) {
            void* p = arena.allocate(alloc_size, 8);
            AllocatorPayload::write_marker(p, alloc_size, i);
            checksum += AllocatorPayload::read_marker(p, alloc_size);
        }
        arena.reset();
        auto end = std::chrono::steady_clock::now();

        do_not_optimize(checksum);
        clobber_memory();

        if (run >= config.warmups) {
            double duration_us = std::chrono::duration<double, std::micro>(end - start).count();
            samples_us.push_back(duration_us);
        }
    }

    return make_bench_result(config, samples_us);
}

} // namespace

int main(int argc, char** argv) {
    BenchConfig base_config;
    base_config.name = "allocator_churn";
    base_config.warmups = 2;
    base_config.iterations = 10;

    if (!BenchCLI::parse(argc, argv, base_config)) {
        return 1;
    }

    std::cout << "=================================================================================================\n";
    std::cout << " BENCHMARK SUITE: Memory Allocator Churn & Arena / Bump Allocator Benchmarks\n";
    std::cout << "=================================================================================================\n";
    std::cout << " Environment: " << EnvironmentMetadata::compiler() << " | "
              << EnvironmentMetadata::build_config() << " | "
              << EnvironmentMetadata::architecture() << " | "
              << EnvironmentMetadata::os() << " | "
              << EnvironmentMetadata::hardware_threads() << " threads\n";
    std::cout << " Protocol   : " << base_config.warmups << " warmups, "
              << base_config.iterations << " timed iterations\n";
    std::cout << "=================================================================================================\n\n";

    std::vector<BenchResult> all_results;
    all_results.reserve(16);

    auto record_bench = [&](BenchResult res) {
        if (!base_config.quiet) {
            BenchRunner::print_table(res);
        }
        all_results.push_back(std::move(res));
    };

    // -------------------------------------------------------------------------
    // Suite 1: Fixed-Size Allocation Churn (64 Bytes, 200,000 allocations)
    // -------------------------------------------------------------------------
    std::cout << "--- Suite 1: Fixed-Size Allocation Churn (64 Bytes, N=200,000) ---\n";
    record_bench(bench_fixed_malloc(base_config, 200000));
    record_bench(bench_fixed_std_allocator(base_config, 200000));
    record_bench(bench_fixed_pool(base_config, 200000));
    record_bench(bench_fixed_arena(base_config, 200000));

    // -------------------------------------------------------------------------
    // Suite 2: Variable-Size Allocation Churn (100,000 allocations)
    // -------------------------------------------------------------------------
    std::cout << "\n--- Suite 2: Variable-Size Allocation Churn (16-512 Bytes, N=100,000) ---\n";
    record_bench(bench_variable_malloc(base_config, 100000));
    record_bench(bench_variable_std_allocator(base_config, 100000));
    record_bench(bench_variable_arena(base_config, 100000));

    // -------------------------------------------------------------------------
    // Suite 3: Frame / Batch Temporary Allocation (500 frames x 200 allocs = 100,000)
    // -------------------------------------------------------------------------
    std::cout << "\n--- Suite 3: Frame / Batch Temporary Allocation (500 frames x 200 allocs) ---\n";
    record_bench(bench_frames_malloc(base_config, 500, 200));
    record_bench(bench_frames_std_allocator(base_config, 500, 200));
    record_bench(bench_frames_arena(base_config, 500, 200));

    // -------------------------------------------------------------------------
    // Suite 4: Pool Reuse Cycles (1,000 blocks x 200 cycles = 200,000 allocs)
    // -------------------------------------------------------------------------
    std::cout << "\n--- Suite 4: Pool Reuse Cycles (Working Set: 1,000 blocks x 200 cycles) ---\n";
    record_bench(bench_reuse_malloc(base_config, 1000, 200));
    record_bench(bench_reuse_std_allocator(base_config, 1000, 200));
    record_bench(bench_reuse_pool(base_config, 1000, 200));

    // -------------------------------------------------------------------------
    // Suite 5: Linear Arena Capacity Scaling (64 KiB, 1 MiB, 16 MiB)
    // -------------------------------------------------------------------------
    std::cout << "\n--- Suite 5: Linear Arena Capacity Scaling (64 KiB, 1 MiB, 16 MiB) ---\n";
    record_bench(bench_arena_scale(base_config, "arena_scale_64k", 64 * 1024));
    record_bench(bench_arena_scale(base_config, "arena_scale_1m", 1024 * 1024));
    record_bench(bench_arena_scale(base_config, "arena_scale_16m", 16 * 1024 * 1024));

    // =========================================================================
    // CSV EXPORT (Complete 16-Workload Suite)
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

    std::cout << "\n[Allocator Benchmarks Complete] All workloads finished successfully.\n";
    return 0;
}
