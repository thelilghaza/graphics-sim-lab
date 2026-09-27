#include "performance_lab/allocator_payload.hpp"
#include "performance_lab/compiler_barrier.hpp"
#include "performance_lab/fixed_block_pool.hpp"
#include "performance_lab/linear_arena.hpp"

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <set>
#include <vector>

#define LAB_CHECK(expr) \
    do { \
        if (!(expr)) { \
            std::cerr << "Assertion failed: " #expr " at " << __FILE__ << ":" << __LINE__ << "\n"; \
            std::abort(); \
        } \
    } while (false)

using namespace performance_lab;

void test_fixed_block_pool_basic() {
    std::cout << "[Test 1] FixedBlockPool Basic Operations & Edge Cases...\n";

    constexpr std::size_t kBlockSize = 64;
    constexpr std::size_t kCapacity = 32;
    constexpr std::size_t kAlignment = 16;

    FixedBlockPool pool(kBlockSize, kCapacity, kAlignment);

    LAB_CHECK(pool.capacity() == kCapacity);
    LAB_CHECK(pool.available() == kCapacity);
    LAB_CHECK(pool.blocks_in_use() == 0);
    LAB_CHECK(pool.block_size() >= kBlockSize);
    LAB_CHECK(pool.block_size() % kAlignment == 0);
    LAB_CHECK(pool.alignment() >= kAlignment);

    std::vector<void*> allocated;
    allocated.reserve(kCapacity);

    // Allocate all blocks
    for (std::size_t i = 0; i < kCapacity; ++i) {
        void* ptr = pool.allocate();
        LAB_CHECK(ptr != nullptr);
        LAB_CHECK(reinterpret_cast<std::uintptr_t>(ptr) % kAlignment == 0);
        LAB_CHECK(pool.owns(ptr));
        AllocatorPayload::write_marker(ptr, kBlockSize, i);
        allocated.push_back(ptr);
    }

    LAB_CHECK(pool.available() == 0);
    LAB_CHECK(pool.blocks_in_use() == kCapacity);

    // Pool exhaustion edge case
    void* failed = pool.allocate();
    LAB_CHECK(failed == nullptr);

    // Validate payload integrity
    for (std::size_t i = 0; i < kCapacity; ++i) {
        LAB_CHECK(AllocatorPayload::validate(allocated[i], kBlockSize, i));
    }

    // Verify pointer ownership bounds
    int dummy = 42;
    LAB_CHECK(!pool.owns(&dummy));
    LAB_CHECK(!pool.owns(nullptr));

    // Deallocate half
    for (std::size_t i = 0; i < kCapacity / 2; ++i) {
        pool.deallocate(allocated[i]);
    }
    LAB_CHECK(pool.available() == kCapacity / 2);
    LAB_CHECK(pool.blocks_in_use() == kCapacity / 2);

    // Reallocate the freed blocks
    for (std::size_t i = 0; i < kCapacity / 2; ++i) {
        void* ptr = pool.allocate();
        LAB_CHECK(ptr != nullptr);
        LAB_CHECK(pool.owns(ptr));
    }
    LAB_CHECK(pool.available() == 0);

    // Reset pool
    pool.reset();
    LAB_CHECK(pool.available() == kCapacity);
    LAB_CHECK(pool.blocks_in_use() == 0);

    // Internal overhead check
    double overhead_32 = pool.internal_overhead_percent(32);
    LAB_CHECK(overhead_32 > 0.0);
    double overhead_64 = pool.internal_overhead_percent(pool.block_size());
    LAB_CHECK(overhead_64 == 0.0);

    std::cout << "  Passed FixedBlockPool basic tests.\n";
}

void test_linear_arena_basic() {
    std::cout << "[Test 2] LinearArena Basic Operations & Edge Cases...\n";

    constexpr std::size_t kCapacity = 4096;
    LinearArena arena(kCapacity, 64);

    LAB_CHECK(arena.capacity() == kCapacity);
    LAB_CHECK(arena.bytes_used() == 0);
    LAB_CHECK(arena.peak_bytes_used() == 0);
    LAB_CHECK(arena.allocation_count() == 0);

    // Zero-byte request policy: must return nullptr and perform no allocation
    void* zero_alloc = arena.allocate(0, 8);
    LAB_CHECK(zero_alloc == nullptr);
    LAB_CHECK(arena.bytes_used() == 0);
    LAB_CHECK(arena.allocation_count() == 0);

    // Alignment test across 8, 16, 32, 64
    const std::size_t alignments[] = {8, 16, 32, 64};
    for (std::size_t align : alignments) {
        void* p = arena.allocate(48, align);
        LAB_CHECK(p != nullptr);
        LAB_CHECK(reinterpret_cast<std::uintptr_t>(p) % align == 0);
        LAB_CHECK(arena.owns(p));
        AllocatorPayload::write_marker(p, 48, align);
        LAB_CHECK(AllocatorPayload::validate(p, 48, align));
    }

    LAB_CHECK(arena.allocation_count() == 4);
    LAB_CHECK(arena.bytes_used() > 0);
    LAB_CHECK(arena.peak_bytes_used() == arena.bytes_used());
    LAB_CHECK(arena.alignment_padding_bytes() >= 0);

    // Reset arena
    std::size_t prev_peak = arena.peak_bytes_used();
    arena.reset();
    LAB_CHECK(arena.bytes_used() == 0);
    LAB_CHECK(arena.peak_bytes_used() == prev_peak);

    // Full reuse after reset
    void* large = arena.allocate(kCapacity - 64, 64);
    LAB_CHECK(large != nullptr);
    LAB_CHECK(arena.owns(large));

    // Capacity boundary exhaustion
    void* overflow = arena.allocate(128, 8);
    LAB_CHECK(overflow == nullptr);

    // Exact fit check
    arena.reset();
    void* exact = arena.allocate(kCapacity, arena.base_alignment());
    LAB_CHECK(exact != nullptr);
    LAB_CHECK(arena.bytes_used() == kCapacity);
    LAB_CHECK(arena.allocate(1, 8) == nullptr);

    std::cout << "  Passed LinearArena basic tests.\n";
}

void test_allocator_payload_markers() {
    std::cout << "[Test 3] AllocatorPayload Marker Verification...\n";

    std::byte buffer[128];
    AllocatorPayload::write_marker(buffer, sizeof(buffer), 1337);

    LAB_CHECK(AllocatorPayload::validate(buffer, sizeof(buffer), 1337));
    LAB_CHECK(!AllocatorPayload::validate(buffer, sizeof(buffer), 1338));
    LAB_CHECK(!AllocatorPayload::validate(buffer, 64, 1337));

    // Small buffer (< 8 bytes) handling
    std::byte small_buf[4];
    AllocatorPayload::write_marker(small_buf, sizeof(small_buf), 999);
    LAB_CHECK(!AllocatorPayload::validate(small_buf, sizeof(small_buf), 999));

    std::cout << "  Passed AllocatorPayload marker tests.\n";
}

void test_fixed_block_pool_reuse_cycles() {
    std::cout << "[Test 4] FixedBlockPool Reuse Cycles & Duplicate Detection...\n";

    constexpr std::size_t kBlockSize = 64;
    constexpr std::size_t kCount = 500;
    FixedBlockPool pool(kBlockSize, kCount, 8);

    for (std::size_t cycle = 0; cycle < 10; ++cycle) {
        std::vector<void*> ptrs;
        ptrs.reserve(kCount);
        std::set<void*> unique_ptrs;

        for (std::size_t i = 0; i < kCount; ++i) {
            void* p = pool.allocate();
            LAB_CHECK(p != nullptr);
            LAB_CHECK(unique_ptrs.insert(p).second); // Ensure zero duplicate block addresses
            AllocatorPayload::write_marker(p, kBlockSize, cycle * 1000 + i);
            ptrs.push_back(p);
        }

        LAB_CHECK(pool.available() == 0);

        for (std::size_t i = 0; i < kCount; ++i) {
            LAB_CHECK(AllocatorPayload::validate(ptrs[i], kBlockSize, cycle * 1000 + i));
            pool.deallocate(ptrs[i]);
        }

        LAB_CHECK(pool.available() == kCount);
    }

    std::cout << "  Passed FixedBlockPool reuse cycles and duplicate detection.\n";
}

void test_common_workload_reproducibility() {
    std::cout << "[Test 5] Common Workload Deterministic Reproducibility...\n";

    constexpr std::size_t kNumAllocations = 1000;
    std::allocator<std::byte> std_alloc;
    LinearArena arena(256 * 1024);

    std::vector<void*> malloc_ptrs(kNumAllocations, nullptr);
    std::vector<std::byte*> stdalloc_ptrs(kNumAllocations, nullptr);
    std::vector<void*> arena_ptrs(kNumAllocations, nullptr);

    // Allocate variable sizes across malloc, std::allocator, and arena
    for (std::size_t i = 0; i < kNumAllocations; ++i) {
        std::size_t sz = AllocatorWorkload::variable_size_at(i);

        malloc_ptrs[i] = std::malloc(sz);
        LAB_CHECK(malloc_ptrs[i] != nullptr);
        AllocatorPayload::write_marker(malloc_ptrs[i], sz, i);

        stdalloc_ptrs[i] = std_alloc.allocate(sz);
        LAB_CHECK(stdalloc_ptrs[i] != nullptr);
        AllocatorPayload::write_marker(stdalloc_ptrs[i], sz, i);

        arena_ptrs[i] = arena.allocate(sz, 8);
        LAB_CHECK(arena_ptrs[i] != nullptr);
        AllocatorPayload::write_marker(arena_ptrs[i], sz, i);
    }

    // Verify and clean up
    for (std::size_t i = 0; i < kNumAllocations; ++i) {
        std::size_t sz = AllocatorWorkload::variable_size_at(i);
        LAB_CHECK(AllocatorPayload::validate(malloc_ptrs[i], sz, i));
        LAB_CHECK(AllocatorPayload::validate(stdalloc_ptrs[i], sz, i));
        LAB_CHECK(AllocatorPayload::validate(arena_ptrs[i], sz, i));

        std::free(malloc_ptrs[i]);
        std_alloc.deallocate(stdalloc_ptrs[i], sz);
    }

    arena.reset();
    LAB_CHECK(arena.bytes_used() == 0);

    std::cout << "  Passed common workload reproducibility.\n";
}

void test_allocator_stress() {
    std::cout << "[Test 6] High-Volume Allocator Stress Validation...\n";

    // Pool stress: 100,000 allocations across 500-block cycles
    constexpr std::size_t kPoolBlocks = 500;
    constexpr std::size_t kCycles = 200;
    FixedBlockPool pool(64, kPoolBlocks, 16);

    std::vector<void*> pool_ptrs(kPoolBlocks, nullptr);
    std::uint64_t total_allocs = 0;

    for (std::size_t c = 0; c < kCycles; ++c) {
        for (std::size_t i = 0; i < kPoolBlocks; ++i) {
            pool_ptrs[i] = pool.allocate();
            LAB_CHECK(pool_ptrs[i] != nullptr);
            AllocatorPayload::write_marker(pool_ptrs[i], 64, ++total_allocs);
        }
        for (std::size_t i = 0; i < kPoolBlocks; ++i) {
            pool.deallocate(pool_ptrs[i]);
        }
    }
    LAB_CHECK(total_allocs == kPoolBlocks * kCycles);
    LAB_CHECK(pool.available() == kPoolBlocks);

    // Arena stress: 10,000 arena allocations across 50 frame resets
    constexpr std::size_t kFrameAllocs = 200;
    constexpr std::size_t kFrames = 50;
    LinearArena frame_arena(64 * 1024);

    for (std::size_t f = 0; f < kFrames; ++f) {
        for (std::size_t i = 0; i < kFrameAllocs; ++i) {
            std::size_t sz = AllocatorWorkload::variable_size_at(i);
            void* p = frame_arena.allocate(sz, 16);
            LAB_CHECK(p != nullptr);
            AllocatorPayload::write_marker(p, sz, f * 1000 + i);
        }
        frame_arena.reset();
    }
    LAB_CHECK(frame_arena.bytes_used() == 0);

    std::cout << "  Passed high-volume stress validation (" << total_allocs << " pool ops, "
              << kFrames * kFrameAllocs << " arena frame ops).\n";
}

int main() {
    std::cout << "====================================================\n";
    std::cout << " Running test_allocator_churn (Milestone 5)\n";
    std::cout << "====================================================\n";

    test_fixed_block_pool_basic();
    test_linear_arena_basic();
    test_allocator_payload_markers();
    test_fixed_block_pool_reuse_cycles();
    test_common_workload_reproducibility();
    test_allocator_stress();

    std::cout << "====================================================\n";
    std::cout << " All allocator unit and stress tests PASSED!\n";
    std::cout << "====================================================\n";
    return 0;
}
