#include "voxel_lab/chunk.hpp"
#include <cassert>
#include <iostream>
#include <stdexcept>

using namespace voxel_lab;

void test_chunk_constants_and_size() {
    assert(CHUNK_DIM == 32);
    assert(CHUNK_VOXELS == 32768);
    assert(STORAGE_BYTES == 65536);
    assert(sizeof(Chunk) == 65536);

    Chunk chunk;
    assert(chunk.dimension() == 32);
    assert(chunk.voxel_count() == 32768);
    assert(chunk.storage_size_bytes() == 65536);
    std::cout << "[PASS] test_chunk_constants_and_size\n";
}

void test_indexing_mapping() {
    // (0,0,0) -> 0
    assert(Chunk::to_index(0, 0, 0) == 0);

    // Boundary check
    assert(Chunk::to_index(1, 0, 0) == 1);
    assert(Chunk::to_index(0, 1, 0) == 32);
    assert(Chunk::to_index(0, 0, 1) == 1024);

    // (31, 31, 31) -> 32767
    assert(Chunk::to_index(31, 31, 31) == 32767);
    std::cout << "[PASS] test_indexing_mapping\n";
}

void test_read_write_roundtrip_and_aliasing() {
    Chunk chunk;

    // Default clear state check
    for (size_t i = 0; i < CHUNK_VOXELS; ++i) {
        assert(chunk.get_voxel_at_index(i) == Voxel(0, 0));
    }

    Voxel v_stone(1, 1);
    Voxel v_dirt(2, 0);

    // Write to (5, 10, 15)
    assert(chunk.set_voxel(5, 10, 15, v_stone));
    assert(chunk.get_voxel(5, 10, 15) == v_stone);

    // Write to (15, 10, 5)
    assert(chunk.set_voxel(15, 10, 5, v_dirt));
    assert(chunk.get_voxel(15, 10, 5) == v_dirt);

    // Verify non-aliasing
    assert(chunk.get_voxel(5, 10, 15) == v_stone);
    assert(chunk.get_voxel(15, 10, 5) != v_stone);

    // Verify (0,0,0) and (31,31,31)
    assert(chunk.set_voxel(0, 0, 0, Voxel(10, 5)));
    assert(chunk.set_voxel(31, 31, 31, Voxel(99, 7)));

    assert(chunk.get_voxel(0, 0, 0) == Voxel(10, 5));
    assert(chunk.get_voxel(31, 31, 31) == Voxel(99, 7));

    std::cout << "[PASS] test_read_write_roundtrip_and_aliasing\n";
}

void test_clear_and_fill() {
    Chunk chunk;
    Voxel v_gold(5, 2);

    chunk.fill(v_gold);
    assert(chunk.get_voxel(0, 0, 0) == v_gold);
    assert(chunk.get_voxel(16, 16, 16) == v_gold);
    assert(chunk.get_voxel(31, 31, 31) == v_gold);

    chunk.clear();
    assert(chunk.get_voxel(0, 0, 0) == Voxel(0, 0));
    assert(chunk.get_voxel(16, 16, 16) == Voxel(0, 0));
    assert(chunk.get_voxel(31, 31, 31) == Voxel(0, 0));

    std::cout << "[PASS] test_clear_and_fill\n";
}

void test_out_of_bounds_handling() {
    Chunk chunk;
    Voxel dummy;

    // Out of bounds in_bounds check
    assert(Chunk::in_bounds(-1, 0, 0) == false);
    assert(Chunk::in_bounds(32, 0, 0) == false);
    assert(Chunk::in_bounds(0, -1, 0) == false);
    assert(Chunk::in_bounds(0, 32, 0) == false);
    assert(Chunk::in_bounds(0, 0, -1) == false);
    assert(Chunk::in_bounds(0, 0, 32) == false);

    // noexcept get_voxel returns false
    assert(chunk.get_voxel(-1, 0, 0, dummy) == false);
    assert(chunk.get_voxel(32, 0, 0, dummy) == false);

    // noexcept set_voxel returns false
    assert(chunk.set_voxel(-1, 0, 0, Voxel(1, 0)) == false);
    assert(chunk.set_voxel(32, 0, 0, Voxel(1, 0)) == false);

    // Exception throw check for throwing get_voxel
    bool caught_exception = false;
    try {
        (void)chunk.get_voxel(-1, 0, 0);
    } catch (const std::out_of_range&) {
        caught_exception = true;
    }
    assert(caught_exception);

    std::cout << "[PASS] test_out_of_bounds_handling\n";
}

int main() {
    test_chunk_constants_and_size();
    test_indexing_mapping();
    test_read_write_roundtrip_and_aliasing();
    test_clear_and_fill();
    test_out_of_bounds_handling();
    std::cout << "All Chunk storage tests passed successfully!\n";
    return 0;
}
