#include "voxel_lab/coordinates.hpp"
#include <cassert>
#include <iostream>
#include <vector>

using namespace voxel_lab;

void test_positive_coordinate_decomposition() {
    struct TestVector {
        int w;
        int expected_chunk;
        int expected_local;
    };

    const std::vector<TestVector> tests = {
        {0, 0, 0},
        {1, 0, 1},
        {31, 0, 31},
        {32, 1, 0},
        {33, 1, 1},
        {63, 1, 31},
        {64, 2, 0}
    };

    for (const auto& t : tests) {
        auto [chunk, local] = decompose_world_coord_scalar(t.w);
        assert(chunk == t.expected_chunk);
        assert(local == t.expected_local);
        assert(t.w == chunk * CHUNK_DIM + local);
    }

    std::cout << "[PASS] test_positive_coordinate_decomposition\n";
}

void test_negative_coordinate_decomposition() {
    struct TestVector {
        int w;
        int expected_chunk;
        int expected_local;
    };

    const std::vector<TestVector> tests = {
        {-1, -1, 31},
        {-2, -1, 30},
        {-31, -1, 1},
        {-32, -1, 0},
        {-33, -2, 31},
        {-63, -2, 1},
        {-64, -2, 0},
        {-65, -3, 31}
    };

    for (const auto& t : tests) {
        auto [chunk, local] = decompose_world_coord_scalar(t.w);
        assert(chunk == t.expected_chunk);
        assert(local == t.expected_local);
        assert(t.w == chunk * CHUNK_DIM + local);
    }

    std::cout << "[PASS] test_negative_coordinate_decomposition\n";
}

void test_exact_multiples_and_boundaries() {
    for (int multiplier = -5; multiplier <= 5; ++multiplier) {
        int w_exact = multiplier * 32;
        auto [c_exact, l_exact] = decompose_world_coord_scalar(w_exact);
        assert(c_exact == multiplier);
        assert(l_exact == 0);

        int w_below = w_exact - 1;
        auto [c_below, l_below] = decompose_world_coord_scalar(w_below);
        assert(c_below == multiplier - 1);
        assert(l_below == 31);
    }

    std::cout << "[PASS] test_exact_multiples_and_boundaries\n";
}

void test_reconstruction_invariant() {
    for (int w = -500; w <= 500; ++w) {
        auto [chunk, local] = decompose_world_coord_scalar(w);
        assert(0 <= local && local < 32);
        assert(w == chunk * 32 + local);
    }

    WorldCoord w_in(-33, 32, -1);
    auto [c, l] = decompose_world_coord(w_in);
    WorldCoord w_reconstructed = reconstruct_world_coord(c, l);
    assert(w_in == w_reconstructed);

    std::cout << "[PASS] test_reconstruction_invariant\n";
}

void test_coordinate_types() {
    WorldCoord w1(10, 20, 30);
    WorldCoord w2(10, 20, 30);
    WorldCoord w3(10, 20, 31);

    assert(w1 == w2);
    assert(w1 != w3);

    ChunkCoord c1(1, 2, 3);
    ChunkCoord c2(1, 2, 4);
    assert(c1 < c2);

    std::cout << "[PASS] test_coordinate_types\n";
}

int main() {
    test_positive_coordinate_decomposition();
    test_negative_coordinate_decomposition();
    test_exact_multiples_and_boundaries();
    test_reconstruction_invariant();
    test_coordinate_types();
    std::cout << "All coordinate conversion tests passed successfully!\n";
    return 0;
}
