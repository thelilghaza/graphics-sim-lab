#include "performance_lab/data_layouts.hpp"
#include "performance_lab/stride_benchmark.hpp"
#include "performance_lab/workingset_benchmark.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

using namespace performance_lab;

void test_layout_equivalence() {
    std::cout << "[Test 1] AoS vs SoA vs AoSoA Equivalence...\n";
    const size_t count = 10'000;

    auto data_aos = generate_aos(count);
    auto data_soa = generate_soa(count);
    auto data_aosoa = generate_aosoa<DEFAULT_TILE_WIDTH>(count);

    double cs_aos = compute_checksum_aos(data_aos);
    double cs_soa = compute_checksum_soa(data_soa);
    double cs_aosoa = compute_checksum_aosoa<DEFAULT_TILE_WIDTH>(data_aosoa);

    std::cout << "  AoS   Checksum: " << cs_aos << "\n";
    std::cout << "  SoA   Checksum: " << cs_soa << "\n";
    std::cout << "  AoSoA Checksum: " << cs_aosoa << "\n";

    // Numerical equivalence test
    assert(std::abs(cs_aos - cs_soa) < 1e-4);
    assert(std::abs(cs_aos - cs_aosoa) < 1e-4);
}

void test_deterministic_initialization() {
    std::cout << "[Test 2] Deterministic Initialization Repeatability...\n";
    const size_t count = 100;

    auto run1_aos = generate_aos(count);
    auto run2_aos = generate_aos(count);

    for (size_t i = 0; i < count; ++i) {
        assert(run1_aos[i].x == run2_aos[i].x);
        assert(run1_aos[i].y == run2_aos[i].y);
        assert(run1_aos[i].z == run2_aos[i].z);
        assert(run1_aos[i].vx == run2_aos[i].vx);
        assert(run1_aos[i].vy == run2_aos[i].vy);
        assert(run1_aos[i].vz == run2_aos[i].vz);
        assert(run1_aos[i].mass == run2_aos[i].mass);
        assert(run1_aos[i].id == run2_aos[i].id);
    }
}

void test_stride_bounds_and_correctness() {
    std::cout << "[Test 3] Stride Access Bounds and Correctness...\n";
    const size_t access_count = 1000;
    const std::vector<size_t> strides = {1, 2, 4, 8, 16, 32, 64, 128, 256};

    for (size_t stride : strides) {
        auto buffer = generate_stride_buffer(access_count, stride);
        size_t required_elements = access_count * stride;
        assert(buffer.size() == required_elements);
        (void)required_elements;

        // Run stride access and ensure last element accessed is within bounds
        size_t max_index = (access_count - 1) * stride;
        assert(max_index < buffer.size());
        (void)max_index;

        float sum = run_stride_access(buffer, access_count, stride);
        assert(!std::isnan(sum));
        assert(sum > 0.0f);
        (void)sum;
    }
}

void test_workingset_size_calculation() {
    std::cout << "[Test 4] Working-Set Size Calculation and Edge Cases...\n";
    const std::vector<size_t> test_bytes = {4096, 65536, 1048576};

    for (size_t bytes : test_bytes) {
        auto buffer = generate_workingset_buffer(bytes);
        assert(buffer.size() == bytes / sizeof(float));

        float sum = run_workingset_pass(buffer);
        assert(!std::isnan(sum));
        assert(sum > 0.0f);
        (void)sum;
    }
}

void test_edge_cases() {
    std::cout << "[Test 5] Edge Cases (Single Record & Small Tile Boundaries)...\n";
    
    // Single record test
    auto aos1 = generate_aos(1);
    auto soa1 = generate_soa(1);
    auto aosoa1 = generate_aosoa<16>(1);

    assert(aos1.size() == 1);
    assert(soa1.size() == 1);
    assert(aosoa1.size() == 1);
    assert(std::abs(compute_checksum_aos(aos1) - compute_checksum_soa(soa1)) < 1e-5);
    assert(std::abs(compute_checksum_aos(aos1) - compute_checksum_aosoa(aosoa1)) < 1e-5);

    // Non-multiple tile width test (17 records for tile width 16)
    auto aosoa17 = generate_aosoa<16>(17);
    assert(aosoa17.tiles.size() == 2);
    assert(aosoa17.size() == 17);
}

int main() {
    std::cout << "=== Running test_cache_locality ===\n";

    test_layout_equivalence();
    test_deterministic_initialization();
    test_stride_bounds_and_correctness();
    test_workingset_size_calculation();
    test_edge_cases();

    std::cout << "=== All 5 cache locality unit tests PASSED cleanly ===\n";
    return 0;
}
