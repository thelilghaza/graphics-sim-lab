#include "voxel_lab/world_grid.hpp"
#include <cassert>
#include <iostream>

using namespace voxel_lab;

void test_missing_chunk_behavior() {
    WorldGrid world;
    WorldCoord w(-33, 15, 100);

    // Unpopulated chunk checks
    assert(world.has_chunk(world_to_chunk(w)) == false);
    assert(world.has_voxel(w) == false);

    // get_voxel returns default air Voxel(0,0) for missing chunks
    assert(world.get_voxel(w) == Voxel(0, 0));

    // Non-throwing get_voxel returns false for missing chunks
    Voxel out;
    assert(world.get_voxel(w, out) == false);

    std::cout << "[PASS] test_missing_chunk_behavior\n";
}

void test_cross_chunk_boundary_reads_and_writes() {
    WorldGrid world;

    Voxel v_dirt(2, 1);
    Voxel v_stone(1, 0);
    Voxel v_gold(5, 7);

    // 1. Positive Boundary: World X = 31 (chunk 0 local 31) vs World X = 32 (chunk 1 local 0)
    world.set_voxel(31, 10, 10, v_dirt);
    world.set_voxel(32, 10, 10, v_stone);

    assert(world.has_chunk(0, 0, 0) == true);
    assert(world.has_chunk(1, 0, 0) == true);

    assert(world.get_voxel(31, 10, 10) == v_dirt);
    assert(world.get_voxel(32, 10, 10) == v_stone);

    // 2. Negative Boundary: World X = -1 (chunk -1 local 31)
    world.set_voxel(-1, 10, 10, v_gold);
    assert(world.has_chunk(-1, 0, 0) == true);
    assert(world.get_voxel(-1, 10, 10) == v_gold);

    // World X = -32 (chunk -1 local 0)
    world.set_voxel(-32, 10, 10, v_stone);
    assert(world.get_voxel(-32, 10, 10) == v_stone);

    // World X = -33 (chunk -2 local 31)
    world.set_voxel(-33, 10, 10, v_dirt);
    assert(world.has_chunk(-2, 0, 0) == true);
    assert(world.get_voxel(-33, 10, 10) == v_dirt);

    std::cout << "[PASS] test_cross_chunk_boundary_reads_and_writes\n";
}

void test_independent_xyz_axis_boundaries() {
    WorldGrid world;
    Voxel v_test(10, 2);

    // Independent X boundary
    world.set_voxel(-1, 0, 0, v_test);
    assert(world.get_voxel(-1, 0, 0) == v_test);
    assert(world.get_voxel(0, 0, 0) != v_test);

    // Independent Y boundary
    world.set_voxel(0, -1, 0, v_test);
    assert(world.get_voxel(0, -1, 0) == v_test);
    assert(world.get_voxel(0, 0, 0) != v_test);

    // Independent Z boundary
    world.set_voxel(0, 0, -1, v_test);
    assert(world.get_voxel(0, 0, -1) == v_test);
    assert(world.get_voxel(0, 0, 0) != v_test);

    std::cout << "[PASS] test_independent_xyz_axis_boundaries\n";
}

void test_combined_corner_coordinates() {
    WorldGrid world;
    Voxel v_corner(7, 3);

    // (31, 31, 31) -> chunk (0, 0, 0) local (31, 31, 31)
    world.set_voxel(31, 31, 31, v_corner);
    assert(world.get_voxel(31, 31, 31) == v_corner);
    assert(world.has_chunk(0, 0, 0));

    // (32, 31, 31) -> chunk (1, 0, 0) local (0, 31, 31)
    world.set_voxel(32, 31, 31, Voxel(8, 3));
    assert(world.get_voxel(32, 31, 31) == Voxel(8, 3));
    assert(world.has_chunk(1, 0, 0));

    // (-1, -1, -1) -> chunk (-1, -1, -1) local (31, 31, 31)
    world.set_voxel(-1, -1, -1, Voxel(9, 3));
    assert(world.get_voxel(-1, -1, -1) == Voxel(9, 3));
    assert(world.has_chunk(-1, -1, -1));

    // (-32, -32, -32) -> chunk (-1, -1, -1) local (0, 0, 0)
    world.set_voxel(-32, -32, -32, Voxel(10, 3));
    assert(world.get_voxel(-32, -32, -32) == Voxel(10, 3));
    assert(world.has_chunk(-1, -1, -1));

    // (-33, -33, -33) -> chunk (-2, -2, -2) local (31, 31, 31)
    world.set_voxel(-33, -33, -33, Voxel(11, 3));
    assert(world.get_voxel(-33, -33, -33) == Voxel(11, 3));
    assert(world.has_chunk(-2, -2, -2));

    std::cout << "[PASS] test_combined_corner_coordinates\n";
}

int main() {
    test_missing_chunk_behavior();
    test_cross_chunk_boundary_reads_and_writes();
    test_independent_xyz_axis_boundaries();
    test_combined_corner_coordinates();
    std::cout << "All WorldAccessor cross-chunk tests passed successfully!\n";
    return 0;
}
