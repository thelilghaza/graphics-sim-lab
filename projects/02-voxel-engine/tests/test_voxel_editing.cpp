#include "voxel_lab/world_grid.hpp"
#include <cassert>
#include <iostream>

using namespace voxel_lab;

void test_basic_set_get_clear() {
    WorldGrid world;
    WorldCoord target(15, 10, 5);

    Voxel v_stone(1, 0);

    // Initial state: empty/air
    assert(world.get_voxel(target) == Voxel(0, 0));
    assert(world.is_solid(target) == false);

    // Set voxel
    world.set_voxel(target, v_stone);
    assert(world.get_voxel(target) == v_stone);
    assert(world.is_solid(target) == true);

    // Clear voxel
    world.clear_voxel(target);
    assert(world.get_voxel(target) == Voxel(0, 0));
    assert(world.is_solid(target) == false);

    std::cout << "[PASS] test_basic_set_get_clear\n";
}

void test_isolation_unrelated_coordinates() {
    WorldGrid world;
    WorldCoord w1(10, 10, 10);
    WorldCoord w2(10, 10, 11);
    WorldCoord w3(11, 10, 10);

    Voxel v_dirt(2, 1);

    world.set_voxel(w1, v_dirt);

    assert(world.get_voxel(w1) == v_dirt);
    assert(world.get_voxel(w2) == Voxel(0, 0));
    assert(world.get_voxel(w3) == Voxel(0, 0));

    std::cout << "[PASS] test_isolation_unrelated_coordinates\n";
}

void test_boundary_and_cross_chunk_editing() {
    WorldGrid world;

    Voxel v_gold(5, 7);
    Voxel v_iron(6, 2);

    // Boundary at lx = 31 (World X = 31) and lx = 0 (World X = 32)
    world.set_voxel(31, 5, 5, v_gold);
    world.set_voxel(32, 5, 5, v_iron);

    assert(world.get_voxel(31, 5, 5) == v_gold);
    assert(world.get_voxel(32, 5, 5) == v_iron);
    assert(world.has_chunk(0, 0, 0));
    assert(world.has_chunk(1, 0, 0));

    // Clear across boundary
    world.clear_voxel(31, 5, 5);
    assert(world.get_voxel(31, 5, 5) == Voxel(0, 0));
    assert(world.get_voxel(32, 5, 5) == v_iron);

    // Negative boundary editing: World X = -1 (chunk -1 local 31) and World X = 0 (chunk 0 local 0)
    world.set_voxel(-1, -1, -1, v_gold);
    assert(world.get_voxel(-1, -1, -1) == v_gold);
    assert(world.has_chunk(-1, -1, -1));

    std::cout << "[PASS] test_boundary_and_cross_chunk_editing\n";
}

void test_fill_box() {
    WorldGrid world;
    Voxel v_fill(3, 2);

    WorldCoord min_c(10, 10, 10);
    WorldCoord max_c(12, 12, 12);

    world.fill_box(min_c, max_c, v_fill);

    // Inside box check (3x3x3 = 27 voxels)
    size_t count = 0;
    for (int z = 10; z <= 12; ++z) {
        for (int y = 10; y <= 12; ++y) {
            for (int x = 10; x <= 12; ++x) {
                assert(world.get_voxel(x, y, z) == v_fill);
                ++count;
            }
        }
    }
    assert(count == 27);

    // Outside box check
    assert(world.get_voxel(9, 10, 10) == Voxel(0, 0));
    assert(world.get_voxel(13, 12, 12) == Voxel(0, 0));

    std::cout << "[PASS] test_fill_box\n";
}

int main() {
    test_basic_set_get_clear();
    test_isolation_unrelated_coordinates();
    test_boundary_and_cross_chunk_editing();
    test_fill_box();
    std::cout << "All voxel editing tests passed successfully!\n";
    return 0;
}
