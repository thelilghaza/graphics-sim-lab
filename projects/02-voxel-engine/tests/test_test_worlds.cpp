#include "voxel_lab/test_worlds.hpp"
#include "voxel_lab/world_grid.hpp"
#include <cassert>
#include <iostream>

using namespace voxel_lab;

void test_solid_world_generation() {
    WorldGrid world;
    Voxel v_stone(1, 0);

    WorldCoord min_c(0, 0, 0);
    WorldCoord max_c(15, 15, 15);

    generate_solid_world(world, min_c, max_c, v_stone);

    // Inside region
    for (int z = 0; z <= 15; ++z) {
        for (int y = 0; y <= 15; ++y) {
            for (int x = 0; x <= 15; ++x) {
                assert(world.get_voxel(x, y, z) == v_stone);
            }
        }
    }

    // Outside region
    assert(world.get_voxel(16, 0, 0) == Voxel(0, 0));
    assert(world.get_voxel(-1, 0, 0) == Voxel(0, 0));

    std::cout << "[PASS] test_solid_world_generation\n";
}

void test_plane_world_generation() {
    WorldGrid world;
    Voxel v_grass(3, 0);

    WorldCoord min_c(-10, -10, -10);
    WorldCoord max_c(10, 10, 10);
    int plane_y = 0;

    generate_plane_world(world, min_c, max_c, plane_y, PlaneAxis::Y, v_grass);

    // Below or at plane level y <= 0 -> solid grass
    for (int y = -10; y <= 0; ++y) {
        assert(world.get_voxel(0, y, 0) == v_grass);
    }

    // Above plane level y > 0 -> air
    for (int y = 1; y <= 10; ++y) {
        assert(world.get_voxel(0, y, 0) == Voxel(0, 0));
    }

    // Test X-axis plane
    WorldGrid world_x;
    generate_plane_world(world_x, min_c, max_c, 2, PlaneAxis::X, v_grass);
    assert(world_x.get_voxel(2, 0, 0) == v_grass);
    assert(world_x.get_voxel(3, 0, 0) == Voxel(0, 0));

    std::cout << "[PASS] test_plane_world_generation\n";
}

void test_sphere_world_generation() {
    WorldGrid world;
    Voxel v_gold(5, 0);

    WorldCoord center(0, 0, 0);
    int radius = 5;

    generate_sphere_world(world, center, radius, v_gold);

    // Center point
    assert(world.get_voxel(center) == v_gold);

    // Points inside sphere (dx^2 + dy^2 + dz^2 <= 25)
    assert(world.get_voxel(3, 4, 0) == v_gold);  // 9 + 16 + 0 = 25 <= 25
    assert(world.get_voxel(5, 0, 0) == v_gold);  // 25 <= 25

    // Points outside sphere
    assert(world.get_voxel(3, 4, 1) == Voxel(0, 0)); // 9 + 16 + 1 = 26 > 25
    assert(world.get_voxel(6, 0, 0) == Voxel(0, 0));

    std::cout << "[PASS] test_sphere_world_generation\n";
}

void test_cross_chunk_world_generation() {
    WorldGrid world;
    Voxel v_mat(4, 1);

    // Sphere centered on chunk boundary (31, 31, 31) with radius 10
    // Spans chunks (0,0,0), (1,0,0), (0,1,0), (0,0,1), (1,1,1), etc.
    WorldCoord center(31, 31, 31);
    int radius = 10;

    generate_sphere_world(world, center, radius, v_mat);

    // Queries across chunk boundaries
    assert(world.get_voxel(31, 31, 31) == v_mat); // chunk (0,0,0)
    assert(world.get_voxel(32, 31, 31) == v_mat); // chunk (1,0,0)
    assert(world.get_voxel(31, 32, 31) == v_mat); // chunk (0,1,0)
    assert(world.get_voxel(31, 31, 32) == v_mat); // chunk (0,0,1)
    assert(world.get_voxel(32, 32, 32) == v_mat); // chunk (1,1,1)

    assert(world.has_chunk(0, 0, 0));
    assert(world.has_chunk(1, 0, 0));
    assert(world.has_chunk(0, 1, 0));
    assert(world.has_chunk(0, 0, 1));
    assert(world.has_chunk(1, 1, 1));

    // Plane crossing negative chunk boundary: y = -1 from -35 to 35
    WorldGrid plane_world;
    generate_plane_world(plane_world, WorldCoord(-35, -35, -35), WorldCoord(35, 35, 35), -1, PlaneAxis::Y, v_mat);

    assert(plane_world.get_voxel(0, -1, 0) == v_mat);  // chunk (0,-1,0)
    assert(plane_world.get_voxel(0, 0, 0) == Voxel(0, 0));   // chunk (0,0,0)
    assert(plane_world.get_voxel(-33, -1, 0) == v_mat); // chunk (-2,-1,0)

    std::cout << "[PASS] test_cross_chunk_world_generation\n";
}

void test_determinism() {
    WorldGrid worldA;
    WorldGrid worldB;
    Voxel v_sphere(7, 2);

    WorldCoord center(10, 10, 10);
    int radius = 15;

    // Generate into world A
    generate_sphere_world(worldA, center, radius, v_sphere);

    // Generate independently into world B
    generate_sphere_world(worldB, center, radius, v_sphere);

    // Exact equality check across loaded chunks and voxels
    assert(worldA.loaded_chunk_count() == worldB.loaded_chunk_count());
    assert(worldA.count_solid_voxels() == worldB.count_solid_voxels());

    for (int z = -10; z <= 30; ++z) {
        for (int y = -10; y <= 30; ++y) {
            for (int x = -10; x <= 30; ++x) {
                WorldCoord w(x, y, z);
                assert(worldA.get_voxel(w) == worldB.get_voxel(w));
            }
        }
    }

    std::cout << "[PASS] test_determinism\n";
}

int main() {
    test_solid_world_generation();
    test_plane_world_generation();
    test_sphere_world_generation();
    test_cross_chunk_world_generation();
    test_determinism();
    std::cout << "All test world generation tests passed successfully!\n";
    return 0;
}
