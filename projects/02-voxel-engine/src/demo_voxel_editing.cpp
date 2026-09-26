#include "voxel_lab/test_worlds.hpp"
#include "voxel_lab/world_grid.hpp"
#include <iostream>

using namespace voxel_lab;

int main() {
    std::cout << "=== Voxel Engine Milestone 3 — Developer Demo ===\n";

    // 1. Solid World Generation Demo
    WorldGrid solid_grid;
    generate_solid_world(solid_grid, WorldCoord(0, 0, 0), WorldCoord(31, 31, 31), Voxel(1, 0));
    std::cout << "[Solid World] Loaded Chunks: " << solid_grid.loaded_chunk_count()
              << " | Solid Voxels: " << solid_grid.count_solid_voxels() << "\n";

    // 2. Planar World Generation Demo
    WorldGrid plane_grid;
    generate_plane_world(plane_grid, WorldCoord(-32, -32, -32), WorldCoord(63, 63, 63), 0, PlaneAxis::Y, Voxel(3, 0));
    std::cout << "[Plane World] Loaded Chunks: " << plane_grid.loaded_chunk_count()
              << " | Solid Voxels: " << plane_grid.count_solid_voxels() << "\n";

    // 3. Cross-Chunk Sphere Generation Demo
    WorldGrid sphere_grid;
    WorldCoord sphere_center(31, 31, 31);
    int sphere_radius = 12;
    generate_sphere_world(sphere_grid, sphere_center, sphere_radius, Voxel(5, 1));

    std::cout << "[Sphere World] Loaded Chunks: " << sphere_grid.loaded_chunk_count()
              << " | Solid Voxels: " << sphere_grid.count_solid_voxels() << "\n";

    // 4. Cross-Chunk Coordinate Queries
    WorldCoord w_inside_c0(31, 31, 31);
    WorldCoord w_inside_c1(32, 31, 31);
    WorldCoord w_outside(31 + sphere_radius + 1, 31, 31);

    std::cout << "\nCross-Chunk Coordinate Queries:\n";
    std::cout << "  World (31, 31, 31) -> Chunk (0,0,0): "
              << (sphere_grid.is_solid(w_inside_c0) ? "SOLID" : "AIR") << "\n";
    std::cout << "  World (32, 31, 31) -> Chunk (1,0,0): "
              << (sphere_grid.is_solid(w_inside_c1) ? "SOLID" : "AIR") << "\n";
    std::cout << "  World (" << w_outside.x << ", 31, 31) -> Outside: "
              << (sphere_grid.is_solid(w_outside) ? "SOLID" : "AIR") << "\n";

    // 5. Determinism Verification
    WorldGrid sphere_grid_copy;
    generate_sphere_world(sphere_grid_copy, sphere_center, sphere_radius, Voxel(5, 1));
    bool is_deterministic = (sphere_grid.loaded_chunk_count() == sphere_grid_copy.loaded_chunk_count()) &&
                             (sphere_grid.count_solid_voxels() == sphere_grid_copy.count_solid_voxels());

    std::cout << "\nDeterminism Verification: "
              << (is_deterministic ? "[PASS] Exact Match" : "[FAIL] Mismatch") << "\n";

    std::cout << "\n=== Milestone 3 Demo Complete ===\n";
    return 0;
}
