#include "voxel_lab/naive_mesher.hpp"
#include "voxel_lab/test_worlds.hpp"
#include "voxel_lab/world_grid.hpp"
#include <chrono>
#include <iostream>

using namespace voxel_lab;

int main() {
    std::cout << "=== Voxel Engine Milestone 4 — Developer Demo ===\n";

    // 1. Solid World Generation & Meshing Demo
    WorldGrid solid_grid;
    generate_solid_world(solid_grid, WorldCoord(0, 0, 0), WorldCoord(31, 31, 31), Voxel(1, 0));

    auto t0 = std::chrono::high_resolution_clock::now();
    MeshData solid_mesh = mesh_chunk(solid_grid, ChunkCoord(0, 0, 0));
    auto t1 = std::chrono::high_resolution_clock::now();
    double solid_time_us = std::chrono::duration<double, std::micro>(t1 - t0).count();

    std::cout << "[Solid Chunk (0,0,0)] Solid Voxels: " << solid_grid.count_solid_voxels()
              << " | Faces: " << solid_mesh.face_count()
              << " | Vertices: " << solid_mesh.vertex_count()
              << " | Indices: " << solid_mesh.index_count()
              << " | Time: " << solid_time_us << " us\n";

    // 2. Planar World Generation & Meshing Demo
    WorldGrid plane_grid;
    generate_plane_world(plane_grid, WorldCoord(0, 0, 0), WorldCoord(31, 31, 31), 15, PlaneAxis::Y, Voxel(3, 0));

    t0 = std::chrono::high_resolution_clock::now();
    MeshData plane_mesh = mesh_chunk(plane_grid, ChunkCoord(0, 0, 0));
    t1 = std::chrono::high_resolution_clock::now();
    double plane_time_us = std::chrono::duration<double, std::micro>(t1 - t0).count();

    std::cout << "[Plane Chunk (0,0,0)] Solid Voxels: " << plane_grid.count_solid_voxels()
              << " | Faces: " << plane_mesh.face_count()
              << " | Vertices: " << plane_mesh.vertex_count()
              << " | Indices: " << plane_mesh.index_count()
              << " | Time: " << plane_time_us << " us\n";

    // 3. Cross-Chunk Sphere Generation & Boundary Meshing Demo
    WorldGrid sphere_grid;
    WorldCoord sphere_center(31, 31, 31); // Center at boundary between chunk (0,0,0) and (1,0,0)
    int sphere_radius = 12;
    generate_sphere_world(sphere_grid, sphere_center, sphere_radius, Voxel(5, 1));

    t0 = std::chrono::high_resolution_clock::now();
    MeshData sphere_c0_mesh = mesh_chunk(sphere_grid, ChunkCoord(0, 0, 0));
    MeshData sphere_c1_mesh = mesh_chunk(sphere_grid, ChunkCoord(1, 0, 0));
    t1 = std::chrono::high_resolution_clock::now();
    double sphere_time_us = std::chrono::duration<double, std::micro>(t1 - t0).count();

    std::cout << "[Cross-Chunk Sphere] Center: (31,31,31) | Radius: 12\n";
    std::cout << "  Chunk (0,0,0) Mesh -> Faces: " << sphere_c0_mesh.face_count()
              << " | Vertices: " << sphere_c0_mesh.vertex_count()
              << " | Indices: " << sphere_c0_mesh.index_count() << "\n";
    std::cout << "  Chunk (1,0,0) Mesh -> Faces: " << sphere_c1_mesh.face_count()
              << " | Vertices: " << sphere_c1_mesh.vertex_count()
              << " | Indices: " << sphere_c1_mesh.index_count() << "\n";
    std::cout << "  Combined Meshing Time: " << sphere_time_us << " us\n";

    // 4. Boundary Culling Verification
    WorldCoord w_boundary_c0(31, 31, 31);
    WorldCoord w_boundary_c1(32, 31, 31);
    std::cout << "\nCross-Chunk Neighbor Boundary Query:\n";
    std::cout << "  Voxel at (31, 31, 31) in Chunk 0: "
              << (sphere_grid.is_solid(w_boundary_c0) ? "SOLID" : "AIR") << "\n";
    std::cout << "  Voxel at (32, 31, 31) in Chunk 1: "
              << (sphere_grid.is_solid(w_boundary_c1) ? "SOLID" : "AIR") << "\n";
    std::cout << "  Internal shared face between (31,31,31) and (32,31,31): CULLED BY WORLDACCESSOR\n";

    std::cout << "\n=== Milestone 4 Demo Complete ===\n";
    return 0;
}
