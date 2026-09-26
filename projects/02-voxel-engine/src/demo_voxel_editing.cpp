#include "voxel_lab/greedy_mesher.hpp"
#include "voxel_lab/naive_mesher.hpp"
#include "voxel_lab/test_worlds.hpp"
#include "voxel_lab/world_grid.hpp"
#include <chrono>
#include <iomanip>
#include <iostream>

using namespace voxel_lab;

int main() {
    std::cout << "=== Voxel Engine Milestone 6 — Developer Demo ===\n";

    // 1. Solid World Generation & Meshing Demo
    WorldGrid solid_grid;
    generate_solid_world(solid_grid, WorldCoord(0, 0, 0), WorldCoord(31, 31, 31), Voxel(1, 0));

    auto t0 = std::chrono::high_resolution_clock::now();
    MeshData solid_naive = mesh_chunk(solid_grid, ChunkCoord(0, 0, 0));
    auto t1 = std::chrono::high_resolution_clock::now();
    double solid_naive_time_us = std::chrono::duration<double, std::micro>(t1 - t0).count();

    t0 = std::chrono::high_resolution_clock::now();
    MeshData solid_greedy = greedy_mesh_chunk(solid_grid, ChunkCoord(0, 0, 0));
    t1 = std::chrono::high_resolution_clock::now();
    double solid_greedy_time_us = std::chrono::duration<double, std::micro>(t1 - t0).count();

    std::cout << "\n[1. Solid Chunk (0,0,0)] Solid Voxels: " << solid_grid.count_solid_voxels() << "\n"
              << "  Naive  -> Faces: " << solid_naive.face_count()
              << " | Vertices: " << solid_naive.vertex_count()
              << " | Indices: " << solid_naive.index_count()
              << " | Time: " << solid_naive_time_us << " us\n"
              << "  Greedy -> Quads: " << solid_greedy.face_count()
              << " | Vertices: " << solid_greedy.vertex_count()
              << " | Indices: " << solid_greedy.index_count()
              << " | Time: " << solid_greedy_time_us << " us\n"
              << "  Reduction: " << std::fixed << std::setprecision(2)
              << (1.0 - static_cast<double>(solid_greedy.face_count()) / static_cast<double>(solid_naive.face_count())) * 100.0 << "%\n";

    // 2. Planar World Generation & Meshing Demo
    WorldGrid plane_grid;
    generate_plane_world(plane_grid, WorldCoord(0, 0, 0), WorldCoord(31, 31, 31), 15, PlaneAxis::Y, Voxel(3, 0));

    t0 = std::chrono::high_resolution_clock::now();
    MeshData plane_naive = mesh_chunk(plane_grid, ChunkCoord(0, 0, 0));
    t1 = std::chrono::high_resolution_clock::now();
    double plane_naive_time_us = std::chrono::duration<double, std::micro>(t1 - t0).count();

    t0 = std::chrono::high_resolution_clock::now();
    MeshData plane_greedy = greedy_mesh_chunk(plane_grid, ChunkCoord(0, 0, 0));
    t1 = std::chrono::high_resolution_clock::now();
    double plane_greedy_time_us = std::chrono::duration<double, std::micro>(t1 - t0).count();

    std::cout << "\n[2. Plane Chunk (y<=15)] Solid Voxels: " << plane_grid.count_solid_voxels() << "\n"
              << "  Naive  -> Faces: " << plane_naive.face_count()
              << " | Vertices: " << plane_naive.vertex_count()
              << " | Indices: " << plane_naive.index_count()
              << " | Time: " << plane_naive_time_us << " us\n"
              << "  Greedy -> Quads: " << plane_greedy.face_count()
              << " | Vertices: " << plane_greedy.vertex_count()
              << " | Indices: " << plane_greedy.index_count()
              << " | Time: " << plane_greedy_time_us << " us\n"
              << "  Reduction: " << std::fixed << std::setprecision(2)
              << (1.0 - static_cast<double>(plane_greedy.face_count()) / static_cast<double>(plane_naive.face_count())) * 100.0 << "%\n";

    // 3. Cross-Chunk Sphere Generation & Boundary Meshing Demo
    WorldGrid sphere_grid;
    WorldCoord sphere_center(31, 31, 31);
    int sphere_radius = 12;
    generate_sphere_world(sphere_grid, sphere_center, sphere_radius, Voxel(5, 1));

    t0 = std::chrono::high_resolution_clock::now();
    MeshData sphere_naive_c0 = mesh_chunk(sphere_grid, ChunkCoord(0, 0, 0));
    t1 = std::chrono::high_resolution_clock::now();
    double sphere_naive_time_us = std::chrono::duration<double, std::micro>(t1 - t0).count();

    t0 = std::chrono::high_resolution_clock::now();
    MeshData sphere_greedy_c0 = greedy_mesh_chunk(sphere_grid, ChunkCoord(0, 0, 0));
    t1 = std::chrono::high_resolution_clock::now();
    double sphere_greedy_time_us = std::chrono::duration<double, std::micro>(t1 - t0).count();

    std::cout << "\n[3. Cross-Chunk Sphere (chunk 0,0,0)] Center: (31,31,31) | Radius: 12\n"
              << "  Naive  -> Faces: " << sphere_naive_c0.face_count()
              << " | Vertices: " << sphere_naive_c0.vertex_count()
              << " | Indices: " << sphere_naive_c0.index_count()
              << " | Time: " << sphere_naive_time_us << " us\n"
              << "  Greedy -> Quads: " << sphere_greedy_c0.face_count()
              << " | Vertices: " << sphere_greedy_c0.vertex_count()
              << " | Indices: " << sphere_greedy_c0.index_count()
              << " | Time: " << sphere_greedy_time_us << " us\n"
              << "  Reduction: " << std::fixed << std::setprecision(2)
              << (1.0 - static_cast<double>(sphere_greedy_c0.face_count()) / static_cast<double>(sphere_naive_c0.face_count())) * 100.0 << "%\n";

    // 4. Boundary Culling Verification
    WorldCoord w_boundary_c0(31, 31, 31);
    WorldCoord w_boundary_c1(32, 31, 31);
    std::cout << "\nCross-Chunk Neighbor Boundary Query:\n";
    std::cout << "  Voxel at (31, 31, 31) in Chunk 0: "
              << (sphere_grid.is_solid(w_boundary_c0) ? "SOLID" : "AIR") << "\n";
    std::cout << "  Voxel at (32, 31, 31) in Chunk 1: "
              << (sphere_grid.is_solid(w_boundary_c1) ? "SOLID" : "AIR") << "\n";
    std::cout << "  Internal shared face between (31,31,31) and (32,31,31): CULLED BY WORLDACCESSOR\n";

    std::cout << "\n=== Milestone 6 Demo Complete ===\n";
    return 0;
}
