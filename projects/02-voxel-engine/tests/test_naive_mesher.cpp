#include "voxel_lab/naive_mesher.hpp"
#include "voxel_lab/test_worlds.hpp"
#include "voxel_lab/world_grid.hpp"
#include <cassert>
#include <iostream>

using namespace voxel_lab;

void test_empty_chunk() {
    WorldGrid world;
    ChunkCoord c(0, 0, 0);

    MeshData mesh = mesh_chunk(world, c);
    assert(mesh.face_count() == 0);
    assert(mesh.vertex_count() == 0);
    assert(mesh.index_count() == 0);

    std::cout << "[PASS] test_empty_chunk\n";
}

void test_single_voxel() {
    WorldGrid world;
    world.set_voxel(5, 5, 5, Voxel(1, 0));

    MeshData mesh = mesh_chunk(world, ChunkCoord(0, 0, 0));
    assert(mesh.face_count() == 6);
    assert(mesh.vertex_count() == 24);
    assert(mesh.index_count() == 36);

    std::cout << "[PASS] test_single_voxel\n";
}

void test_two_adjacent_voxels() {
    WorldGrid world;
    world.set_voxel(5, 5, 5, Voxel(1, 0));
    world.set_voxel(6, 5, 5, Voxel(1, 0)); // Adjacent along +X

    MeshData mesh = mesh_chunk(world, ChunkCoord(0, 0, 0));
    assert(mesh.face_count() == 10);
    assert(mesh.vertex_count() == 40);
    assert(mesh.index_count() == 60);

    std::cout << "[PASS] test_two_adjacent_voxels\n";
}

void test_full_solid_chunk() {
    WorldGrid world;
    generate_solid_world(world, WorldCoord(0, 0, 0), WorldCoord(31, 31, 31), Voxel(1, 0));

    MeshData mesh = mesh_chunk(world, ChunkCoord(0, 0, 0));
    assert(mesh.face_count() == 6144);
    assert(mesh.vertex_count() == 24576);
    assert(mesh.index_count() == 36864);

    std::cout << "[PASS] test_full_solid_chunk\n";
}

void test_cross_chunk_boundary() {
    WorldGrid world;
    Voxel v_stone(1, 0);

    // X-axis boundary: Chunk 0 (31, 10, 10) vs Chunk 1 (32, 10, 10)
    world.set_voxel(31, 10, 10, v_stone);
    world.set_voxel(32, 10, 10, v_stone);

    MeshData mesh0_both = mesh_chunk(world, ChunkCoord(0, 0, 0));
    MeshData mesh1_both = mesh_chunk(world, ChunkCoord(1, 0, 0));

    // Single isolated voxel has 6 faces; adjacent across chunk boundary hides 1 face (+X for c0, -X for c1)
    assert(mesh0_both.face_count() == 5);
    assert(mesh1_both.face_count() == 5);

    // Remove voxel at (32, 10, 10), face +X on (31, 10, 10) must reappear
    world.clear_voxel(32, 10, 10);
    MeshData mesh0_after = mesh_chunk(world, ChunkCoord(0, 0, 0));
    assert(mesh0_after.face_count() == 6);

    // Y-axis boundary: Chunk 0 (10, 31, 10) vs Chunk 1 (10, 32, 10)
    WorldGrid world_y;
    world_y.set_voxel(10, 31, 10, v_stone);
    world_y.set_voxel(10, 32, 10, v_stone);
    assert(mesh_chunk(world_y, ChunkCoord(0, 0, 0)).face_count() == 5);
    assert(mesh_chunk(world_y, ChunkCoord(0, 1, 0)).face_count() == 5);

    // Z-axis boundary: Chunk 0 (10, 10, 31) vs Chunk 1 (10, 10, 32)
    WorldGrid world_z;
    world_z.set_voxel(10, 10, 31, v_stone);
    world_z.set_voxel(10, 10, 32, v_stone);
    assert(mesh_chunk(world_z, ChunkCoord(0, 0, 0)).face_count() == 5);
    assert(mesh_chunk(world_z, ChunkCoord(0, 0, 1)).face_count() == 5);

    std::cout << "[PASS] test_cross_chunk_boundary\n";
}

void test_negative_chunk_coordinates() {
    WorldGrid world;
    Voxel v_stone(1, 0);

    // Voxels in Chunk (-1, -1, -1)
    world.set_voxel(-1, -1, -1, v_stone);  // local (31, 31, 31)
    world.set_voxel(-2, -1, -1, v_stone);  // local (30, 31, 31)

    MeshData mesh_neg = mesh_chunk(world, ChunkCoord(-1, -1, -1));
    assert(mesh_neg.face_count() == 10);
    assert(mesh_neg.vertex_count() == 40);

    std::cout << "[PASS] test_negative_chunk_coordinates\n";
}

void test_face_normals_and_geometry_bounds() {
    WorldGrid world;
    world.set_voxel(0, 0, 0, Voxel(1, 0)); // Local (0,0,0) in Chunk (0,0,0)

    MeshData mesh = mesh_chunk(world, ChunkCoord(0, 0, 0));
    assert(mesh.face_count() == 6);

    // Verify outward normals for all 6 faces
    bool has_pos_x = false, has_neg_x = false;
    bool has_pos_y = false, has_neg_y = false;
    bool has_pos_z = false, has_neg_z = false;

    for (const auto& v : mesh.vertices) {
        // Bounds check: local positions must lie within [0.0, 1.0] for voxel at (0,0,0)
        assert(v.x >= 0.0f && v.x <= 1.0f);
        assert(v.y >= 0.0f && v.y <= 1.0f);
        assert(v.z >= 0.0f && v.z <= 1.0f);

        if (v.nx == 1.0f) has_pos_x = true;
        if (v.nx == -1.0f) has_neg_x = true;
        if (v.ny == 1.0f) has_pos_y = true;
        if (v.ny == -1.0f) has_neg_y = true;
        if (v.nz == 1.0f) has_pos_z = true;
        if (v.nz == -1.0f) has_neg_z = true;
    }

    assert(has_pos_x && has_neg_x && has_pos_y && has_neg_y && has_pos_z && has_neg_z);

    std::cout << "[PASS] test_face_normals_and_geometry_bounds\n";
}

void test_determinism() {
    WorldGrid world;
    generate_sphere_world(world, WorldCoord(15, 15, 15), 10, Voxel(1, 0));

    MeshData meshA = mesh_chunk(world, ChunkCoord(0, 0, 0));
    MeshData meshB = mesh_chunk(world, ChunkCoord(0, 0, 0));

    assert(meshA == meshB);

    std::cout << "[PASS] test_determinism\n";
}

void test_test_world_integration() {
    WorldGrid plane_world;
    generate_plane_world(plane_world, WorldCoord(0, 0, 0), WorldCoord(31, 31, 31), 15, PlaneAxis::Y, Voxel(1, 0));

    MeshData plane_mesh = mesh_chunk(plane_world, ChunkCoord(0, 0, 0));
    // Plane at y = 15: top exposed face at y = 15 (+Y faces = 32x32 = 1024 faces), bottom exposed face at y = 0 (-Y faces = 1024 faces),
    // and 4 side boundaries (+X, -X, +Z, -Z: 4 * 16 * 32 = 2048 faces). Total = 4096 faces.
    assert(plane_mesh.face_count() == 4096);

    std::cout << "[PASS] test_test_world_integration\n";
}

int main() {
    test_empty_chunk();
    test_single_voxel();
    test_two_adjacent_voxels();
    test_full_solid_chunk();
    test_cross_chunk_boundary();
    test_negative_chunk_coordinates();
    test_face_normals_and_geometry_bounds();
    test_determinism();
    test_test_world_integration();
    std::cout << "All naive mesher unit tests passed successfully!\n";
    return 0;
}
