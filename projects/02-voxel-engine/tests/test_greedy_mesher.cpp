#include "voxel_lab/greedy_mesher.hpp"
#include "voxel_lab/naive_mesher.hpp"
#include "voxel_lab/test_worlds.hpp"
#include "voxel_lab/world_grid.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <set>
#include <tuple>

using namespace voxel_lab;

// Canonical representation of a unit 1x1 face on a voxel at local (x, y, z)
struct CanonicalUnitFace {
    int dir; // 0: +X, 1: -X, 2: +Y, 3: -Y, 4: +Z, 5: -Z
    int x;
    int y;
    int z;

    bool operator<(const CanonicalUnitFace& o) const noexcept {
        return std::tie(dir, x, y, z) < std::tie(o.dir, o.x, o.y, o.z);
    }
    bool operator==(const CanonicalUnitFace& o) const noexcept {
        return dir == o.dir && x == o.x && y == o.y && z == o.z;
    }
};

// Extracts canonical unit faces from a mesh
std::set<CanonicalUnitFace> extract_canonical_faces(const MeshData& mesh) {
    std::set<CanonicalUnitFace> faces;
    size_t quad_count = mesh.indices.size() / 6;

    for (size_t q = 0; q < quad_count; ++q) {
        uint32_t i0 = mesh.indices[q * 6 + 0];
        uint32_t i1 = mesh.indices[q * 6 + 1];
        uint32_t i2 = mesh.indices[q * 6 + 2];
        uint32_t i3 = mesh.indices[q * 6 + 5];

        const MeshVertex& v0 = mesh.vertices[i0];
        const MeshVertex& v1 = mesh.vertices[i1];
        const MeshVertex& v2 = mesh.vertices[i2];
        const MeshVertex& v3 = mesh.vertices[i3];

        int dir = -1;
        if (v0.nx > 0.5f) dir = 0;       // +X
        else if (v0.nx < -0.5f) dir = 1;  // -X
        else if (v0.ny > 0.5f) dir = 2;  // +Y
        else if (v0.ny < -0.5f) dir = 3; // -Y
        else if (v0.nz > 0.5f) dir = 4;  // +Z
        else if (v0.nz < -0.5f) dir = 5; // -Z

        assert(dir != -1);

        float min_x = std::min({v0.x, v1.x, v2.x, v3.x});
        float max_x = std::max({v0.x, v1.x, v2.x, v3.x});
        float min_y = std::min({v0.y, v1.y, v2.y, v3.y});
        float max_y = std::max({v0.y, v1.y, v2.y, v3.y});
        float min_z = std::min({v0.z, v1.z, v2.z, v3.z});
        float max_z = std::max({v0.z, v1.z, v2.z, v3.z});

        switch (dir) {
            case 0: { // +X face at x = max_x, voxel is at x = max_x - 1
                int vx = static_cast<int>(std::round(max_x - 1.0f));
                int start_y = static_cast<int>(std::round(min_y));
                int end_y = static_cast<int>(std::round(max_y));
                int start_z = static_cast<int>(std::round(min_z));
                int end_z = static_cast<int>(std::round(max_z));
                for (int y = start_y; y < end_y; ++y) {
                    for (int z = start_z; z < end_z; ++z) {
                        faces.insert(CanonicalUnitFace{0, vx, y, z});
                    }
                }
                break;
            }
            case 1: { // -X face at x = min_x, voxel is at x = min_x
                int vx = static_cast<int>(std::round(min_x));
                int start_y = static_cast<int>(std::round(min_y));
                int end_y = static_cast<int>(std::round(max_y));
                int start_z = static_cast<int>(std::round(min_z));
                int end_z = static_cast<int>(std::round(max_z));
                for (int y = start_y; y < end_y; ++y) {
                    for (int z = start_z; z < end_z; ++z) {
                        faces.insert(CanonicalUnitFace{1, vx, y, z});
                    }
                }
                break;
            }
            case 2: { // +Y face at y = max_y, voxel is at y = max_y - 1
                int vy = static_cast<int>(std::round(max_y - 1.0f));
                int start_x = static_cast<int>(std::round(min_x));
                int end_x = static_cast<int>(std::round(max_x));
                int start_z = static_cast<int>(std::round(min_z));
                int end_z = static_cast<int>(std::round(max_z));
                for (int x = start_x; x < end_x; ++x) {
                    for (int z = start_z; z < end_z; ++z) {
                        faces.insert(CanonicalUnitFace{2, x, vy, z});
                    }
                }
                break;
            }
            case 3: { // -Y face at y = min_y, voxel is at y = min_y
                int vy = static_cast<int>(std::round(min_y));
                int start_x = static_cast<int>(std::round(min_x));
                int end_x = static_cast<int>(std::round(max_x));
                int start_z = static_cast<int>(std::round(min_z));
                int end_z = static_cast<int>(std::round(max_z));
                for (int x = start_x; x < end_x; ++x) {
                    for (int z = start_z; z < end_z; ++z) {
                        faces.insert(CanonicalUnitFace{3, x, vy, z});
                    }
                }
                break;
            }
            case 4: { // +Z face at z = max_z, voxel is at z = max_z - 1
                int vz = static_cast<int>(std::round(max_z - 1.0f));
                int start_x = static_cast<int>(std::round(min_x));
                int end_x = static_cast<int>(std::round(max_x));
                int start_y = static_cast<int>(std::round(min_y));
                int end_y = static_cast<int>(std::round(max_y));
                for (int x = start_x; x < end_x; ++x) {
                    for (int y = start_y; y < end_y; ++y) {
                        faces.insert(CanonicalUnitFace{4, x, y, vz});
                    }
                }
                break;
            }
            case 5: { // -Z face at z = min_z, voxel is at z = min_z
                int vz = static_cast<int>(std::round(min_z));
                int start_x = static_cast<int>(std::round(min_x));
                int end_x = static_cast<int>(std::round(max_x));
                int start_y = static_cast<int>(std::round(min_y));
                int end_y = static_cast<int>(std::round(max_y));
                for (int x = start_x; x < end_x; ++x) {
                    for (int y = start_y; y < end_y; ++y) {
                        faces.insert(CanonicalUnitFace{5, x, y, vz});
                    }
                }
                break;
            }
        }
    }
    return faces;
}

void verify_surface_equivalence(const WorldAccessor& world, const ChunkCoord& c, const char* case_name) {
    MeshData naive_mesh = mesh_chunk(world, c);
    MeshData greedy_mesh = greedy_mesh_chunk(world, c);

    auto naive_faces = extract_canonical_faces(naive_mesh);
    auto greedy_faces = extract_canonical_faces(greedy_mesh);

    if (naive_faces.size() != naive_mesh.face_count() || naive_faces != greedy_faces) {
        std::cerr << "[FAIL] Surface Equivalence failed for " << case_name << "\n";
        std::abort();
    }

    std::cout << "[PASS] Surface Equivalence for " << case_name
              << ": Naive " << naive_mesh.face_count() << " faces -> Greedy "
              << greedy_mesh.face_count() << " quads (Exact match: "
              << naive_faces.size() << " canonical unit faces)\n";
}

void test_empty_chunk() {
    WorldGrid world;
    ChunkCoord c(0, 0, 0);

    MeshData greedy = greedy_mesh_chunk(world, c);
    assert(greedy.face_count() == 0);
    assert(greedy.vertex_count() == 0);
    assert(greedy.index_count() == 0);

    verify_surface_equivalence(world, c, "Empty Chunk");
    std::cout << "[PASS] test_empty_chunk\n";
}

void test_single_voxel() {
    WorldGrid world;
    world.set_voxel(10, 10, 10, Voxel(1, 0));
    ChunkCoord c(0, 0, 0);

    MeshData greedy = greedy_mesh_chunk(world, c);
    assert(greedy.face_count() == 6);
    assert(greedy.vertex_count() == 24);
    assert(greedy.index_count() == 36);

    verify_surface_equivalence(world, c, "Single Voxel");
    std::cout << "[PASS] test_single_voxel\n";
}

void test_two_adjacent_voxels() {
    WorldGrid world;
    world.set_voxel(10, 10, 10, Voxel(1, 0));
    world.set_voxel(11, 10, 10, Voxel(1, 0)); // Adjacent along X
    ChunkCoord c(0, 0, 0);

    MeshData naive = mesh_chunk(world, c);
    MeshData greedy = greedy_mesh_chunk(world, c);

    assert(naive.face_count() == 10);
    // 4 shared sides along Y/Z merge into 2x1 quads (4 quads) + 2 end caps (2 quads) = 6 quads
    assert(greedy.face_count() == 6);

    verify_surface_equivalence(world, c, "Two Adjacent Voxels");
    std::cout << "[PASS] test_two_adjacent_voxels\n";
}

void test_full_solid_chunk() {
    WorldGrid world;
    generate_solid_world(world, WorldCoord(0, 0, 0), WorldCoord(31, 31, 31), Voxel(1, 0));
    ChunkCoord c(0, 0, 0);

    MeshData greedy = greedy_mesh_chunk(world, c);

    // Exactly 6 quads: one for each of the 6 sides of the full chunk
    assert(greedy.face_count() == 6);
    assert(greedy.vertex_count() == 24);
    assert(greedy.index_count() == 36);

    verify_surface_equivalence(world, c, "Full Solid Chunk");
    std::cout << "[PASS] test_full_solid_chunk\n";
}

void test_planar_world() {
    WorldGrid world;
    generate_plane_world(world, WorldCoord(0, 0, 0), WorldCoord(31, 31, 31), 15, PlaneAxis::Y, Voxel(1, 0));
    ChunkCoord c(0, 0, 0);

    MeshData naive = mesh_chunk(world, c);
    MeshData greedy = greedy_mesh_chunk(world, c);

    // Naive has 4096 faces; greedy collapses top plane (32x32) into 1 quad, bottom into 1 quad, and 4 sides into 1 quad each = 6 quads
    assert(naive.face_count() == 4096);
    assert(greedy.face_count() == 6);

    verify_surface_equivalence(world, c, "Planar World");
    std::cout << "[PASS] test_planar_world\n";
}

void test_sphere_world() {
    WorldGrid world;
    generate_sphere_world(world, WorldCoord(15, 15, 15), 12, Voxel(1, 0));
    ChunkCoord c(0, 0, 0);

    MeshData naive = mesh_chunk(world, c);
    MeshData greedy = greedy_mesh_chunk(world, c);

    assert(naive.face_count() == 2646);
    assert(greedy.face_count() < naive.face_count());

    verify_surface_equivalence(world, c, "Sphere World");
    std::cout << "[PASS] test_sphere_world\n";
}

void test_cross_chunk_boundary() {
    WorldGrid world;
    Voxel v_stone(1, 0);

    // Cross-chunk solid boundary at chunk 0 (31, 10, 10) vs chunk 1 (32, 10, 10)
    world.set_voxel(31, 10, 10, v_stone);
    world.set_voxel(32, 10, 10, v_stone);

    MeshData greedy0 = greedy_mesh_chunk(world, ChunkCoord(0, 0, 0));
    MeshData greedy1 = greedy_mesh_chunk(world, ChunkCoord(1, 0, 0));

    // Internal shared face between (31,10,10) and (32,10,10) must be culled
    assert(greedy0.face_count() == 5);
    assert(greedy1.face_count() == 5);

    verify_surface_equivalence(world, ChunkCoord(0, 0, 0), "Cross-Chunk Boundary (Chunk 0)");
    verify_surface_equivalence(world, ChunkCoord(1, 0, 0), "Cross-Chunk Boundary (Chunk 1)");

    std::cout << "[PASS] test_cross_chunk_boundary\n";
}

void test_negative_chunk_coordinates() {
    WorldGrid world;
    Voxel v_stone(1, 0);

    world.set_voxel(-1, -1, -1, v_stone);
    world.set_voxel(-2, -1, -1, v_stone);
    ChunkCoord c(-1, -1, -1);

    verify_surface_equivalence(world, c, "Negative Chunk Coordinates");
    std::cout << "[PASS] test_negative_chunk_coordinates\n";
}

void test_face_normals_and_geometry_bounds() {
    WorldGrid world;
    world.set_voxel(0, 0, 0, Voxel(1, 0));
    ChunkCoord c(0, 0, 0);

    MeshData greedy = greedy_mesh_chunk(world, c);
    assert(greedy.face_count() == 6);

    for (const auto& v : greedy.vertices) {
        assert(v.x >= 0.0f && v.x <= 32.0f);
        assert(v.y >= 0.0f && v.y <= 32.0f);
        assert(v.z >= 0.0f && v.z <= 32.0f);

        float len = std::sqrt(v.nx * v.nx + v.ny * v.ny + v.nz * v.nz);
        assert(std::abs(len - 1.0f) < 0.0001f);
        static_cast<void>(len);
    }

    std::cout << "[PASS] test_face_normals_and_geometry_bounds\n";
}

void test_deterministic_output() {
    WorldGrid world;
    generate_sphere_world(world, WorldCoord(15, 15, 15), 10, Voxel(1, 0));
    ChunkCoord c(0, 0, 0);

    MeshData greedy1 = greedy_mesh_chunk(world, c);
    MeshData greedy2 = greedy_mesh_chunk(world, c);

    assert(greedy1 == greedy2);
    std::cout << "[PASS] test_deterministic_output\n";
}

void test_voxel_type_compatibility() {
    WorldGrid world;
    // Two adjacent voxels with DIFFERENT type IDs
    world.set_voxel(10, 10, 10, Voxel(1, 0)); // Stone (type 1)
    world.set_voxel(11, 10, 10, Voxel(2, 0)); // Dirt (type 2)
    ChunkCoord c(0, 0, 0);

    MeshData greedy = greedy_mesh_chunk(world, c);

    // Side faces along +Y, -Y, +Z, -Z must NOT merge across type 1 and type 2!
    // Each voxel has 5 exposed faces (shared face between 10 and 11 is culled).
    // Because type 1 != type 2, the side faces cannot merge, resulting in 5 + 5 = 10 quads!
    assert(greedy.face_count() == 10);

    verify_surface_equivalence(world, c, "Voxel Type Compatibility (Non-Mergeable Types)");
    std::cout << "[PASS] test_voxel_type_compatibility\n";
}

void test_cross_chunk_sphere() {
    WorldGrid world;
    generate_sphere_world(world, WorldCoord(31, 31, 31), 12, Voxel(1, 0));
    for (int cz = 0; cz <= 1; ++cz) {
        for (int cy = 0; cy <= 1; ++cy) {
            for (int cx = 0; cx <= 1; ++cx) {
                ChunkCoord c(cx, cy, cz);
                verify_surface_equivalence(world, c, "Cross-Chunk Sphere");
            }
        }
    }
    std::cout << "[PASS] test_cross_chunk_sphere\n";
}

int main() {
    test_empty_chunk();
    test_single_voxel();
    test_two_adjacent_voxels();
    test_full_solid_chunk();
    test_planar_world();
    test_sphere_world();
    test_cross_chunk_boundary();
    test_cross_chunk_sphere();
    test_negative_chunk_coordinates();
    test_face_normals_and_geometry_bounds();
    test_deterministic_output();
    test_voxel_type_compatibility();
    std::cout << "All greedy mesher unit tests passed successfully!\n";
    return 0;
}
