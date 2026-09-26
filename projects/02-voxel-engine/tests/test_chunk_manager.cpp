#include "voxel_lab/chunk_manager.hpp"
#include "voxel_lab/test_worlds.hpp"

#include <cassert>
#include <cstdlib>
#include <iostream>
#include <set>
#include <vector>

using namespace voxel_lab;

#define CHECK(expr) do { \
    if (!(expr)) { \
        std::cerr << "CHECK failed at " << __FILE__ << ":" << __LINE__ << " (" #expr ")\n"; \
        std::abort(); \
    } \
} while (0)

void test_initial_load() {
    StreamingConfig config;
    config.load_radius = 1;
    config.unload_radius = 2;

    ChunkManager mgr(config, MesherType::Naive);
    CHECK(mgr.loaded_chunk_count() == 0);

    bool changed = mgr.update_streaming(Vec3(0.0f, 0.0f, 0.0f));
    CHECK(changed);
    CHECK(mgr.has_camera_chunk());
    CHECK((mgr.get_camera_chunk() == ChunkCoord(0, 0, 0)));

    // (2 * 1 + 1)^3 = 27 chunks
    CHECK(mgr.loaded_chunk_count() == 27);
    const auto& metrics = mgr.get_metrics();
    CHECK(metrics.chunks_loaded_this_update == 27);
    CHECK(metrics.chunks_unloaded_this_update == 0);
    CHECK(metrics.total_chunks_loaded == 27);
    CHECK(metrics.currently_loaded_chunks == 27);

    for (int dz = -1; dz <= 1; ++dz) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                CHECK(mgr.is_loaded(ChunkCoord(dx, dy, dz)));
                CHECK(mgr.get_mesh(ChunkCoord(dx, dy, dz)) != nullptr);
            }
        }
    }

    std::cout << "[PASS] test_initial_load\n";
}

void test_movement_within_same_chunk() {
    StreamingConfig config;
    config.load_radius = 1;
    config.unload_radius = 2;

    ChunkManager mgr(config, MesherType::Naive);
    mgr.update_streaming(Vec3(0.0f, 0.0f, 0.0f));
    CHECK(mgr.loaded_chunk_count() == 27);

    // Move camera to (15.5, 20.0, 31.9) - still in chunk (0,0,0)
    bool changed = mgr.update_streaming(Vec3(15.5f, 20.0f, 31.9f));
    CHECK(!changed);

    const auto& metrics = mgr.get_metrics();
    CHECK(metrics.chunks_loaded_this_update == 0);
    CHECK(metrics.chunks_unloaded_this_update == 0);
    CHECK(metrics.currently_loaded_chunks == 27);
    CHECK(mgr.loaded_chunk_count() == 27);

    std::cout << "[PASS] test_movement_within_same_chunk\n";
}

void test_crossing_pos_x_boundary() {
    StreamingConfig config;
    config.load_radius = 1;
    config.unload_radius = 2;

    ChunkManager mgr(config, MesherType::Naive);
    mgr.update_streaming(Vec3(0.0f, 0.0f, 0.0f));
    CHECK(mgr.loaded_chunk_count() == 27);

    // Move to chunk (1, 0, 0)
    bool changed = mgr.update_streaming(Vec3(32.0f, 0.0f, 0.0f));
    CHECK(changed);
    CHECK((mgr.get_camera_chunk() == ChunkCoord(1, 0, 0)));

    // New chunks at cx = 2 must now be loaded (3 * 3 = 9 chunks)
    for (int dz = -1; dz <= 1; ++dz) {
        for (int dy = -1; dy <= 1; ++dy) {
            CHECK(mgr.is_loaded(ChunkCoord(2, dy, dz)));
        }
    }

    // Chunks at cx = -1 have distance |1 - (-1)| = 2 <= unload_radius, so they remain loaded
    for (int dz = -1; dz <= 1; ++dz) {
        for (int dy = -1; dy <= 1; ++dy) {
            CHECK(mgr.is_loaded(ChunkCoord(-1, dy, dz)));
        }
    }

    // Now move to chunk (2, 0, 0)
    changed = mgr.update_streaming(Vec3(64.0f, 0.0f, 0.0f));
    CHECK(changed);
    CHECK((mgr.get_camera_chunk() == ChunkCoord(2, 0, 0)));

    // Now chunks at cx = -1 have distance |2 - (-1)| = 3 > unload_radius, so they must be UNLOADED
    for (int dz = -1; dz <= 1; ++dz) {
        for (int dy = -1; dy <= 1; ++dy) {
            CHECK(!mgr.is_loaded(ChunkCoord(-1, dy, dz)));
            CHECK(mgr.get_mesh(ChunkCoord(-1, dy, dz)) == nullptr);
        }
    }

    CHECK(mgr.get_metrics().chunks_unloaded_this_update == 9);
    std::cout << "[PASS] test_crossing_pos_x_boundary\n";
}

void test_crossing_neg_x_boundary() {
    StreamingConfig config;
    config.load_radius = 1;
    config.unload_radius = 2;

    ChunkManager mgr(config, MesherType::Naive);
    mgr.update_streaming(Vec3(0.0f, 0.0f, 0.0f));

    // Move into negative X space: chunk (-1, 0, 0)
    bool changed = mgr.update_streaming(Vec3(-1.0f, 0.0f, 0.0f));
    CHECK(changed);
    CHECK((mgr.get_camera_chunk() == ChunkCoord(-1, 0, 0)));

    for (int dz = -1; dz <= 1; ++dz) {
        for (int dy = -1; dy <= 1; ++dy) {
            CHECK(mgr.is_loaded(ChunkCoord(-2, dy, dz)));
        }
    }

    // Move to chunk (-2, 0, 0) -> distance to cx = 1 is |-2 - 1| = 3 > 2 -> cx = 1 unloads
    changed = mgr.update_streaming(Vec3(-33.0f, 0.0f, 0.0f));
    CHECK(changed);
    CHECK((mgr.get_camera_chunk() == ChunkCoord(-2, 0, 0)));

    for (int dz = -1; dz <= 1; ++dz) {
        for (int dy = -1; dy <= 1; ++dy) {
            CHECK(!mgr.is_loaded(ChunkCoord(1, dy, dz)));
        }
    }

    std::cout << "[PASS] test_crossing_neg_x_boundary\n";
}

void test_y_boundary() {
    StreamingConfig config;
    config.load_radius = 1;
    config.unload_radius = 2;

    ChunkManager mgr(config, MesherType::Naive);
    mgr.update_streaming(Vec3(0.0f, 0.0f, 0.0f));

    // Move to chunk (0, 1, 0)
    bool changed = mgr.update_streaming(Vec3(0.0f, 32.0f, 0.0f));
    CHECK(changed);
    CHECK((mgr.get_camera_chunk() == ChunkCoord(0, 1, 0)));
    CHECK(mgr.is_loaded(ChunkCoord(0, 2, 0)));

    // Move to chunk (0, 2, 0) -> cy = -1 must unload
    changed = mgr.update_streaming(Vec3(0.0f, 64.0f, 0.0f));
    CHECK(changed);
    CHECK(!mgr.is_loaded(ChunkCoord(0, -1, 0)));

    // Move to chunk (0, -1, 0)
    changed = mgr.update_streaming(Vec3(0.0f, -1.0f, 0.0f));
    CHECK(changed);
    CHECK((mgr.get_camera_chunk() == ChunkCoord(0, -1, 0)));
    CHECK(mgr.is_loaded(ChunkCoord(0, -2, 0)));

    std::cout << "[PASS] test_y_boundary\n";
}

void test_z_boundary() {
    StreamingConfig config;
    config.load_radius = 1;
    config.unload_radius = 2;

    ChunkManager mgr(config, MesherType::Naive);
    mgr.update_streaming(Vec3(0.0f, 0.0f, 0.0f));

    // Move to chunk (0, 0, 1)
    bool changed = mgr.update_streaming(Vec3(0.0f, 0.0f, 32.0f));
    CHECK(changed);
    CHECK((mgr.get_camera_chunk() == ChunkCoord(0, 0, 1)));
    CHECK(mgr.is_loaded(ChunkCoord(0, 0, 2)));

    // Move to chunk (0, 0, 2) -> cz = -1 unloads
    changed = mgr.update_streaming(Vec3(0.0f, 0.0f, 64.0f));
    CHECK(changed);
    CHECK(!mgr.is_loaded(ChunkCoord(0, 0, -1)));

    // Move to chunk (0, 0, -1)
    changed = mgr.update_streaming(Vec3(0.0f, 0.0f, -1.0f));
    CHECK(changed);
    CHECK((mgr.get_camera_chunk() == ChunkCoord(0, 0, -1)));
    CHECK(mgr.is_loaded(ChunkCoord(0, 0, -2)));

    std::cout << "[PASS] test_z_boundary\n";
}

void test_deterministic_reload() {
    ChunkManager mgr;
    ChunkCoord c(5, 5, 5);

    CHECK(!mgr.is_loaded(c));
    bool loaded = mgr.load_chunk(c);
    CHECK(loaded);
    CHECK(mgr.is_loaded(c));

    // Record copy of all voxels in the chunk
    const Chunk* chunk1 = mgr.get_world().get_chunk(c);
    CHECK(chunk1 != nullptr);
    std::vector<Voxel> saved_voxels;
    saved_voxels.reserve(CHUNK_VOXELS);
    for (int lz = 0; lz < CHUNK_DIM; ++lz) {
        for (int ly = 0; ly < CHUNK_DIM; ++ly) {
            for (int lx = 0; lx < CHUNK_DIM; ++lx) {
                saved_voxels.push_back(chunk1->get_voxel(lx, ly, lz));
            }
        }
    }

    // Unload the chunk
    bool unloaded = mgr.unload_chunk(c);
    CHECK(unloaded);
    CHECK(!mgr.is_loaded(c));
    CHECK(mgr.get_world().get_chunk(c) == nullptr);

    // Reload the chunk
    loaded = mgr.load_chunk(c);
    CHECK(loaded);
    CHECK(mgr.is_loaded(c));

    const Chunk* chunk2 = mgr.get_world().get_chunk(c);
    CHECK(chunk2 != nullptr);
    size_t idx = 0;
    for (int lz = 0; lz < CHUNK_DIM; ++lz) {
        for (int ly = 0; ly < CHUNK_DIM; ++ly) {
            for (int lx = 0; lx < CHUNK_DIM; ++lx) {
                CHECK(chunk2->get_voxel(lx, ly, lz) == saved_voxels[idx++]);
            }
        }
    }

    std::cout << "[PASS] test_deterministic_reload\n";
}

void test_hysteresis() {
    StreamingConfig config;
    config.load_radius = 1;
    config.unload_radius = 2; // Hysteresis band is distance 2

    ChunkManager mgr(config, MesherType::Naive);
    mgr.update_streaming(Vec3(0.0f, 0.0f, 0.0f));

    ChunkCoord origin_neighbor(-1, 0, 0);
    CHECK(mgr.is_loaded(origin_neighbor));

    // Move to (1, 0, 0): origin_neighbor has Chebyshev distance |1 - (-1)| = 2
    // Since 2 <= unload_radius, it MUST remain loaded
    mgr.update_streaming(Vec3(32.0f, 0.0f, 0.0f));
    CHECK(mgr.is_loaded(origin_neighbor));
    CHECK(mgr.get_metrics().chunks_unloaded_this_update == 0);

    // Move to (2, 0, 0): origin_neighbor has Chebyshev distance |2 - (-1)| = 3
    // Since 3 > unload_radius, it MUST be unloaded
    mgr.update_streaming(Vec3(64.0f, 0.0f, 0.0f));
    CHECK(!mgr.is_loaded(origin_neighbor));
    CHECK(mgr.get_metrics().chunks_unloaded_this_update > 0);

    std::cout << "[PASS] test_hysteresis\n";
}

void test_missing_chunk_semantics() {
    ChunkManager mgr;
    ChunkCoord c(7, -3, 4);
    CHECK(!mgr.is_loaded(c));

    // Missing chunk reports missing semantics through WorldAccessor
    const WorldGrid& world = mgr.get_world();
    CHECK(!world.has_chunk(c));

    WorldCoord w = reconstruct_world_coord(c, LocalCoord(5, 5, 5));
    CHECK(!world.has_voxel(w));
    CHECK(world.get_voxel(w) == Voxel(0, 0)); // Default air
    Voxel out_v;
    CHECK(!world.get_voxel(w, out_v));

    std::cout << "[PASS] test_missing_chunk_semantics\n";
}

void test_neighbor_mesh_invalidation() {
    for (auto mesher : {MesherType::Naive, MesherType::Greedy}) {
        auto boundary_gen = [](WorldAccessor& world, const ChunkCoord& c) {
            WorldGrid* grid = dynamic_cast<WorldGrid*>(&world);
            Chunk* chunk = grid ? grid->get_chunk(c) : nullptr;
            if (chunk) {
                // Place solid voxels on chunk boundary centers
                chunk->set_voxel(31, 16, 16, Voxel(1, 0)); // +X face
                chunk->set_voxel(0, 16, 16, Voxel(1, 0));  // -X face
                chunk->set_voxel(16, 31, 16, Voxel(1, 0)); // +Y face
                chunk->set_voxel(16, 0, 16, Voxel(1, 0));  // -Y face
                chunk->set_voxel(16, 16, 31, Voxel(1, 0)); // +Z face
                chunk->set_voxel(16, 16, 0, Voxel(1, 0));  // -Z face
            }
        };

        ChunkManager mgr(StreamingConfig{}, mesher, boundary_gen);

        // 1. Test X Boundary Invalidation
        ChunkCoord c0(0, 0, 0);
        ChunkCoord c1(1, 0, 0);

        mgr.load_chunk(c0);
        const MeshData* m0 = mgr.get_mesh(c0);
        CHECK(m0 != nullptr);
        // c0 has 6 isolated voxels, each having 6 faces = 36 faces (in naive: 36 faces, in greedy: 36 quads)
        const size_t isolated_face_count = m0->face_count();
        CHECK(isolated_face_count == 36);

        // Now load c1 directly adjacent to c0 along +X
        mgr.load_chunk(c1);
        const MeshData* m0_after = mgr.get_mesh(c0);
        const MeshData* m1 = mgr.get_mesh(c1);
        CHECK(m0_after != nullptr);
        CHECK(m1 != nullptr);

        // Shared internal boundary face between (31, 16, 16) in c0 and (0, 16, 16) in c1 is culled!
        CHECK(m0_after->face_count() == isolated_face_count - 1);
        CHECK(m1->face_count() == isolated_face_count - 1);

        // Now unload c1 -> c0 must be invalidated and regenerated to restore the exposed face!
        mgr.unload_chunk(c1);
        const MeshData* m0_restored = mgr.get_mesh(c0);
        CHECK(m0_restored != nullptr);
        CHECK(m0_restored->face_count() == isolated_face_count);

        // 2. Test Y Boundary Invalidation
        ChunkCoord cy1(0, 1, 0);
        mgr.load_chunk(cy1);
        CHECK(mgr.get_mesh(c0)->face_count() == isolated_face_count - 1);
        CHECK(mgr.get_mesh(cy1)->face_count() == isolated_face_count - 1);
        mgr.unload_chunk(cy1);
        CHECK(mgr.get_mesh(c0)->face_count() == isolated_face_count);

        // 3. Test Z Boundary Invalidation
        ChunkCoord cz1(0, 0, 1);
        mgr.load_chunk(cz1);
        CHECK(mgr.get_mesh(c0)->face_count() == isolated_face_count - 1);
        CHECK(mgr.get_mesh(cz1)->face_count() == isolated_face_count - 1);
        mgr.unload_chunk(cz1);
        CHECK(mgr.get_mesh(c0)->face_count() == isolated_face_count);

        // 4. Test Negative Coordinates Invalidation
        ChunkCoord c_neg(-1, 0, 0);
        mgr.load_chunk(c_neg);
        CHECK(mgr.get_mesh(c0)->face_count() == isolated_face_count - 1);
        CHECK(mgr.get_mesh(c_neg)->face_count() == isolated_face_count - 1);
        mgr.unload_chunk(c_neg);
        CHECK(mgr.get_mesh(c0)->face_count() == isolated_face_count);
    }

    std::cout << "[PASS] test_neighbor_mesh_invalidation\n";
}

void test_no_duplicate_loads() {
    StreamingConfig config;
    config.load_radius = 1;
    config.unload_radius = 2;

    ChunkManager mgr(config, MesherType::Naive);

    // Repeated updates at the exact same position
    for (int i = 0; i < 10; ++i) {
        mgr.update_streaming(Vec3(0.0f, 0.0f, 0.0f));
    }

    CHECK(mgr.loaded_chunk_count() == 27);
    auto coords = mgr.get_loaded_chunk_coordinates();
    CHECK(coords.size() == 27);

    std::set<ChunkCoord> unique_coords(coords.begin(), coords.end());
    CHECK(unique_coords.size() == 27);

    // Cumulative loaded metric should be exactly 27, not 270!
    CHECK(mgr.get_metrics().total_chunks_loaded == 27);

    std::cout << "[PASS] test_no_duplicate_loads\n";
}

void test_deterministic_loaded_set() {
    StreamingConfig config;
    config.load_radius = 1;
    config.unload_radius = 2;

    ChunkManager mgr1(config, MesherType::Naive);
    ChunkManager mgr2(config, MesherType::Naive);

    const Vec3 path[] = {
        Vec3(0.0f, 0.0f, 0.0f),
        Vec3(32.0f, 0.0f, 0.0f),
        Vec3(64.0f, 32.0f, 0.0f),
        Vec3(64.0f, 32.0f, -64.0f),
        Vec3(-32.0f, -32.0f, 32.0f)
    };

    for (const auto& pos : path) {
        mgr1.update_streaming(pos);
        mgr2.update_streaming(pos);
    }

    CHECK(mgr1.loaded_chunk_count() == mgr2.loaded_chunk_count());
    auto coords1 = mgr1.get_loaded_chunk_coordinates();
    auto coords2 = mgr2.get_loaded_chunk_coordinates();

    std::set<ChunkCoord> set1(coords1.begin(), coords1.end());
    std::set<ChunkCoord> set2(coords2.begin(), coords2.end());
    CHECK(set1 == set2);

    std::cout << "[PASS] test_deterministic_loaded_set\n";
}

int main() {
    std::cout << "Running test_initial_load..." << std::endl;
    test_initial_load();
    std::cout << "Running test_movement_within_same_chunk..." << std::endl;
    test_movement_within_same_chunk();
    std::cout << "Running test_crossing_pos_x_boundary..." << std::endl;
    test_crossing_pos_x_boundary();
    std::cout << "Running test_crossing_neg_x_boundary..." << std::endl;
    test_crossing_neg_x_boundary();
    std::cout << "Running test_y_boundary..." << std::endl;
    test_y_boundary();
    std::cout << "Running test_z_boundary..." << std::endl;
    test_z_boundary();
    std::cout << "Running test_deterministic_reload..." << std::endl;
    test_deterministic_reload();
    std::cout << "Running test_hysteresis..." << std::endl;
    test_hysteresis();
    std::cout << "Running test_missing_chunk_semantics..." << std::endl;
    test_missing_chunk_semantics();
    std::cout << "Running test_neighbor_mesh_invalidation..." << std::endl;
    test_neighbor_mesh_invalidation();
    std::cout << "Running test_no_duplicate_loads..." << std::endl;
    test_no_duplicate_loads();
    std::cout << "Running test_deterministic_loaded_set..." << std::endl;
    test_deterministic_loaded_set();

    std::cout << "All chunk manager unit tests passed successfully!" << std::endl;
    return 0;
}
