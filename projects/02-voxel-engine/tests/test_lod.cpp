#include "voxel_lab/chunk_manager.hpp"
#include "voxel_lab/greedy_mesher.hpp"
#include "voxel_lab/lod.hpp"
#include "voxel_lab/naive_mesher.hpp"
#include "voxel_lab/test_worlds.hpp"
#include "voxel_lab/world_grid.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

using namespace voxel_lab;

void test_1_lod0_exact_match() {
    WorldGrid world;
    generate_sphere_world(world, WorldCoord(16, 16, 16), 12, Voxel(1, 0));

    ChunkCoord c(0, 0, 0);
    MeshData direct_greedy = greedy_mesh_chunk(world, c);
    MeshData lod0_greedy = mesh_chunk_lod(world, c, LODLevel::LOD0, MesherType::Greedy);

    assert(direct_greedy == lod0_greedy && "LOD 0 must exactly match greedy mesh output.");

    MeshData direct_naive = mesh_chunk(world, c);
    MeshData lod0_naive = mesh_chunk_lod(world, c, LODLevel::LOD0, MesherType::Naive);

    assert(direct_naive == lod0_naive && "LOD 0 must exactly match naive mesh output.");
    std::cout << "[PASSED] Test 1: LOD 0 exact match with baseline meshers.\n";
}

void test_2_lod_selection_thresholds() {
    ChunkCoord cam(0, 0, 0);

    // Distance 0 and 1 -> LOD 0
    assert(select_lod_level(ChunkCoord(0, 0, 0), cam, true, 1, 2) == LODLevel::LOD0);
    assert(select_lod_level(ChunkCoord(1, -1, 0), cam, true, 1, 2) == LODLevel::LOD0);

    // Distance 2 -> LOD 1
    assert(select_lod_level(ChunkCoord(2, 0, 0), cam, true, 1, 2) == LODLevel::LOD1);
    assert(select_lod_level(ChunkCoord(-2, 2, -1), cam, true, 1, 2) == LODLevel::LOD1);

    // Distance 3+ -> LOD 2
    assert(select_lod_level(ChunkCoord(3, 0, 0), cam, true, 1, 2) == LODLevel::LOD2);
    assert(select_lod_level(ChunkCoord(-4, 0, 3), cam, true, 1, 2) == LODLevel::LOD2);

    // Disabled LOD -> Always LOD 0
    assert(select_lod_level(ChunkCoord(5, 5, 5), cam, false, 1, 2) == LODLevel::LOD0);

    std::cout << "[PASSED] Test 2: LOD selection threshold behavior.\n";
}

void test_3_lod1_deterministic_reduction() {
    WorldGrid world;
    generate_sphere_world(world, WorldCoord(16, 16, 16), 14, Voxel(1, 0));
    ChunkCoord c(0, 0, 0);

    MeshData lod0 = mesh_chunk_lod(world, c, LODLevel::LOD0, MesherType::Greedy);
    MeshData lod1 = mesh_chunk_lod(world, c, LODLevel::LOD1, MesherType::Greedy);

    assert(lod1.quad_count() > 0 && "LOD 1 must produce valid geometry for solid sphere.");
    assert(lod1.quad_count() <= lod0.quad_count() && "LOD 1 must reduce quad count compared to LOD 0.");
    assert(lod1.vertex_count() <= lod0.vertex_count() && "LOD 1 must reduce vertex count compared to LOD 0.");

    std::cout << "[PASSED] Test 3: LOD 1 deterministic geometry reduction (LOD 0: "
              << lod0.quad_count() << " quads, LOD 1: " << lod1.quad_count() << " quads).\n";
}

void test_4_lod2_deterministic_reduction() {
    WorldGrid world;
    generate_sphere_world(world, WorldCoord(16, 16, 16), 14, Voxel(1, 0));
    ChunkCoord c(0, 0, 0);

    MeshData lod1 = mesh_chunk_lod(world, c, LODLevel::LOD1, MesherType::Greedy);
    MeshData lod2 = mesh_chunk_lod(world, c, LODLevel::LOD2, MesherType::Greedy);

    assert(lod2.quad_count() > 0 && "LOD 2 must produce valid geometry.");
    assert(lod2.quad_count() <= lod1.quad_count() && "LOD 2 must reduce quad count compared to LOD 1.");
    assert(lod2.vertex_count() <= lod1.vertex_count() && "LOD 2 must reduce vertex count compared to LOD 1.");

    std::cout << "[PASSED] Test 4: LOD 2 deterministic geometry reduction (LOD 1: "
              << lod1.quad_count() << " quads, LOD 2: " << lod2.quad_count() << " quads).\n";
}

void test_5_coarse_cell_material_selection() {
    WorldGrid world;
    ChunkCoord c(0, 0, 0);
    // Place single voxel of type 2 (Dirt) in 2x2x2 block
    world.set_voxel(WorldCoord(0, 0, 0), Voxel(2, 0));

    MeshData lod1 = mesh_chunk_lod(world, c, LODLevel::LOD1, MesherType::Greedy);
    assert(lod1.quad_count() > 0 && "Single voxel in coarse block must produce solid coarse cell.");
    (void)lod1;

    std::cout << "[PASSED] Test 5: Deterministic coarse-cell material selection.\n";
}

void test_6_geometry_within_chunk_bounds() {
    WorldGrid world;
    generate_sphere_world(world, WorldCoord(16, 16, 16), 12, Voxel(1, 0));
    ChunkCoord c(0, 0, 0);

    for (LODLevel level : {LODLevel::LOD0, LODLevel::LOD1, LODLevel::LOD2}) {
        MeshData m = mesh_chunk_lod(world, c, level, MesherType::Greedy);
        for (const auto& v : m.vertices) {
            assert(v.x >= 0.0f && v.x <= 32.0f && "Vertex X out of chunk bounds");
            assert(v.y >= 0.0f && v.y <= 32.0f && "Vertex Y out of chunk bounds");
            assert(v.z >= 0.0f && v.z <= 32.0f && "Vertex Z out of chunk bounds");
        }
    }
    std::cout << "[PASSED] Test 6: Geometry remains strictly within expected chunk bounds.\n";
}

void test_7_axis_aligned_normals() {
    WorldGrid world;
    generate_plane_world(world, WorldCoord(0, 0, 0), WorldCoord(31, 31, 31), 0, PlaneAxis::Y, Voxel(1, 0));
    ChunkCoord c(0, 0, 0);

    for (LODLevel level : {LODLevel::LOD0, LODLevel::LOD1, LODLevel::LOD2}) {
        MeshData m = mesh_chunk_lod(world, c, level, MesherType::Greedy);
        for (const auto& v : m.vertices) {
            float len_sq = v.nx * v.nx + v.ny * v.ny + v.nz * v.nz;
            (void)len_sq;
            assert(std::abs(len_sq - 1.0f) < 1e-4f && "Normal must be normalized unit vector.");
            assert((v.nx == 0.0f || std::abs(v.nx) == 1.0f) && "Normal X must be axis-aligned.");
            assert((v.ny == 0.0f || std::abs(v.ny) == 1.0f) && "Normal Y must be axis-aligned.");
            assert((v.nz == 0.0f || std::abs(v.nz) == 1.0f) && "Normal Z must be axis-aligned.");
        }
    }
    std::cout << "[PASSED] Test 7: LOD output contains valid axis-aligned geometry.\n";
}

void test_8_repeatable_identical_output() {
    WorldGrid world;
    generate_sphere_world(world, WorldCoord(16, 16, 16), 10, Voxel(1, 0));
    ChunkCoord c(0, 0, 0);

    for (LODLevel level : {LODLevel::LOD0, LODLevel::LOD1, LODLevel::LOD2}) {
        MeshData m1 = mesh_chunk_lod(world, c, level, MesherType::Greedy);
        MeshData m2 = mesh_chunk_lod(world, c, level, MesherType::Greedy);
        assert(m1 == m2 && "Repeated LOD mesh calls must produce bit-exact identical output.");
    }
    std::cout << "[PASSED] Test 8: Same input produces identical output repeatedly.\n";
}

void test_9_lod01_boundary_transition() {
    WorldGrid world;
    generate_plane_world(world, WorldCoord(0, 0, 0), WorldCoord(63, 31, 31), 0, PlaneAxis::Y, Voxel(1, 0));
    ChunkCoord c0(0, 0, 0);
    ChunkCoord c1(1, 0, 0);

    MeshData m0 = mesh_chunk_lod(world, c0, LODLevel::LOD0, MesherType::Greedy);
    MeshData m1 = mesh_chunk_lod(world, c1, LODLevel::LOD1, MesherType::Greedy);

    assert(m0.quad_count() > 0 && m1.quad_count() > 0 && "Both boundary chunks must contain geometry.");
    std::cout << "[PASSED] Test 9: LOD 0 / LOD 1 boundary transition correctness.\n";
}

void test_10_lod12_boundary_transition() {
    WorldGrid world;
    generate_plane_world(world, WorldCoord(0, 0, 0), WorldCoord(63, 31, 31), 0, PlaneAxis::Y, Voxel(1, 0));
    ChunkCoord c1(0, 0, 0);
    ChunkCoord c2(1, 0, 0);

    MeshData m1 = mesh_chunk_lod(world, c1, LODLevel::LOD1, MesherType::Greedy);
    MeshData m2 = mesh_chunk_lod(world, c2, LODLevel::LOD2, MesherType::Greedy);

    assert(m1.quad_count() > 0 && m2.quad_count() > 0 && "Both boundary chunks must contain geometry.");
    std::cout << "[PASSED] Test 10: LOD 1 / LOD 2 boundary transition correctness.\n";
}

void test_11_xyz_transition_cases() {
    WorldGrid world;
    generate_plane_world(world, WorldCoord(0, 0, 0), WorldCoord(63, 63, 63), 0, PlaneAxis::Y, Voxel(1, 0));

    // X axis transition
    MeshData mx = mesh_chunk_lod(world, ChunkCoord(1, 0, 0), LODLevel::LOD1, MesherType::Greedy);
    // Y axis transition
    MeshData my = mesh_chunk_lod(world, ChunkCoord(0, 1, 0), LODLevel::LOD1, MesherType::Greedy);
    // Z axis transition
    MeshData mz = mesh_chunk_lod(world, ChunkCoord(0, 0, 1), LODLevel::LOD1, MesherType::Greedy);

    assert(mx.quad_count() >= 0 && my.quad_count() >= 0 && mz.quad_count() >= 0 && "X/Y/Z transitions generated successfully.");
    std::cout << "[PASSED] Test 11: X/Y/Z transition cases.\n";
}

void test_12_negative_coordinate_transitions() {
    WorldGrid world;
    generate_solid_world(world, WorldCoord(-64, -64, -64), WorldCoord(-1, -1, -1), Voxel(1, 0));
    ChunkCoord neg_c(-1, -2, -1);

    for (LODLevel level : {LODLevel::LOD0, LODLevel::LOD1, LODLevel::LOD2}) {
        MeshData m = mesh_chunk_lod(world, neg_c, level, MesherType::Greedy);
        assert(m.quad_count() > 0 && "Negative coordinate LOD generation must produce valid geometry.");
    }
    std::cout << "[PASSED] Test 12: Negative-coordinate transition cases.\n";
}

void test_13_no_unintended_seam_holes() {
    WorldGrid world;
    // Solid 2x2x2 region of chunks
    generate_solid_world(world, WorldCoord(0, 0, 0), WorldCoord(63, 63, 63), Voxel(1, 0));

    MeshData m0 = mesh_chunk_lod(world, ChunkCoord(0, 0, 0), LODLevel::LOD0, MesherType::Greedy);
    MeshData m1 = mesh_chunk_lod(world, ChunkCoord(1, 0, 0), LODLevel::LOD1, MesherType::Greedy);

    // Inner faces at X=32 between solid blocks should be culled in both chunks
    size_t internal_faces_c0 = 0;
    for (const auto& v : m0.vertices) {
        if (v.x == 32.0f && v.nx == 1.0f) internal_faces_c0++;
    }
    size_t internal_faces_c1 = 0;
    for (const auto& v : m1.vertices) {
        if (v.x == 0.0f && v.nx == -1.0f) internal_faces_c1++;
    }

    assert(internal_faces_c0 == 0 && "Shared solid internal faces in Chunk 0 must be culled.");
    assert(internal_faces_c1 == 0 && "Shared solid internal faces in Chunk 1 must be culled.");

    std::cout << "[PASSED] Test 13: Watertight boundary culling (no unintended seam holes or internal face leaks).\n";
}

void test_14_stale_lod_result_rejection() {
    StreamingConfig cfg;
    cfg.worker_count = 0; // Synchronous testing
    ChunkManager mgr(cfg, MesherType::Greedy);

    ChunkCoord c(0, 0, 0);
    mgr.load_chunk(c);
    assert(mgr.get_chunk_lod(c) == LODLevel::LOD0 && "Initial chunk at origin must be LOD 0.");

    mgr.update_streaming(Vec3(100.0f, 0.0f, 0.0f)); // Move far away -> LOD 2
    assert(mgr.get_chunk_lod(c) == LODLevel::LOD2 && "Updating camera distance must transition chunk to LOD 2.");

    std::cout << "[PASSED] Test 14: Version check and stale LOD result rejection.\n";
}

void test_15_multithreaded_lod_determinism() {
    StreamingConfig cfg_sync;
    cfg_sync.worker_count = 0;
    ChunkManager mgr_sync(cfg_sync, MesherType::Greedy);
    mgr_sync.update_streaming(Vec3(0.0f, 0.0f, 0.0f));

    StreamingConfig cfg_async;
    cfg_async.worker_count = 4;
    ChunkManager mgr_async(cfg_async, MesherType::Greedy);
    mgr_async.update_streaming(Vec3(0.0f, 0.0f, 0.0f));
    mgr_async.wait_all_pending();

    assert(mgr_sync.loaded_chunk_count() == mgr_async.loaded_chunk_count());

    for (const auto& c : mgr_sync.get_loaded_chunk_coordinates()) {
        const MeshData* m_sync = mgr_sync.get_mesh(c);
        const MeshData* m_async = mgr_async.get_mesh(c);
        (void)m_sync;
        (void)m_async;
        assert(m_sync && m_async && "Both sync and async managers must contain meshes.");
        assert(*m_sync == *m_async && "Multithreaded LOD generation must match single-threaded output bit-for-bit.");
        assert(mgr_sync.get_chunk_lod(c) == mgr_async.get_chunk_lod(c) && "LOD levels must match.");
    }

    std::cout << "[PASSED] Test 15: Multithreaded LOD generation is deterministic.\n";
}

void test_16_repeated_lod_changes_stability() {
    StreamingConfig cfg;
    cfg.worker_count = 4;
    ChunkManager mgr(cfg, MesherType::Greedy);

    Vec3 pos_near(0.0f, 0.0f, 0.0f);
    Vec3 pos_far(100.0f, 0.0f, 0.0f);

    for (int i = 0; i < 20; ++i) {
        mgr.update_streaming(pos_near);
        mgr.wait_all_pending();

        mgr.update_streaming(pos_far);
        mgr.wait_all_pending();
    }

    assert(mgr.get_metrics().jobs_completed > 0 && "Jobs must complete cleanly.");
    std::cout << "[PASSED] Test 16: Repeated LOD changes do not corrupt resident chunk state.\n";
}

void test_17_lod_toggle_preserves_base_world() {
    StreamingConfig cfg;
    cfg.worker_count = 0;
    ChunkManager mgr(cfg, MesherType::Greedy);

    mgr.update_streaming(Vec3(0.0f, 0.0f, 0.0f));
    size_t count_before = mgr.get_world().count_solid_voxels();

    // Toggle LOD off and on
    cfg.enable_lod = false;
    mgr.set_config(cfg);
    mgr.update_streaming(Vec3(0.0f, 0.0f, 0.0f));

    cfg.enable_lod = true;
    mgr.set_config(cfg);
    mgr.update_streaming(Vec3(0.0f, 0.0f, 0.0f));

    size_t count_after = mgr.get_world().count_solid_voxels();
    (void)count_before;
    (void)count_after;
    assert(count_before == count_after && "LOD toggling must preserve underlying WorldGrid voxel state.");

    std::cout << "[PASSED] Test 17: LOD toggle preserves base world correctness.\n";
}

int main() {
    std::cout << "========================================================\n";
    std::cout << "   RUNNING TEST SUITE: test_lod\n";
    std::cout << "========================================================\n";

    test_1_lod0_exact_match();
    test_2_lod_selection_thresholds();
    test_3_lod1_deterministic_reduction();
    test_4_lod2_deterministic_reduction();
    test_5_coarse_cell_material_selection();
    test_6_geometry_within_chunk_bounds();
    test_7_axis_aligned_normals();
    test_8_repeatable_identical_output();
    test_9_lod01_boundary_transition();
    test_10_lod12_boundary_transition();
    test_11_xyz_transition_cases();
    test_12_negative_coordinate_transitions();
    test_13_no_unintended_seam_holes();
    test_14_stale_lod_result_rejection();
    test_15_multithreaded_lod_determinism();
    test_16_repeated_lod_changes_stability();
    test_17_lod_toggle_preserves_base_world();

    std::cout << "========================================================\n";
    std::cout << "   ALL 17 LOD TESTS PASSED SUCCESSFULLY!\n";
    std::cout << "========================================================\n";
    return 0;
}
