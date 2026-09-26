#include "voxel_lab/chunk_manager.hpp"
#include "voxel_lab/naive_mesher.hpp"
#include "voxel_lab/greedy_mesher.hpp"
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

// Test 1: Chunk storage invariants remain unchanged
void test_chunk_invariants() {
    static_assert(sizeof(Voxel) == 2, "Voxel must be exactly 2 bytes");
    static_assert(sizeof(Chunk) == 65536, "Chunk must be contiguous 32^3 65536 bytes");
    static_assert(sizeof(MeshVertex) == 24, "MeshVertex must be exactly 24 bytes");

    Chunk c;
    c.fill(Voxel(1, 0));
    CHECK(c.get_voxel(0, 0, 0) == Voxel(1, 0));
    CHECK(c.get_voxel(31, 31, 31) == Voxel(1, 0));

    std::cout << "[PASS] test_chunk_invariants\n";
}

// Test 2: MeshData reuse preserves exact output
void test_mesh_data_reuse_exactness() {
    WorldGrid world;
    ChunkCoord c0(0, 0, 0);
    generate_sphere_world(world, WorldCoord(16, 16, 16), 12);

    // 1. Fresh naive mesh
    MeshData fresh_naive;
    mesh_chunk(world, c0, fresh_naive);
    CHECK(fresh_naive.face_count() > 0);

    // 2. Clear and reuse for naive mesh
    MeshData reused_naive = fresh_naive; // has allocated capacity
    reused_naive.clear();
    CHECK(reused_naive.vertex_count() == 0);
    CHECK(reused_naive.index_count() == 0);
    CHECK(reused_naive.vertices.capacity() >= fresh_naive.vertex_count());
    mesh_chunk(world, c0, reused_naive);
    CHECK(reused_naive == fresh_naive);

    // 3. Fresh greedy mesh
    MeshData fresh_greedy;
    greedy_mesh_chunk(world, c0, fresh_greedy);
    CHECK(fresh_greedy.face_count() > 0);

    // 4. Clear and reuse for greedy mesh
    MeshData reused_greedy = fresh_greedy;
    reused_greedy.clear();
    CHECK(reused_greedy.vertex_count() == 0);
    CHECK(reused_greedy.index_count() == 0);
    CHECK(reused_greedy.vertices.capacity() >= fresh_greedy.vertex_count());
    greedy_mesh_chunk(world, c0, reused_greedy);
    CHECK(reused_greedy == fresh_greedy);

    std::cout << "[PASS] test_mesh_data_reuse_exactness\n";
}

// Test 3: Reused worker buffers do not leak previous voxel/mesh data
void test_buffer_no_data_leakage() {
    WorldGrid world;
    ChunkCoord c_solid(0, 0, 0);
    ChunkCoord c_empty(1, 0, 0);
    generate_solid_world(world, WorldCoord(0, 0, 0), WorldCoord(31, 31, 31), Voxel(1, 0));

    // Mesh solid chunk into buffer
    MeshData buf;
    mesh_chunk(world, c_solid, buf);
    CHECK(buf.face_count() == 6144);

    // Clear and mesh empty chunk into the same buffer
    buf.clear();
    mesh_chunk(world, c_empty, buf);
    CHECK(buf.face_count() == 0);
    CHECK(buf.vertex_count() == 0);
    CHECK(buf.index_count() == 0);

    // Now test with Greedy mesher
    greedy_mesh_chunk(world, c_solid, buf);
    CHECK(buf.face_count() == 6); // Collapses to 6 quads

    buf.clear();
    greedy_mesh_chunk(world, c_empty, buf);
    CHECK(buf.face_count() == 0);
    CHECK(buf.vertex_count() == 0);
    CHECK(buf.index_count() == 0);

    std::cout << "[PASS] test_buffer_no_data_leakage\n";
}

// Test 4: Repeated chunk load/unload leaves no stale world state
void test_repeated_load_unload_state_clean() {
    StreamingConfig cfg;
    cfg.load_radius = 1;
    cfg.unload_radius = 1;
    cfg.worker_count = 0; // Synchronous
    cfg.enable_mesh_buffer_reuse = true;

    ChunkManager mgr(cfg, MesherType::Naive);

    // Load and unload chunk 50 times
    ChunkCoord c(5, 5, 5);
    for (int i = 0; i < 50; ++i) {
        bool loaded = mgr.load_chunk(c);
        CHECK(loaded);
        CHECK(mgr.is_loaded(c));
        CHECK(mgr.get_mesh(c) != nullptr);

        bool unloaded = mgr.unload_chunk(c);
        CHECK(unloaded);
        CHECK(!mgr.is_loaded(c));
        CHECK(mgr.get_mesh(c) == nullptr);
        CHECK(!mgr.get_world().has_chunk(c));
    }

    CHECK(mgr.loaded_chunk_count() == 0);
    CHECK(mgr.get_recycled_mesh_buffer_count() > 0);

    std::cout << "[PASS] test_repeated_load_unload_state_clean\n";
}

// Test 5: Repeated mesh replacement leaves correct current mesh
void test_repeated_mesh_replacement() {
    StreamingConfig cfg;
    cfg.worker_count = 0;
    cfg.enable_mesh_buffer_reuse = true;

    ChunkManager mgr(cfg, MesherType::Naive);
    ChunkCoord c(0, 0, 0);
    mgr.load_chunk(c);

    const MeshData* naive_m = mgr.get_mesh(c);
    CHECK(naive_m != nullptr);
    size_t naive_faces = naive_m->face_count();

    // Switch mesher to Greedy (triggers remesh and buffer recycling)
    mgr.set_mesher_type(MesherType::Greedy);
    const MeshData* greedy_m = mgr.get_mesh(c);
    CHECK(greedy_m != nullptr);
    size_t greedy_faces = greedy_m->face_count();
    CHECK(greedy_faces < naive_faces);

    // Switch back to Naive
    mgr.set_mesher_type(MesherType::Naive);
    const MeshData* restored_naive_m = mgr.get_mesh(c);
    CHECK(restored_naive_m != nullptr);
    CHECK(restored_naive_m->face_count() == naive_faces);

    // Verify buffer recycling happened
    const auto& metrics = mgr.get_metrics();
    CHECK(metrics.mesh_buffers_reused > 0);

    std::cout << "[PASS] test_repeated_mesh_replacement\n";
}

// Test 6: Multiple workers can reuse their own buffers safely
void test_multithreaded_buffer_reuse_safety() {
    StreamingConfig cfg;
    cfg.load_radius = 2; // 125 chunks
    cfg.unload_radius = 3;
    cfg.worker_count = 4;
    cfg.enable_mesh_buffer_reuse = true;

    ChunkManager mgr(cfg, MesherType::Greedy);
    mgr.update_streaming(Vec3(0.0f, 0.0f, 0.0f));
    mgr.wait_all_pending();
    CHECK(mgr.loaded_chunk_count() == 125);

    // Move camera to trigger unloads and loads with 4 active worker threads
    mgr.update_streaming(Vec3(32.0f, 0.0f, 0.0f));
    mgr.wait_all_pending();
    CHECK(mgr.loaded_chunk_count() == 150);

    // Move again to cause unloads
    mgr.update_streaming(Vec3(64.0f, 0.0f, 0.0f));
    mgr.wait_all_pending();
    CHECK(mgr.loaded_chunk_count() == 150);

    const auto& metrics = mgr.get_metrics();
    CHECK(metrics.mesh_buffers_reused > 0);
    CHECK(metrics.mesh_buffers_recycled > 0);

    // Verify all resident meshes are valid and non-empty
    for (const auto& c : mgr.get_loaded_chunk_coordinates()) {
        const MeshData* m = mgr.get_mesh(c);
        CHECK(m != nullptr);
        CHECK(m->vertex_count() == m->face_count() * 4);
        CHECK(m->index_count() == m->face_count() * 6);
    }

    std::cout << "[PASS] test_multithreaded_buffer_reuse_safety\n";
}

// Test 7: Deterministic output remains identical after reuse
void test_deterministic_output_after_reuse() {
    StreamingConfig cfg_no_reuse;
    cfg_no_reuse.load_radius = 1; // 27 chunks
    cfg_no_reuse.unload_radius = 2;
    cfg_no_reuse.worker_count = 0;
    cfg_no_reuse.enable_mesh_buffer_reuse = false;

    StreamingConfig cfg_reuse = cfg_no_reuse;
    cfg_reuse.enable_mesh_buffer_reuse = true;

    // Run baseline (no reuse)
    ChunkManager mgr_baseline(cfg_no_reuse, MesherType::Greedy);
    mgr_baseline.update_streaming(Vec3(0.0f, 0.0f, 0.0f));

    // Run with reuse
    ChunkManager mgr_optimized(cfg_reuse, MesherType::Greedy);
    // Cycle a few times first to populate recycled buffers
    mgr_optimized.load_chunk(ChunkCoord(10, 10, 10));
    mgr_optimized.unload_chunk(ChunkCoord(10, 10, 10));
    mgr_optimized.load_chunk(ChunkCoord(11, 11, 11));
    mgr_optimized.unload_chunk(ChunkCoord(11, 11, 11));

    mgr_optimized.update_streaming(Vec3(0.0f, 0.0f, 0.0f));

    // Compare all 27 meshes for exact equality
    CHECK(mgr_baseline.loaded_chunk_count() == 27);
    CHECK(mgr_optimized.loaded_chunk_count() == 27);

    for (const auto& c : mgr_baseline.get_loaded_chunk_coordinates()) {
        const MeshData* m_base = mgr_baseline.get_mesh(c);
        const MeshData* m_opt = mgr_optimized.get_mesh(c);
        CHECK(m_base != nullptr);
        CHECK(m_opt != nullptr);
        CHECK(*m_base == *m_opt); // 100% exact vertex and index match
    }

    std::cout << "[PASS] test_deterministic_output_after_reuse\n";
}

// Test 8: Streaming behavior remains functionally unchanged
void test_streaming_behavior_unchanged() {
    StreamingConfig cfg;
    cfg.load_radius = 2;
    cfg.unload_radius = 3;
    cfg.worker_count = 2;
    cfg.enable_mesh_buffer_reuse = true;

    ChunkManager mgr(cfg, MesherType::Naive);
    mgr.update_streaming(Vec3(0.0f, 0.0f, 0.0f));
    mgr.wait_all_pending();
    CHECK(mgr.loaded_chunk_count() == 125);

    // Same chunk move -> no change
    bool changed = mgr.update_streaming(Vec3(5.0f, 5.0f, 5.0f));
    CHECK(!changed);
    CHECK(mgr.loaded_chunk_count() == 125);

    // Cross boundary
    changed = mgr.update_streaming(Vec3(32.0f, 0.0f, 0.0f));
    CHECK(changed);
    mgr.wait_all_pending();
    CHECK(mgr.loaded_chunk_count() == 150);

    std::cout << "[PASS] test_streaming_behavior_unchanged\n";
}

// Test 9: Negative coordinates remain correct
void test_negative_coordinates_correct() {
    StreamingConfig cfg;
    cfg.load_radius = 1;
    cfg.unload_radius = 2;
    cfg.worker_count = 2;
    cfg.enable_mesh_buffer_reuse = true;

    ChunkManager mgr(cfg, MesherType::Greedy);
    mgr.update_streaming(Vec3(-64.0f, -32.0f, -64.0f));
    mgr.wait_all_pending();

    CHECK(mgr.has_camera_chunk());
    CHECK(mgr.get_camera_chunk() == ChunkCoord(-2, -1, -2));
    CHECK(mgr.loaded_chunk_count() == 27);

    for (const auto& c : mgr.get_loaded_chunk_coordinates()) {
        const MeshData* m = mgr.get_mesh(c);
        CHECK(m != nullptr);
    }

    std::cout << "[PASS] test_negative_coordinates_correct\n";
}

// Test 10: Existing stale-result protection still works with reuse
void test_stale_result_protection_with_reuse() {
    StreamingConfig cfg;
    cfg.load_radius = 2;
    cfg.unload_radius = 3;
    cfg.worker_count = 4;
    cfg.enable_mesh_buffer_reuse = true;

    ChunkManager mgr(cfg, MesherType::Naive);

    // Initial position at (0, 0, 0)
    mgr.update_streaming(Vec3(0.0f, 0.0f, 0.0f));

    // Immediately move camera far away before workers complete!
    // Teleport to (3200, 3200, 3200) -> chunk (100, 100, 100)
    mgr.update_streaming(Vec3(3200.0f, 3200.0f, 3200.0f));
    mgr.wait_all_pending();

    // Chunks around (0,0,0) must NOT be resident or resurrected
    CHECK(!mgr.is_loaded(ChunkCoord(0, 0, 0)));
    CHECK(!mgr.is_loaded(ChunkCoord(1, 0, 0)));
    CHECK(!mgr.is_loaded(ChunkCoord(-1, 0, 0)));

    // Only chunks around (100, 100, 100) should be loaded
    CHECK(mgr.is_loaded(ChunkCoord(100, 100, 100)));

    const auto& metrics = mgr.get_metrics();
    CHECK(metrics.jobs_discarded_stale > 0);
    // Discarded stale jobs must have had their buffers safely recycled
    CHECK(mgr.get_recycled_mesh_buffer_count() > 0);

    std::cout << "[PASS] test_stale_result_protection_with_reuse\n";
}

// Test 11: Shutdown with reusable buffers/pools does not leak or deadlock
void test_shutdown_with_reusable_pools() {
    {
        StreamingConfig cfg;
        cfg.load_radius = 2;
        cfg.unload_radius = 3;
        cfg.worker_count = 4;
        cfg.enable_mesh_buffer_reuse = true;

        ChunkManager mgr(cfg, MesherType::Greedy);
        mgr.update_streaming(Vec3(0.0f, 0.0f, 0.0f));
        // Destroy manager while pool has active and recycled buffers
    }

    std::cout << "[PASS] test_shutdown_with_reusable_pools\n";
}

// Test 12: Memory stats reporting logic
void test_memory_stats_reporting() {
    StreamingConfig cfg;
    cfg.load_radius = 1;
    cfg.unload_radius = 2;
    cfg.worker_count = 0;
    cfg.enable_mesh_buffer_reuse = true;

    ChunkManager mgr(cfg, MesherType::Naive);
    mgr.update_streaming(Vec3(0.0f, 0.0f, 0.0f));

    ChunkManagerMemoryStats stats = mgr.get_memory_stats();
    CHECK(stats.resident_chunk_count == 27);
    CHECK(stats.raw_chunk_payload_bytes == 27 * 65536);
    CHECK(stats.mesh_count == 27);
    CHECK(stats.total_mesh_logical_bytes > 0);
    CHECK(stats.total_mesh_capacity_bytes >= stats.total_mesh_logical_bytes);
    CHECK(stats.estimated_total_logical_bytes == stats.raw_chunk_payload_bytes + stats.total_mesh_logical_bytes);

    std::cout << "[PASS] test_memory_stats_reporting\n";
}

int main() {
    std::cout << "=== Running Milestone 9 Memory Optimization Test Suite ===\n";
    test_chunk_invariants();
    test_mesh_data_reuse_exactness();
    test_buffer_no_data_leakage();
    test_repeated_load_unload_state_clean();
    test_repeated_mesh_replacement();
    test_multithreaded_buffer_reuse_safety();
    test_deterministic_output_after_reuse();
    test_streaming_behavior_unchanged();
    test_negative_coordinates_correct();
    test_stale_result_protection_with_reuse();
    test_shutdown_with_reusable_pools();
    test_memory_stats_reporting();
    std::cout << "All Milestone 9 memory optimization unit tests passed successfully!\n";
    return 0;
}
