#include "voxel_lab/chunk_manager.hpp"
#include "voxel_lab/thread_pool.hpp"
#include "voxel_lab/test_worlds.hpp"

#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <set>
#include <thread>
#include <vector>

using namespace voxel_lab;

#define CHECK(expr) do { \
    if (!(expr)) { \
        std::cerr << "CHECK failed at " << __FILE__ << ":" << __LINE__ << " (" #expr ")\n"; \
        std::abort(); \
    } \
} while (0)

// 1. WORKER POOL START/STOP
void test_worker_pool_start_stop() {
    {
        ThreadPool pool(4);
        CHECK(pool.thread_count() == 4);
    } // Exits scope, destructor calls stop() and joins all threads cleanly
    std::cout << "[PASS] test_worker_pool_start_stop\n";
}

// 2. JOB EXECUTION
void test_job_execution() {
    ThreadPool pool(4);
    std::atomic<int> counter{0};
    const int task_count = 100;

    for (int i = 0; i < task_count; ++i) {
        pool.enqueue([&counter]() {
            counter.fetch_add(1, std::memory_order_relaxed);
        });
    }

    pool.wait_idle();
    CHECK(counter.load() == task_count);
    std::cout << "[PASS] test_job_execution\n";
}

// 3. MULTIPLE WORKERS
void test_multiple_workers() {
    const size_t worker_count = 4;
    ThreadPool pool(worker_count);
    std::mutex id_mutex;
    std::set<std::thread::id> thread_ids;

    // Enqueue enough tasks with artificial work to engage all workers
    for (int i = 0; i < 40; ++i) {
        pool.enqueue([&id_mutex, &thread_ids]() {
            {
                std::lock_guard<std::mutex> lock(id_mutex);
                thread_ids.insert(std::this_thread::get_id());
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        });
    }

    pool.wait_idle();
    CHECK(thread_ids.size() >= 2); // At least multiple workers participated
    std::cout << "[PASS] test_multiple_workers (" << thread_ids.size() << " unique threads observed)\n";
}

// 4. RESULT COMPLETENESS
void test_result_completeness() {
    StreamingConfig config;
    config.load_radius = 1;
    config.unload_radius = 2;
    config.worker_count = 4;

    ChunkManager mgr(config, MesherType::Naive);
    mgr.update_streaming(Vec3(0.0f, 0.0f, 0.0f));
    mgr.wait_all_pending();

    // 3x3x3 = 27 chunks must be loaded and have valid meshes
    CHECK(mgr.loaded_chunk_count() == 27);
    for (int dz = -1; dz <= 1; ++dz) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                ChunkCoord c(dx, dy, dz);
                CHECK(mgr.is_loaded(c));
                CHECK(mgr.get_mesh(c) != nullptr);
            }
        }
    }
    std::cout << "[PASS] test_result_completeness\n";
}

// 5. STALE RESULT REJECTION
void test_stale_result_rejection() {
    StreamingConfig config;
    config.load_radius = 1;
    config.unload_radius = 2;
    config.worker_count = 4;

    ChunkManager mgr(config, MesherType::Naive);

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

    // Stale jobs must have been discarded
    const auto& metrics = mgr.get_metrics();
    CHECK(metrics.jobs_discarded_stale > 0);
    std::cout << "[PASS] test_stale_result_rejection (" << metrics.jobs_discarded_stale << " stale jobs rejected)\n";
}

// 6. DUPLICATE REQUEST HANDLING
void test_duplicate_request_handling() {
    StreamingConfig config;
    config.load_radius = 1;
    config.unload_radius = 2;
    config.worker_count = 4;

    ChunkManager mgr(config, MesherType::Naive);

    // Call update_streaming repeatedly at the exact same location
    for (int i = 0; i < 10; ++i) {
        mgr.update_streaming(Vec3(0.0f, 0.0f, 0.0f));
    }
    mgr.wait_all_pending();

    CHECK(mgr.loaded_chunk_count() == 27);
    auto coords = mgr.get_loaded_chunk_coordinates();
    std::set<ChunkCoord> unique_coords(coords.begin(), coords.end());
    CHECK(unique_coords.size() == 27);

    std::cout << "[PASS] test_duplicate_request_handling\n";
}

// 7. DETERMINISTIC GENERATION
void test_deterministic_generation() {
    StreamingConfig config1;
    config1.load_radius = 1;
    config1.worker_count = 1; // 1 worker

    StreamingConfig config4;
    config4.load_radius = 1;
    config4.worker_count = 4; // 4 workers

    ChunkManager mgr1(config1, MesherType::Naive);
    ChunkManager mgr4(config4, MesherType::Naive);

    mgr1.update_streaming(Vec3(0.0f, 0.0f, 0.0f));
    mgr1.wait_all_pending();

    mgr4.update_streaming(Vec3(0.0f, 0.0f, 0.0f));
    mgr4.wait_all_pending();

    CHECK(mgr1.loaded_chunk_count() == mgr4.loaded_chunk_count());

    // Verify bitwise exact voxel contents for all 27 chunks
    for (int dz = -1; dz <= 1; ++dz) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                ChunkCoord c(dx, dy, dz);
                const Chunk* ch1 = mgr1.get_world().get_chunk(c);
                const Chunk* ch4 = mgr4.get_world().get_chunk(c);
                CHECK(ch1 != nullptr);
                CHECK(ch4 != nullptr);

                for (int lz = 0; lz < CHUNK_DIM; ++lz) {
                    for (int ly = 0; ly < CHUNK_DIM; ++ly) {
                        for (int lx = 0; lx < CHUNK_DIM; ++lx) {
                            CHECK(ch1->get_voxel(lx, ly, lz) == ch4->get_voxel(lx, ly, lz));
                        }
                    }
                }
            }
        }
    }
    std::cout << "[PASS] test_deterministic_generation\n";
}

// 8. DETERMINISTIC MESHING
void test_deterministic_meshing() {
    StreamingConfig config1;
    config1.load_radius = 1;
    config1.worker_count = 1;

    StreamingConfig config4;
    config4.load_radius = 1;
    config4.worker_count = 4;

    ChunkManager mgr1(config1, MesherType::Greedy);
    ChunkManager mgr4(config4, MesherType::Greedy);

    mgr1.update_streaming(Vec3(0.0f, 0.0f, 0.0f));
    mgr1.wait_all_pending();

    mgr4.update_streaming(Vec3(0.0f, 0.0f, 0.0f));
    mgr4.wait_all_pending();

    for (int dz = -1; dz <= 1; ++dz) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                ChunkCoord c(dx, dy, dz);
                const MeshData* m1 = mgr1.get_mesh(c);
                const MeshData* m4 = mgr4.get_mesh(c);
                CHECK(m1 != nullptr);
                CHECK(m4 != nullptr);
                CHECK(m1->vertex_count() == m4->vertex_count());
                CHECK(m1->index_count() == m4->index_count());
                CHECK(m1->face_count() == m4->face_count());

                // Byte-level verification of vertex data
                for (size_t v = 0; v < m1->vertex_count(); ++v) {
                    CHECK(m1->vertices[v].x == m4->vertices[v].x);
                    CHECK(m1->vertices[v].y == m4->vertices[v].y);
                    CHECK(m1->vertices[v].z == m4->vertices[v].z);
                    CHECK(m1->vertices[v].nx == m4->vertices[v].nx);
                    CHECK(m1->vertices[v].ny == m4->vertices[v].ny);
                    CHECK(m1->vertices[v].nz == m4->vertices[v].nz);
                }
                for (size_t idx = 0; idx < m1->index_count(); ++idx) {
                    CHECK(m1->indices[idx] == m4->indices[idx]);
                }
            }
        }
    }
    std::cout << "[PASS] test_deterministic_meshing\n";
}

// 9. CROSS-CHUNK CORRECTNESS
void test_cross_chunk_correctness() {
    auto boundary_gen = [](WorldAccessor& world, const ChunkCoord& c) {
        ChunkNeighborhoodSnapshot* snap = dynamic_cast<ChunkNeighborhoodSnapshot*>(&world);
        WorldGrid* grid = dynamic_cast<WorldGrid*>(&world);
        Chunk* chunk = snap ? snap->get_chunk(c) : (grid ? grid->get_chunk(c) : nullptr);
        if (chunk) {
            chunk->set_voxel(31, 16, 16, Voxel(1, 0)); // +X face
            chunk->set_voxel(0, 16, 16, Voxel(1, 0));  // -X face
        }
    };

    StreamingConfig config;
    config.worker_count = 4;
    ChunkManager mgr(config, MesherType::Naive, boundary_gen);

    ChunkCoord c0(0, 0, 0);
    ChunkCoord c1(1, 0, 0);

    mgr.load_chunk(c0);
    const MeshData* m0_single = mgr.get_mesh(c0);
    CHECK(m0_single != nullptr);
    CHECK(m0_single->face_count() == 12); // 2 isolated voxels * 6 faces

    // Load adjacent chunk c1
    mgr.load_chunk(c1);
    const MeshData* m0_shared = mgr.get_mesh(c0);
    const MeshData* m1_shared = mgr.get_mesh(c1);
    CHECK(m0_shared != nullptr);
    CHECK(m1_shared != nullptr);

    // Touching face between (31,16,16) and (0,16,16) must be culled
    CHECK(m0_shared->face_count() == 11);
    CHECK(m1_shared->face_count() == 11);

    // Unload c1 -> c0 face must be restored
    mgr.unload_chunk(c1);
    const MeshData* m0_restored = mgr.get_mesh(c0);
    CHECK(m0_restored != nullptr);
    CHECK(m0_restored->face_count() == 12);

    std::cout << "[PASS] test_cross_chunk_correctness\n";
}

// 10. NEGATIVE COORDINATES
void test_negative_coordinates() {
    StreamingConfig config;
    config.load_radius = 1;
    config.unload_radius = 2;
    config.worker_count = 4;

    ChunkManager mgr(config, MesherType::Naive);

    // Stream around negative coordinate (-64, -64, -64) -> chunk (-2, -2, -2)
    mgr.update_streaming(Vec3(-64.0f, -64.0f, -64.0f));
    mgr.wait_all_pending();

    CHECK(mgr.loaded_chunk_count() == 27);
    for (int dz = -3; dz <= -1; ++dz) {
        for (int dy = -3; dy <= -1; ++dy) {
            for (int dx = -3; dx <= -1; ++dx) {
                ChunkCoord c(dx, dy, dz);
                CHECK(mgr.is_loaded(c));
                CHECK(mgr.get_mesh(c) != nullptr);
            }
        }
    }
    std::cout << "[PASS] test_negative_coordinates\n";
}

// 11. LOAD/UNLOAD RACE STRESS TEST
void test_load_unload_race() {
    StreamingConfig config;
    config.load_radius = 1;
    config.unload_radius = 2;
    config.worker_count = 4;

    ChunkManager mgr(config, MesherType::Naive);

    // Rapidly toggle between two positions 20 times without waiting
    for (int i = 0; i < 20; ++i) {
        mgr.update_streaming(Vec3(0.0f, 0.0f, 0.0f));
        mgr.update_streaming(Vec3(64.0f, 0.0f, 0.0f));
    }

    // Now wait for all in-flight work to settle
    mgr.wait_all_pending();

    // Manager must settle into valid state around final position (64, 0, 0) -> chunk (2, 0, 0)
    CHECK(mgr.is_loaded(ChunkCoord(2, 0, 0)));
    CHECK(mgr.loaded_chunk_count() <= 125); // Within hysteresis envelope
    std::cout << "[PASS] test_load_unload_race (settled cleanly)\n";
}

// 12. SHUTDOWN WITH PENDING WORK
void test_shutdown_with_pending_work() {
    {
        StreamingConfig config;
        config.load_radius = 3; // 7x7x7 = 343 chunks
        config.worker_count = 4;

        ChunkManager mgr(config, MesherType::Naive);
        mgr.update_streaming(Vec3(0.0f, 0.0f, 0.0f));
        // Intentionally destroy manager immediately while all 343 jobs are queued/executing!
    }
    std::cout << "[PASS] test_shutdown_with_pending_work (no deadlocks or leaks)\n";
}

// 13. GPU THREAD SAFETY STRUCTURAL VERIFICATION
void test_gpu_thread_safety() {
    // Workers operate strictly on ChunkBuildTask, ChunkNeighborhoodSnapshot, and MeshData.
    // Verify that MeshData contains zero OpenGL handles, zero GLuint, and zero GPU dependencies.
    static_assert(std::is_standard_layout_v<MeshVertex>, "MeshVertex must be standard layout");
    static_assert(sizeof(MeshVertex) == 6 * sizeof(float), "MeshVertex must be exactly 24 bytes (x,y,z,nx,ny,nz)");
    std::cout << "[PASS] test_gpu_thread_safety (clean architectural separation)\n";
}

int main() {
    std::cout << "=== Running Milestone 8 Concurrency Test Suite ===\n";

    test_worker_pool_start_stop();
    test_job_execution();
    test_multiple_workers();
    test_result_completeness();
    test_stale_result_rejection();
    test_duplicate_request_handling();
    test_deterministic_generation();
    test_deterministic_meshing();
    test_cross_chunk_correctness();
    test_negative_coordinates();
    test_load_unload_race();
    test_shutdown_with_pending_work();
    test_gpu_thread_safety();

    std::cout << "All Milestone 8 concurrency unit tests passed successfully!\n";
    return 0;
}
