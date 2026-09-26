#pragma once

#include "voxel_lab/chunk.hpp"
#include "voxel_lab/chunk_snapshot.hpp"
#include "voxel_lab/coordinates.hpp"
#include "voxel_lab/greedy_mesher.hpp"
#include "voxel_lab/math.hpp"
#include "voxel_lab/mesh.hpp"
#include "voxel_lab/naive_mesher.hpp"
#include "voxel_lab/thread_pool.hpp"
#include "voxel_lab/world_grid.hpp"

#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <vector>

namespace voxel_lab {

struct StreamingConfig {
    int load_radius{2};      // Chebyshev distance in chunk coordinates for loading
    int unload_radius{3};    // Chebyshev distance in chunk coordinates for unloading (unload_radius >= load_radius)
    size_t worker_count{0};  // Number of worker threads (0 = synchronous execution, >=1 = multithreaded worker pool)

    // Milestone 9: Memory & buffer reuse configuration
    bool enable_mesh_buffer_reuse{true};   // When true, recycles and reuses MeshData vector capacity
    size_t max_recycled_mesh_buffers{128}; // Upper bound on retained idle mesh buffers in pool
};

struct StreamingMetrics {
    // Basic streaming & residency metrics
    size_t chunks_loaded_this_update{0};
    size_t chunks_unloaded_this_update{0};
    size_t total_chunks_loaded{0};
    size_t total_chunks_unloaded{0};
    size_t currently_loaded_chunks{0};
    size_t chunks_meshed_this_update{0};
    size_t total_chunks_meshed{0};
    size_t total_faces_or_quads{0};
    size_t total_vertices{0};
    size_t total_indices{0};

    // Concurrency metrics (Milestone 8)
    size_t worker_count{0};
    size_t jobs_submitted{0};
    size_t jobs_completed{0};
    size_t jobs_discarded_stale{0};
    size_t jobs_pending{0};
    size_t chunks_generated{0};
    size_t chunks_integrated{0};
    double total_generation_time_us{0.0};
    double total_mesh_time_us{0.0};
    double total_cpu_build_time_us{0.0};
    double last_streaming_update_time_us{0.0};

    // Milestone 9: Memory & buffer reuse metrics
    size_t mesh_buffers_reused{0};
    size_t mesh_buffers_allocated_fresh{0};
    size_t mesh_buffers_recycled{0};
    size_t mesh_buffers_evicted_from_pool{0};
};

struct ChunkManagerMemoryStats {
    size_t resident_chunk_count{0};
    size_t raw_chunk_payload_bytes{0};       // count * 65536
    size_t mesh_count{0};
    size_t total_mesh_logical_bytes{0};       // vertex size (24B) + index size (4B)
    size_t total_mesh_capacity_bytes{0};      // vertex cap (24B) + index cap (4B)
    size_t recycled_mesh_buffer_count{0};
    size_t recycled_mesh_capacity_bytes{0};
    size_t estimated_world_grid_node_bytes{0}; // count * 65584
    size_t estimated_total_logical_bytes{0};
    size_t process_working_set_bytes{0};      // OS resident memory
    size_t process_private_bytes{0};          // OS private commit
};

using ChunkGenerator = std::function<void(WorldAccessor& world, const ChunkCoord& coord)>;

// Default deterministic terrain generator function
void generate_default_terrain_chunk(WorldAccessor& world, const ChunkCoord& coord);

struct ChunkBuildTask {
    ChunkCoord coord;
    uint64_t version{0};
    bool need_generation{false};
    MesherType mesher_type{MesherType::Naive};
    ChunkNeighborhoodSnapshot snapshot;
    ChunkGenerator generator;
    MeshData mesh; // Milestone 9: reusable mesh buffer passed into task
};

struct ChunkBuildResult {
    ChunkCoord coord;
    uint64_t version{0};
    bool need_generation{false};
    Chunk chunk;
    MeshData mesh;
    MesherType mesher_type{MesherType::Naive};
    double generation_time_us{0.0};
    double mesh_time_us{0.0};
};

class ChunkManager {
public:
    explicit ChunkManager(StreamingConfig config = StreamingConfig{},
                          MesherType mesher = MesherType::Naive,
                          ChunkGenerator generator = nullptr);
    ~ChunkManager();

    ChunkManager(const ChunkManager&) = delete;
    ChunkManager& operator=(const ChunkManager&) = delete;

    // Updates streaming around camera position. Integrates completed async jobs and queues new ones.
    // Returns true if loaded set or meshes changed.
    bool update_streaming(const Vec3& camera_world_position, bool force = false);
    bool update_streaming(const WorldCoord& camera_world_coord, bool force = false);

    // Drains and integrates all completed async worker jobs on the calling thread
    size_t integrate_completed_jobs();

    // Blocks until all pending worker jobs have completed and integrates their results
    void wait_all_pending();

    // Queries
    bool is_loaded(const ChunkCoord& chunk_coord) const noexcept;
    bool is_desired(const ChunkCoord& chunk_coord) const noexcept;
    size_t loaded_chunk_count() const noexcept;
    std::vector<ChunkCoord> get_loaded_chunk_coordinates() const;

    // Explicit manual chunk operations (used for testing and direct control)
    bool load_chunk(const ChunkCoord& chunk_coord);
    bool unload_chunk(const ChunkCoord& chunk_coord);

    // Access to world representation
    const WorldGrid& get_world() const noexcept { return world_grid; }
    WorldGrid& get_world() noexcept { return world_grid; }

    // Access to mesh data
    const MeshData* get_mesh(const ChunkCoord& chunk_coord) const noexcept;
    const std::map<ChunkCoord, MeshData>& get_all_meshes() const noexcept { return meshes; }

    // Change mesher algorithm (regenerates all currently loaded meshes)
    void set_mesher_type(MesherType mesher);
    MesherType get_mesher_type() const noexcept { return mesher_type; }

    // Config & generator customization
    void set_config(const StreamingConfig& new_config);
    const StreamingConfig& get_config() const noexcept { return config; }

    void set_generator(ChunkGenerator gen);

    // Concurrency queries
    size_t get_worker_count() const noexcept { return config.worker_count; }
    size_t get_pending_job_count() const noexcept;

    // Metrics
    const StreamingMetrics& get_metrics() const noexcept { return metrics; }
    void reset_cumulative_metrics() noexcept;

    // Camera chunk tracking
    bool has_camera_chunk() const noexcept { return has_cam_chunk; }
    const ChunkCoord& get_camera_chunk() const noexcept { return current_cam_chunk; }

    // Milestone 9: Memory queries & buffer reuse control
    ChunkManagerMemoryStats get_memory_stats() const;
    size_t get_recycled_mesh_buffer_count() const noexcept { return recycled_mesh_buffers.size(); }
    void clear_recycled_mesh_buffers() noexcept;

    // Incremental GPU sync lists from last update
    const std::vector<ChunkCoord>& get_recently_updated_mesh_coords() const noexcept { return recently_updated_meshes; }
    const std::vector<ChunkCoord>& get_recently_unloaded_mesh_coords() const noexcept { return recently_unloaded_meshes; }

private:
    void queue_build(const ChunkCoord& coord, bool need_generation);
    void queue_remesh(const ChunkCoord& coord);
    void execute_task_sync(ChunkBuildTask task);
    void remesh_chunk_sync(const ChunkCoord& coord);
    void recalculate_aggregate_mesh_metrics();

    // Milestone 9: Buffer reuse helpers
    MeshData acquire_mesh_buffer();
    void recycle_mesh_buffer(MeshData mesh);

    StreamingConfig config;
    MesherType mesher_type{MesherType::Naive};
    ChunkGenerator generator;

    WorldGrid world_grid;
    std::set<ChunkCoord> desired_chunks;
    std::set<ChunkCoord> loaded_chunks;
    std::map<ChunkCoord, MeshData> meshes;
    std::vector<MeshData> recycled_mesh_buffers; // Milestone 9: Pool of retained reusable MeshData buffers
    std::map<ChunkCoord, uint64_t> chunk_versions;
    uint64_t next_version{0};

    bool has_cam_chunk{false};
    ChunkCoord current_cam_chunk{0, 0, 0};

    // Concurrency components
    std::unique_ptr<ThreadPool> thread_pool;
    std::deque<ChunkBuildResult> completed_queue;
    mutable std::mutex completed_mutex;

    StreamingMetrics metrics;
    std::vector<ChunkCoord> recently_updated_meshes;
    std::vector<ChunkCoord> recently_unloaded_meshes;
};

} // namespace voxel_lab
