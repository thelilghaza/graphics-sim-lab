#pragma once

#include "voxel_lab/chunk.hpp"
#include "voxel_lab/coordinates.hpp"
#include "voxel_lab/greedy_mesher.hpp"
#include "voxel_lab/math.hpp"
#include "voxel_lab/mesh.hpp"
#include "voxel_lab/naive_mesher.hpp"
#include "voxel_lab/world_grid.hpp"

#include <functional>
#include <map>
#include <set>
#include <vector>

namespace voxel_lab {

struct StreamingConfig {
    int load_radius{2};   // Chebyshev distance in chunk coordinates for loading
    int unload_radius{3}; // Chebyshev distance in chunk coordinates for unloading (unload_radius >= load_radius)
};

struct StreamingMetrics {
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
};

using ChunkGenerator = std::function<void(WorldAccessor& world, const ChunkCoord& coord)>;

// Default deterministic terrain generator function
void generate_default_terrain_chunk(WorldAccessor& world, const ChunkCoord& coord);

class ChunkManager {
public:
    explicit ChunkManager(StreamingConfig config = StreamingConfig{},
                          MesherType mesher = MesherType::Naive,
                          ChunkGenerator generator = nullptr);

    // Updates streaming around camera position. Returns true if loaded set or meshes changed.
    bool update_streaming(const Vec3& camera_world_position, bool force = false);
    bool update_streaming(const WorldCoord& camera_world_coord, bool force = false);

    // Queries
    bool is_loaded(const ChunkCoord& chunk_coord) const noexcept;
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

    // Metrics
    const StreamingMetrics& get_metrics() const noexcept { return metrics; }
    void reset_cumulative_metrics() noexcept;

    // Camera chunk tracking
    bool has_camera_chunk() const noexcept { return has_cam_chunk; }
    const ChunkCoord& get_camera_chunk() const noexcept { return current_cam_chunk; }

    // Incremental GPU sync lists from last update
    const std::vector<ChunkCoord>& get_recently_updated_mesh_coords() const noexcept { return recently_updated_meshes; }
    const std::vector<ChunkCoord>& get_recently_unloaded_mesh_coords() const noexcept { return recently_unloaded_meshes; }

private:
    void remesh_chunk(const ChunkCoord& coord);
    void recalculate_aggregate_mesh_metrics();

    StreamingConfig config;
    MesherType mesher_type{MesherType::Greedy};
    ChunkGenerator generator;

    WorldGrid world_grid;
    std::set<ChunkCoord> loaded_chunks;
    std::map<ChunkCoord, MeshData> meshes;

    bool has_cam_chunk{false};
    ChunkCoord current_cam_chunk{0, 0, 0};

    StreamingMetrics metrics;
    std::vector<ChunkCoord> recently_updated_meshes;
    std::vector<ChunkCoord> recently_unloaded_meshes;
};

} // namespace voxel_lab
