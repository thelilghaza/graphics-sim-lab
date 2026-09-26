#pragma once

#include "voxel_lab/camera.hpp"
#include "voxel_lab/chunk_manager.hpp"
#include "voxel_lab/gl_mesh.hpp"
#include "voxel_lab/gl_shader.hpp"
#include "voxel_lab/mesh.hpp"
#include "voxel_lab/world_grid.hpp"

#include <map>
#include <string>

namespace voxel_lab {

struct SceneStats {
    std::string name;
    MesherType mesher{MesherType::Greedy};
    size_t chunk_count{0};
    size_t solid_voxel_count{0};
    size_t face_count{0};
    size_t vertex_count{0};
    size_t index_count{0};
    Vec3 base_color{0.8f, 0.8f, 0.8f};
    ChunkCoord cam_chunk{0, 0, 0};
    size_t chunks_loaded_last_update{0};
    size_t chunks_unloaded_last_update{0};
    size_t total_chunks_loaded{0};
    size_t total_chunks_unloaded{0};
};

class SceneManager {
public:
    SceneManager();

    // Switches active scene (1: Solid Chunk, 2: Plane World, 3: Cross-Chunk Sphere, 4: Dynamic Streaming World)
    bool switch_scene(int scene_index, Camera& camera);

    // Updates per-frame logic (e.g. streaming around camera in Scene 4)
    void update(const Camera& camera);

    // Switches meshing algorithm and regenerates mesh keeping the current camera
    void set_mesher(MesherType mesher);

    void render(const GLShader& shader, const Mat4& view, const Mat4& proj) const;

    const SceneStats& get_current_stats() const noexcept { return current_stats; }
    int get_current_scene_index() const noexcept { return current_scene_index; }
    MesherType get_current_mesher() const noexcept { return current_mesher; }

    const ChunkManager& get_chunk_manager() const noexcept { return chunk_manager; }
    ChunkManager& get_chunk_manager() noexcept { return chunk_manager; }

private:
    void rebuild_current_scene(bool reset_camera, Camera* camera = nullptr);
    void build_solid_chunk_scene(bool reset_camera, Camera* camera = nullptr);
    void build_plane_world_scene(bool reset_camera, Camera* camera = nullptr);
    void build_cross_chunk_sphere_scene(bool reset_camera, Camera* camera = nullptr);
    void build_streaming_scene(bool reset_camera, Camera* camera = nullptr);
    void sync_streaming_gpu_meshes();

    int current_scene_index{4}; // Default to Milestone 7 streaming scene
    MesherType current_mesher{MesherType::Greedy};
    SceneStats current_stats;
    GLMesh current_gl_mesh;

    ChunkManager chunk_manager;
    std::map<ChunkCoord, GLMesh> chunk_gl_meshes;
};

} // namespace voxel_lab
