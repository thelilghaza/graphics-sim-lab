#pragma once

#include "voxel_lab/camera.hpp"
#include "voxel_lab/gl_mesh.hpp"
#include "voxel_lab/gl_shader.hpp"
#include "voxel_lab/mesh.hpp"
#include "voxel_lab/world_grid.hpp"
#include <string>

namespace voxel_lab {

enum class MesherType {
    Naive,
    Greedy
};

inline const char* mesher_type_name(MesherType type) noexcept {
    switch (type) {
        case MesherType::Naive: return "Naive";
        case MesherType::Greedy: return "Greedy";
    }
    return "Unknown";
}

struct SceneStats {
    std::string name;
    MesherType mesher{MesherType::Greedy};
    size_t chunk_count{0};
    size_t solid_voxel_count{0};
    size_t face_count{0};
    size_t vertex_count{0};
    size_t index_count{0};
    Vec3 base_color{0.8f, 0.8f, 0.8f};
};

class SceneManager {
public:
    SceneManager();

    // Switches active scene (1: Solid Chunk, 2: Plane World, 3: Cross-Chunk Sphere)
    bool switch_scene(int scene_index, Camera& camera);

    // Switches meshing algorithm and regenerates mesh keeping the current camera
    void set_mesher(MesherType mesher);

    void render(const GLShader& shader, const Mat4& view, const Mat4& proj) const;

    const SceneStats& get_current_stats() const noexcept { return current_stats; }
    int get_current_scene_index() const noexcept { return current_scene_index; }
    MesherType get_current_mesher() const noexcept { return current_mesher; }

private:
    void rebuild_current_scene(bool reset_camera, Camera* camera = nullptr);
    void build_solid_chunk_scene(bool reset_camera, Camera* camera = nullptr);
    void build_plane_world_scene(bool reset_camera, Camera* camera = nullptr);
    void build_cross_chunk_sphere_scene(bool reset_camera, Camera* camera = nullptr);

    int current_scene_index{1};
    MesherType current_mesher{MesherType::Greedy};
    SceneStats current_stats;
    GLMesh current_gl_mesh;
};

} // namespace voxel_lab
