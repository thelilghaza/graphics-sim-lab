#pragma once

#include "voxel_lab/camera.hpp"
#include "voxel_lab/gl_mesh.hpp"
#include "voxel_lab/gl_shader.hpp"
#include "voxel_lab/mesh.hpp"
#include "voxel_lab/world_grid.hpp"
#include <string>

namespace voxel_lab {

struct SceneStats {
    std::string name;
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

    void render(const GLShader& shader, const Mat4& view, const Mat4& proj) const;

    const SceneStats& get_current_stats() const noexcept { return current_stats; }
    int get_current_scene_index() const noexcept { return current_scene_index; }

private:
    void build_solid_chunk_scene(Camera& camera);
    void build_plane_world_scene(Camera& camera);
    void build_cross_chunk_sphere_scene(Camera& camera);

    int current_scene_index{1};
    SceneStats current_stats;
    GLMesh current_gl_mesh;
};

} // namespace voxel_lab
