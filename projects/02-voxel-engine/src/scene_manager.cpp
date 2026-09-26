#include "voxel_lab/scene_manager.hpp"
#include "voxel_lab/naive_mesher.hpp"
#include "voxel_lab/test_worlds.hpp"
#include <iostream>

namespace voxel_lab {

SceneManager::SceneManager() {
    // Default initial scene built upon first switch
}

bool SceneManager::switch_scene(int scene_index, Camera& camera) {
    switch (scene_index) {
        case 1:
            build_solid_chunk_scene(camera);
            current_scene_index = 1;
            return true;
        case 2:
            build_plane_world_scene(camera);
            current_scene_index = 2;
            return true;
        case 3:
            build_cross_chunk_sphere_scene(camera);
            current_scene_index = 3;
            return true;
        default:
            std::cerr << "[SceneManager] Invalid scene index: " << scene_index << " (valid: 1, 2, 3)\n";
            return false;
    }
}

void SceneManager::build_solid_chunk_scene(Camera& camera) {
    WorldGrid world;
    generate_solid_world(world, WorldCoord(0, 0, 0), WorldCoord(31, 31, 31), Voxel(1, 0));

    MeshData mesh = mesh_chunk(world, ChunkCoord(0, 0, 0));
    current_gl_mesh.upload(mesh);

    current_stats.name = "Scene 1: Solid Chunk (32^3)";
    current_stats.chunk_count = world.loaded_chunk_count();
    current_stats.solid_voxel_count = world.count_solid_voxels();
    current_stats.face_count = mesh.face_count();
    current_stats.vertex_count = mesh.vertex_count();
    current_stats.index_count = mesh.index_count();
    current_stats.base_color = Vec3(0.75f, 0.78f, 0.82f); // Slate gray

    camera.reset(Vec3(48.0f, 48.0f, 64.0f), -120.0f, -25.0f);
}

void SceneManager::build_plane_world_scene(Camera& camera) {
    WorldGrid world;
    generate_plane_world(world, WorldCoord(0, 0, 0), WorldCoord(31, 31, 31), 15, PlaneAxis::Y, Voxel(3, 0));

    MeshData mesh = mesh_chunk(world, ChunkCoord(0, 0, 0));
    current_gl_mesh.upload(mesh);

    current_stats.name = "Scene 2: Planar World (y <= 15)";
    current_stats.chunk_count = world.loaded_chunk_count();
    current_stats.solid_voxel_count = world.count_solid_voxels();
    current_stats.face_count = mesh.face_count();
    current_stats.vertex_count = mesh.vertex_count();
    current_stats.index_count = mesh.index_count();
    current_stats.base_color = Vec3(0.35f, 0.75f, 0.40f); // Terrain green

    camera.reset(Vec3(48.0f, 36.0f, 64.0f), -120.0f, -20.0f);
}

void SceneManager::build_cross_chunk_sphere_scene(Camera& camera) {
    WorldGrid world;
    WorldCoord center(31, 31, 31);
    int radius = 12;
    generate_sphere_world(world, center, radius, Voxel(5, 1));

    // Mesh all 8 neighboring chunks surrounding the cross-chunk boundary
    MeshData combined_mesh;
    for (int cz = 0; cz <= 1; ++cz) {
        for (int cy = 0; cy <= 1; ++cy) {
            for (int cx = 0; cx <= 1; ++cx) {
                ChunkCoord c(cx, cy, cz);
                if (world.has_chunk(c)) {
                    MeshData chunk_mesh = mesh_chunk(world, c);
                    uint32_t base_index = static_cast<uint32_t>(combined_mesh.vertices.size());
                    float offset_x = static_cast<float>(cx * CHUNK_DIM);
                    float offset_y = static_cast<float>(cy * CHUNK_DIM);
                    float offset_z = static_cast<float>(cz * CHUNK_DIM);

                    for (const auto& v : chunk_mesh.vertices) {
                        combined_mesh.vertices.emplace_back(
                            v.x + offset_x,
                            v.y + offset_y,
                            v.z + offset_z,
                            v.nx, v.ny, v.nz
                        );
                    }
                    for (auto idx : chunk_mesh.indices) {
                        combined_mesh.indices.push_back(base_index + idx);
                    }
                }
            }
        }
    }

    current_gl_mesh.upload(combined_mesh);

    current_stats.name = "Scene 3: Cross-Chunk Sphere (r=12 at (31,31,31))";
    current_stats.chunk_count = world.loaded_chunk_count();
    current_stats.solid_voxel_count = world.count_solid_voxels();
    current_stats.face_count = combined_mesh.face_count();
    current_stats.vertex_count = combined_mesh.vertex_count();
    current_stats.index_count = combined_mesh.index_count();
    current_stats.base_color = Vec3(0.92f, 0.72f, 0.28f); // Golden amber

    camera.reset(Vec3(65.0f, 55.0f, 75.0f), -125.0f, -20.0f);
}

void SceneManager::render(const GLShader& shader, const Mat4& view, const Mat4& proj) const {
    shader.use();
    shader.set_mat4("uModel", Mat4::identity());
    shader.set_mat4("uView", view);
    shader.set_mat4("uProjection", proj);
    shader.set_vec3("uLightDir", Vec3(0.6f, 0.8f, 0.4f));
    shader.set_vec3("uBaseColor", current_stats.base_color);

    current_gl_mesh.draw();
}

} // namespace voxel_lab
