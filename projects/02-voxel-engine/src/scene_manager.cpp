#include "voxel_lab/scene_manager.hpp"
#include "voxel_lab/greedy_mesher.hpp"
#include "voxel_lab/naive_mesher.hpp"
#include "voxel_lab/test_worlds.hpp"

#include <iostream>

namespace voxel_lab {

namespace {

MeshData mesh_for_chunk(MesherType mesher, const WorldAccessor& world, const ChunkCoord& coord) {
    if (mesher == MesherType::Naive) {
        return mesh_chunk(world, coord);
    } else {
        return greedy_mesh_chunk(world, coord);
    }
}

} // anonymous namespace

SceneManager::SceneManager() {
    StreamingConfig cfg;
    cfg.load_radius = 2;
    cfg.unload_radius = 3;
    cfg.worker_count = 4; // Use 4 worker threads for dynamic viewer streaming
    chunk_manager.set_config(cfg);
}

bool SceneManager::switch_scene(int scene_index, Camera& camera) {
    if (scene_index < 1 || scene_index > 4) {
        std::cerr << "[SceneManager] Invalid scene index: " << scene_index << " (valid: 1, 2, 3, 4)\n";
        return false;
    }
    current_scene_index = scene_index;
    rebuild_current_scene(true, &camera);
    std::cout << "[SceneManager] Switched scene to: " << current_stats.name
              << " | Mesher: [" << mesher_type_name(current_mesher) << "]"
              << " | Faces/Quads: " << current_stats.face_count
              << " | Vertices: " << current_stats.vertex_count << "\n";
    return true;
}

void SceneManager::update(const Camera& camera) {
    if (current_scene_index == 4) {
        bool changed = chunk_manager.update_streaming(camera.get_position());
        if (changed) {
            sync_streaming_gpu_meshes();
        }

        const auto& m = chunk_manager.get_metrics();
        current_stats.chunk_count = chunk_manager.loaded_chunk_count();
        current_stats.face_count = m.total_faces_or_quads;
        current_stats.vertex_count = m.total_vertices;
        current_stats.index_count = m.total_indices;
        current_stats.cam_chunk = chunk_manager.get_camera_chunk();
        current_stats.chunks_loaded_last_update = m.chunks_loaded_this_update;
        current_stats.chunks_unloaded_last_update = m.chunks_unloaded_this_update;
        current_stats.total_chunks_loaded = m.total_chunks_loaded;
        current_stats.total_chunks_unloaded = m.total_chunks_unloaded;
        current_stats.worker_count = m.worker_count;
        current_stats.jobs_pending = m.jobs_pending;
        current_stats.jobs_completed = m.jobs_completed;
        current_stats.jobs_discarded_stale = m.jobs_discarded_stale;
        current_stats.total_cpu_build_time_ms = m.total_cpu_build_time_us / 1000.0;
        current_stats.lod_enabled = chunk_manager.get_config().enable_lod;
        current_stats.lod0_chunks = m.lod0_chunk_count;
        current_stats.lod1_chunks = m.lod1_chunk_count;
        current_stats.lod2_chunks = m.lod2_chunk_count;
        current_stats.lod_changes = m.lod_changes;
    }
}

void SceneManager::toggle_lod() {
    StreamingConfig cfg = chunk_manager.get_config();
    cfg.enable_lod = !cfg.enable_lod;
    chunk_manager.set_config(cfg);
    if (current_scene_index == 4) {
        ChunkCoord cc = chunk_manager.get_camera_chunk();
        WorldCoord wc(cc.x * 32 + 16, cc.y * 32 + 16, cc.z * 32 + 16);
        chunk_manager.update_streaming(wc, true);
        chunk_manager.wait_all_pending();
        sync_streaming_gpu_meshes();
    }
    std::cout << "[SceneManager] LOD system toggled: [" << (cfg.enable_lod ? "ON" : "OFF") << "]\n";
}

void SceneManager::set_mesher(MesherType mesher) {
    if (current_mesher == mesher) {
        return;
    }
    current_mesher = mesher;
    if (current_scene_index == 4) {
        chunk_manager.set_mesher_type(mesher);
        for (const auto& c : chunk_manager.get_recently_updated_mesh_coords()) {
            const MeshData* m = chunk_manager.get_mesh(c);
            if (m) {
                chunk_gl_meshes[c].upload(*m);
            }
        }
        sync_streaming_gpu_meshes();
    } else {
        rebuild_current_scene(false, nullptr);
    }
    std::cout << "[SceneManager] Switched mesher to: [" << mesher_type_name(current_mesher) << "]"
              << " | Scene: " << current_stats.name
              << " | Faces/Quads: " << current_stats.face_count
              << " | Vertices: " << current_stats.vertex_count << "\n";
}

void SceneManager::rebuild_current_scene(bool reset_camera, Camera* camera) {
    switch (current_scene_index) {
        case 1:
            build_solid_chunk_scene(reset_camera, camera);
            break;
        case 2:
            build_plane_world_scene(reset_camera, camera);
            break;
        case 3:
            build_cross_chunk_sphere_scene(reset_camera, camera);
            break;
        case 4:
            build_streaming_scene(reset_camera, camera);
            break;
    }
}

void SceneManager::build_solid_chunk_scene(bool reset_camera, Camera* camera) {
    chunk_gl_meshes.clear();
    WorldGrid world;
    generate_solid_world(world, WorldCoord(0, 0, 0), WorldCoord(31, 31, 31), Voxel(1, 0));

    MeshData mesh = mesh_for_chunk(current_mesher, world, ChunkCoord(0, 0, 0));
    current_gl_mesh.upload(mesh);

    current_stats.name = "Scene 1: Solid Chunk (32^3)";
    current_stats.mesher = current_mesher;
    current_stats.chunk_count = world.loaded_chunk_count();
    current_stats.solid_voxel_count = world.count_solid_voxels();
    current_stats.face_count = mesh.face_count();
    current_stats.vertex_count = mesh.vertex_count();
    current_stats.index_count = mesh.index_count();
    current_stats.base_color = Vec3(0.75f, 0.78f, 0.82f); // Slate gray
    current_stats.cam_chunk = ChunkCoord(0, 0, 0);
    current_stats.chunks_loaded_last_update = 0;
    current_stats.chunks_unloaded_last_update = 0;

    if (reset_camera && camera) {
        camera->reset(Vec3(48.0f, 48.0f, 64.0f), -120.0f, -25.0f);
    }
}

void SceneManager::build_plane_world_scene(bool reset_camera, Camera* camera) {
    chunk_gl_meshes.clear();
    WorldGrid world;
    generate_plane_world(world, WorldCoord(0, 0, 0), WorldCoord(31, 31, 31), 15, PlaneAxis::Y, Voxel(3, 0));

    MeshData mesh = mesh_for_chunk(current_mesher, world, ChunkCoord(0, 0, 0));
    current_gl_mesh.upload(mesh);

    current_stats.name = "Scene 2: Planar World (y <= 15)";
    current_stats.mesher = current_mesher;
    current_stats.chunk_count = world.loaded_chunk_count();
    current_stats.solid_voxel_count = world.count_solid_voxels();
    current_stats.face_count = mesh.face_count();
    current_stats.vertex_count = mesh.vertex_count();
    current_stats.index_count = mesh.index_count();
    current_stats.base_color = Vec3(0.35f, 0.75f, 0.40f); // Terrain green
    current_stats.cam_chunk = ChunkCoord(0, 0, 0);
    current_stats.chunks_loaded_last_update = 0;
    current_stats.chunks_unloaded_last_update = 0;

    if (reset_camera && camera) {
        camera->reset(Vec3(48.0f, 36.0f, 64.0f), -120.0f, -20.0f);
    }
}

void SceneManager::build_cross_chunk_sphere_scene(bool reset_camera, Camera* camera) {
    chunk_gl_meshes.clear();
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
                    MeshData chunk_mesh = mesh_for_chunk(current_mesher, world, c);
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
    current_stats.mesher = current_mesher;
    current_stats.chunk_count = world.loaded_chunk_count();
    current_stats.solid_voxel_count = world.count_solid_voxels();
    current_stats.face_count = combined_mesh.face_count();
    current_stats.vertex_count = combined_mesh.vertex_count();
    current_stats.index_count = combined_mesh.index_count();
    current_stats.base_color = Vec3(0.92f, 0.72f, 0.28f); // Golden amber
    current_stats.cam_chunk = ChunkCoord(0, 0, 0);
    current_stats.chunks_loaded_last_update = 0;
    current_stats.chunks_unloaded_last_update = 0;

    if (reset_camera && camera) {
        camera->reset(Vec3(65.0f, 55.0f, 75.0f), -125.0f, -20.0f);
    }
}

void SceneManager::build_streaming_scene(bool reset_camera, Camera* camera) {
    chunk_gl_meshes.clear();
    current_gl_mesh.destroy();

    chunk_manager.set_mesher_type(current_mesher);
    Vec3 cam_pos(0.0f, 25.0f, 50.0f);

    if (reset_camera && camera) {
        camera->reset(cam_pos, -90.0f, -20.0f);
    }

    chunk_manager.update_streaming(cam_pos, true);

    for (const auto& [coord, mesh] : chunk_manager.get_all_meshes()) {
        chunk_gl_meshes[coord].upload(mesh);
    }

    current_stats.name = "Scene 4: Dynamic Streaming World";
    current_stats.mesher = current_mesher;
    current_stats.base_color = Vec3(0.42f, 0.78f, 0.48f); // Terrain green
    sync_streaming_gpu_meshes();
}

void SceneManager::sync_streaming_gpu_meshes() {
    for (const auto& c : chunk_manager.get_recently_unloaded_mesh_coords()) {
        chunk_gl_meshes.erase(c);
    }
    for (const auto& c : chunk_manager.get_recently_updated_mesh_coords()) {
        const MeshData* m = chunk_manager.get_mesh(c);
        if (m) {
            chunk_gl_meshes[c].upload(*m);
        }
    }

    const auto& metrics = chunk_manager.get_metrics();
    current_stats.chunk_count = chunk_manager.loaded_chunk_count();
    current_stats.solid_voxel_count = chunk_manager.get_world().count_solid_voxels();
    current_stats.face_count = metrics.total_faces_or_quads;
    current_stats.vertex_count = metrics.total_vertices;
    current_stats.index_count = metrics.total_indices;
    current_stats.cam_chunk = chunk_manager.get_camera_chunk();
    current_stats.chunks_loaded_last_update = metrics.chunks_loaded_this_update;
    current_stats.chunks_unloaded_last_update = metrics.chunks_unloaded_this_update;
    current_stats.total_chunks_loaded = metrics.total_chunks_loaded;
    current_stats.total_chunks_unloaded = metrics.total_chunks_unloaded;
}

void SceneManager::render(const GLShader& shader, const Mat4& view, const Mat4& proj) const {
    shader.use();
    shader.set_mat4("uView", view);
    shader.set_mat4("uProjection", proj);
    shader.set_vec3("uLightDir", Vec3(0.6f, 0.8f, 0.4f));
    shader.set_vec3("uBaseColor", current_stats.base_color);

    if (current_scene_index == 4) {
        for (const auto& [coord, mesh] : chunk_gl_meshes) {
            Mat4 model = Mat4::translate(Vec3(
                static_cast<float>(coord.x * CHUNK_DIM),
                static_cast<float>(coord.y * CHUNK_DIM),
                static_cast<float>(coord.z * CHUNK_DIM)
            ));
            shader.set_mat4("uModel", model);
            mesh.draw();
        }
    } else {
        shader.set_mat4("uModel", Mat4::identity());
        current_gl_mesh.draw();
    }
}

} // namespace voxel_lab
