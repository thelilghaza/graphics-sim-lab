#include "voxel_lab/chunk_manager.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace voxel_lab {

namespace {

inline int continuous_terrain_height(int wx, int wz) {
    int mx = ((wx % 128) + 128) % 128;
    int hx = (mx < 64) ? mx : (128 - mx); // 0 .. 64

    int mz = ((wz % 128) + 128) % 128;
    int hz = (mz < 64) ? mz : (128 - mz); // 0 .. 64

    // Base height -6 + (hx + hz) / 4 -> range [-6, 26]
    return -6 + (hx + hz) / 4;
}

std::array<ChunkCoord, 6> get_chunk_neighbors(const ChunkCoord& c) noexcept {
    return {
        ChunkCoord(c.x + 1, c.y, c.z),
        ChunkCoord(c.x - 1, c.y, c.z),
        ChunkCoord(c.x, c.y + 1, c.z),
        ChunkCoord(c.x, c.y - 1, c.z),
        ChunkCoord(c.x, c.y, c.z + 1),
        ChunkCoord(c.x, c.y, c.z - 1)
    };
}

int chebyshev_distance(const ChunkCoord& a, const ChunkCoord& b) noexcept {
    return std::max({std::abs(a.x - b.x), std::abs(a.y - b.y), std::abs(a.z - b.z)});
}

} // anonymous namespace

void generate_default_terrain_chunk(WorldAccessor& world, const ChunkCoord& coord) {
    int min_y = coord.y * CHUNK_DIM;
    int max_y = (coord.y + 1) * CHUNK_DIM - 1;

    WorldGrid* grid = dynamic_cast<WorldGrid*>(&world);
    Chunk* chunk = grid ? grid->get_chunk(coord) : nullptr;

    // Completely above terrain surface (open sky)
    if (min_y > 26) {
        if (chunk) {
            chunk->clear();
        }
        return;
    }

    // Completely underground (solid bedrock)
    if (max_y < -6) {
        if (chunk) {
            chunk->fill(Voxel(1, 0));
        } else {
            for (int lz = 0; lz < CHUNK_DIM; ++lz) {
                for (int ly = 0; ly < CHUNK_DIM; ++ly) {
                    for (int lx = 0; lx < CHUNK_DIM; ++lx) {
                        WorldCoord w = reconstruct_world_coord(coord, LocalCoord(lx, ly, lz));
                        world.set_voxel(w, Voxel(1, 0)); // Stone
                    }
                }
            }
        }
        return;
    }

    // Intersects surface
    for (int lz = 0; lz < CHUNK_DIM; ++lz) {
        int wz = coord.z * CHUNK_DIM + lz;
        for (int lx = 0; lx < CHUNK_DIM; ++lx) {
            int wx = coord.x * CHUNK_DIM + lx;
            int h = continuous_terrain_height(wx, wz);

            for (int ly = 0; ly < CHUNK_DIM; ++ly) {
                int wy = coord.y * CHUNK_DIM + ly;
                Voxel v(0, 0);
                if (wy < h - 2) {
                    v = Voxel(1, 0); // Stone (type 1)
                } else if (wy <= h) {
                    v = Voxel(2, 0); // Dirt/Grass (type 2)
                }

                if (chunk) {
                    chunk->set_voxel(lx, ly, lz, v);
                } else {
                    world.set_voxel(WorldCoord(wx, wy, wz), v);
                }
            }
        }
    }
}

ChunkManager::ChunkManager(StreamingConfig cfg, MesherType mesher, ChunkGenerator gen)
    : config(cfg), mesher_type(mesher), generator(std::move(gen)) {}

bool ChunkManager::update_streaming(const Vec3& camera_world_position, bool force) {
    WorldCoord w(
        static_cast<int>(std::floor(camera_world_position.x)),
        static_cast<int>(std::floor(camera_world_position.y)),
        static_cast<int>(std::floor(camera_world_position.z))
    );
    return update_streaming(w, force);
}

bool ChunkManager::update_streaming(const WorldCoord& camera_world_coord, bool force) {
    ChunkCoord new_cam_chunk = world_to_chunk(camera_world_coord);

    if (!force && has_cam_chunk && new_cam_chunk == current_cam_chunk) {
        metrics.chunks_loaded_this_update = 0;
        metrics.chunks_unloaded_this_update = 0;
        metrics.chunks_meshed_this_update = 0;
        recently_updated_meshes.clear();
        recently_unloaded_meshes.clear();
        return false;
    }

    has_cam_chunk = true;
    current_cam_chunk = new_cam_chunk;
    recently_updated_meshes.clear();
    recently_unloaded_meshes.clear();
    metrics.chunks_loaded_this_update = 0;
    metrics.chunks_unloaded_this_update = 0;
    metrics.chunks_meshed_this_update = 0;

    int r_load = std::max(0, config.load_radius);
    int r_unload = std::max(r_load, config.unload_radius);

    // 1. Identify chunks outside unload radius
    std::vector<ChunkCoord> to_unload;
    for (const auto& c : loaded_chunks) {
        if (chebyshev_distance(c, current_cam_chunk) > r_unload) {
            to_unload.push_back(c);
        }
    }

    // 2. Identify missing chunks within load radius
    std::vector<ChunkCoord> to_load;
    for (int dz = -r_load; dz <= r_load; ++dz) {
        for (int dy = -r_load; dy <= r_load; ++dy) {
            for (int dx = -r_load; dx <= r_load; ++dx) {
                ChunkCoord c(current_cam_chunk.x + dx, current_cam_chunk.y + dy, current_cam_chunk.z + dz);
                if (loaded_chunks.find(c) == loaded_chunks.end()) {
                    to_load.push_back(c);
                }
            }
        }
    }

    if (to_unload.empty() && to_load.empty() && !force) {
        return false;
    }

    std::set<ChunkCoord> dirty_mesh_chunks;

    // Execute unloads
    for (const auto& c : to_unload) {
        for (const auto& n : get_chunk_neighbors(c)) {
            if (loaded_chunks.find(n) != loaded_chunks.end() &&
                std::find(to_unload.begin(), to_unload.end(), n) == to_unload.end()) {
                dirty_mesh_chunks.insert(n);
            }
        }

        loaded_chunks.erase(c);
        meshes.erase(c);
        world_grid.remove_chunk(c);
        recently_unloaded_meshes.push_back(c);

        metrics.chunks_unloaded_this_update++;
        metrics.total_chunks_unloaded++;
    }

    // Execute loads
    for (const auto& c : to_load) {
        world_grid.get_or_create_chunk(c);
        if (generator) {
            generator(world_grid, c);
        } else {
            generate_default_terrain_chunk(world_grid, c);
        }

        loaded_chunks.insert(c);
        dirty_mesh_chunks.insert(c);

        for (const auto& n : get_chunk_neighbors(c)) {
            if (loaded_chunks.find(n) != loaded_chunks.end()) {
                dirty_mesh_chunks.insert(n);
            }
        }

        metrics.chunks_loaded_this_update++;
        metrics.total_chunks_loaded++;
    }

    // Execute mesh updates
    for (const auto& c : dirty_mesh_chunks) {
        if (loaded_chunks.find(c) != loaded_chunks.end()) {
            remesh_chunk(c);
            recently_updated_meshes.push_back(c);
            metrics.chunks_meshed_this_update++;
            metrics.total_chunks_meshed++;
        }
    }

    metrics.currently_loaded_chunks = loaded_chunks.size();
    recalculate_aggregate_mesh_metrics();

    return true;
}

bool ChunkManager::is_loaded(const ChunkCoord& chunk_coord) const noexcept {
    return loaded_chunks.find(chunk_coord) != loaded_chunks.end();
}

size_t ChunkManager::loaded_chunk_count() const noexcept {
    return loaded_chunks.size();
}

std::vector<ChunkCoord> ChunkManager::get_loaded_chunk_coordinates() const {
    return std::vector<ChunkCoord>(loaded_chunks.begin(), loaded_chunks.end());
}

bool ChunkManager::load_chunk(const ChunkCoord& chunk_coord) {
    if (is_loaded(chunk_coord)) {
        return false;
    }

    world_grid.get_or_create_chunk(chunk_coord);
    if (generator) {
        generator(world_grid, chunk_coord);
    } else {
        generate_default_terrain_chunk(world_grid, chunk_coord);
    }

    loaded_chunks.insert(chunk_coord);

    std::set<ChunkCoord> dirty_chunks{chunk_coord};
    for (const auto& n : get_chunk_neighbors(chunk_coord)) {
        if (loaded_chunks.find(n) != loaded_chunks.end()) {
            dirty_chunks.insert(n);
        }
    }

    for (const auto& c : dirty_chunks) {
        remesh_chunk(c);
        recently_updated_meshes.push_back(c);
        metrics.total_chunks_meshed++;
    }

    metrics.total_chunks_loaded++;
    metrics.currently_loaded_chunks = loaded_chunks.size();
    recalculate_aggregate_mesh_metrics();
    return true;
}

bool ChunkManager::unload_chunk(const ChunkCoord& chunk_coord) {
    if (!is_loaded(chunk_coord)) {
        return false;
    }

    std::set<ChunkCoord> dirty_neighbors;
    for (const auto& n : get_chunk_neighbors(chunk_coord)) {
        if (loaded_chunks.find(n) != loaded_chunks.end() && n != chunk_coord) {
            dirty_neighbors.insert(n);
        }
    }

    loaded_chunks.erase(chunk_coord);
    meshes.erase(chunk_coord);
    world_grid.remove_chunk(chunk_coord);
    recently_unloaded_meshes.push_back(chunk_coord);

    for (const auto& n : dirty_neighbors) {
        remesh_chunk(n);
        recently_updated_meshes.push_back(n);
        metrics.total_chunks_meshed++;
    }

    metrics.total_chunks_unloaded++;
    metrics.currently_loaded_chunks = loaded_chunks.size();
    recalculate_aggregate_mesh_metrics();
    return true;
}

const MeshData* ChunkManager::get_mesh(const ChunkCoord& chunk_coord) const noexcept {
    auto it = meshes.find(chunk_coord);
    if (it != meshes.end()) {
        return &it->second;
    }
    return nullptr;
}

void ChunkManager::set_mesher_type(MesherType mesher) {
    if (mesher_type == mesher) {
        return;
    }
    mesher_type = mesher;
    recently_updated_meshes.clear();

    for (const auto& c : loaded_chunks) {
        remesh_chunk(c);
        recently_updated_meshes.push_back(c);
        metrics.total_chunks_meshed++;
    }

    recalculate_aggregate_mesh_metrics();
}

void ChunkManager::set_config(const StreamingConfig& new_config) {
    config = new_config;
}

void ChunkManager::set_generator(ChunkGenerator gen) {
    generator = std::move(gen);
}

void ChunkManager::reset_cumulative_metrics() noexcept {
    metrics.total_chunks_loaded = 0;
    metrics.total_chunks_unloaded = 0;
    metrics.total_chunks_meshed = 0;
}

void ChunkManager::remesh_chunk(const ChunkCoord& coord) {
    MeshData mesh;
    if (mesher_type == MesherType::Naive) {
        mesh_chunk(world_grid, coord, mesh);
    } else {
        greedy_mesh_chunk(world_grid, coord, mesh);
    }
    meshes[coord] = std::move(mesh);
}

void ChunkManager::recalculate_aggregate_mesh_metrics() {
    size_t faces = 0;
    size_t vertices = 0;
    size_t indices = 0;

    for (const auto& [coord, mesh] : meshes) {
        faces += mesh.face_count();
        vertices += mesh.vertex_count();
        indices += mesh.index_count();
    }

    metrics.total_faces_or_quads = faces;
    metrics.total_vertices = vertices;
    metrics.total_indices = indices;
}

} // namespace voxel_lab
