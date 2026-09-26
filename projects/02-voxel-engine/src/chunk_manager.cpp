#include "voxel_lab/chunk_manager.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <psapi.h>
#endif

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
    ChunkNeighborhoodSnapshot* snap = dynamic_cast<ChunkNeighborhoodSnapshot*>(&world);
    Chunk* chunk = grid ? grid->get_chunk(coord) : (snap ? snap->get_chunk(coord) : nullptr);

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
    : config(cfg), mesher_type(mesher), generator(std::move(gen)) {
    if (config.worker_count > 0) {
        thread_pool = std::make_unique<ThreadPool>(config.worker_count);
    }
    metrics.worker_count = config.worker_count;
}

ChunkManager::~ChunkManager() {
    if (thread_pool) {
        thread_pool->stop();
    }
}

bool ChunkManager::update_streaming(const Vec3& camera_world_position, bool force) {
    WorldCoord w(
        static_cast<int>(std::floor(camera_world_position.x)),
        static_cast<int>(std::floor(camera_world_position.y)),
        static_cast<int>(std::floor(camera_world_position.z))
    );
    return update_streaming(w, force);
}

bool ChunkManager::update_streaming(const WorldCoord& camera_world_coord, bool force) {
    auto t_start = std::chrono::high_resolution_clock::now();

    recently_updated_meshes.clear();
    recently_unloaded_meshes.clear();
    metrics.chunks_loaded_this_update = 0;
    metrics.chunks_unloaded_this_update = 0;
    metrics.chunks_meshed_this_update = 0;

    // 1. First integrate any completed async results
    size_t integrated = integrate_completed_jobs();

    ChunkCoord new_cam_chunk = world_to_chunk(camera_world_coord);

    if (!force && has_cam_chunk && new_cam_chunk == current_cam_chunk) {
        auto t_end = std::chrono::high_resolution_clock::now();
        metrics.last_streaming_update_time_us = std::chrono::duration<double, std::micro>(t_end - t_start).count();
        return integrated > 0;
    }

    has_cam_chunk = true;
    current_cam_chunk = new_cam_chunk;

    int r_load = std::max(0, config.load_radius);
    int r_unload = std::max(r_load, config.unload_radius);

    // 2. Identify newly desired chunks within load radius
    std::set<ChunkCoord> new_desired;
    for (int dz = -r_load; dz <= r_load; ++dz) {
        for (int dy = -r_load; dy <= r_load; ++dy) {
            for (int dx = -r_load; dx <= r_load; ++dx) {
                new_desired.insert(ChunkCoord(current_cam_chunk.x + dx, current_cam_chunk.y + dy, current_cam_chunk.z + dz));
            }
        }
    }

    // Retain desired chunks that are within unload radius (hysteresis band)
    for (const auto& c : desired_chunks) {
        if (chebyshev_distance(c, current_cam_chunk) <= r_unload) {
            new_desired.insert(c);
        }
    }

    // 3. Identify chunks outside unload radius to evict
    std::vector<ChunkCoord> to_unload;
    for (const auto& c : desired_chunks) {
        if (new_desired.find(c) == new_desired.end()) {
            to_unload.push_back(c);
        }
    }
    for (const auto& c : loaded_chunks) {
        if (new_desired.find(c) == new_desired.end() && std::find(to_unload.begin(), to_unload.end(), c) == to_unload.end()) {
            to_unload.push_back(c);
        }
    }

    // 4. Identify missing chunks to load or chunks needing LOD update
    std::vector<std::pair<ChunkCoord, LODLevel>> to_load;
    std::vector<std::pair<ChunkCoord, LODLevel>> to_remesh_lod;

    for (const auto& c : new_desired) {
        LODLevel target_lod = select_lod_level(c, current_cam_chunk, config.enable_lod, config.lod0_radius, config.lod1_radius);
        if (loaded_chunks.find(c) == loaded_chunks.end()) {
            to_load.push_back({c, target_lod});
        } else {
            LODLevel current_lod = get_chunk_lod(c);
            if (current_lod != target_lod || force) {
                if (current_lod != target_lod) {
                    metrics.lod_changes++;
                }
                to_remesh_lod.push_back({c, target_lod});
            }
        }
    }

    desired_chunks = std::move(new_desired);

    // Execute unloads
    for (const auto& c : to_unload) {
        chunk_versions[c] = ++next_version; // Invalidate any in-flight jobs for this coord
        loaded_chunks.erase(c);
        meshes.erase(c);
        chunk_lods.erase(c);
        world_grid.remove_chunk(c);
        recently_unloaded_meshes.push_back(c);

        metrics.chunks_unloaded_this_update++;
        metrics.total_chunks_unloaded++;

        // Notify resident neighbors that c was unloaded
        for (const auto& n : get_chunk_neighbors(c)) {
            if (is_loaded(n) && desired_chunks.find(n) != desired_chunks.end()) {
                queue_remesh(n, get_chunk_lod(n));
            }
        }
    }

    // Execute loads
    for (const auto& [c, lod] : to_load) {
        queue_build(c, true, lod);
    }

    // Execute LOD updates
    for (const auto& [c, lod] : to_remesh_lod) {
        queue_remesh(c, lod);
    }

    // If synchronous mode (worker_count == 0), wait for all immediate jobs
    if (config.worker_count == 0) {
        integrate_completed_jobs();
    }

    metrics.currently_loaded_chunks = loaded_chunks.size();
    if (thread_pool) {
        metrics.jobs_pending = thread_pool->pending_tasks();
    }
    recalculate_aggregate_mesh_metrics();

    auto t_end = std::chrono::high_resolution_clock::now();
    metrics.last_streaming_update_time_us = std::chrono::duration<double, std::micro>(t_end - t_start).count();

    return integrated > 0 || !to_unload.empty() || !to_load.empty();
}

size_t ChunkManager::integrate_completed_jobs() {
    size_t total_integrated = 0;

    while (true) {
        std::vector<ChunkBuildResult> results;
        {
            std::lock_guard<std::mutex> lock(completed_mutex);
            if (completed_queue.empty()) {
                break;
            }
            results.reserve(completed_queue.size());
            while (!completed_queue.empty()) {
                results.push_back(std::move(completed_queue.front()));
                completed_queue.pop_front();
            }
        }

        std::vector<ChunkCoord> new_chunks_for_neighbor_remesh;

        for (auto& res : results) {
            metrics.jobs_completed++;
            metrics.total_generation_time_us += res.generation_time_us;
            metrics.total_mesh_time_us += res.mesh_time_us;
            metrics.total_cpu_build_time_us += (res.generation_time_us + res.mesh_time_us);

            // Stale Job Protection:
            // 1. Must still be in chunk_versions
            // 2. Version must match the latest version requested
            // 3. Chunk must currently be desired
            // 4. Mesher must match current mesher
            auto it = chunk_versions.find(res.coord);
            if (it == chunk_versions.end() || it->second != res.version ||
                desired_chunks.find(res.coord) == desired_chunks.end() ||
                res.mesher_type != mesher_type) {
                metrics.jobs_discarded_stale++;
                recycle_mesh_buffer(std::move(res.mesh));
                continue;
            }

            // Integrate voxel chunk
            if (res.need_generation) {
                world_grid.set_chunk(res.coord, std::move(res.chunk));
                loaded_chunks.insert(res.coord);
                metrics.total_chunks_loaded++;
                metrics.chunks_loaded_this_update++;
                metrics.chunks_generated++;
                new_chunks_for_neighbor_remesh.push_back(res.coord);
            }

            // Integrate mesh (recycle previous mesh buffer if replacing)
            auto mit = meshes.find(res.coord);
            if (mit != meshes.end()) {
                recycle_mesh_buffer(std::move(mit->second));
                mit->second = std::move(res.mesh);
            } else {
                meshes[res.coord] = std::move(res.mesh);
            }
            chunk_lods[res.coord] = res.lod;
            recently_updated_meshes.push_back(res.coord);
            metrics.chunks_meshed_this_update++;
            metrics.total_chunks_meshed++;
            metrics.chunks_integrated++;
            total_integrated++;
        }

        // For any newly loaded chunks, trigger neighbor remeshing
        for (const auto& c : new_chunks_for_neighbor_remesh) {
            for (const auto& n : get_chunk_neighbors(c)) {
                if (is_loaded(n) && desired_chunks.find(n) != desired_chunks.end()) {
                    queue_remesh(n, get_chunk_lod(n));
                }
            }
        }

        // In multithreaded mode, new neighbor remeshes are queued to workers; break.
        // In synchronous mode (no thread_pool), new neighbor remeshes were executed into completed_queue, so loop to integrate them.
        if (thread_pool) {
            break;
        }
    }

    metrics.currently_loaded_chunks = loaded_chunks.size();
    if (thread_pool) {
        metrics.jobs_pending = thread_pool->pending_tasks();
    }
    recalculate_aggregate_mesh_metrics();

    return total_integrated;
}

LODLevel ChunkManager::get_chunk_lod(const ChunkCoord& chunk_coord) const noexcept {
    auto it = chunk_lods.find(chunk_coord);
    if (it != chunk_lods.end()) {
        return it->second;
    }
    return LODLevel::LOD0;
}

void ChunkManager::wait_all_pending() {
    if (thread_pool) {
        thread_pool->wait_idle();
        integrate_completed_jobs();

        // Loop until cascading neighbor remeshes and completion queue are completely drained
        while (thread_pool->pending_tasks() > 0) {
            thread_pool->wait_idle();
            integrate_completed_jobs();
        }
    } else {
        integrate_completed_jobs();
    }
}

bool ChunkManager::is_loaded(const ChunkCoord& chunk_coord) const noexcept {
    return loaded_chunks.find(chunk_coord) != loaded_chunks.end();
}

bool ChunkManager::is_desired(const ChunkCoord& chunk_coord) const noexcept {
    return desired_chunks.find(chunk_coord) != desired_chunks.end();
}

size_t ChunkManager::loaded_chunk_count() const noexcept {
    return loaded_chunks.size();
}

std::vector<ChunkCoord> ChunkManager::get_loaded_chunk_coordinates() const {
    return std::vector<ChunkCoord>(loaded_chunks.begin(), loaded_chunks.end());
}

bool ChunkManager::load_chunk(const ChunkCoord& chunk_coord) {
    if (is_loaded(chunk_coord) || desired_chunks.find(chunk_coord) != desired_chunks.end()) {
        return false;
    }

    desired_chunks.insert(chunk_coord);
    LODLevel target_lod = select_lod_level(chunk_coord, current_cam_chunk, config.enable_lod, config.lod0_radius, config.lod1_radius);
    queue_build(chunk_coord, true, target_lod);

    if (config.worker_count > 0) {
        wait_all_pending();
    } else {
        integrate_completed_jobs();
    }
    return true;
}

bool ChunkManager::unload_chunk(const ChunkCoord& chunk_coord) {
    if (!is_loaded(chunk_coord) && desired_chunks.find(chunk_coord) == desired_chunks.end()) {
        return false;
    }

    chunk_versions[chunk_coord] = ++next_version; // Invalidate any in-flight work
    desired_chunks.erase(chunk_coord);
    loaded_chunks.erase(chunk_coord);
    chunk_lods.erase(chunk_coord);

    auto mit = meshes.find(chunk_coord);
    if (mit != meshes.end()) {
        recycle_mesh_buffer(std::move(mit->second));
        meshes.erase(mit);
    }

    world_grid.remove_chunk(chunk_coord);
    recently_unloaded_meshes.push_back(chunk_coord);

    metrics.chunks_unloaded_this_update++;
    metrics.total_chunks_unloaded++;

    // Invalidate neighbors
    for (const auto& n : get_chunk_neighbors(chunk_coord)) {
        if (is_loaded(n) && desired_chunks.find(n) != desired_chunks.end()) {
            queue_remesh(n, get_chunk_lod(n));
        }
    }

    if (config.worker_count > 0) {
        wait_all_pending();
    } else {
        integrate_completed_jobs();
    }
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

    // Requeue remesh for all resident chunks with new mesher
    for (const auto& c : loaded_chunks) {
        if (desired_chunks.find(c) != desired_chunks.end()) {
            queue_remesh(c, get_chunk_lod(c));
        }
    }

    if (config.worker_count > 0) {
        wait_all_pending();
    } else {
        integrate_completed_jobs();
    }
}

void ChunkManager::set_config(const StreamingConfig& new_config) {
    if (config.worker_count != new_config.worker_count) {
        wait_all_pending();
        if (thread_pool) {
            thread_pool->stop();
            thread_pool.reset();
        }
        if (new_config.worker_count > 0) {
            thread_pool = std::make_unique<ThreadPool>(new_config.worker_count);
        }
        metrics.worker_count = new_config.worker_count;
    }
    if (!new_config.enable_mesh_buffer_reuse) {
        clear_recycled_mesh_buffers();
    }
    config = new_config;
}

void ChunkManager::set_generator(ChunkGenerator gen) {
    generator = std::move(gen);
}

size_t ChunkManager::get_pending_job_count() const noexcept {
    if (thread_pool) {
        return thread_pool->pending_tasks();
    }
    return 0;
}

void ChunkManager::reset_cumulative_metrics() noexcept {
    metrics.total_chunks_loaded = 0;
    metrics.total_chunks_unloaded = 0;
    metrics.total_chunks_meshed = 0;
    metrics.jobs_submitted = 0;
    metrics.jobs_completed = 0;
    metrics.jobs_discarded_stale = 0;
    metrics.chunks_generated = 0;
    metrics.chunks_integrated = 0;
    metrics.total_generation_time_us = 0.0;
    metrics.total_mesh_time_us = 0.0;
    metrics.total_cpu_build_time_us = 0.0;
    metrics.mesh_buffers_reused = 0;
    metrics.mesh_buffers_allocated_fresh = 0;
    metrics.mesh_buffers_recycled = 0;
    metrics.mesh_buffers_evicted_from_pool = 0;
}

void ChunkManager::clear_recycled_mesh_buffers() noexcept {
    recycled_mesh_buffers.clear();
    recycled_mesh_buffers.shrink_to_fit();
}

MeshData ChunkManager::acquire_mesh_buffer() {
    if (config.enable_mesh_buffer_reuse && !recycled_mesh_buffers.empty()) {
        MeshData buf = std::move(recycled_mesh_buffers.back());
        recycled_mesh_buffers.pop_back();
        buf.clear();
        metrics.mesh_buffers_reused++;
        return buf;
    }
    metrics.mesh_buffers_allocated_fresh++;
    MeshData buf;
    if (config.enable_mesh_buffer_reuse) {
        // Pre-reserve a baseline capacity to minimize initial micro-reallocations
        buf.vertices.reserve(1024);
        buf.indices.reserve(1536);
    }
    return buf;
}

void ChunkManager::recycle_mesh_buffer(MeshData mesh) {
    if (!config.enable_mesh_buffer_reuse) {
        return;
    }
    if (recycled_mesh_buffers.size() < config.max_recycled_mesh_buffers) {
        mesh.clear();
        recycled_mesh_buffers.push_back(std::move(mesh));
        metrics.mesh_buffers_recycled++;
    } else {
        metrics.mesh_buffers_evicted_from_pool++;
    }
}

ChunkManagerMemoryStats ChunkManager::get_memory_stats() const {
    ChunkManagerMemoryStats stats;
    stats.resident_chunk_count = loaded_chunks.size();
    stats.raw_chunk_payload_bytes = stats.resident_chunk_count * sizeof(Chunk);
    stats.mesh_count = meshes.size();

    for (const auto& [coord, mesh] : meshes) {
        stats.total_mesh_logical_bytes += mesh.total_logical_bytes();
        stats.total_mesh_capacity_bytes += mesh.total_capacity_bytes();
    }

    stats.recycled_mesh_buffer_count = recycled_mesh_buffers.size();
    for (const auto& mesh : recycled_mesh_buffers) {
        stats.recycled_mesh_capacity_bytes += mesh.total_capacity_bytes();
    }

    // std::map node in 64-bit MSVC: ChunkCoord (12B) + Chunk (65536B) + 3 pointers (24B) + color/flags (8B) ~ 65584B
    stats.estimated_world_grid_node_bytes = stats.resident_chunk_count * (sizeof(Chunk) + 48);
    stats.estimated_total_logical_bytes = stats.raw_chunk_payload_bytes + stats.total_mesh_logical_bytes;

#ifdef _WIN32
    PROCESS_MEMORY_COUNTERS_EX pmc;
    if (GetProcessMemoryInfo(GetCurrentProcess(), reinterpret_cast<PROCESS_MEMORY_COUNTERS*>(&pmc), sizeof(pmc))) {
        stats.process_working_set_bytes = static_cast<size_t>(pmc.WorkingSetSize);
        stats.process_private_bytes = static_cast<size_t>(pmc.PrivateUsage);
    }
#endif

    return stats;
}

void ChunkManager::queue_build(const ChunkCoord& coord, bool need_generation, LODLevel lod) {
    uint64_t v = ++next_version;
    chunk_versions[coord] = v;

    ChunkNeighborhoodSnapshot snap = ChunkNeighborhoodSnapshot::capture(world_grid, coord, !need_generation);

    ChunkBuildTask task{
        coord,
        v,
        need_generation,
        mesher_type,
        lod,
        std::move(snap),
        generator,
        acquire_mesh_buffer()
    };

    metrics.jobs_submitted++;

    if (thread_pool) {
        thread_pool->enqueue([this, t = std::move(task)]() mutable {
            execute_task_sync(std::move(t));
        });
    } else {
        execute_task_sync(std::move(task));
    }
}

void ChunkManager::queue_remesh(const ChunkCoord& coord, LODLevel lod) {
    queue_build(coord, false, lod);
}

void ChunkManager::execute_task_sync(ChunkBuildTask task) {
    ChunkBuildResult res;
    res.coord = task.coord;
    res.version = task.version;
    res.need_generation = task.need_generation;
    res.mesher_type = task.mesher_type;
    res.lod = task.lod;

    if (task.need_generation) {
        auto t0 = std::chrono::high_resolution_clock::now();
        if (task.generator) {
            task.generator(task.snapshot, task.coord);
        } else {
            generate_default_terrain_chunk(task.snapshot, task.coord);
        }
        auto t1 = std::chrono::high_resolution_clock::now();
        res.generation_time_us = std::chrono::duration<double, std::micro>(t1 - t0).count();
    }

    auto t2 = std::chrono::high_resolution_clock::now();
    task.mesh.clear();
    mesh_chunk_lod(task.snapshot, task.coord, task.lod, task.mesher_type, task.mesh);
    auto t3 = std::chrono::high_resolution_clock::now();
    res.mesh_time_us = std::chrono::duration<double, std::micro>(t3 - t2).count();

    res.mesh = std::move(task.mesh);

    if (task.need_generation) {
        res.chunk = std::move(task.snapshot.center_chunk);
    }

    {
        std::lock_guard<std::mutex> lock(completed_mutex);
        completed_queue.push_back(std::move(res));
    }
}

void ChunkManager::recalculate_aggregate_mesh_metrics() {
    size_t faces = 0;
    size_t vertices = 0;
    size_t indices = 0;
    size_t l0_cnt = 0;
    size_t l1_cnt = 0;
    size_t l2_cnt = 0;

    for (const auto& [coord, mesh] : meshes) {
        faces += mesh.face_count();
        vertices += mesh.vertex_count();
        indices += mesh.index_count();

        auto lit = chunk_lods.find(coord);
        LODLevel l = (lit != chunk_lods.end()) ? lit->second : LODLevel::LOD0;
        if (l == LODLevel::LOD0) l0_cnt++;
        else if (l == LODLevel::LOD1) l1_cnt++;
        else l2_cnt++;
    }

    metrics.total_faces_or_quads = faces;
    metrics.total_vertices = vertices;
    metrics.total_indices = indices;
    metrics.lod0_chunk_count = l0_cnt;
    metrics.lod1_chunk_count = l1_cnt;
    metrics.lod2_chunk_count = l2_cnt;
}

} // namespace voxel_lab
