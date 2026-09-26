#include "voxel_lab/world_grid.hpp"

namespace voxel_lab {

Voxel WorldGrid::get_voxel(const WorldCoord& w) const {
    ChunkCoord c = world_to_chunk(w);
    const Chunk* chunk = get_chunk(c);
    if (!chunk) {
        return Voxel(0, 0); // Default air voxel for unpopulated chunks
    }
    LocalCoord l = world_to_local(w);
    return chunk->get_voxel(l.x, l.y, l.z);
}

bool WorldGrid::get_voxel(const WorldCoord& w, Voxel& out_voxel) const noexcept {
    ChunkCoord c = world_to_chunk(w);
    const Chunk* chunk = get_chunk(c);
    if (!chunk) {
        return false;
    }
    LocalCoord l = world_to_local(w);
    return chunk->get_voxel(l.x, l.y, l.z, out_voxel);
}

bool WorldGrid::set_voxel(const WorldCoord& w, const Voxel& voxel) {
    auto [c, l] = decompose_world_coord(w);
    Chunk& chunk = get_or_create_chunk(c);
    return chunk.set_voxel(l.x, l.y, l.z, voxel);
}

bool WorldGrid::has_chunk(const ChunkCoord& c) const noexcept {
    return chunks.find(c) != chunks.end();
}

bool WorldGrid::has_voxel(const WorldCoord& w) const noexcept {
    return has_chunk(world_to_chunk(w));
}

Chunk* WorldGrid::get_chunk(const ChunkCoord& c) noexcept {
    auto it = chunks.find(c);
    if (it == chunks.end()) {
        return nullptr;
    }
    return &it->second;
}

const Chunk* WorldGrid::get_chunk(const ChunkCoord& c) const noexcept {
    auto it = chunks.find(c);
    if (it == chunks.end()) {
        return nullptr;
    }
    return &it->second;
}

Chunk& WorldGrid::get_or_create_chunk(const ChunkCoord& c) {
    return chunks[c];
}

bool WorldGrid::remove_chunk(const ChunkCoord& c) noexcept {
    return chunks.erase(c) > 0;
}

size_t WorldGrid::count_solid_voxels() const {
    size_t count = 0;
    for (const auto& entry : chunks) {
        const Chunk& chunk = entry.second;
        for (int z = 0; z < CHUNK_DIM; ++z) {
            for (int y = 0; y < CHUNK_DIM; ++y) {
                for (int x = 0; x < CHUNK_DIM; ++x) {
                    if (chunk.get_voxel(x, y, z).is_solid()) {
                        ++count;
                    }
                }
            }
        }
    }
    return count;
}

} // namespace voxel_lab
