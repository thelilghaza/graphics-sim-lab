#pragma once

#include "voxel_lab/chunk.hpp"
#include "voxel_lab/coordinates.hpp"
#include "voxel_lab/world_accessor.hpp"

#include <map>
#include <cstddef>

namespace voxel_lab {

class WorldGrid : public WorldAccessor {
public:
    WorldGrid() = default;

    using WorldAccessor::get_voxel;
    using WorldAccessor::set_voxel;
    using WorldAccessor::clear_voxel;
    using WorldAccessor::is_solid;
    using WorldAccessor::fill_box;
    using WorldAccessor::has_chunk;
    using WorldAccessor::has_voxel;

    Voxel get_voxel(const WorldCoord& w) const override;
    bool get_voxel(const WorldCoord& w, Voxel& out_voxel) const noexcept override;
    bool set_voxel(const WorldCoord& w, const Voxel& voxel) override;

    bool has_chunk(const ChunkCoord& c) const noexcept override;
    bool has_voxel(const WorldCoord& w) const noexcept override;

    Chunk* get_chunk(const ChunkCoord& c) noexcept;
    const Chunk* get_chunk(const ChunkCoord& c) const noexcept;
    Chunk& get_or_create_chunk(const ChunkCoord& c);

    bool remove_chunk(const ChunkCoord& c) noexcept;
    size_t loaded_chunk_count() const noexcept { return chunks.size(); }
    size_t count_solid_voxels() const;
    void clear() noexcept { chunks.clear(); }

private:
    std::map<ChunkCoord, Chunk> chunks;
};

} // namespace voxel_lab
