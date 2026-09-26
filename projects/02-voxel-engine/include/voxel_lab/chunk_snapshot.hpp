#pragma once

#include "voxel_lab/chunk.hpp"
#include "voxel_lab/coordinates.hpp"
#include "voxel_lab/world_accessor.hpp"
#include "voxel_lab/world_grid.hpp"

#include <array>

namespace voxel_lab {

// Read-only isolated snapshot of a target chunk and its 6 orthogonal boundary neighbor planes.
// Provides a consistent, race-free view of world data for worker threads during surface meshing.
class ChunkNeighborhoodSnapshot : public WorldGrid {
public:
    ChunkCoord center_coord;
    Chunk center_chunk;
    bool has_center_chunk{false};

    struct BoundaryFace {
        bool present{false};
        std::array<Voxel, CHUNK_DIM * CHUNK_DIM> voxels{};
    };

    // 0: PosX (+X: neighbor cx + 1, face lx = 0)
    // 1: NegX (-X: neighbor cx - 1, face lx = 31)
    // 2: PosY (+Y: neighbor cy + 1, face ly = 0)
    // 3: NegY (-Y: neighbor cy - 1, face ly = 31)
    // 4: PosZ (+Z: neighbor cz + 1, face lz = 0)
    // 5: NegZ (-Z: neighbor cz - 1, face lz = 31)
    std::array<BoundaryFace, 6> neighbor_faces{};

    ChunkNeighborhoodSnapshot() = default;
    explicit ChunkNeighborhoodSnapshot(const ChunkCoord& center) : center_coord(center) {}

    // Factory method to capture snapshot from WorldGrid safely on the main thread
    static ChunkNeighborhoodSnapshot capture(const WorldGrid& world, const ChunkCoord& coord, bool copy_center = true);

    Chunk* get_chunk(const ChunkCoord& c) noexcept override {
        if (c == center_coord) {
            has_center_chunk = true;
            return &center_chunk;
        }
        return nullptr;
    }

    const Chunk* get_chunk(const ChunkCoord& c) const noexcept override {
        if (c == center_coord && has_center_chunk) {
            return &center_chunk;
        }
        return nullptr;
    }

    // WorldAccessor / WorldGrid overrides
    using WorldGrid::get_voxel;
    using WorldGrid::set_voxel;
    using WorldGrid::clear_voxel;
    using WorldGrid::is_solid;
    using WorldGrid::fill_box;
    using WorldGrid::has_chunk;
    using WorldGrid::has_voxel;

    Voxel get_voxel(const WorldCoord& w) const override;
    bool get_voxel(const WorldCoord& w, Voxel& out_voxel) const noexcept override;
    bool set_voxel(const WorldCoord& w, const Voxel& voxel) override;
    bool has_chunk(const ChunkCoord& c) const noexcept override;
    bool has_voxel(const WorldCoord& w) const noexcept override;
};

} // namespace voxel_lab
