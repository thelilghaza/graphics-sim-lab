#pragma once

#include "voxel_lab/coordinates.hpp"
#include "voxel_lab/voxel.hpp"

namespace voxel_lab {

class WorldAccessor {
public:
    virtual ~WorldAccessor() = default;

    virtual Voxel get_voxel(const WorldCoord& w) const = 0;
    virtual bool get_voxel(const WorldCoord& w, Voxel& out_voxel) const noexcept = 0;
    virtual bool set_voxel(const WorldCoord& w, const Voxel& voxel) = 0;

    virtual bool has_chunk(const ChunkCoord& c) const noexcept = 0;
    virtual bool has_voxel(const WorldCoord& w) const noexcept = 0;

    // Convenience overloads taking scalar coordinates
    Voxel get_voxel(int wx, int wy, int wz) const {
        return get_voxel(WorldCoord(wx, wy, wz));
    }

    bool get_voxel(int wx, int wy, int wz, Voxel& out_voxel) const noexcept {
        return get_voxel(WorldCoord(wx, wy, wz), out_voxel);
    }

    bool set_voxel(int wx, int wy, int wz, const Voxel& voxel) {
        return set_voxel(WorldCoord(wx, wy, wz), voxel);
    }

    bool has_chunk(int cx, int cy, int cz) const noexcept {
        return has_chunk(ChunkCoord(cx, cy, cz));
    }

    bool has_voxel(int wx, int wy, int wz) const noexcept {
        return has_voxel(WorldCoord(wx, wy, wz));
    }
};

} // namespace voxel_lab
