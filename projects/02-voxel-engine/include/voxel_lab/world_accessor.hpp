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

    // Voxel editing helpers
    virtual bool clear_voxel(const WorldCoord& w) {
        return set_voxel(w, Voxel(0, 0));
    }

    virtual bool is_solid(const WorldCoord& w) const {
        return get_voxel(w).is_solid();
    }

    virtual void fill_box(const WorldCoord& min_coord, const WorldCoord& max_coord, const Voxel& voxel) {
        int start_x = (min_coord.x < max_coord.x) ? min_coord.x : max_coord.x;
        int end_x = (min_coord.x > max_coord.x) ? min_coord.x : max_coord.x;
        int start_y = (min_coord.y < max_coord.y) ? min_coord.y : max_coord.y;
        int end_y = (min_coord.y > max_coord.y) ? min_coord.y : max_coord.y;
        int start_z = (min_coord.z < max_coord.z) ? min_coord.z : max_coord.z;
        int end_z = (min_coord.z > max_coord.z) ? min_coord.z : max_coord.z;

        for (int z = start_z; z <= end_z; ++z) {
            for (int y = start_y; y <= end_y; ++y) {
                for (int x = start_x; x <= end_x; ++x) {
                    set_voxel(WorldCoord(x, y, z), voxel);
                }
            }
        }
    }

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

    bool clear_voxel(int wx, int wy, int wz) {
        return clear_voxel(WorldCoord(wx, wy, wz));
    }

    bool is_solid(int wx, int wy, int wz) const {
        return is_solid(WorldCoord(wx, wy, wz));
    }

    void fill_box(int min_x, int min_y, int min_z, int max_x, int max_y, int max_z, const Voxel& voxel) {
        fill_box(WorldCoord(min_x, min_y, min_z), WorldCoord(max_x, max_y, max_z), voxel);
    }

    bool has_chunk(int cx, int cy, int cz) const noexcept {
        return has_chunk(ChunkCoord(cx, cy, cz));
    }

    bool has_voxel(int wx, int wy, int wz) const noexcept {
        return has_voxel(WorldCoord(wx, wy, wz));
    }
};

} // namespace voxel_lab
