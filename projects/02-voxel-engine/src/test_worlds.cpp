#include "voxel_lab/test_worlds.hpp"
#include <algorithm>

namespace voxel_lab {

void generate_solid_world(WorldAccessor& world,
                          const WorldCoord& min_coord,
                          const WorldCoord& max_coord,
                          const Voxel& solid_voxel) {
    int start_x = std::min(min_coord.x, max_coord.x);
    int end_x = std::max(min_coord.x, max_coord.x);
    int start_y = std::min(min_coord.y, max_coord.y);
    int end_y = std::max(min_coord.y, max_coord.y);
    int start_z = std::min(min_coord.z, max_coord.z);
    int end_z = std::max(min_coord.z, max_coord.z);

    for (int z = start_z; z <= end_z; ++z) {
        for (int y = start_y; y <= end_y; ++y) {
            for (int x = start_x; x <= end_x; ++x) {
                world.set_voxel(WorldCoord(x, y, z), solid_voxel);
            }
        }
    }
}

void generate_plane_world(WorldAccessor& world,
                          const WorldCoord& min_coord,
                          const WorldCoord& max_coord,
                          int plane_coord,
                          PlaneAxis axis,
                          const Voxel& solid_voxel) {
    int start_x = std::min(min_coord.x, max_coord.x);
    int end_x = std::max(min_coord.x, max_coord.x);
    int start_y = std::min(min_coord.y, max_coord.y);
    int end_y = std::max(min_coord.y, max_coord.y);
    int start_z = std::min(min_coord.z, max_coord.z);
    int end_z = std::max(min_coord.z, max_coord.z);

    for (int z = start_z; z <= end_z; ++z) {
        for (int y = start_y; y <= end_y; ++y) {
            for (int x = start_x; x <= end_x; ++x) {
                int c = (axis == PlaneAxis::X) ? x : ((axis == PlaneAxis::Y) ? y : z);
                if (c <= plane_coord) {
                    world.set_voxel(WorldCoord(x, y, z), solid_voxel);
                } else {
                    world.set_voxel(WorldCoord(x, y, z), Voxel(0, 0));
                }
            }
        }
    }
}

void generate_sphere_world(WorldAccessor& world,
                           const WorldCoord& center,
                           int radius,
                           const Voxel& solid_voxel) {
    if (radius < 0) {
        return;
    }
    int radius_sq = radius * radius;
    for (int z = center.z - radius; z <= center.z + radius; ++z) {
        int dz = z - center.z;
        int dz_sq = dz * dz;
        for (int y = center.y - radius; y <= center.y + radius; ++y) {
            int dy = y - center.y;
            int dy_sq = dy * dy;
            for (int x = center.x - radius; x <= center.x + radius; ++x) {
                int dx = x - center.x;
                int dist_sq = dx * dx + dy_sq + dz_sq;
                if (dist_sq <= radius_sq) {
                    world.set_voxel(WorldCoord(x, y, z), solid_voxel);
                } else {
                    world.set_voxel(WorldCoord(x, y, z), Voxel(0, 0));
                }
            }
        }
    }
}

} // namespace voxel_lab
