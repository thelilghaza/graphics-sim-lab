#pragma once

#include "voxel_lab/coordinates.hpp"
#include "voxel_lab/voxel.hpp"
#include "voxel_lab/world_accessor.hpp"

namespace voxel_lab {

enum class PlaneAxis {
    X,
    Y,
    Z
};

// Generates a solid block of voxels within [min_coord, max_coord] inclusive
void generate_solid_world(WorldAccessor& world,
                          const WorldCoord& min_coord,
                          const WorldCoord& max_coord,
                          const Voxel& solid_voxel = Voxel(1, 1));

// Scalar convenience overload for generate_solid_world
inline void generate_solid_world(WorldAccessor& world,
                                 int min_x, int min_y, int min_z,
                                 int max_x, int max_y, int max_z,
                                 const Voxel& solid_voxel = Voxel(1, 1)) {
    generate_solid_world(world, WorldCoord(min_x, min_y, min_z), WorldCoord(max_x, max_y, max_z), solid_voxel);
}

// Generates a planar surface where coordinates along 'axis' <= 'plane_coord' are filled with solid_voxel,
// and voxels > 'plane_coord' within [min_coord, max_coord] are filled with air Voxel(0,0).
void generate_plane_world(WorldAccessor& world,
                          const WorldCoord& min_coord,
                          const WorldCoord& max_coord,
                          int plane_coord,
                          PlaneAxis axis = PlaneAxis::Y,
                          const Voxel& solid_voxel = Voxel(1, 1));

// Scalar convenience overload for generate_plane_world
inline void generate_plane_world(WorldAccessor& world,
                                 int min_x, int min_y, int min_z,
                                 int max_x, int max_y, int max_z,
                                 int plane_coord,
                                 PlaneAxis axis = PlaneAxis::Y,
                                 const Voxel& solid_voxel = Voxel(1, 1)) {
    generate_plane_world(world, WorldCoord(min_x, min_y, min_z), WorldCoord(max_x, max_y, max_z), plane_coord, axis, solid_voxel);
}

// Generates a sphere centered at 'center' with integer 'radius', filled with solid_voxel
void generate_sphere_world(WorldAccessor& world,
                           const WorldCoord& center,
                           int radius,
                           const Voxel& solid_voxel = Voxel(1, 1));

// Scalar convenience overload for generate_sphere_world
inline void generate_sphere_world(WorldAccessor& world,
                                 int center_x, int center_y, int center_z,
                                 int radius,
                                 const Voxel& solid_voxel = Voxel(1, 1)) {
    generate_sphere_world(world, WorldCoord(center_x, center_y, center_z), radius, solid_voxel);
}

} // namespace voxel_lab
