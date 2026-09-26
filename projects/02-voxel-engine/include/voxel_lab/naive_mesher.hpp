#pragma once

#include "voxel_lab/coordinates.hpp"
#include "voxel_lab/mesh.hpp"
#include "voxel_lab/world_accessor.hpp"

namespace voxel_lab {

enum class Direction {
    PosX, // +X
    NegX, // -X
    PosY, // +Y
    NegY, // -Y
    PosZ, // +Z
    NegZ  // -Z
};

// Generates mesh data for a single chunk using naive exposed-face culling
MeshData mesh_chunk(const WorldAccessor& world, const ChunkCoord& chunk_coord);

// Appends naive exposed-face mesh data for a single chunk to out_mesh
void mesh_chunk(const WorldAccessor& world, const ChunkCoord& chunk_coord, MeshData& out_mesh);

} // namespace voxel_lab
