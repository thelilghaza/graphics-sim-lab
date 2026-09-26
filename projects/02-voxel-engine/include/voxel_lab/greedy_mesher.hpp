#pragma once

#include "voxel_lab/coordinates.hpp"
#include "voxel_lab/mesh.hpp"
#include "voxel_lab/world_accessor.hpp"

namespace voxel_lab {

// Generates mesh data for a single chunk using greedy meshing
MeshData greedy_mesh_chunk(const WorldAccessor& world, const ChunkCoord& chunk_coord);

// Appends greedy mesh data for a single chunk to out_mesh
void greedy_mesh_chunk(const WorldAccessor& world, const ChunkCoord& chunk_coord, MeshData& out_mesh);

} // namespace voxel_lab
