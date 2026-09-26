#pragma once

#include "voxel_lab/coordinates.hpp"
#include "voxel_lab/greedy_mesher.hpp"
#include "voxel_lab/mesh.hpp"
#include "voxel_lab/naive_mesher.hpp"
#include "voxel_lab/world_accessor.hpp"

namespace voxel_lab {

// Determines LOD level based on Chebyshev distance between chunk and camera chunk
LODLevel select_lod_level(const ChunkCoord& chunk,
                         const ChunkCoord& cam_chunk,
                         bool enable_lod,
                         int lod0_radius,
                         int lod1_radius) noexcept;

// Generates mesh data for a single chunk at specified LOD level and mesher algorithm
MeshData mesh_chunk_lod(const WorldAccessor& world,
                       const ChunkCoord& chunk_coord,
                       LODLevel lod,
                       MesherType mesher = MesherType::Greedy);

// Appends mesh data for a single chunk at specified LOD level to out_mesh
void mesh_chunk_lod(const WorldAccessor& world,
                    const ChunkCoord& chunk_coord,
                    LODLevel lod,
                    MesherType mesher,
                    MeshData& out_mesh);

} // namespace voxel_lab
