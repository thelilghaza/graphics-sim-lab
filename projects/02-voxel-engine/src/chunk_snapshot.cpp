#include "voxel_lab/chunk_snapshot.hpp"

namespace voxel_lab {

ChunkNeighborhoodSnapshot ChunkNeighborhoodSnapshot::capture(const WorldGrid& world, const ChunkCoord& coord, bool copy_center) {
    ChunkNeighborhoodSnapshot snap(coord);
    if (copy_center) {
        const Chunk* center = world.get_chunk(coord);
        if (center) {
            snap.center_chunk = *center;
            snap.has_center_chunk = true;
        }
    }

    // 0: PosX (+X: neighbor cx + 1, face lx = 0)
    if (const Chunk* nb = world.get_chunk(ChunkCoord(coord.x + 1, coord.y, coord.z))) {
        snap.neighbor_faces[0].present = true;
        for (int lz = 0; lz < CHUNK_DIM; ++lz) {
            for (int ly = 0; ly < CHUNK_DIM; ++ly) {
                snap.neighbor_faces[0].voxels[ly + lz * CHUNK_DIM] = nb->get_voxel(0, ly, lz);
            }
        }
    }

    // 1: NegX (-X: neighbor cx - 1, face lx = 31)
    if (const Chunk* nb = world.get_chunk(ChunkCoord(coord.x - 1, coord.y, coord.z))) {
        snap.neighbor_faces[1].present = true;
        for (int lz = 0; lz < CHUNK_DIM; ++lz) {
            for (int ly = 0; ly < CHUNK_DIM; ++ly) {
                snap.neighbor_faces[1].voxels[ly + lz * CHUNK_DIM] = nb->get_voxel(31, ly, lz);
            }
        }
    }

    // 2: PosY (+Y: neighbor cy + 1, face ly = 0)
    if (const Chunk* nb = world.get_chunk(ChunkCoord(coord.x, coord.y + 1, coord.z))) {
        snap.neighbor_faces[2].present = true;
        for (int lz = 0; lz < CHUNK_DIM; ++lz) {
            for (int lx = 0; lx < CHUNK_DIM; ++lx) {
                snap.neighbor_faces[2].voxels[lx + lz * CHUNK_DIM] = nb->get_voxel(lx, 0, lz);
            }
        }
    }

    // 3: NegY (-Y: neighbor cy - 1, face ly = 31)
    if (const Chunk* nb = world.get_chunk(ChunkCoord(coord.x, coord.y - 1, coord.z))) {
        snap.neighbor_faces[3].present = true;
        for (int lz = 0; lz < CHUNK_DIM; ++lz) {
            for (int lx = 0; lx < CHUNK_DIM; ++lx) {
                snap.neighbor_faces[3].voxels[lx + lz * CHUNK_DIM] = nb->get_voxel(lx, 31, lz);
            }
        }
    }

    // 4: PosZ (+Z: neighbor cz + 1, face lz = 0)
    if (const Chunk* nb = world.get_chunk(ChunkCoord(coord.x, coord.y, coord.z + 1))) {
        snap.neighbor_faces[4].present = true;
        for (int ly = 0; ly < CHUNK_DIM; ++ly) {
            for (int lx = 0; lx < CHUNK_DIM; ++lx) {
                snap.neighbor_faces[4].voxels[lx + ly * CHUNK_DIM] = nb->get_voxel(lx, ly, 0);
            }
        }
    }

    // 5: NegZ (-Z: neighbor cz - 1, face lz = 31)
    if (const Chunk* nb = world.get_chunk(ChunkCoord(coord.x, coord.y, coord.z - 1))) {
        snap.neighbor_faces[5].present = true;
        for (int ly = 0; ly < CHUNK_DIM; ++ly) {
            for (int lx = 0; lx < CHUNK_DIM; ++lx) {
                snap.neighbor_faces[5].voxels[lx + ly * CHUNK_DIM] = nb->get_voxel(lx, ly, 31);
            }
        }
    }

    return snap;
}

Voxel ChunkNeighborhoodSnapshot::get_voxel(const WorldCoord& w) const {
    Voxel v(0, 0);
    get_voxel(w, v);
    return v;
}

bool ChunkNeighborhoodSnapshot::get_voxel(const WorldCoord& w, Voxel& out_voxel) const noexcept {
    auto [c, l] = decompose_world_coord(w);

    if (c == center_coord) {
        if (has_center_chunk) {
            out_voxel = center_chunk.get_voxel(l.x, l.y, l.z);
            return true;
        }
        return false;
    }

    // Check 6 boundary planes
    if (c.x == center_coord.x + 1 && c.y == center_coord.y && c.z == center_coord.z && l.x == 0) {
        if (neighbor_faces[0].present) {
            out_voxel = neighbor_faces[0].voxels[l.y + l.z * CHUNK_DIM];
            return true;
        }
        return false;
    }
    if (c.x == center_coord.x - 1 && c.y == center_coord.y && c.z == center_coord.z && l.x == 31) {
        if (neighbor_faces[1].present) {
            out_voxel = neighbor_faces[1].voxels[l.y + l.z * CHUNK_DIM];
            return true;
        }
        return false;
    }
    if (c.y == center_coord.y + 1 && c.x == center_coord.x && c.z == center_coord.z && l.y == 0) {
        if (neighbor_faces[2].present) {
            out_voxel = neighbor_faces[2].voxels[l.x + l.z * CHUNK_DIM];
            return true;
        }
        return false;
    }
    if (c.y == center_coord.y - 1 && c.x == center_coord.x && c.z == center_coord.z && l.y == 31) {
        if (neighbor_faces[3].present) {
            out_voxel = neighbor_faces[3].voxels[l.x + l.z * CHUNK_DIM];
            return true;
        }
        return false;
    }
    if (c.z == center_coord.z + 1 && c.x == center_coord.x && c.y == center_coord.y && l.z == 0) {
        if (neighbor_faces[4].present) {
            out_voxel = neighbor_faces[4].voxels[l.x + l.y * CHUNK_DIM];
            return true;
        }
        return false;
    }
    if (c.z == center_coord.z - 1 && c.x == center_coord.x && c.y == center_coord.y && l.z == 31) {
        if (neighbor_faces[5].present) {
            out_voxel = neighbor_faces[5].voxels[l.x + l.y * CHUNK_DIM];
            return true;
        }
        return false;
    }

    return false;
}

bool ChunkNeighborhoodSnapshot::set_voxel(const WorldCoord& w, const Voxel& voxel) {
    auto [c, l] = decompose_world_coord(w);
    if (c == center_coord) {
        has_center_chunk = true;
        return center_chunk.set_voxel(l.x, l.y, l.z, voxel);
    }
    return false;
}

bool ChunkNeighborhoodSnapshot::has_chunk(const ChunkCoord& c) const noexcept {
    if (c == center_coord) {
        return has_center_chunk;
    }
    if (c == ChunkCoord(center_coord.x + 1, center_coord.y, center_coord.z)) return neighbor_faces[0].present;
    if (c == ChunkCoord(center_coord.x - 1, center_coord.y, center_coord.z)) return neighbor_faces[1].present;
    if (c == ChunkCoord(center_coord.x, center_coord.y + 1, center_coord.z)) return neighbor_faces[2].present;
    if (c == ChunkCoord(center_coord.x, center_coord.y - 1, center_coord.z)) return neighbor_faces[3].present;
    if (c == ChunkCoord(center_coord.x, center_coord.y, center_coord.z + 1)) return neighbor_faces[4].present;
    if (c == ChunkCoord(center_coord.x, center_coord.y, center_coord.z - 1)) return neighbor_faces[5].present;
    return false;
}

bool ChunkNeighborhoodSnapshot::has_voxel(const WorldCoord& w) const noexcept {
    return has_chunk(world_to_chunk(w));
}

} // namespace voxel_lab
