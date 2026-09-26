#include "voxel_lab/lod.hpp"
#include "voxel_lab/chunk.hpp"
#include "voxel_lab/greedy_mesher.hpp"
#include "voxel_lab/naive_mesher.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace voxel_lab {

namespace {

enum class FaceDir {
    PosX, // +X
    NegX, // -X
    PosY, // +Y
    NegY, // -Y
    PosZ, // +Z
    NegZ  // -Z
};

struct CoarseCell {
    bool solid{false};
    uint8_t type_id{0};
};

CoarseCell evaluate_coarse_cell(const WorldAccessor& world,
                               const ChunkCoord& chunk_coord,
                               int cu, int cv, int cw,
                               int step) {
    CoarseCell cell;
    int lx_base = cu * step;
    int ly_base = cv * step;
    int lz_base = cw * step;

    for (int dz = 0; dz < step; ++dz) {
        for (int dy = 0; dy < step; ++dy) {
            for (int dx = 0; dx < step; ++dx) {
                WorldCoord w = reconstruct_world_coord(chunk_coord, LocalCoord(lx_base + dx, ly_base + dy, lz_base + dz));
                Voxel v = world.get_voxel(w);
                if (v.is_solid()) {
                    if (!cell.solid) {
                        cell.solid = true;
                        cell.type_id = v.type_id;
                    }
                }
            }
        }
    }
    return cell;
}

bool is_neighbor_block_solid(const WorldAccessor& world,
                            const ChunkCoord& chunk_coord,
                            FaceDir dir,
                            int slice, int u, int v,
                            int step, int grid_dim) {
    int ndx = 0, ndy = 0, ndz = 0;
    switch (dir) {
        case FaceDir::PosX: ndx = 1; break;
        case FaceDir::NegX: ndx = -1; break;
        case FaceDir::PosY: ndy = 1; break;
        case FaceDir::NegY: ndy = -1; break;
        case FaceDir::PosZ: ndz = 1; break;
        case FaceDir::NegZ: ndz = -1; break;
    }

    int nslice = slice + (ndx != 0 ? ndx : (ndy != 0 ? ndy : ndz));

    if (nslice >= 0 && nslice < grid_dim) {
        // Neighbor is within chunk bounds
        int ncu = 0, ncv = 0, ncw = 0;
        switch (dir) {
            case FaceDir::PosX: case FaceDir::NegX:
                ncu = nslice; ncv = u; ncw = v; break;
            case FaceDir::PosY: case FaceDir::NegY:
                ncu = u; ncv = nslice; ncw = v; break;
            case FaceDir::PosZ: case FaceDir::NegZ:
                ncu = u; ncv = v; ncw = nslice; break;
        }
        CoarseCell nc = evaluate_coarse_cell(world, chunk_coord, ncu, ncv, ncw, step);
        return nc.solid;
    } else {
        // Neighbor is in adjacent chunk - query world voxel block
        int base_lx = 0, base_ly = 0, base_lz = 0;
        switch (dir) {
            case FaceDir::PosX: base_lx = CHUNK_DIM; base_ly = u * step; base_lz = v * step; break;
            case FaceDir::NegX: base_lx = -1; base_ly = u * step; base_lz = v * step; break;
            case FaceDir::PosY: base_lx = u * step; base_ly = CHUNK_DIM; base_lz = v * step; break;
            case FaceDir::NegY: base_lx = u * step; base_ly = -1; base_lz = v * step; break;
            case FaceDir::PosZ: base_lx = u * step; base_ly = v * step; base_lz = CHUNK_DIM; break;
            case FaceDir::NegZ: base_lx = u * step; base_ly = v * step; base_lz = -1; break;
        }

        bool all_solid = true;
        int dim1 = step;
        int dim2 = step;

        for (int d2 = 0; d2 < dim2; ++d2) {
            for (int d1 = 0; d1 < dim1; ++d1) {
                int qx = base_lx;
                int qy = base_ly;
                int qz = base_lz;

                switch (dir) {
                    case FaceDir::PosX: case FaceDir::NegX:
                        qy = base_ly + d1; qz = base_lz + d2; break;
                    case FaceDir::PosY: case FaceDir::NegY:
                        qx = base_lx + d1; qz = base_lz + d2; break;
                    case FaceDir::PosZ: case FaceDir::NegZ:
                        qx = base_lx + d1; qy = base_ly + d2; break;
                }

                WorldCoord w = reconstruct_world_coord(chunk_coord, LocalCoord(0, 0, 0));
                WorldCoord nw(w.x + qx, w.y + qy, w.z + qz);

                if (!world.is_solid(nw)) {
                    all_solid = false;
                    break;
                }
            }
            if (!all_solid) break;
        }
        return all_solid;
    }
}

void emit_quad_lod(FaceDir dir, float fx, float fy, float fz, float w, float h, float step, MeshData& out_mesh) {
    uint32_t base_index = static_cast<uint32_t>(out_mesh.vertices.size());

    switch (dir) {
        case FaceDir::PosX:
            out_mesh.vertices.emplace_back(fx + step, fy,     fz,     1.0f, 0.0f, 0.0f);
            out_mesh.vertices.emplace_back(fx + step, fy + w, fz,     1.0f, 0.0f, 0.0f);
            out_mesh.vertices.emplace_back(fx + step, fy + w, fz + h, 1.0f, 0.0f, 0.0f);
            out_mesh.vertices.emplace_back(fx + step, fy,     fz + h, 1.0f, 0.0f, 0.0f);
            break;
        case FaceDir::NegX:
            out_mesh.vertices.emplace_back(fx, fy,     fz + h, -1.0f, 0.0f, 0.0f);
            out_mesh.vertices.emplace_back(fx, fy + w, fz + h, -1.0f, 0.0f, 0.0f);
            out_mesh.vertices.emplace_back(fx, fy + w, fz,     -1.0f, 0.0f, 0.0f);
            out_mesh.vertices.emplace_back(fx, fy,     fz,     -1.0f, 0.0f, 0.0f);
            break;
        case FaceDir::PosY:
            out_mesh.vertices.emplace_back(fx,     fy + step, fz + h, 0.0f, 1.0f, 0.0f);
            out_mesh.vertices.emplace_back(fx + w, fy + step, fz + h, 0.0f, 1.0f, 0.0f);
            out_mesh.vertices.emplace_back(fx + w, fy + step, fz,     0.0f, 1.0f, 0.0f);
            out_mesh.vertices.emplace_back(fx,     fy + step, fz,     0.0f, 1.0f, 0.0f);
            break;
        case FaceDir::NegY:
            out_mesh.vertices.emplace_back(fx,     fy, fz,     0.0f, -1.0f, 0.0f);
            out_mesh.vertices.emplace_back(fx + w, fy, fz,     0.0f, -1.0f, 0.0f);
            out_mesh.vertices.emplace_back(fx + w, fy, fz + h, 0.0f, -1.0f, 0.0f);
            out_mesh.vertices.emplace_back(fx,     fy, fz + h, 0.0f, -1.0f, 0.0f);
            break;
        case FaceDir::PosZ:
            out_mesh.vertices.emplace_back(fx,     fy,     fz + step, 0.0f, 0.0f, 1.0f);
            out_mesh.vertices.emplace_back(fx + w, fy,     fz + step, 0.0f, 0.0f, 1.0f);
            out_mesh.vertices.emplace_back(fx + w, fy + h, fz + step, 0.0f, 0.0f, 1.0f);
            out_mesh.vertices.emplace_back(fx,     fy + h, fz + step, 0.0f, 0.0f, 1.0f);
            break;
        case FaceDir::NegZ:
            out_mesh.vertices.emplace_back(fx,     fy + h, fz, 0.0f, 0.0f, -1.0f);
            out_mesh.vertices.emplace_back(fx + w, fy + h, fz, 0.0f, 0.0f, -1.0f);
            out_mesh.vertices.emplace_back(fx + w, fy,     fz, 0.0f, 0.0f, -1.0f);
            out_mesh.vertices.emplace_back(fx,     fy,     fz, 0.0f, 0.0f, -1.0f);
            break;
    }

    out_mesh.indices.push_back(base_index + 0);
    out_mesh.indices.push_back(base_index + 1);
    out_mesh.indices.push_back(base_index + 2);
    out_mesh.indices.push_back(base_index + 0);
    out_mesh.indices.push_back(base_index + 2);
    out_mesh.indices.push_back(base_index + 3);
}

void mesh_face_direction_lod(FaceDir dir,
                             const WorldAccessor& world,
                             const ChunkCoord& chunk_coord,
                             int step, int grid_dim,
                             MesherType mesher,
                             MeshData& out_mesh) {
    std::vector<std::vector<uint8_t>> mask(grid_dim, std::vector<uint8_t>(grid_dim, 0));

    for (int slice = 0; slice < grid_dim; ++slice) {
        // Step 1: Build visibility mask for this slice
        for (int v = 0; v < grid_dim; ++v) {
            for (int u = 0; u < grid_dim; ++u) {
                int cu = 0, cv = 0, cw = 0;
                switch (dir) {
                    case FaceDir::PosX: case FaceDir::NegX:
                        cu = slice; cv = u; cw = v; break;
                    case FaceDir::PosY: case FaceDir::NegY:
                        cu = u; cv = slice; cw = v; break;
                    case FaceDir::PosZ: case FaceDir::NegZ:
                        cu = u; cv = v; cw = slice; break;
                }

                CoarseCell cell = evaluate_coarse_cell(world, chunk_coord, cu, cv, cw, step);
                if (cell.solid) {
                    bool neighbor_solid = is_neighbor_block_solid(world, chunk_coord, dir, slice, u, v, step, grid_dim);
                    if (!neighbor_solid) {
                        mask[u][v] = cell.type_id;
                    } else {
                        mask[u][v] = 0;
                    }
                } else {
                    mask[u][v] = 0;
                }
            }
        }

        float step_f = static_cast<float>(step);

        if (mesher == MesherType::Greedy) {
            // Step 2: 2D Greedy merge across slice mask
            for (int v = 0; v < grid_dim; ++v) {
                for (int u = 0; u < grid_dim; ++u) {
                    uint8_t type = mask[u][v];
                    if (type == 0) continue;

                    int width = 1;
                    while (u + width < grid_dim && mask[u + width][v] == type) {
                        width++;
                    }

                    int height = 1;
                    while (v + height < grid_dim) {
                        bool row_matches = true;
                        for (int k = 0; k < width; ++k) {
                            if (mask[u + k][v + height] != type) {
                                row_matches = false;
                                break;
                            }
                        }
                        if (!row_matches) break;
                        height++;
                    }

                    float fx = 0.0f, fy = 0.0f, fz = 0.0f;
                    switch (dir) {
                        case FaceDir::PosX: case FaceDir::NegX:
                            fx = static_cast<float>(slice) * step_f;
                            fy = static_cast<float>(u) * step_f;
                            fz = static_cast<float>(v) * step_f;
                            break;
                        case FaceDir::PosY: case FaceDir::NegY:
                            fx = static_cast<float>(u) * step_f;
                            fy = static_cast<float>(slice) * step_f;
                            fz = static_cast<float>(v) * step_f;
                            break;
                        case FaceDir::PosZ: case FaceDir::NegZ:
                            fx = static_cast<float>(u) * step_f;
                            fy = static_cast<float>(v) * step_f;
                            fz = static_cast<float>(slice) * step_f;
                            break;
                    }

                    float quad_w = static_cast<float>(width) * step_f;
                    float quad_h = static_cast<float>(height) * step_f;

                    emit_quad_lod(dir, fx, fy, fz, quad_w, quad_h, step_f, out_mesh);

                    for (int dv = 0; dv < height; ++dv) {
                        for (int du = 0; du < width; ++du) {
                            mask[u + du][v + dv] = 0;
                        }
                    }
                }
            }
        } else {
            // Naive 1x1 cell emitting
            for (int v = 0; v < grid_dim; ++v) {
                for (int u = 0; u < grid_dim; ++u) {
                    if (mask[u][v] == 0) continue;

                    float fx = 0.0f, fy = 0.0f, fz = 0.0f;
                    switch (dir) {
                        case FaceDir::PosX: case FaceDir::NegX:
                            fx = static_cast<float>(slice) * step_f;
                            fy = static_cast<float>(u) * step_f;
                            fz = static_cast<float>(v) * step_f;
                            break;
                        case FaceDir::PosY: case FaceDir::NegY:
                            fx = static_cast<float>(u) * step_f;
                            fy = static_cast<float>(slice) * step_f;
                            fz = static_cast<float>(v) * step_f;
                            break;
                        case FaceDir::PosZ: case FaceDir::NegZ:
                            fx = static_cast<float>(u) * step_f;
                            fy = static_cast<float>(v) * step_f;
                            fz = static_cast<float>(slice) * step_f;
                            break;
                    }

                    emit_quad_lod(dir, fx, fy, fz, step_f, step_f, step_f, out_mesh);
                }
            }
        }
    }
}

} // anonymous namespace

LODLevel select_lod_level(const ChunkCoord& chunk,
                         const ChunkCoord& cam_chunk,
                         bool enable_lod,
                         int lod0_radius,
                         int lod1_radius) noexcept {
    if (!enable_lod) {
        return LODLevel::LOD0;
    }
    int dx = std::abs(chunk.x - cam_chunk.x);
    int dy = std::abs(chunk.y - cam_chunk.y);
    int dz = std::abs(chunk.z - cam_chunk.z);
    int dist = std::max({dx, dy, dz});

    if (dist <= lod0_radius) {
        return LODLevel::LOD0;
    } else if (dist <= lod1_radius) {
        return LODLevel::LOD1;
    } else {
        return LODLevel::LOD2;
    }
}

void mesh_chunk_lod(const WorldAccessor& world,
                    const ChunkCoord& chunk_coord,
                    LODLevel lod,
                    MesherType mesher,
                    MeshData& out_mesh) {
    if (lod == LODLevel::LOD0) {
        if (mesher == MesherType::Greedy) {
            greedy_mesh_chunk(world, chunk_coord, out_mesh);
        } else {
            mesh_chunk(world, chunk_coord, out_mesh);
        }
        return;
    }

    int step = lod_step_size(lod);
    int grid_dim = CHUNK_DIM / step;

    mesh_face_direction_lod(FaceDir::PosX, world, chunk_coord, step, grid_dim, mesher, out_mesh);
    mesh_face_direction_lod(FaceDir::NegX, world, chunk_coord, step, grid_dim, mesher, out_mesh);
    mesh_face_direction_lod(FaceDir::PosY, world, chunk_coord, step, grid_dim, mesher, out_mesh);
    mesh_face_direction_lod(FaceDir::NegY, world, chunk_coord, step, grid_dim, mesher, out_mesh);
    mesh_face_direction_lod(FaceDir::PosZ, world, chunk_coord, step, grid_dim, mesher, out_mesh);
    mesh_face_direction_lod(FaceDir::NegZ, world, chunk_coord, step, grid_dim, mesher, out_mesh);
}

MeshData mesh_chunk_lod(const WorldAccessor& world,
                       const ChunkCoord& chunk_coord,
                       LODLevel lod,
                       MesherType mesher) {
    MeshData mesh;
    mesh_chunk_lod(world, chunk_coord, lod, mesher, mesh);
    return mesh;
}

} // namespace voxel_lab
