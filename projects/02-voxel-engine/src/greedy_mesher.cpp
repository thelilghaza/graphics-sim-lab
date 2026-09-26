#include "voxel_lab/greedy_mesher.hpp"
#include "voxel_lab/chunk.hpp"
#include <array>
#include <cstdint>

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

void emit_quad(FaceDir dir, float fx, float fy, float fz, float w, float h, MeshData& out_mesh) {
    uint32_t base_index = static_cast<uint32_t>(out_mesh.vertices.size());

    switch (dir) {
        case FaceDir::PosX: // +X face at x = fx + 1, spanning y in [fy, fy + w] and z in [fz, fz + h]
            out_mesh.vertices.emplace_back(fx + 1.0f, fy,     fz,     1.0f, 0.0f, 0.0f);
            out_mesh.vertices.emplace_back(fx + 1.0f, fy + w, fz,     1.0f, 0.0f, 0.0f);
            out_mesh.vertices.emplace_back(fx + 1.0f, fy + w, fz + h, 1.0f, 0.0f, 0.0f);
            out_mesh.vertices.emplace_back(fx + 1.0f, fy,     fz + h, 1.0f, 0.0f, 0.0f);
            break;
        case FaceDir::NegX: // -X face at x = fx, spanning y in [fy, fy + w] and z in [fz, fz + h]
            out_mesh.vertices.emplace_back(fx, fy,     fz + h, -1.0f, 0.0f, 0.0f);
            out_mesh.vertices.emplace_back(fx, fy + w, fz + h, -1.0f, 0.0f, 0.0f);
            out_mesh.vertices.emplace_back(fx, fy + w, fz,     -1.0f, 0.0f, 0.0f);
            out_mesh.vertices.emplace_back(fx, fy,     fz,     -1.0f, 0.0f, 0.0f);
            break;
        case FaceDir::PosY: // +Y face at y = fy + 1, spanning x in [fx, fx + w] and z in [fz, fz + h]
            out_mesh.vertices.emplace_back(fx,     fy + 1.0f, fz + h, 0.0f, 1.0f, 0.0f);
            out_mesh.vertices.emplace_back(fx + w, fy + 1.0f, fz + h, 0.0f, 1.0f, 0.0f);
            out_mesh.vertices.emplace_back(fx + w, fy + 1.0f, fz,     0.0f, 1.0f, 0.0f);
            out_mesh.vertices.emplace_back(fx,     fy + 1.0f, fz,     0.0f, 1.0f, 0.0f);
            break;
        case FaceDir::NegY: // -Y face at y = fy, spanning x in [fx, fx + w] and z in [fz, fz + h]
            out_mesh.vertices.emplace_back(fx,     fy, fz,     0.0f, -1.0f, 0.0f);
            out_mesh.vertices.emplace_back(fx + w, fy, fz,     0.0f, -1.0f, 0.0f);
            out_mesh.vertices.emplace_back(fx + w, fy, fz + h, 0.0f, -1.0f, 0.0f);
            out_mesh.vertices.emplace_back(fx,     fy, fz + h, 0.0f, -1.0f, 0.0f);
            break;
        case FaceDir::PosZ: // +Z face at z = fz + 1, spanning x in [fx, fx + w] and y in [fy, fy + h]
            out_mesh.vertices.emplace_back(fx,     fy,     fz + 1.0f, 0.0f, 0.0f, 1.0f);
            out_mesh.vertices.emplace_back(fx + w, fy,     fz + 1.0f, 0.0f, 0.0f, 1.0f);
            out_mesh.vertices.emplace_back(fx + w, fy + h, fz + 1.0f, 0.0f, 0.0f, 1.0f);
            out_mesh.vertices.emplace_back(fx,     fy + h, fz + 1.0f, 0.0f, 0.0f, 1.0f);
            break;
        case FaceDir::NegZ: // -Z face at z = fz, spanning x in [fx, fx + w] and y in [fy, fy + h]
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

void mesh_face_direction(FaceDir dir, const WorldAccessor& world, const ChunkCoord& chunk_coord, MeshData& out_mesh) {
    std::array<std::array<uint8_t, CHUNK_DIM>, CHUNK_DIM> mask{};

    // Iterate through all 32 slices along the normal axis
    for (int slice = 0; slice < CHUNK_DIM; ++slice) {
        // Step 1: Build the 2D visibility and voxel-type mask for this slice
        for (int v = 0; v < CHUNK_DIM; ++v) {
            for (int u = 0; u < CHUNK_DIM; ++u) {
                int lx = 0, ly = 0, lz = 0;
                int ndx = 0, ndy = 0, ndz = 0;

                switch (dir) {
                    case FaceDir::PosX:
                        lx = slice; ly = u; lz = v;
                        ndx = 1; ndy = 0; ndz = 0;
                        break;
                    case FaceDir::NegX:
                        lx = slice; ly = u; lz = v;
                        ndx = -1; ndy = 0; ndz = 0;
                        break;
                    case FaceDir::PosY:
                        lx = u; ly = slice; lz = v;
                        ndx = 0; ndy = 1; ndz = 0;
                        break;
                    case FaceDir::NegY:
                        lx = u; ly = slice; lz = v;
                        ndx = 0; ndy = -1; ndz = 0;
                        break;
                    case FaceDir::PosZ:
                        lx = u; ly = v; lz = slice;
                        ndx = 0; ndy = 0; ndz = 1;
                        break;
                    case FaceDir::NegZ:
                        lx = u; ly = v; lz = slice;
                        ndx = 0; ndy = 0; ndz = -1;
                        break;
                }

                WorldCoord w = reconstruct_world_coord(chunk_coord, LocalCoord(lx, ly, lz));
                Voxel voxel = world.get_voxel(w);

                if (voxel.is_solid()) {
                    WorldCoord neighbor_w(w.x + ndx, w.y + ndy, w.z + ndz);
                    if (!world.is_solid(neighbor_w)) {
                        mask[u][v] = voxel.type_id;
                    } else {
                        mask[u][v] = 0;
                    }
                } else {
                    mask[u][v] = 0;
                }
            }
        }

        // Step 2: Greedy 2D merge across the mask slice
        for (int v = 0; v < CHUNK_DIM; ++v) {
            for (int u = 0; u < CHUNK_DIM; ++u) {
                uint8_t type = mask[u][v];
                if (type == 0) {
                    continue;
                }

                // Determine maximum width along u with identical voxel type
                int width = 1;
                while (u + width < CHUNK_DIM && mask[u + width][v] == type) {
                    width++;
                }

                // Determine maximum height along v where the full row of width 'width' matches
                int height = 1;
                while (v + height < CHUNK_DIM) {
                    bool row_matches = true;
                    for (int k = 0; k < width; ++k) {
                        if (mask[u + k][v + height] != type) {
                            row_matches = false;
                            break;
                        }
                    }
                    if (!row_matches) {
                        break;
                    }
                    height++;
                }

                // Calculate base position (fx, fy, fz)
                float fx = 0.0f, fy = 0.0f, fz = 0.0f;
                switch (dir) {
                    case FaceDir::PosX:
                    case FaceDir::NegX:
                        fx = static_cast<float>(slice);
                        fy = static_cast<float>(u);
                        fz = static_cast<float>(v);
                        break;
                    case FaceDir::PosY:
                    case FaceDir::NegY:
                        fx = static_cast<float>(u);
                        fy = static_cast<float>(slice);
                        fz = static_cast<float>(v);
                        break;
                    case FaceDir::PosZ:
                    case FaceDir::NegZ:
                        fx = static_cast<float>(u);
                        fy = static_cast<float>(v);
                        fz = static_cast<float>(slice);
                        break;
                }

                // Emit the merged quad
                emit_quad(dir, fx, fy, fz, static_cast<float>(width), static_cast<float>(height), out_mesh);

                // Clear the merged cells in the mask
                for (int dv = 0; dv < height; ++dv) {
                    for (int du = 0; du < width; ++du) {
                        mask[u + du][v + dv] = 0;
                    }
                }
            }
        }
    }
}

} // anonymous namespace

void greedy_mesh_chunk(const WorldAccessor& world, const ChunkCoord& chunk_coord, MeshData& out_mesh) {
    // Deterministic direction order: PosX, NegX, PosY, NegY, PosZ, NegZ
    mesh_face_direction(FaceDir::PosX, world, chunk_coord, out_mesh);
    mesh_face_direction(FaceDir::NegX, world, chunk_coord, out_mesh);
    mesh_face_direction(FaceDir::PosY, world, chunk_coord, out_mesh);
    mesh_face_direction(FaceDir::NegY, world, chunk_coord, out_mesh);
    mesh_face_direction(FaceDir::PosZ, world, chunk_coord, out_mesh);
    mesh_face_direction(FaceDir::NegZ, world, chunk_coord, out_mesh);
}

MeshData greedy_mesh_chunk(const WorldAccessor& world, const ChunkCoord& chunk_coord) {
    MeshData mesh;
    greedy_mesh_chunk(world, chunk_coord, mesh);
    return mesh;
}

} // namespace voxel_lab
