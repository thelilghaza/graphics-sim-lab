#include "voxel_lab/naive_mesher.hpp"
#include "voxel_lab/chunk.hpp"

namespace voxel_lab {

namespace {

struct DirectionInfo {
    int dx;
    int dy;
    int dz;
    float nx;
    float ny;
    float nz;
};

constexpr DirectionInfo DIRECTIONS[6] = {
    { 1,  0,  0,  1.0f,  0.0f,  0.0f}, // PosX
    {-1,  0,  0, -1.0f,  0.0f,  0.0f}, // NegX
    { 0,  1,  0,  0.0f,  1.0f,  0.0f}, // PosY
    { 0, -1,  0,  0.0f, -1.0f,  0.0f}, // NegY
    { 0,  0,  1,  0.0f,  0.0f,  1.0f}, // PosZ
    { 0,  0, -1,  0.0f,  0.0f, -1.0f}  // NegZ
};

void emit_face(float fx, float fy, float fz, Direction dir, MeshData& out_mesh) {
    uint32_t base_index = static_cast<uint32_t>(out_mesh.vertices.size());
    const DirectionInfo& info = DIRECTIONS[static_cast<int>(dir)];
    float nx = info.nx;
    float ny = info.ny;
    float nz = info.nz;

    switch (dir) {
        case Direction::PosX: // +X face at x = fx + 1
            out_mesh.vertices.emplace_back(fx + 1.0f, fy,        fz,        nx, ny, nz);
            out_mesh.vertices.emplace_back(fx + 1.0f, fy + 1.0f, fz,        nx, ny, nz);
            out_mesh.vertices.emplace_back(fx + 1.0f, fy + 1.0f, fz + 1.0f, nx, ny, nz);
            out_mesh.vertices.emplace_back(fx + 1.0f, fy,        fz + 1.0f, nx, ny, nz);
            break;
        case Direction::NegX: // -X face at x = fx
            out_mesh.vertices.emplace_back(fx, fy,        fz + 1.0f, nx, ny, nz);
            out_mesh.vertices.emplace_back(fx, fy + 1.0f, fz + 1.0f, nx, ny, nz);
            out_mesh.vertices.emplace_back(fx, fy + 1.0f, fz,        nx, ny, nz);
            out_mesh.vertices.emplace_back(fx, fy,        fz,        nx, ny, nz);
            break;
        case Direction::PosY: // +Y face at y = fy + 1
            out_mesh.vertices.emplace_back(fx,        fy + 1.0f, fz + 1.0f, nx, ny, nz);
            out_mesh.vertices.emplace_back(fx + 1.0f, fy + 1.0f, fz + 1.0f, nx, ny, nz);
            out_mesh.vertices.emplace_back(fx + 1.0f, fy + 1.0f, fz,        nx, ny, nz);
            out_mesh.vertices.emplace_back(fx,        fy + 1.0f, fz,        nx, ny, nz);
            break;
        case Direction::NegY: // -Y face at y = fy
            out_mesh.vertices.emplace_back(fx,        fy, fz,        nx, ny, nz);
            out_mesh.vertices.emplace_back(fx + 1.0f, fy, fz,        nx, ny, nz);
            out_mesh.vertices.emplace_back(fx + 1.0f, fy, fz + 1.0f, nx, ny, nz);
            out_mesh.vertices.emplace_back(fx,        fy, fz + 1.0f, nx, ny, nz);
            break;
        case Direction::PosZ: // +Z face at z = fz + 1
            out_mesh.vertices.emplace_back(fx,        fy,        fz + 1.0f, nx, ny, nz);
            out_mesh.vertices.emplace_back(fx + 1.0f, fy,        fz + 1.0f, nx, ny, nz);
            out_mesh.vertices.emplace_back(fx + 1.0f, fy + 1.0f, fz + 1.0f, nx, ny, nz);
            out_mesh.vertices.emplace_back(fx,        fy + 1.0f, fz + 1.0f, nx, ny, nz);
            break;
        case Direction::NegZ: // -Z face at z = fz
            out_mesh.vertices.emplace_back(fx,        fy + 1.0f, fz, nx, ny, nz);
            out_mesh.vertices.emplace_back(fx + 1.0f, fy + 1.0f, fz, nx, ny, nz);
            out_mesh.vertices.emplace_back(fx + 1.0f, fy,        fz, nx, ny, nz);
            out_mesh.vertices.emplace_back(fx,        fy,        fz, nx, ny, nz);
            break;
    }

    out_mesh.indices.push_back(base_index + 0);
    out_mesh.indices.push_back(base_index + 1);
    out_mesh.indices.push_back(base_index + 2);
    out_mesh.indices.push_back(base_index + 0);
    out_mesh.indices.push_back(base_index + 2);
    out_mesh.indices.push_back(base_index + 3);
}

} // anonymous namespace

void mesh_chunk(const WorldAccessor& world, const ChunkCoord& chunk_coord, MeshData& out_mesh) {
    for (int lz = 0; lz < CHUNK_DIM; ++lz) {
        for (int ly = 0; ly < CHUNK_DIM; ++ly) {
            for (int lx = 0; lx < CHUNK_DIM; ++lx) {
                WorldCoord w = reconstruct_world_coord(chunk_coord, LocalCoord(lx, ly, lz));
                if (!world.is_solid(w)) {
                    continue;
                }

                float fx = static_cast<float>(lx);
                float fy = static_cast<float>(ly);
                float fz = static_cast<float>(lz);

                for (int d = 0; d < 6; ++d) {
                    const DirectionInfo& info = DIRECTIONS[d];
                    WorldCoord neighbor_w(w.x + info.dx, w.y + info.dy, w.z + info.dz);
                    if (!world.is_solid(neighbor_w)) {
                        emit_face(fx, fy, fz, static_cast<Direction>(d), out_mesh);
                    }
                }
            }
        }
    }
}

MeshData mesh_chunk(const WorldAccessor& world, const ChunkCoord& chunk_coord) {
    MeshData mesh;
    mesh_chunk(world, chunk_coord, mesh);
    return mesh;
}

} // namespace voxel_lab
