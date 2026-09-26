#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace voxel_lab {

enum class MesherType {
    Naive,
    Greedy
};

inline const char* mesher_type_name(MesherType type) noexcept {
    switch (type) {
        case MesherType::Naive: return "Naive";
        case MesherType::Greedy: return "Greedy";
    }
    return "Unknown";
}

enum class LODLevel : uint8_t {
    LOD0 = 0,
    LOD1 = 1,
    LOD2 = 2
};

inline const char* lod_level_name(LODLevel level) noexcept {
    switch (level) {
        case LODLevel::LOD0: return "LOD 0 (Full)";
        case LODLevel::LOD1: return "LOD 1 (2x)";
        case LODLevel::LOD2: return "LOD 2 (4x)";
    }
    return "Unknown";
}

inline int lod_step_size(LODLevel level) noexcept {
    switch (level) {
        case LODLevel::LOD0: return 1;
        case LODLevel::LOD1: return 2;
        case LODLevel::LOD2: return 4;
    }
    return 1;
}

struct MeshVertex {
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};

    float nx{0.0f};
    float ny{0.0f};
    float nz{0.0f};

    constexpr MeshVertex() = default;
    constexpr MeshVertex(float vx, float vy, float vz, float vnx, float vny, float vnz)
        : x(vx), y(vy), z(vz), nx(vnx), ny(vny), nz(vnz) {}

    constexpr bool operator==(const MeshVertex& other) const noexcept {
        return x == other.x && y == other.y && z == other.z &&
               nx == other.nx && ny == other.ny && nz == other.nz;
    }

    constexpr bool operator!=(const MeshVertex& other) const noexcept {
        return !(*this == other);
    }
};

static_assert(sizeof(MeshVertex) == 24, "MeshVertex must be exactly 24 bytes (3 floats pos + 3 floats normal)");

struct MeshData {
    std::vector<MeshVertex> vertices;
    std::vector<uint32_t> indices;

    size_t vertex_count() const noexcept { return vertices.size(); }
    size_t index_count() const noexcept { return indices.size(); }
    size_t face_count() const noexcept { return indices.size() / 6; }
    size_t quad_count() const noexcept { return face_count(); }

    size_t vertex_bytes() const noexcept { return vertices.size() * sizeof(MeshVertex); }
    size_t index_bytes() const noexcept { return indices.size() * sizeof(uint32_t); }
    size_t total_logical_bytes() const noexcept { return vertex_bytes() + index_bytes(); }

    size_t vertex_capacity_bytes() const noexcept { return vertices.capacity() * sizeof(MeshVertex); }
    size_t index_capacity_bytes() const noexcept { return indices.capacity() * sizeof(uint32_t); }
    size_t total_capacity_bytes() const noexcept { return vertex_capacity_bytes() + index_capacity_bytes(); }

    void clear() noexcept {
        vertices.clear();
        indices.clear();
    }

    void shrink_to_fit() {
        vertices.shrink_to_fit();
        indices.shrink_to_fit();
    }

    bool operator==(const MeshData& other) const noexcept {
        return vertices == other.vertices && indices == other.indices;
    }

    bool operator!=(const MeshData& other) const noexcept {
        return !(*this == other);
    }
};

} // namespace voxel_lab
