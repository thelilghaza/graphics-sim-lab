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

struct MeshData {
    std::vector<MeshVertex> vertices;
    std::vector<uint32_t> indices;

    size_t vertex_count() const noexcept { return vertices.size(); }
    size_t index_count() const noexcept { return indices.size(); }
    size_t face_count() const noexcept { return indices.size() / 6; }
    size_t quad_count() const noexcept { return face_count(); }

    void clear() noexcept {
        vertices.clear();
        indices.clear();
    }

    bool operator==(const MeshData& other) const noexcept {
        return vertices == other.vertices && indices == other.indices;
    }

    bool operator!=(const MeshData& other) const noexcept {
        return !(*this == other);
    }
};

} // namespace voxel_lab
