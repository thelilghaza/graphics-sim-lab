#pragma once

#include "voxel_lab/chunk.hpp"
#include <cstddef>
#include <cstdint>
#include <tuple>

namespace voxel_lab {

struct WorldCoord {
    int x{0};
    int y{0};
    int z{0};

    constexpr WorldCoord() = default;
    constexpr WorldCoord(int wx, int wy, int wz) : x(wx), y(wy), z(wz) {}

    constexpr bool operator==(const WorldCoord& other) const noexcept {
        return x == other.x && y == other.y && z == other.z;
    }

    constexpr bool operator!=(const WorldCoord& other) const noexcept {
        return !(*this == other);
    }
};

struct ChunkCoord {
    int x{0};
    int y{0};
    int z{0};

    constexpr ChunkCoord() = default;
    constexpr ChunkCoord(int cx, int cy, int cz) : x(cx), y(cy), z(cz) {}

    constexpr bool operator==(const ChunkCoord& other) const noexcept {
        return x == other.x && y == other.y && z == other.z;
    }

    constexpr bool operator!=(const ChunkCoord& other) const noexcept {
        return !(*this == other);
    }

    constexpr bool operator<(const ChunkCoord& other) const noexcept {
        return std::tie(x, y, z) < std::tie(other.x, other.y, other.z);
    }
};

struct LocalCoord {
    int x{0};
    int y{0};
    int z{0};

    constexpr LocalCoord() = default;
    constexpr LocalCoord(int lx, int ly, int lz) : x(lx), y(ly), z(lz) {}

    constexpr bool operator==(const LocalCoord& other) const noexcept {
        return x == other.x && y == other.y && z == other.z;
    }

    constexpr bool operator!=(const LocalCoord& other) const noexcept {
        return !(*this == other);
    }
};

// Deterministic integer floor division for coordinate decomposition (step size S = 32)
constexpr int floor_div_32(int value) noexcept {
    return (value < 0) ? ((value - 31) / CHUNK_DIM) : (value / CHUNK_DIM);
}

// Local coordinate extraction in range [0, 31]
constexpr int floor_mod_32(int value) noexcept {
    int m = value % CHUNK_DIM;
    return (m < 0) ? (m + CHUNK_DIM) : m;
}

// Converts world coordinate scalar W to (chunk, local) where 0 <= local < 32 and W == chunk * 32 + local
constexpr std::pair<int, int> decompose_world_coord_scalar(int w) noexcept {
    int chunk = floor_div_32(w);
    int local = floor_mod_32(w);
    return {chunk, local};
}

// Converts WorldCoord to ChunkCoord
constexpr ChunkCoord world_to_chunk(const WorldCoord& w) noexcept {
    return ChunkCoord(floor_div_32(w.x), floor_div_32(w.y), floor_div_32(w.z));
}

// Converts WorldCoord to LocalCoord [0, 31]
constexpr LocalCoord world_to_local(const WorldCoord& w) noexcept {
    return LocalCoord(floor_mod_32(w.x), floor_mod_32(w.y), floor_mod_32(w.z));
}

// Decomposes WorldCoord into ChunkCoord and LocalCoord
constexpr std::pair<ChunkCoord, LocalCoord> decompose_world_coord(const WorldCoord& w) noexcept {
    return {world_to_chunk(w), world_to_local(w)};
}

// Reconstructs WorldCoord from ChunkCoord and LocalCoord
constexpr WorldCoord reconstruct_world_coord(const ChunkCoord& c, const LocalCoord& l) noexcept {
    return WorldCoord(c.x * CHUNK_DIM + l.x, c.y * CHUNK_DIM + l.y, c.z * CHUNK_DIM + l.z);
}

} // namespace voxel_lab
