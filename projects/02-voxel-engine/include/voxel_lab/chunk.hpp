#pragma once

#include "voxel_lab/voxel.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace voxel_lab {

constexpr int CHUNK_DIM = 32;
constexpr size_t CHUNK_VOXELS = static_cast<size_t>(CHUNK_DIM * CHUNK_DIM * CHUNK_DIM);
constexpr size_t STORAGE_BYTES = CHUNK_VOXELS * sizeof(Voxel);

class Chunk {
public:
    Chunk();

    static constexpr bool in_bounds(int lx, int ly, int lz) noexcept {
        return lx >= 0 && lx < CHUNK_DIM &&
               ly >= 0 && ly < CHUNK_DIM &&
               lz >= 0 && lz < CHUNK_DIM;
    }

    static constexpr size_t to_index(int lx, int ly, int lz) noexcept {
        return static_cast<size_t>(lx + ly * CHUNK_DIM + lz * (CHUNK_DIM * CHUNK_DIM));
    }

    Voxel get_voxel(int lx, int ly, int lz) const;
    bool get_voxel(int lx, int ly, int lz, Voxel& out_voxel) const noexcept;
    bool set_voxel(int lx, int ly, int lz, const Voxel& voxel) noexcept;

    const Voxel& get_voxel_at_index(size_t index) const;
    bool set_voxel_at_index(size_t index, const Voxel& voxel) noexcept;

    void fill(const Voxel& voxel) noexcept;
    void clear() noexcept;

    constexpr int dimension() const noexcept { return CHUNK_DIM; }
    constexpr size_t voxel_count() const noexcept { return CHUNK_VOXELS; }
    constexpr size_t storage_size_bytes() const noexcept { return STORAGE_BYTES; }

    const Voxel* data() const noexcept { return voxels.data(); }
    Voxel* data() noexcept { return voxels.data(); }

private:
    std::array<Voxel, CHUNK_VOXELS> voxels;
};

static_assert(sizeof(Chunk) == STORAGE_BYTES, "Chunk storage must be exactly contiguous 65,536 bytes");

} // namespace voxel_lab
