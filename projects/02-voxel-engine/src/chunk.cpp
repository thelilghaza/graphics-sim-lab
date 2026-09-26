#include "voxel_lab/chunk.hpp"
#include <stdexcept>

namespace voxel_lab {

Chunk::Chunk() {
    clear();
}

Voxel Chunk::get_voxel(int lx, int ly, int lz) const {
    if (!in_bounds(lx, ly, lz)) {
        throw std::out_of_range("Local voxel coordinates out of bounds");
    }
    return voxels[to_index(lx, ly, lz)];
}

bool Chunk::get_voxel(int lx, int ly, int lz, Voxel& out_voxel) const noexcept {
    if (!in_bounds(lx, ly, lz)) {
        return false;
    }
    out_voxel = voxels[to_index(lx, ly, lz)];
    return true;
}

bool Chunk::set_voxel(int lx, int ly, int lz, const Voxel& voxel) noexcept {
    if (!in_bounds(lx, ly, lz)) {
        return false;
    }
    voxels[to_index(lx, ly, lz)] = voxel;
    return true;
}

const Voxel& Chunk::get_voxel_at_index(size_t index) const {
    if (index >= CHUNK_VOXELS) {
        throw std::out_of_range("Chunk voxel index out of bounds");
    }
    return voxels[index];
}

bool Chunk::set_voxel_at_index(size_t index, const Voxel& voxel) noexcept {
    if (index >= CHUNK_VOXELS) {
        return false;
    }
    voxels[index] = voxel;
    return true;
}

void Chunk::fill(const Voxel& voxel) noexcept {
    voxels.fill(voxel);
}

void Chunk::clear() noexcept {
    voxels.fill(Voxel(0, 0));
}

} // namespace voxel_lab
