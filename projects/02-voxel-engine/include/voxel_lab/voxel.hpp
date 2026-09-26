#pragma once

#include <cstdint>
#include <type_traits>

namespace voxel_lab {

struct Voxel {
    uint8_t type_id{0};
    uint8_t flags{0};

    constexpr Voxel() = default;
    constexpr Voxel(uint8_t type, uint8_t f = 0) : type_id(type), flags(f) {}

    constexpr bool operator==(const Voxel& other) const {
        return type_id == other.type_id && flags == other.flags;
    }

    constexpr bool operator!=(const Voxel& other) const {
        return !(*this == other);
    }

    constexpr bool is_air() const noexcept {
        return type_id == 0;
    }

    constexpr bool is_solid() const noexcept {
        return type_id != 0;
    }
};

static_assert(sizeof(Voxel) == 2, "Voxel struct payload must be exactly 2 bytes");
static_assert(std::is_trivially_copyable_v<Voxel>, "Voxel struct must be trivially copyable");

} // namespace voxel_lab
