#ifndef PERFORMANCE_LAB_ALLOCATOR_PAYLOAD_HPP
#define PERFORMANCE_LAB_ALLOCATOR_PAYLOAD_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace performance_lab {

/**
 * @brief Deterministic workload parameters and payload validation for allocator benchmarks.
 */
struct AllocatorWorkload {
    static constexpr std::size_t kFixedSize = 64;
    static constexpr std::size_t kSecondaryFixedSize = 128;
    static constexpr std::size_t kNumVariableSizes = 9;
    static constexpr std::array<std::size_t, kNumVariableSizes> kVariableSizes = {
        16, 32, 48, 64, 96, 128, 192, 256, 512
    };

    /**
     * @brief Computes deterministic allocation size for index i in variable workloads.
     */
    static constexpr std::size_t variable_size_at(std::size_t index) noexcept {
        return kVariableSizes[index % kNumVariableSizes];
    }
};

/**
 * @brief Lightweight deterministic payload marker to prevent dead-code elimination
 *        without dominating allocator measurement overhead.
 */
class AllocatorPayload {
public:
    static constexpr std::uint64_t kMagic = 0x5A5A5A5AA5A5A5A5ULL;

    /**
     * @brief Writes a deterministic 64-bit verification pattern into the allocated block.
     */
    static inline void write_marker(void* ptr, std::size_t size, std::uint64_t seq) noexcept {
        if (!ptr || size < sizeof(std::uint64_t)) {
            return;
        }
        std::uint64_t marker = (seq << 32) ^ (kMagic ^ static_cast<std::uint64_t>(size));
        std::memcpy(ptr, &marker, sizeof(marker));
    }

    /**
     * @brief Reads the 64-bit verification pattern from the allocated block.
     */
    static inline std::uint64_t read_marker(const void* ptr, std::size_t size) noexcept {
        if (!ptr || size < sizeof(std::uint64_t)) {
            return 0;
        }
        std::uint64_t marker = 0;
        std::memcpy(&marker, ptr, sizeof(marker));
        return marker;
    }

    /**
     * @brief Validates that the payload marker matches the expected deterministic pattern.
     */
    static inline bool validate(const void* ptr, std::size_t size, std::uint64_t seq) noexcept {
        if (!ptr || size < sizeof(std::uint64_t)) {
            return false;
        }
        std::uint64_t expected = (seq << 32) ^ (kMagic ^ static_cast<std::uint64_t>(size));
        return read_marker(ptr, size) == expected;
    }
};

} // namespace performance_lab

#endif // PERFORMANCE_LAB_ALLOCATOR_PAYLOAD_HPP
