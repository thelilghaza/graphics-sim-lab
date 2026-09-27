#ifndef PERFORMANCE_LAB_QUEUE_PAYLOAD_HPP
#define PERFORMANCE_LAB_QUEUE_PAYLOAD_HPP

#include <cstdint>
#include <type_traits>

namespace performance_lab {

/**
 * @brief Fixed-size, trivially copyable queue payload.
 *
 * Total footprint: exactly 24 bytes (three 64-bit unsigned integers).
 * Contains a sequence ID and two deterministic payload words derived from the sequence.
 * Used across all queue implementations to ensure fair and realistic data movement
 * without dynamic allocations or pointer ownership overhead.
 */
struct QueueItem {
    std::uint64_t sequence{0};
    std::uint64_t payload_a{0};
    std::uint64_t payload_b{0};

    /**
     * @brief Constructs a deterministic QueueItem from a sequence number.
     */
    static constexpr QueueItem make(std::uint64_t seq) noexcept {
        return QueueItem{
            seq,
            seq ^ 0xDEADBEEFCAFEBABEULL,
            ~seq + 1ULL
        };
    }

    /**
     * @brief Validates payload integrity against sequence number.
     * @return true if payload matches deterministic sequence derivation.
     */
    [[nodiscard]] constexpr bool is_valid() const noexcept {
        return (payload_a == (sequence ^ 0xDEADBEEFCAFEBABEULL)) &&
               (payload_b == (~sequence + 1ULL));
    }
};

static_assert(sizeof(QueueItem) == 24, "QueueItem must be exactly 24 bytes");
static_assert(std::is_trivially_copyable_v<QueueItem>, "QueueItem must be trivially copyable");
static_assert(std::is_trivially_destructible_v<QueueItem>, "QueueItem must be trivially destructible");

} // namespace performance_lab

#endif // PERFORMANCE_LAB_QUEUE_PAYLOAD_HPP
