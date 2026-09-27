#ifndef PERFORMANCE_LAB_MPMC_BOUNDED_QUEUE_HPP
#define PERFORMANCE_LAB_MPMC_BOUNDED_QUEUE_HPP

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <new>
#include <stdexcept>
#include <vector>

namespace performance_lab {

#if defined(__cpp_lib_hardware_interference_size)
static constexpr size_t kMpmcCacheLineSize = std::hardware_destructive_interference_size;
#else
static constexpr size_t kMpmcCacheLineSize = 64;
#endif

/**
 * @brief Multi-Producer Multi-Consumer (MPMC) bounded lock-free queue.
 *
 * Implements Dmitry Vyukov's bounded MPMC queue with per-slot sequence numbers:
 * - Fixed capacity (must be power of two).
 * - No dynamic heap allocations during push/pop operations.
 * - Sequence numbers per slot track write/read readiness and prevent ABA wrap-around issues.
 * - Enqueue and dequeue positions reside on isolated cache lines to prevent false sharing.
 * - Memory ordering:
 *     * Slot sequence loads use acquire to synchronize with previous cell writes/reads.
 *     * Slot sequence stores use release to publish data or slot availability.
 */
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable: 4324) // structure was padded due to alignment specifier
#endif

template <typename T>
class MpmcBoundedQueue {
public:
    explicit MpmcBoundedQueue(size_t capacity)
        : capacity_(round_up_power_of_two(capacity))
        , mask_(capacity_ - 1)
        , buffer_(std::make_unique<Cell[]>(capacity_))
    {
        if (capacity < 2) {
            throw std::invalid_argument("MpmcBoundedQueue capacity must be at least 2");
        }
        for (size_t i = 0; i < capacity_; ++i) {
            buffer_[i].sequence.store(i, std::memory_order_relaxed);
        }
    }

    ~MpmcBoundedQueue() = default;

    MpmcBoundedQueue(const MpmcBoundedQueue&) = delete;
    MpmcBoundedQueue& operator=(const MpmcBoundedQueue&) = delete;
    MpmcBoundedQueue(MpmcBoundedQueue&&) = delete;
    MpmcBoundedQueue& operator=(MpmcBoundedQueue&&) = delete;

    /**
     * @brief Attempts to enqueue an item without blocking.
     * @return true if enqueued successfully, false if queue is full.
     */
    bool try_push(const T& data) noexcept {
        Cell* cell = nullptr;
        size_t pos = enqueue_pos_.load(std::memory_order_relaxed);

        for (;;) {
            cell = &buffer_[pos & mask_];
            const size_t seq = cell->sequence.load(std::memory_order_acquire);
            const intptr_t diff = static_cast<intptr_t>(seq) - static_cast<intptr_t>(pos);

            if (diff == 0) {
                // Slot is ready for this enqueue position; attempt to claim it
                if (enqueue_pos_.compare_exchange_weak(pos, pos + 1, std::memory_order_relaxed)) {
                    break;
                }
            } else if (diff < 0) {
                // Queue is full or slot has not been freed by consumer yet
                return false;
            } else {
                // Another producer advanced enqueue_pos_; refresh pos
                pos = enqueue_pos_.load(std::memory_order_relaxed);
            }
        }

        cell->data = data;
        // Release store signals that data is fully written and ready for dequeue at pos + 1
        cell->sequence.store(pos + 1, std::memory_order_release);
        return true;
    }

    /**
     * @brief Attempts to dequeue an item without blocking.
     * @return true if dequeued successfully, false if queue is empty.
     */
    bool try_pop(T& data) noexcept {
        Cell* cell = nullptr;
        size_t pos = dequeue_pos_.load(std::memory_order_relaxed);

        for (;;) {
            cell = &buffer_[pos & mask_];
            const size_t seq = cell->sequence.load(std::memory_order_acquire);
            const intptr_t diff = static_cast<intptr_t>(seq) - static_cast<intptr_t>(pos + 1);

            if (diff == 0) {
                // Slot has data published at pos + 1; attempt to claim it
                if (dequeue_pos_.compare_exchange_weak(pos, pos + 1, std::memory_order_relaxed)) {
                    break;
                }
            } else if (diff < 0) {
                // Queue is empty or slot has not been published by producer yet
                return false;
            } else {
                // Another consumer advanced dequeue_pos_; refresh pos
                pos = dequeue_pos_.load(std::memory_order_relaxed);
            }
        }

        data = cell->data;
        // Release store signals that slot has been read and is ready for next wrap-around enqueue
        cell->sequence.store(pos + mask_ + 1, std::memory_order_release);
        return true;
    }

    /**
     * @brief Approximate emptiness check.
     */
    [[nodiscard]] bool empty() const noexcept {
        const size_t enq = enqueue_pos_.load(std::memory_order_relaxed);
        const size_t deq = dequeue_pos_.load(std::memory_order_relaxed);
        return enq <= deq;
    }

    /**
     * @brief Approximate size.
     */
    [[nodiscard]] size_t size() const noexcept {
        const size_t enq = enqueue_pos_.load(std::memory_order_relaxed);
        const size_t deq = dequeue_pos_.load(std::memory_order_relaxed);
        return (enq > deq) ? (enq - deq) : 0;
    }

    /**
     * @brief Queue capacity.
     */
    [[nodiscard]] size_t capacity() const noexcept {
        return capacity_;
    }

private:
    struct Cell {
        std::atomic<size_t> sequence{0};
        T data{};
    };

    static size_t round_up_power_of_two(size_t v) noexcept {
        if (v < 2) return 2;
        --v;
        v |= v >> 1;
        v |= v >> 2;
        v |= v >> 4;
        v |= v >> 8;
        v |= v >> 16;
#if INTPTR_MAX == INT64_MAX
        v |= v >> 32;
#endif
        return v + 1;
    }

    const size_t capacity_;
    const size_t mask_;
    std::unique_ptr<Cell[]> buffer_;

    alignas(kMpmcCacheLineSize) std::atomic<size_t> enqueue_pos_{0};
    alignas(kMpmcCacheLineSize) std::atomic<size_t> dequeue_pos_{0};
};

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

} // namespace performance_lab

#endif // PERFORMANCE_LAB_MPMC_BOUNDED_QUEUE_HPP
