#ifndef PERFORMANCE_LAB_SPSC_QUEUE_HPP
#define PERFORMANCE_LAB_SPSC_QUEUE_HPP

#include <atomic>
#include <cstddef>
#include <new>
#include <stdexcept>
#include <vector>

namespace performance_lab {

#if defined(__cpp_lib_hardware_interference_size)
static constexpr size_t kCacheLineSize = std::hardware_destructive_interference_size;
#else
static constexpr size_t kCacheLineSize = 64;
#endif

/**
 * @brief Single-Producer Single-Consumer (SPSC) bounded lock-free ring buffer.
 *
 * Implements a cache-friendly ring buffer where:
 * - Exactly one producer thread writes to head.
 * - Exactly one consumer thread reads from tail.
 * - Producer and consumer control variables reside on isolated cache lines (alignas(kCacheLineSize)).
 * - Employs cached head/tail shadow copies to minimize inter-core cache-line invalidation traffic.
 * - Memory ordering: acquire/release semantics ensure safe synchronization of payload elements
 *   without requiring expensive sequential consistency barriers.
 */
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable: 4324) // structure was padded due to alignment specifier
#endif

template <typename T>
class SpscQueue {
public:
    explicit SpscQueue(size_t capacity)
        : capacity_(round_up_power_of_two(capacity))
        , mask_(capacity_ - 1)
        , buffer_(capacity_)
    {
        if (capacity == 0) {
            throw std::invalid_argument("SpscQueue capacity must be greater than zero");
        }
    }

    ~SpscQueue() = default;

    SpscQueue(const SpscQueue&) = delete;
    SpscQueue& operator=(const SpscQueue&) = delete;
    SpscQueue(SpscQueue&&) = delete;
    SpscQueue& operator=(SpscQueue&&) = delete;

    /**
     * @brief Attempts to push an item (called strictly by producer thread).
     * @return true if enqueued, false if queue is full.
     */
    bool try_push(const T& item) noexcept {
        const size_t current_head = head_.load(std::memory_order_relaxed);
        
        // Fast-path: check against producer's cached tail
        if (current_head - cached_tail_ == capacity_) {
            // Slow-path: refresh cached tail from atomic consumer variable with acquire
            cached_tail_ = tail_.load(std::memory_order_acquire);
            if (current_head - cached_tail_ == capacity_) {
                return false; // Queue is full
            }
        }

        buffer_[current_head & mask_] = item;
        // Release store ensures the written buffer element is visible before head advances
        head_.store(current_head + 1, std::memory_order_release);
        return true;
    }

    /**
     * @brief Attempts to pop an item (called strictly by consumer thread).
     * @return true if dequeued, false if queue is empty.
     */
    bool try_pop(T& item) noexcept {
        const size_t current_tail = tail_.load(std::memory_order_relaxed);

        // Fast-path: check against consumer's cached head
        if (current_tail == cached_head_) {
            // Slow-path: refresh cached head from atomic producer variable with acquire
            cached_head_ = head_.load(std::memory_order_acquire);
            if (current_tail == cached_head_) {
                return false; // Queue is empty
            }
        }

        item = buffer_[current_tail & mask_];
        // Release store ensures the consumer has finished reading the slot before tail advances
        tail_.store(current_tail + 1, std::memory_order_release);
        return true;
    }

    /**
     * @brief Checks if queue appears empty.
     */
    [[nodiscard]] bool empty() const noexcept {
        return head_.load(std::memory_order_relaxed) == tail_.load(std::memory_order_relaxed);
    }

    /**
     * @brief Approximate current element count.
     */
    [[nodiscard]] size_t size() const noexcept {
        const size_t h = head_.load(std::memory_order_relaxed);
        const size_t t = tail_.load(std::memory_order_relaxed);
        return (h >= t) ? (h - t) : 0;
    }

    /**
     * @brief Queue capacity.
     */
    [[nodiscard]] size_t capacity() const noexcept {
        return capacity_;
    }

private:
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
    std::vector<T> buffer_;

    // Producer state (cache-line isolated)
    alignas(kCacheLineSize) std::atomic<size_t> head_{0};
    alignas(kCacheLineSize) size_t cached_tail_{0};

    // Consumer state (cache-line isolated)
    alignas(kCacheLineSize) std::atomic<size_t> tail_{0};
    alignas(kCacheLineSize) size_t cached_head_{0};
};

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

} // namespace performance_lab

#endif // PERFORMANCE_LAB_SPSC_QUEUE_HPP
