#ifndef PERFORMANCE_LAB_LINEAR_ARENA_HPP
#define PERFORMANCE_LAB_LINEAR_ARENA_HPP

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <new>
#include <stdexcept>

namespace performance_lab {

/**
 * @brief Monotonic linear arena (bump allocator).
 *
 * Preallocates a contiguous backing buffer. Allocations advance a bump pointer
 * monotonically with explicit alignment padding. Individual deallocation is
 * intentionally unsupported; all allocated memory is bulk-reclaimed via reset().
 */
class LinearArena {
public:
    explicit LinearArena(std::size_t capacity, std::size_t base_alignment = 64)
        : capacity_(capacity)
        , base_alignment_(std::max(base_alignment, alignof(void*)))
        , offset_(0)
        , peak_bytes_used_(0)
        , total_payload_bytes_(0)
        , total_padding_bytes_(0)
        , allocation_count_(0)
        , buffer_(nullptr)
    {
        if (capacity_ == 0) {
            throw std::invalid_argument("LinearArena capacity must be greater than zero");
        }
        buffer_ = static_cast<std::byte*>(::operator new(capacity_, std::align_val_t{base_alignment_}));
    }

    ~LinearArena() {
        if (buffer_) {
            ::operator delete(buffer_, std::align_val_t{base_alignment_});
            buffer_ = nullptr;
        }
    }

    LinearArena(const LinearArena&) = delete;
    LinearArena& operator=(const LinearArena&) = delete;

    LinearArena(LinearArena&& other) noexcept
        : capacity_(other.capacity_)
        , base_alignment_(other.base_alignment_)
        , offset_(other.offset_)
        , peak_bytes_used_(other.peak_bytes_used_)
        , total_payload_bytes_(other.total_payload_bytes_)
        , total_padding_bytes_(other.total_padding_bytes_)
        , allocation_count_(other.allocation_count_)
        , buffer_(other.buffer_)
    {
        other.buffer_ = nullptr;
        other.capacity_ = 0;
        other.offset_ = 0;
        other.peak_bytes_used_ = 0;
        other.total_payload_bytes_ = 0;
        other.total_padding_bytes_ = 0;
        other.allocation_count_ = 0;
    }

    LinearArena& operator=(LinearArena&& other) noexcept {
        if (this != &other) {
            if (buffer_) {
                ::operator delete(buffer_, std::align_val_t{base_alignment_});
            }
            capacity_ = other.capacity_;
            base_alignment_ = other.base_alignment_;
            offset_ = other.offset_;
            peak_bytes_used_ = other.peak_bytes_used_;
            total_payload_bytes_ = other.total_payload_bytes_;
            total_padding_bytes_ = other.total_padding_bytes_;
            allocation_count_ = other.allocation_count_;
            buffer_ = other.buffer_;

            other.buffer_ = nullptr;
            other.capacity_ = 0;
            other.offset_ = 0;
            other.peak_bytes_used_ = 0;
            other.total_payload_bytes_ = 0;
            other.total_padding_bytes_ = 0;
            other.allocation_count_ = 0;
        }
        return *this;
    }

    /**
     * @brief Allocates size bytes with specified alignment from the arena.
     *
     * Zero-byte request policy: Returns nullptr without advancing the bump pointer
     * and without modifying allocation accounting.
     *
     * @param size Number of bytes requested.
     * @param alignment Alignment boundary (must be power of two).
     * @return Pointer to aligned storage, or nullptr if arena capacity is exceeded.
     */
    void* allocate(std::size_t size, std::size_t alignment = 8) noexcept {
        if (size == 0) {
            return nullptr;
        }

        std::uintptr_t current_addr = reinterpret_cast<std::uintptr_t>(buffer_ + offset_);
        std::size_t rem = current_addr % alignment;
        std::size_t padding = (rem == 0) ? 0 : (alignment - rem);

        if (offset_ + padding + size > capacity_) {
            return nullptr;
        }

        void* result = static_cast<void*>(buffer_ + offset_ + padding);
        offset_ += padding + size;
        total_padding_bytes_ += padding;
        total_payload_bytes_ += size;
        ++allocation_count_;

        if (offset_ > peak_bytes_used_) {
            peak_bytes_used_ = offset_;
        }

        return result;
    }

    /**
     * @brief Bulk-reclaims all allocations by resetting the bump pointer to 0.
     *        Does not deallocate backing buffer storage.
     */
    void reset() noexcept {
        offset_ = 0;
    }

    /**
     * @brief Resets cumulative statistical counters for peak usage, padding, and payloads.
     */
    void reset_stats() noexcept {
        peak_bytes_used_ = offset_;
        total_payload_bytes_ = 0;
        total_padding_bytes_ = 0;
        allocation_count_ = 0;
    }

    /**
     * @brief Checks if a pointer lies within the active allocation window of this arena.
     */
    bool owns(const void* ptr) const noexcept {
        if (!ptr || !buffer_) {
            return false;
        }
        const std::byte* byte_ptr = static_cast<const std::byte*>(ptr);
        return (byte_ptr >= buffer_ && byte_ptr < buffer_ + offset_);
    }

    std::size_t capacity() const noexcept { return capacity_; }
    std::size_t bytes_used() const noexcept { return offset_; }
    std::size_t peak_bytes_used() const noexcept { return peak_bytes_used_; }
    std::size_t payload_bytes_requested() const noexcept { return total_payload_bytes_; }
    std::size_t alignment_padding_bytes() const noexcept { return total_padding_bytes_; }
    std::size_t allocation_count() const noexcept { return allocation_count_; }
    std::size_t base_alignment() const noexcept { return base_alignment_; }

    /**
     * @brief Utilization percentage: (payload_bytes / capacity) * 100.
     */
    double utilization_percent() const noexcept {
        if (capacity_ == 0) {
            return 0.0;
        }
        return (static_cast<double>(total_payload_bytes_) / capacity_) * 100.0;
    }

private:
    std::size_t capacity_;
    std::size_t base_alignment_;
    std::size_t offset_;
    std::size_t peak_bytes_used_;
    std::size_t total_payload_bytes_;
    std::size_t total_padding_bytes_;
    std::size_t allocation_count_;
    std::byte* buffer_;
};

} // namespace performance_lab

#endif // PERFORMANCE_LAB_LINEAR_ARENA_HPP
