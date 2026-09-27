#ifndef PERFORMANCE_LAB_FIXED_BLOCK_POOL_HPP
#define PERFORMANCE_LAB_FIXED_BLOCK_POOL_HPP

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <new>
#include <stdexcept>

namespace performance_lab {

/**
 * @brief Fixed-size block memory pool using an intrusive free list.
 *
 * Preallocates a contiguous backing buffer during construction. Subsequent
 * allocate() and deallocate() operations run in O(1) time without performing
 * any heap allocations or OS kernel calls.
 */
class FixedBlockPool {
public:
    FixedBlockPool(std::size_t requested_block_size, std::size_t block_count, std::size_t alignment = alignof(std::max_align_t))
        : requested_block_size_(requested_block_size)
        , block_count_(block_count)
        , alignment_(std::max(alignment, alignof(void*)))
        , aligned_block_size_(calculate_aligned_block_size(requested_block_size, alignment_))
        , total_buffer_size_(aligned_block_size_ * block_count)
        , available_blocks_(block_count)
        , buffer_(nullptr)
        , free_list_(nullptr)
    {
        if (block_count_ == 0) {
            throw std::invalid_argument("FixedBlockPool block_count must be greater than zero");
        }
        if (requested_block_size_ == 0) {
            throw std::invalid_argument("FixedBlockPool block_size must be greater than zero");
        }

        // Allocate aligned contiguous backing storage
        buffer_ = static_cast<std::byte*>(::operator new(total_buffer_size_, std::align_val_t{alignment_}));
        reset();
    }

    ~FixedBlockPool() {
        if (buffer_) {
            ::operator delete(buffer_, std::align_val_t{alignment_});
            buffer_ = nullptr;
        }
    }

    FixedBlockPool(const FixedBlockPool&) = delete;
    FixedBlockPool& operator=(const FixedBlockPool&) = delete;

    FixedBlockPool(FixedBlockPool&& other) noexcept
        : requested_block_size_(other.requested_block_size_)
        , block_count_(other.block_count_)
        , alignment_(other.alignment_)
        , aligned_block_size_(other.aligned_block_size_)
        , total_buffer_size_(other.total_buffer_size_)
        , available_blocks_(other.available_blocks_)
        , buffer_(other.buffer_)
        , free_list_(other.free_list_)
    {
        other.buffer_ = nullptr;
        other.free_list_ = nullptr;
        other.available_blocks_ = 0;
        other.block_count_ = 0;
        other.total_buffer_size_ = 0;
    }

    FixedBlockPool& operator=(FixedBlockPool&& other) noexcept {
        if (this != &other) {
            if (buffer_) {
                ::operator delete(buffer_, std::align_val_t{alignment_});
            }
            requested_block_size_ = other.requested_block_size_;
            block_count_ = other.block_count_;
            alignment_ = other.alignment_;
            aligned_block_size_ = other.aligned_block_size_;
            total_buffer_size_ = other.total_buffer_size_;
            available_blocks_ = other.available_blocks_;
            buffer_ = other.buffer_;
            free_list_ = other.free_list_;

            other.buffer_ = nullptr;
            other.free_list_ = nullptr;
            other.available_blocks_ = 0;
            other.block_count_ = 0;
            other.total_buffer_size_ = 0;
        }
        return *this;
    }

    /**
     * @brief Allocates a single fixed-size block from the free list.
     * @return Pointer to aligned block storage, or nullptr if the pool is exhausted.
     */
    void* allocate() noexcept {
        if (!free_list_) {
            return nullptr;
        }
        FreeNode* node = free_list_;
        free_list_ = node->next;
        --available_blocks_;
        return static_cast<void*>(node);
    }

    /**
     * @brief Returns an allocated block to the free list.
     */
    void deallocate(void* ptr) noexcept {
        if (!ptr) {
            return;
        }
        FreeNode* node = static_cast<FreeNode*>(ptr);
        node->next = free_list_;
        free_list_ = node;
        ++available_blocks_;
    }

    /**
     * @brief Resets the free list to contain all blocks in contiguous order.
     *        Does not reallocate or free backing storage.
     */
    void reset() noexcept {
        available_blocks_ = block_count_;
        if (block_count_ == 0 || !buffer_) {
            free_list_ = nullptr;
            return;
        }
        for (std::size_t i = 0; i < block_count_ - 1; ++i) {
            FreeNode* curr = reinterpret_cast<FreeNode*>(buffer_ + (i * aligned_block_size_));
            FreeNode* next = reinterpret_cast<FreeNode*>(buffer_ + ((i + 1) * aligned_block_size_));
            curr->next = next;
        }
        FreeNode* last = reinterpret_cast<FreeNode*>(buffer_ + ((block_count_ - 1) * aligned_block_size_));
        last->next = nullptr;
        free_list_ = reinterpret_cast<FreeNode*>(buffer_);
    }

    /**
     * @brief Checks if a pointer originated from this pool and matches block alignment.
     */
    bool owns(const void* ptr) const noexcept {
        if (!ptr || !buffer_) {
            return false;
        }
        const std::byte* byte_ptr = static_cast<const std::byte*>(ptr);
        if (byte_ptr < buffer_ || byte_ptr >= buffer_ + total_buffer_size_) {
            return false;
        }
        std::size_t offset = static_cast<std::size_t>(byte_ptr - buffer_);
        return (offset % aligned_block_size_) == 0;
    }

    std::size_t block_size() const noexcept { return aligned_block_size_; }
    std::size_t requested_block_size() const noexcept { return requested_block_size_; }
    std::size_t capacity() const noexcept { return block_count_; }
    std::size_t available() const noexcept { return available_blocks_; }
    std::size_t blocks_in_use() const noexcept { return block_count_ - available_blocks_; }
    std::size_t alignment() const noexcept { return alignment_; }
    std::size_t reserved_bytes() const noexcept { return total_buffer_size_; }

    /**
     * @brief Calculates percentage of internal fragmentation waste for a given payload request size.
     */
    double internal_overhead_percent(std::size_t payload_size) const noexcept {
        if (payload_size >= aligned_block_size_) {
            return 0.0;
        }
        return (static_cast<double>(aligned_block_size_ - payload_size) / aligned_block_size_) * 100.0;
    }

private:
    struct FreeNode {
        FreeNode* next;
    };

    static std::size_t calculate_aligned_block_size(std::size_t raw_size, std::size_t align) noexcept {
        std::size_t base = std::max(raw_size, sizeof(FreeNode));
        std::size_t rem = base % align;
        return (rem == 0) ? base : (base + (align - rem));
    }

    std::size_t requested_block_size_;
    std::size_t block_count_;
    std::size_t alignment_;
    std::size_t aligned_block_size_;
    std::size_t total_buffer_size_;
    std::size_t available_blocks_;
    std::byte* buffer_;
    FreeNode* free_list_;
};

} // namespace performance_lab

#endif // PERFORMANCE_LAB_FIXED_BLOCK_POOL_HPP
