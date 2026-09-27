#ifndef PERFORMANCE_LAB_MUTEX_BOUNDED_QUEUE_HPP
#define PERFORMANCE_LAB_MUTEX_BOUNDED_QUEUE_HPP

#include <condition_variable>
#include <cstddef>
#include <mutex>
#include <stdexcept>
#include <vector>

namespace performance_lab {

/**
 * @brief Thread-safe bounded queue synchronized via std::mutex and std::condition_variable.
 *
 * Serves as the conventional synchronization baseline for M4 benchmarks.
 * Employs a pre-allocated fixed-capacity ring buffer to prevent per-item heap allocations.
 */
template <typename T>
class MutexBoundedQueue {
public:
    explicit MutexBoundedQueue(size_t capacity)
        : capacity_(capacity)
        , buffer_(capacity)
    {
        if (capacity == 0) {
            throw std::invalid_argument("MutexBoundedQueue capacity must be greater than zero");
        }
    }

    ~MutexBoundedQueue() = default;

    MutexBoundedQueue(const MutexBoundedQueue&) = delete;
    MutexBoundedQueue& operator=(const MutexBoundedQueue&) = delete;
    MutexBoundedQueue(MutexBoundedQueue&&) = delete;
    MutexBoundedQueue& operator=(MutexBoundedQueue&&) = delete;

    /**
     * @brief Attempts to enqueue an item without blocking.
     * @return true if enqueued successfully, false if queue is full.
     */
    bool try_push(const T& item) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (count_ == capacity_) {
            return false;
        }
        buffer_[tail_] = item;
        tail_ = (tail_ + 1 == capacity_) ? 0 : tail_ + 1;
        ++count_;
        lock.unlock();
        cv_not_empty_.notify_one();
        return true;
    }

    /**
     * @brief Attempts to dequeue an item without blocking.
     * @return true if dequeued successfully, false if queue is empty.
     */
    bool try_pop(T& item) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (count_ == 0) {
            return false;
        }
        item = buffer_[head_];
        head_ = (head_ + 1 == capacity_) ? 0 : head_ + 1;
        --count_;
        lock.unlock();
        cv_not_full_.notify_one();
        return true;
    }

    /**
     * @brief Enqueues an item, blocking if the queue is full.
     */
    void push(const T& item) {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_not_full_.wait(lock, [this]() { return count_ < capacity_; });
        buffer_[tail_] = item;
        tail_ = (tail_ + 1 == capacity_) ? 0 : tail_ + 1;
        ++count_;
        lock.unlock();
        cv_not_empty_.notify_one();
    }

    /**
     * @brief Dequeues an item, blocking if the queue is empty.
     */
    void pop(T& item) {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_not_empty_.wait(lock, [this]() { return count_ > 0; });
        item = buffer_[head_];
        head_ = (head_ + 1 == capacity_) ? 0 : head_ + 1;
        --count_;
        lock.unlock();
        cv_not_full_.notify_one();
    }

    /**
     * @brief Checks if queue is empty.
     */
    [[nodiscard]] bool empty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return count_ == 0;
    }

    /**
     * @brief Returns current number of elements.
     */
    [[nodiscard]] size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return count_;
    }

    /**
     * @brief Returns queue capacity.
     */
    [[nodiscard]] size_t capacity() const noexcept {
        return capacity_;
    }

private:
    const size_t capacity_;
    std::vector<T> buffer_;
    size_t head_{0};
    size_t tail_{0};
    size_t count_{0};
    mutable std::mutex mutex_;
    std::condition_variable cv_not_full_;
    std::condition_variable cv_not_empty_;
};

} // namespace performance_lab

#endif // PERFORMANCE_LAB_MUTEX_BOUNDED_QUEUE_HPP
