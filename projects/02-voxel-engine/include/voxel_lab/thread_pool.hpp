#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace voxel_lab {

class ThreadPool {
public:
    explicit ThreadPool(size_t thread_count);
    ~ThreadPool();

    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;
    ThreadPool(ThreadPool&&) = delete;
    ThreadPool& operator=(ThreadPool&&) = delete;

    // Enqueues a task for worker threads
    void enqueue(std::function<void()> task);

    // Waits until all queued tasks and currently running tasks have finished
    void wait_idle();

    // Requests clean shutdown and joins all worker threads
    void stop();

    size_t thread_count() const noexcept { return workers.size(); }
    size_t pending_tasks() const;
    size_t active_tasks_count() const noexcept { return active_tasks.load(std::memory_order_relaxed); }

private:
    std::vector<std::thread> workers;
    std::deque<std::function<void()>> tasks;
    mutable std::mutex queue_mutex;
    std::condition_variable cv_task;
    std::condition_variable cv_idle;
    std::atomic<bool> stopping{false};
    std::atomic<size_t> active_tasks{0};
};

} // namespace voxel_lab
