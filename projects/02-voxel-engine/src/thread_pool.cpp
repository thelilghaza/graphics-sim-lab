#include "voxel_lab/thread_pool.hpp"

namespace voxel_lab {

ThreadPool::ThreadPool(size_t thread_count) {
    if (thread_count == 0) {
        thread_count = 1;
    }
    workers.reserve(thread_count);

    for (size_t i = 0; i < thread_count; ++i) {
        workers.emplace_back([this]() {
            while (true) {
                std::function<void()> task;
                {
                    std::unique_lock<std::mutex> lock(queue_mutex);
                    cv_task.wait(lock, [this]() {
                        return stopping.load(std::memory_order_relaxed) || !tasks.empty();
                    });

                    if (stopping.load(std::memory_order_relaxed) && tasks.empty()) {
                        return;
                    }

                    task = std::move(tasks.front());
                    tasks.pop_front();
                    active_tasks.fetch_add(1, std::memory_order_relaxed);
                }

                task();

                {
                    std::unique_lock<std::mutex> lock(queue_mutex);
                    active_tasks.fetch_sub(1, std::memory_order_relaxed);
                    if (tasks.empty() && active_tasks.load(std::memory_order_relaxed) == 0) {
                        cv_idle.notify_all();
                    }
                }
            }
        });
    }
}

ThreadPool::~ThreadPool() {
    stop();
}

void ThreadPool::enqueue(std::function<void()> task) {
    {
        std::unique_lock<std::mutex> lock(queue_mutex);
        if (stopping.load(std::memory_order_relaxed)) {
            return;
        }
        tasks.push_back(std::move(task));
    }
    cv_task.notify_one();
}

void ThreadPool::wait_idle() {
    std::unique_lock<std::mutex> lock(queue_mutex);
    cv_idle.wait(lock, [this]() {
        return tasks.empty() && active_tasks.load(std::memory_order_relaxed) == 0;
    });
}

void ThreadPool::stop() {
    {
        std::unique_lock<std::mutex> lock(queue_mutex);
        if (stopping.load(std::memory_order_relaxed)) {
            return;
        }
        stopping.store(true, std::memory_order_relaxed);
    }
    cv_task.notify_all();

    for (auto& worker : workers) {
        if (worker.joinable()) {
            worker.join();
        }
    }
    workers.clear();
}

size_t ThreadPool::pending_tasks() const {
    std::unique_lock<std::mutex> lock(queue_mutex);
    return tasks.size() + active_tasks.load(std::memory_order_relaxed);
}

} // namespace voxel_lab
