#include "performance_lab/atomic_lockfree_traits.hpp"
#include "performance_lab/mpmc_bounded_queue.hpp"
#include "performance_lab/mutex_bounded_queue.hpp"
#include "performance_lab/queue_payload.hpp"
#include "performance_lab/spsc_queue.hpp"

#include <cassert>
#include <iostream>
#include <latch>
#include <thread>
#include <vector>

#if defined(_M_X64) || defined(__x86_64__)
#include <immintrin.h>
#endif

using namespace performance_lab;

void test_atomic_traits() {
    std::cout << "[Test 1] Atomic Lock-Free Status Traits...\n";
    AtomicLockFreeTraits::print();
    assert(AtomicLockFreeTraits::is_size_t_lock_free());
    assert(AtomicLockFreeTraits::is_uint64_lock_free());
}

void test_spsc_basic() {
    std::cout << "[Test 2] SPSC Basic Operations & FIFO...\n";
    SpscQueue<QueueItem> queue(4); // Capacity rounded to 4
    assert(queue.capacity() == 4);
    assert(queue.empty());
    assert(queue.size() == 0);

    QueueItem item{};
    assert(!queue.try_pop(item));

    // Fill to capacity
    for (uint64_t i = 0; i < 4; ++i) {
        assert(queue.try_push(QueueItem::make(i)));
    }
    // Full
    assert(!queue.try_push(QueueItem::make(999)));

    // Pop and verify FIFO order
    for (uint64_t i = 0; i < 4; ++i) {
        assert(queue.try_pop(item));
        assert(item.sequence == i);
        assert(item.is_valid());
    }
    assert(queue.empty());
    assert(!queue.try_pop(item));

    // Wrap-around test (exercise several cycles)
    for (uint64_t cycle = 0; cycle < 100; ++cycle) {
        assert(queue.try_push(QueueItem::make(cycle)));
        assert(queue.try_pop(item));
        assert(item.sequence == cycle);
        assert(item.is_valid());
    }
}

void test_spsc_threaded() {
    std::cout << "[Test 3] SPSC Single-Producer Single-Consumer Threaded...\n";
    constexpr size_t total_items = 200'000;
    SpscQueue<QueueItem> queue(1024);

    std::thread producer([&]() {
        for (uint64_t i = 0; i < total_items; ++i) {
            QueueItem item = QueueItem::make(i);
            while (!queue.try_push(item)) {
                #if defined(_M_X64) || defined(__x86_64__)
                _mm_pause();
                #else
                std::this_thread::yield();
                #endif
            }
        }
    });

    std::thread consumer([&]() {
        QueueItem item{};
        for (uint64_t expected_seq = 0; expected_seq < total_items; ++expected_seq) {
            while (!queue.try_pop(item)) {
                #if defined(_M_X64) || defined(__x86_64__)
                _mm_pause();
                #else
                std::this_thread::yield();
                #endif
            }
            assert(item.sequence == expected_seq);
            assert(item.is_valid());
        }
    });

    producer.join();
    consumer.join();
}

void test_mutex_queue_basic_and_threaded() {
    std::cout << "[Test 4] Mutex Bounded Queue Basic & Threaded...\n";
    MutexBoundedQueue<QueueItem> queue(4);
    assert(queue.capacity() == 4);
    assert(queue.empty());

    QueueItem item{};
    assert(!queue.try_pop(item));

    for (uint64_t i = 0; i < 4; ++i) {
        assert(queue.try_push(QueueItem::make(i)));
    }
    assert(!queue.try_push(QueueItem::make(999)));

    for (uint64_t i = 0; i < 4; ++i) {
        assert(queue.try_pop(item));
        assert(item.sequence == i);
        assert(item.is_valid());
    }
    assert(queue.empty());

    // Multi-threaded 2P / 2C test
    constexpr size_t total_items = 50'000;
    constexpr size_t num_producers = 2;
    constexpr size_t num_consumers = 2;
    constexpr size_t items_per_producer = total_items / num_producers;

    MutexBoundedQueue<QueueItem> m_queue(64);
    std::vector<uint32_t> received(total_items, 0);
    std::mutex received_mutex;

    std::vector<std::thread> producers;
    producers.reserve(num_producers);
    for (size_t p = 0; p < num_producers; ++p) {
        producers.emplace_back([&, p]() {
            const uint64_t start_seq = p * items_per_producer;
            const uint64_t end_seq = start_seq + items_per_producer;
            for (uint64_t seq = start_seq; seq < end_seq; ++seq) {
                m_queue.push(QueueItem::make(seq));
            }
        });
    }

    std::atomic<size_t> consumed_count{0};
    std::vector<std::thread> consumers;
    consumers.reserve(num_consumers);
    for (size_t c = 0; c < num_consumers; ++c) {
        consumers.emplace_back([&]() {
            while (consumed_count.load(std::memory_order_relaxed) < total_items) {
                QueueItem pop_item{};
                if (m_queue.try_pop(pop_item)) {
                    assert(pop_item.is_valid());
                    {
                        std::lock_guard<std::mutex> lk(received_mutex);
                        assert(pop_item.sequence < total_items);
                        received[pop_item.sequence]++;
                    }
                    consumed_count.fetch_add(1, std::memory_order_relaxed);
                } else {
                    std::this_thread::yield();
                }
            }
        });
    }

    for (auto& t : producers) t.join();
    for (auto& t : consumers) t.join();

    assert(consumed_count.load() == total_items);
    for (size_t i = 0; i < total_items; ++i) {
        assert(received[i] == 1);
    }
}

void test_mpmc_basic_and_threaded() {
    std::cout << "[Test 5] MPMC Bounded Queue Basic & Threaded...\n";
    MpmcBoundedQueue<QueueItem> queue(4);
    assert(queue.capacity() == 4);

    QueueItem item{};
    assert(!queue.try_pop(item));

    for (uint64_t i = 0; i < 4; ++i) {
        assert(queue.try_push(QueueItem::make(i)));
    }
    assert(!queue.try_push(QueueItem::make(999)));

    for (uint64_t i = 0; i < 4; ++i) {
        assert(queue.try_pop(item));
        assert(item.sequence == i);
        assert(item.is_valid());
    }
    assert(!queue.try_pop(item));

    // Multi-threaded 4P / 4C test
    constexpr size_t total_items = 100'000;
    constexpr size_t num_producers = 4;
    constexpr size_t num_consumers = 4;
    constexpr size_t items_per_producer = total_items / num_producers;

    MpmcBoundedQueue<QueueItem> mpmc(256);
    std::vector<std::atomic<uint32_t>> received(total_items);
    for (auto& v : received) {
        v.store(0, std::memory_order_relaxed);
    }

    std::vector<std::thread> producers;
    producers.reserve(num_producers);
    for (size_t p = 0; p < num_producers; ++p) {
        producers.emplace_back([&, p]() {
            const uint64_t start_seq = p * items_per_producer;
            const uint64_t end_seq = start_seq + items_per_producer;
            for (uint64_t seq = start_seq; seq < end_seq; ++seq) {
                QueueItem push_item = QueueItem::make(seq);
                while (!mpmc.try_push(push_item)) {
                    #if defined(_M_X64) || defined(__x86_64__)
                    _mm_pause();
                    #else
                    std::this_thread::yield();
                    #endif
                }
            }
        });
    }

    std::atomic<size_t> consumed_count{0};
    std::vector<std::thread> consumers;
    consumers.reserve(num_consumers);
    for (size_t c = 0; c < num_consumers; ++c) {
        consumers.emplace_back([&]() {
            while (consumed_count.load(std::memory_order_relaxed) < total_items) {
                QueueItem pop_item{};
                if (mpmc.try_pop(pop_item)) {
                    assert(pop_item.is_valid());
                    assert(pop_item.sequence < total_items);
                    received[pop_item.sequence].fetch_add(1, std::memory_order_relaxed);
                    consumed_count.fetch_add(1, std::memory_order_relaxed);
                } else {
                    #if defined(_M_X64) || defined(__x86_64__)
                    _mm_pause();
                    #else
                    std::this_thread::yield();
                    #endif
                }
            }
        });
    }

    for (auto& t : producers) t.join();
    for (auto& t : consumers) t.join();

    assert(consumed_count.load() == total_items);
    for (size_t i = 0; i < total_items; ++i) {
        assert(received[i].load() == 1);
    }
}

void test_stress_validation() {
    std::cout << "[Test 6] Stress Validation (MPMC 4P/4C with 1,000,000 items)...\n";
    constexpr size_t total_items = 1'000'000;
    constexpr size_t num_producers = 4;
    constexpr size_t num_consumers = 4;
    constexpr size_t items_per_producer = total_items / num_producers;
    constexpr size_t queue_capacity = 1024;

    MpmcBoundedQueue<QueueItem> queue(queue_capacity);
    std::vector<std::atomic<uint8_t>> received(total_items);
    for (auto& v : received) {
        v.store(0, std::memory_order_relaxed);
    }

    std::atomic<size_t> total_produced{0};
    std::atomic<size_t> total_consumed{0};
    std::atomic<size_t> corruption_count{0};

    std::vector<std::thread> producers;
    producers.reserve(num_producers);
    for (size_t p = 0; p < num_producers; ++p) {
        producers.emplace_back([&, p]() {
            const uint64_t start_seq = p * items_per_producer;
            const uint64_t end_seq = start_seq + items_per_producer;
            for (uint64_t seq = start_seq; seq < end_seq; ++seq) {
                QueueItem item = QueueItem::make(seq);
                while (!queue.try_push(item)) {
                    #if defined(_M_X64) || defined(__x86_64__)
                    _mm_pause();
                    #else
                    std::this_thread::yield();
                    #endif
                }
                total_produced.fetch_add(1, std::memory_order_relaxed);
            }
        });
    }

    std::vector<std::thread> consumers;
    consumers.reserve(num_consumers);
    for (size_t c = 0; c < num_consumers; ++c) {
        consumers.emplace_back([&]() {
            while (total_consumed.load(std::memory_order_relaxed) < total_items) {
                QueueItem item{};
                if (queue.try_pop(item)) {
                    if (!item.is_valid()) {
                        corruption_count.fetch_add(1, std::memory_order_relaxed);
                    }
                    if (item.sequence < total_items) {
                        received[item.sequence].fetch_add(1, std::memory_order_relaxed);
                    }
                    total_consumed.fetch_add(1, std::memory_order_relaxed);
                } else {
                    #if defined(_M_X64) || defined(__x86_64__)
                    _mm_pause();
                    #else
                    std::this_thread::yield();
                    #endif
                }
            }
        });
    }

    for (auto& t : producers) t.join();
    for (auto& t : consumers) t.join();

    size_t missing_count = 0;
    size_t duplicate_count = 0;
    for (size_t i = 0; i < total_items; ++i) {
        uint8_t count = received[i].load(std::memory_order_relaxed);
        if (count == 0) {
            missing_count++;
        } else if (count > 1) {
            duplicate_count += (count - 1);
        }
    }

    std::cout << "  Expected items   : " << total_items << "\n";
    std::cout << "  Produced items   : " << total_produced.load() << "\n";
    std::cout << "  Consumed items   : " << total_consumed.load() << "\n";
    std::cout << "  Missing items    : " << missing_count << "\n";
    std::cout << "  Duplicate items  : " << duplicate_count << "\n";
    std::cout << "  Corruption items : " << corruption_count.load() << "\n";

    assert(total_produced.load() == total_items);
    assert(total_consumed.load() == total_items);
    assert(missing_count == 0);
    assert(duplicate_count == 0);
    assert(corruption_count.load() == 0);
}

int main() {
    std::cout << "=== Running test_lockfree_queues ===\n";

    test_atomic_traits();
    test_spsc_basic();
    test_spsc_threaded();
    test_mutex_queue_basic_and_threaded();
    test_mpmc_basic_and_threaded();
    test_stress_validation();

    std::cout << "=== All Queue Unit Tests & Stress Validation PASSED cleanly ===\n";
    return 0;
}
