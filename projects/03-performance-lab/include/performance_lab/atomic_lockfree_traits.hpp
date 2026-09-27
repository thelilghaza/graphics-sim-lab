#ifndef PERFORMANCE_LAB_ATOMIC_LOCKFREE_TRAITS_HPP
#define PERFORMANCE_LAB_ATOMIC_LOCKFREE_TRAITS_HPP

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string>

namespace performance_lab {

/**
 * @brief Reports lock-free status of atomic types used in queue index and sequence operations.
 */
struct AtomicLockFreeTraits {
    /**
     * @brief Checks compile-time and runtime lock-free property of std::atomic<std::size_t>.
     */
    static bool is_size_t_lock_free() noexcept {
        static const std::atomic<std::size_t> test_val{0};
        return test_val.is_lock_free();
    }

    /**
     * @brief Checks compile-time and runtime lock-free property of std::atomic<std::uint64_t>.
     */
    static bool is_uint64_lock_free() noexcept {
        static const std::atomic<std::uint64_t> test_val{0};
        return test_val.is_lock_free();
    }

    /**
     * @brief Checks compile-time and runtime lock-free property of std::atomic<std::uint32_t>.
     */
    static bool is_uint32_lock_free() noexcept {
        static const std::atomic<std::uint32_t> test_val{0};
        return test_val.is_lock_free();
    }

    /**
     * @brief Returns a descriptive human-readable summary of atomic lock-free support.
     */
    static std::string summary() {
        std::string s;
        s += "size_t: ";
        s += (std::atomic<std::size_t>::is_always_lock_free ? "always lock-free" :
              is_size_t_lock_free() ? "runtime lock-free" : "not lock-free");
        s += " | uint64: ";
        s += (std::atomic<std::uint64_t>::is_always_lock_free ? "always lock-free" :
              is_uint64_lock_free() ? "runtime lock-free" : "not lock-free");
        s += " | uint32: ";
        s += (std::atomic<std::uint32_t>::is_always_lock_free ? "always lock-free" :
              is_uint32_lock_free() ? "runtime lock-free" : "not lock-free");
        return s;
    }

    /**
     * @brief Prints atomic traits to standard output.
     */
    static void print(std::ostream& os = std::cout) {
        os << "[Atomic Lock-Free Status] " << summary() << "\n";
    }
};

} // namespace performance_lab

#endif // PERFORMANCE_LAB_ATOMIC_LOCKFREE_TRAITS_HPP
