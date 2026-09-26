#ifndef PERFORMANCE_LAB_COMPILER_BARRIER_HPP
#define PERFORMANCE_LAB_COMPILER_BARRIER_HPP

#if defined(_MSC_VER)
#include <intrin.h>
#endif

namespace performance_lab {

/**
 * @brief Prevents compiler dead-code elimination by forcing a value to be consumed.
 * 
 * Guarantees:
 * - On GCC/Clang: Uses inline assembly with memory clobber ("+r,m") to prevent the
 *   compiler from optimizing away computations whose result is passed into this function.
 * - On MSVC: Uses a volatile pointer write/read barrier to ensure the compiler cannot
 *   assume the value is unused.
 * 
 * Limitations:
 * - Does not prevent the compiler from optimizing or unrolling operations *inside*
 *   the benchmark loop before the barrier call.
 */
template <typename T>
inline void do_not_optimize(T&& value) {
#if defined(__GNUC__) || defined(__clang__)
    asm volatile("" : "+r,m"(value) :: "memory");
#elif defined(_MSC_VER)
    // Force compiler to write to memory and prevent dead-code stripping
    char volatile* p = reinterpret_cast<char volatile*>(&value);
    (void)*p;
    _ReadWriteBarrier();
#else
    static volatile T sink = value;
    (void)sink;
#endif
}

/**
 * @brief Memory clobber barrier preventing memory operation reordering across the barrier.
 */
inline void clobber_memory() {
#if defined(__GNUC__) || defined(__clang__)
    asm volatile("" ::: "memory");
#elif defined(_MSC_VER)
    _ReadWriteBarrier();
#else
    // Fallback
#endif
}

} // namespace performance_lab

#endif // PERFORMANCE_LAB_COMPILER_BARRIER_HPP
