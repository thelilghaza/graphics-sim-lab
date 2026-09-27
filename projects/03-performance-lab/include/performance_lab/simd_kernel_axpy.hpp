#ifndef PERFORMANCE_LAB_SIMD_KERNEL_AXPY_HPP
#define PERFORMANCE_LAB_SIMD_KERNEL_AXPY_HPP

#include "performance_lab/simd_caps.hpp"
#include <cstddef>
#include <cstdint>
#include <vector>

namespace performance_lab {

// ============================================================================
// Kernel B: AXPY Vector Transform (y[i] = a * x[i] + y[i])
// ============================================================================

/**
 * @brief Reference Scalar Implementation (Auto-Vectorization Explicitly Disabled).
 */
#if defined(__GNUC__) && !defined(__clang__)
__attribute__((optimize("no-tree-vectorize")))
#endif
inline void kernel_axpy_scalar_reference(float alpha, const float* x, float* y, size_t n) {
#if defined(_MSC_VER)
    #pragma loop(no_vector)
#elif defined(__clang__)
    #pragma clang loop vectorize(disable)
#endif
    for (size_t i = 0; i < n; ++i) {
        y[i] = alpha * x[i] + y[i];
    }
}

/**
 * @brief Compiler-Optimized Implementation (Allows Compiler Auto-Vectorization).
 */
inline void kernel_axpy_compiler_opt(float alpha, const float* x, float* y, size_t n) {
    for (size_t i = 0; i < n; ++i) {
        y[i] = alpha * x[i] + y[i];
    }
}

/**
 * @brief Explicit SSE2 Implementation (4 Floats per Step).
 */
inline void kernel_axpy_sse(float alpha, const float* x, float* y, size_t n) {
#if defined(PERFORMANCE_LAB_HAS_SSE_INTRINSICS)
    size_t vec_len = n & ~size_t(3); // n / 4 * 4
    __m128 valpha = _mm_set1_ps(alpha);

    for (size_t i = 0; i < vec_len; i += 4) {
        __m128 vx = _mm_loadu_ps(x + i);
        __m128 vy = _mm_loadu_ps(y + i);
        __m128 vres = _mm_add_ps(_mm_mul_ps(valpha, vx), vy);
        _mm_storeu_ps(y + i, vres);
    }

    // Scalar Tail Loop
    for (size_t i = vec_len; i < n; ++i) {
        y[i] = alpha * x[i] + y[i];
    }
#else
    kernel_axpy_scalar_reference(alpha, x, y, n);
#endif
}

/**
 * @brief Explicit AVX2 Implementation (8 Floats per Step using FMA).
 */
#if (defined(__GNUC__) || defined(__clang__)) && defined(PERFORMANCE_LAB_ARCH_X86)
__attribute__((target("avx2,fma")))
#endif
inline void kernel_axpy_avx2(float alpha, const float* x, float* y, size_t n) {
#if defined(PERFORMANCE_LAB_HAS_AVX2_INTRINSICS)
    if (!SimdCapabilities::has_avx2()) {
        kernel_axpy_sse(alpha, x, y, n);
        return;
    }

    size_t vec_len = n & ~size_t(7); // n / 8 * 8
    __m256 valpha = _mm256_set1_ps(alpha);

    for (size_t i = 0; i < vec_len; i += 8) {
        __m256 vx = _mm256_loadu_ps(x + i);
        __m256 vy = _mm256_loadu_ps(y + i);
        __m256 vres = _mm256_fmadd_ps(valpha, vx, vy);
        _mm256_storeu_ps(y + i, vres);
    }

    // Scalar Tail Loop
    for (size_t i = vec_len; i < n; ++i) {
        y[i] = alpha * x[i] + y[i];
    }
#else
    kernel_axpy_sse(alpha, x, y, n);
#endif
}

/**
 * @brief Explicit ARM NEON Implementation (4 Floats per Step).
 */
inline void kernel_axpy_neon(float alpha, const float* x, float* y, size_t n) {
#if defined(PERFORMANCE_LAB_HAS_NEON_INTRINSICS)
    size_t vec_len = n & ~size_t(3);
    float32x4_t valpha = vdupq_n_f32(alpha);

    for (size_t i = 0; i < vec_len; i += 4) {
        float32x4_t vx = vld1q_f32(x + i);
        float32x4_t vy = vld1q_f32(y + i);
        float32x4_t vres = vmlaq_f32(vy, valpha, vx);
        vst1q_f32(y + i, vres);
    }

    for (size_t i = vec_len; i < n; ++i) {
        y[i] = alpha * x[i] + y[i];
    }
#else
    kernel_axpy_scalar_reference(alpha, x, y, n);
#endif
}

} // namespace performance_lab

#endif // PERFORMANCE_LAB_SIMD_KERNEL_AXPY_HPP
