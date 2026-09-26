#ifndef PERFORMANCE_LAB_SIMD_KERNEL_DOT_HPP
#define PERFORMANCE_LAB_SIMD_KERNEL_DOT_HPP

#include "performance_lab/simd_caps.hpp"
#include <cstddef>
#include <cstdint>
#include <vector>
#include <cmath>

namespace performance_lab {

// ============================================================================
// Kernel A: Vector Fused Arithmetic (sum += a[i] * b[i] + c[i])
// ============================================================================

/**
 * @brief Reference Scalar Implementation (Auto-Vectorization Explicitly Disabled).
 */
inline float kernel_dot_scalar_reference(const float* a, const float* b, const float* c, size_t n) {
    float sum = 0.0f;
#if defined(_MSC_VER)
    #pragma loop(no_vector)
#elif defined(__clang__)
    #pragma clang loop vectorize(disable)
#elif defined(__GNUC__)
    #pragma GCC optimize("no-tree-vectorize")
#endif
    for (size_t i = 0; i < n; ++i) {
        sum += a[i] * b[i] + c[i];
    }
    return sum;
}

/**
 * @brief Compiler-Optimized Implementation (Allows Compiler Auto-Vectorization).
 */
inline float kernel_dot_compiler_opt(const float* a, const float* b, const float* c, size_t n) {
    float sum = 0.0f;
    for (size_t i = 0; i < n; ++i) {
        sum += a[i] * b[i] + c[i];
    }
    return sum;
}

/**
 * @brief Explicit SSE2/SSE4.1 Implementation (4 Floats per Step).
 */
inline float kernel_dot_sse(const float* a, const float* b, const float* c, size_t n) {
    float sum = 0.0f;
#if defined(PERFORMANCE_LAB_HAS_SSE_INTRINSICS)
    size_t vec_len = n & ~size_t(3); // n / 4 * 4
    __m128 vsum = _mm_setzero_ps();

    for (size_t i = 0; i < vec_len; i += 4) {
        __m128 va = _mm_loadu_ps(a + i);
        __m128 vb = _mm_loadu_ps(b + i);
        __m128 vc = _mm_loadu_ps(c + i);
        __m128 vprod = _mm_mul_ps(va, vb);
        __m128 vres = _mm_add_ps(vprod, vc);
        vsum = _mm_add_ps(vsum, vres);
    }

    alignas(16) float tmp[4];
    _mm_storeu_ps(tmp, vsum);
    sum = tmp[0] + tmp[1] + tmp[2] + tmp[3];

    // Scalar Tail Loop
    for (size_t i = vec_len; i < n; ++i) {
        sum += a[i] * b[i] + c[i];
    }
#else
    sum = kernel_dot_scalar_reference(a, b, c, n);
#endif
    return sum;
}

/**
 * @brief Explicit AVX2 Implementation (8 Floats per Step using FMA).
 */
inline float kernel_dot_avx2(const float* a, const float* b, const float* c, size_t n) {
    float sum = 0.0f;
#if defined(PERFORMANCE_LAB_HAS_AVX2_INTRINSICS)
    if (!SimdCapabilities::has_avx2()) {
        return kernel_dot_sse(a, b, c, n);
    }

    size_t vec_len = n & ~size_t(7); // n / 8 * 8
    __m256 vsum = _mm256_setzero_ps();

    for (size_t i = 0; i < vec_len; i += 8) {
        __m256 va = _mm256_loadu_ps(a + i);
        __m256 vb = _mm256_loadu_ps(b + i);
        __m256 vc = _mm256_loadu_ps(c + i);
        __m256 vres = _mm256_fmadd_ps(va, vb, vc);
        vsum = _mm256_add_ps(vsum, vres);
    }

    alignas(32) float tmp[8];
    _mm256_storeu_ps(tmp, vsum);
    sum = tmp[0] + tmp[1] + tmp[2] + tmp[3] + tmp[4] + tmp[5] + tmp[6] + tmp[7];

    // Scalar Tail Loop
    for (size_t i = vec_len; i < n; ++i) {
        sum += a[i] * b[i] + c[i];
    }
#else
    sum = kernel_dot_sse(a, b, c, n);
#endif
    return sum;
}

/**
 * @brief Explicit ARM NEON Implementation (4 Floats per Step).
 */
inline float kernel_dot_neon(const float* a, const float* b, const float* c, size_t n) {
    float sum = 0.0f;
#if defined(PERFORMANCE_LAB_HAS_NEON_INTRINSICS)
    size_t vec_len = n & ~size_t(3);
    float32x4_t vsum = vdupq_n_f32(0.0f);

    for (size_t i = 0; i < vec_len; i += 4) {
        float32x4_t va = vld1q_f32(a + i);
        float32x4_t vb = vld1q_f32(b + i);
        float32x4_t vc = vld1q_f32(c + i);
        float32x4_t vres = vmlaq_f32(vc, va, vb);
        vsum = vaddq_f32(vsum, vres);
    }

    float tmp[4];
    vst1q_f32(tmp, vsum);
    sum = tmp[0] + tmp[1] + tmp[2] + tmp[3];

    for (size_t i = vec_len; i < n; ++i) {
        sum += a[i] * b[i] + c[i];
    }
#else
    sum = kernel_dot_scalar_reference(a, b, c, n);
#endif
    return sum;
}

} // namespace performance_lab

#endif // PERFORMANCE_LAB_SIMD_KERNEL_DOT_HPP
