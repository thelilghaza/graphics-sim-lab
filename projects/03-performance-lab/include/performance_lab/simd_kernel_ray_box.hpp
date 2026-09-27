#ifndef PERFORMANCE_LAB_SIMD_KERNEL_RAY_BOX_HPP
#define PERFORMANCE_LAB_SIMD_KERNEL_RAY_BOX_HPP

#include "performance_lab/simd_caps.hpp"
#include <cstddef>
#include <cstdint>
#include <vector>
#include <algorithm>

namespace performance_lab {

struct AABB3D {
    float min_x{-1.0f}, min_y{-1.0f}, min_z{-1.0f};
    float max_x{1.0f},  max_y{1.0f},  max_z{1.0f};
};

struct RayPacket4 {
    float orig_x[4];
    float orig_y[4];
    float orig_z[4];
    float inv_dir_x[4];
    float inv_dir_y[4];
    float inv_dir_z[4];
};

struct RayPacket8 {
    float orig_x[8];
    float orig_y[8];
    float orig_z[8];
    float inv_dir_x[8];
    float inv_dir_y[8];
    float inv_dir_z[8];
};

// ============================================================================
// Kernel C: 4-Ray & 8-Ray Batch Ray-AABB Intersection
// ============================================================================

/**
 * @brief Reference Scalar Implementation (Auto-Vectorization Explicitly Disabled).
 */
#if defined(__GNUC__) && !defined(__clang__)
__attribute__((optimize("no-tree-vectorize")))
#endif
inline uint32_t kernel_ray_box_4_scalar_reference(const RayPacket4& rays, const AABB3D& box) {
    uint32_t hit_mask = 0;
#if defined(_MSC_VER)
    #pragma loop(no_vector)
#elif defined(__clang__)
    #pragma clang loop vectorize(disable)
#endif
    for (int k = 0; k < 4; ++k) {
        float t1_x = (box.min_x - rays.orig_x[k]) * rays.inv_dir_x[k];
        float t2_x = (box.max_x - rays.orig_x[k]) * rays.inv_dir_x[k];
        float tmin_x = (std::min)(t1_x, t2_x);
        float tmax_x = (std::max)(t1_x, t2_x);

        float t1_y = (box.min_y - rays.orig_y[k]) * rays.inv_dir_y[k];
        float t2_y = (box.max_y - rays.orig_y[k]) * rays.inv_dir_y[k];
        float tmin_y = (std::min)(t1_y, t2_y);
        float tmax_y = (std::max)(t1_y, t2_y);

        float t1_z = (box.min_z - rays.orig_z[k]) * rays.inv_dir_z[k];
        float t2_z = (box.max_z - rays.orig_z[k]) * rays.inv_dir_z[k];
        float tmin_z = (std::min)(t1_z, t2_z);
        float tmax_z = (std::max)(t1_z, t2_z);

        float t_near = (std::max)((std::max)(tmin_x, tmin_y), tmin_z);
        float t_far  = (std::min)((std::min)(tmax_x, tmax_y), tmax_z);

        if (t_far >= t_near && t_far >= 0.0f) {
            hit_mask |= (1u << k);
        }
    }
    return hit_mask;
}

/**
 * @brief Compiler-Optimized Implementation (Allows Compiler Auto-Vectorization).
 */
inline uint32_t kernel_ray_box_4_compiler_opt(const RayPacket4& rays, const AABB3D& box) {
    uint32_t hit_mask = 0;
    for (int k = 0; k < 4; ++k) {
        float t1_x = (box.min_x - rays.orig_x[k]) * rays.inv_dir_x[k];
        float t2_x = (box.max_x - rays.orig_x[k]) * rays.inv_dir_x[k];
        float tmin_x = (std::min)(t1_x, t2_x);
        float tmax_x = (std::max)(t1_x, t2_x);

        float t1_y = (box.min_y - rays.orig_y[k]) * rays.inv_dir_y[k];
        float t2_y = (box.max_y - rays.orig_y[k]) * rays.inv_dir_y[k];
        float tmin_y = (std::min)(t1_y, t2_y);
        float tmax_y = (std::max)(t1_y, t2_y);

        float t1_z = (box.min_z - rays.orig_z[k]) * rays.inv_dir_z[k];
        float t2_z = (box.max_z - rays.orig_z[k]) * rays.inv_dir_z[k];
        float tmin_z = (std::min)(t1_z, t2_z);
        float tmax_z = (std::max)(t1_z, t2_z);

        float t_near = (std::max)((std::max)(tmin_x, tmin_y), tmin_z);
        float t_far  = (std::min)((std::min)(tmax_x, tmax_y), tmax_z);

        if (t_far >= t_near && t_far >= 0.0f) {
            hit_mask |= (1u << k);
        }
    }
    return hit_mask;
}

/**
 * @brief Explicit SSE2 Implementation (4-Wide SIMD Ray-AABB Intersection).
 */
inline uint32_t kernel_ray_box_4_sse(const RayPacket4& rays, const AABB3D& box) {
#if defined(PERFORMANCE_LAB_HAS_SSE_INTRINSICS)
    __m128 vorig_x = _mm_loadu_ps(rays.orig_x);
    __m128 vorig_y = _mm_loadu_ps(rays.orig_y);
    __m128 vorig_z = _mm_loadu_ps(rays.orig_z);

    __m128 vinv_x = _mm_loadu_ps(rays.inv_dir_x);
    __m128 vinv_y = _mm_loadu_ps(rays.inv_dir_y);
    __m128 vinv_z = _mm_loadu_ps(rays.inv_dir_z);

    __m128 vmin_x = _mm_set1_ps(box.min_x);
    __m128 vmin_y = _mm_set1_ps(box.min_y);
    __m128 vmin_z = _mm_set1_ps(box.min_z);

    __m128 vmax_x = _mm_set1_ps(box.max_x);
    __m128 vmax_y = _mm_set1_ps(box.max_y);
    __m128 vmax_z = _mm_set1_ps(box.max_z);

    __m128 vt1_x = _mm_mul_ps(_mm_sub_ps(vmin_x, vorig_x), vinv_x);
    __m128 vt2_x = _mm_mul_ps(_mm_sub_ps(vmax_x, vorig_x), vinv_x);
    __m128 vtmin_x = _mm_min_ps(vt1_x, vt2_x);
    __m128 vtmax_x = _mm_max_ps(vt1_x, vt2_x);

    __m128 vt1_y = _mm_mul_ps(_mm_sub_ps(vmin_y, vorig_y), vinv_y);
    __m128 vt2_y = _mm_mul_ps(_mm_sub_ps(vmax_y, vorig_y), vinv_y);
    __m128 vtmin_y = _mm_min_ps(vt1_y, vt2_y);
    __m128 vtmax_y = _mm_max_ps(vt1_y, vt2_y);

    __m128 vt1_z = _mm_mul_ps(_mm_sub_ps(vmin_z, vorig_z), vinv_z);
    __m128 vt2_z = _mm_mul_ps(_mm_sub_ps(vmax_z, vorig_z), vinv_z);
    __m128 vtmin_z = _mm_min_ps(vt1_z, vt2_z);
    __m128 vtmax_z = _mm_max_ps(vt1_z, vt2_z);

    __m128 vnear = _mm_max_ps(_mm_max_ps(vtmin_x, vtmin_y), vtmin_z);
    __m128 vfar  = _mm_min_ps(_mm_min_ps(vtmax_x, vtmax_y), vtmax_z);

    __m128 vzero = _mm_setzero_ps();
    __m128 vcond1 = _mm_cmple_ps(vnear, vfar); // near <= far
    __m128 vcond2 = _mm_cmpge_ps(vfar, vzero); // far >= 0
    __m128 vhit   = _mm_and_ps(vcond1, vcond2);

    return static_cast<uint32_t>(_mm_movemask_ps(vhit));
#else
    return kernel_ray_box_4_scalar_reference(rays, box);
#endif
}

/**
 * @brief Explicit AVX2 Implementation (8-Wide SIMD Ray-AABB Intersection).
 */
#if (defined(__GNUC__) || defined(__clang__)) && defined(PERFORMANCE_LAB_ARCH_X86)
__attribute__((target("avx2,fma")))
#endif
inline uint32_t kernel_ray_box_8_avx2(const RayPacket8& rays, const AABB3D& box) {
#if defined(PERFORMANCE_LAB_HAS_AVX2_INTRINSICS)
    if (!SimdCapabilities::has_avx2()) {
        RayPacket4 r1, r2;
        for (int i = 0; i < 4; ++i) {
            r1.orig_x[i] = rays.orig_x[i]; r1.orig_y[i] = rays.orig_y[i]; r1.orig_z[i] = rays.orig_z[i];
            r1.inv_dir_x[i] = rays.inv_dir_x[i]; r1.inv_dir_y[i] = rays.inv_dir_y[i]; r1.inv_dir_z[i] = rays.inv_dir_z[i];

            r2.orig_x[i] = rays.orig_x[i+4]; r2.orig_y[i] = rays.orig_y[i+4]; r2.orig_z[i] = rays.orig_z[i+4];
            r2.inv_dir_x[i] = rays.inv_dir_x[i+4]; r2.inv_dir_y[i] = rays.inv_dir_y[i+4]; r2.inv_dir_z[i] = rays.inv_dir_z[i+4];
        }
        return kernel_ray_box_4_sse(r1, box) | (kernel_ray_box_4_sse(r2, box) << 4);
    }

    __m256 vorig_x = _mm256_loadu_ps(rays.orig_x);
    __m256 vorig_y = _mm256_loadu_ps(rays.orig_y);
    __m256 vorig_z = _mm256_loadu_ps(rays.orig_z);

    __m256 vinv_x = _mm256_loadu_ps(rays.inv_dir_x);
    __m256 vinv_y = _mm256_loadu_ps(rays.inv_dir_y);
    __m256 vinv_z = _mm256_loadu_ps(rays.inv_dir_z);

    __m256 vmin_x = _mm256_set1_ps(box.min_x);
    __m256 vmin_y = _mm256_set1_ps(box.min_y);
    __m256 vmin_z = _mm256_set1_ps(box.min_z);

    __m256 vmax_x = _mm256_set1_ps(box.max_x);
    __m256 vmax_y = _mm256_set1_ps(box.max_y);
    __m256 vmax_z = _mm256_set1_ps(box.max_z);

    __m256 vt1_x = _mm256_mul_ps(_mm256_sub_ps(vmin_x, vorig_x), vinv_x);
    __m256 vt2_x = _mm256_mul_ps(_mm256_sub_ps(vmax_x, vorig_x), vinv_x);
    __m256 vtmin_x = _mm256_min_ps(vt1_x, vt2_x);
    __m256 vtmax_x = _mm256_max_ps(vt1_x, vt2_x);

    __m256 vt1_y = _mm256_mul_ps(_mm256_sub_ps(vmin_y, vorig_y), vinv_y);
    __m256 vt2_y = _mm256_mul_ps(_mm256_sub_ps(vmax_y, vorig_y), vinv_y);
    __m256 vtmin_y = _mm256_min_ps(vt1_y, vt2_y);
    __m256 vtmax_y = _mm256_max_ps(vt1_y, vt2_y);

    __m256 vt1_z = _mm256_mul_ps(_mm256_sub_ps(vmin_z, vorig_z), vinv_z);
    __m256 vt2_z = _mm256_mul_ps(_mm256_sub_ps(vmax_z, vorig_z), vinv_z);
    __m256 vtmin_z = _mm256_min_ps(vt1_z, vt2_z);
    __m256 vtmax_z = _mm256_max_ps(vt1_z, vt2_z);

    __m256 vnear = _mm256_max_ps(_mm256_max_ps(vtmin_x, vtmin_y), vtmin_z);
    __m256 vfar  = _mm256_min_ps(_mm256_min_ps(vtmax_x, vtmax_y), vtmax_z);

    __m256 vzero = _mm256_setzero_ps();
    __m256 vcond1 = _mm256_cmp_ps(vnear, vfar, _CMP_LE_OQ);
    __m256 vcond2 = _mm256_cmp_ps(vfar, vzero, _CMP_GE_OQ);
    __m256 vhit   = _mm256_and_ps(vcond1, vcond2);

    return static_cast<uint32_t>(_mm256_movemask_ps(vhit));
#else
    RayPacket4 r1, r2;
    for (int i = 0; i < 4; ++i) {
        r1.orig_x[i] = rays.orig_x[i]; r1.orig_y[i] = rays.orig_y[i]; r1.orig_z[i] = rays.orig_z[i];
        r1.inv_dir_x[i] = rays.inv_dir_x[i]; r1.inv_dir_y[i] = rays.inv_dir_y[i]; r1.inv_dir_z[i] = rays.inv_dir_z[i];

        r2.orig_x[i] = rays.orig_x[i+4]; r2.orig_y[i] = rays.orig_y[i+4]; r2.orig_z[i] = rays.orig_z[i+4];
        r2.inv_dir_x[i] = rays.inv_dir_x[i+4]; r2.inv_dir_y[i] = rays.inv_dir_y[i+4]; r2.inv_dir_z[i] = rays.inv_dir_z[i+4];
    }
    return kernel_ray_box_4_scalar_reference(r1, box) | (kernel_ray_box_4_scalar_reference(r2, box) << 4);
#endif
}

} // namespace performance_lab

#endif // PERFORMANCE_LAB_SIMD_KERNEL_RAY_BOX_HPP
