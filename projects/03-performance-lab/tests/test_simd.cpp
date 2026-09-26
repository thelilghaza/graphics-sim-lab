#include "performance_lab/simd_caps.hpp"
#include "performance_lab/simd_kernel_dot.hpp"
#include "performance_lab/simd_kernel_axpy.hpp"
#include "performance_lab/simd_kernel_ray_box.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

using namespace performance_lab;

void test_simd_capabilities() {
    std::cout << "[Test 1] SIMD Capabilities Layer...\n";
    std::cout << "  Active ISA: " << SimdCapabilities::active_isa_string() << "\n";
    assert(SimdCapabilities::has_sse2() == true); // On x64 MSVC, SSE2 is always supported
}

void test_kernel_dot_lengths_and_tails() {
    std::cout << "[Test 2] Kernel A (Dot) Lengths & Tail Handling...\n";
    const std::vector<size_t> test_lengths = {0, 1, 3, 4, 7, 8, 15, 16, 17, 31, 32, 33, 1000, 16384};

    for (size_t n : test_lengths) {
        std::vector<float> a(n, 1.25f);
        std::vector<float> b(n, 2.50f);
        std::vector<float> c(n, 0.75f);

        float res_scalar = kernel_dot_scalar_reference(a.data(), b.data(), c.data(), n);
        float res_opt    = kernel_dot_compiler_opt(a.data(), b.data(), c.data(), n);
        float res_sse    = kernel_dot_sse(a.data(), b.data(), c.data(), n);
        float res_avx2   = kernel_dot_avx2(a.data(), b.data(), c.data(), n);
        float res_neon   = kernel_dot_neon(a.data(), b.data(), c.data(), n);

        (void)res_scalar;
        (void)res_opt;
        (void)res_sse;
        (void)res_avx2;
        (void)res_neon;

        if (n == 0) {
            assert(res_scalar == 0.0f);
            assert(res_opt == 0.0f);
            assert(res_sse == 0.0f);
            assert(res_avx2 == 0.0f);
            assert(res_neon == 0.0f);
        } else {
            assert(std::abs(res_scalar - res_opt) / res_scalar < 1e-4f);
            assert(std::abs(res_scalar - res_sse) / res_scalar < 1e-4f);
            assert(std::abs(res_scalar - res_avx2) / res_scalar < 1e-4f);
            assert(std::abs(res_scalar - res_neon) / res_scalar < 1e-4f);
        }
    }
}

void test_kernel_axpy_lengths_and_tails() {
    std::cout << "[Test 3] Kernel B (AXPY) Lengths & Tail Handling...\n";
    const std::vector<size_t> test_lengths = {0, 1, 3, 4, 7, 8, 15, 16, 17, 31, 32, 33, 1000};
    float alpha = 3.14f;

    for (size_t n : test_lengths) {
        std::vector<float> x(n);
        std::vector<float> y_base(n);
        for (size_t i = 0; i < n; ++i) {
            x[i] = static_cast<float>(i + 1) * 0.1f;
            y_base[i] = static_cast<float>(i + 1) * 0.5f;
        }

        std::vector<float> y_scalar = y_base;
        std::vector<float> y_opt    = y_base;
        std::vector<float> y_sse    = y_base;
        std::vector<float> y_avx2   = y_base;
        std::vector<float> y_neon   = y_base;

        kernel_axpy_scalar_reference(alpha, x.data(), y_scalar.data(), n);
        kernel_axpy_compiler_opt(alpha, x.data(), y_opt.data(), n);
        kernel_axpy_sse(alpha, x.data(), y_sse.data(), n);
        kernel_axpy_avx2(alpha, x.data(), y_avx2.data(), n);
        kernel_axpy_neon(alpha, x.data(), y_neon.data(), n);

        for (size_t i = 0; i < n; ++i) {
            assert(std::abs(y_scalar[i] - y_opt[i]) < 1e-4f);
            assert(std::abs(y_scalar[i] - y_sse[i]) < 1e-4f);
            assert(std::abs(y_scalar[i] - y_avx2[i]) < 1e-4f);
            assert(std::abs(y_scalar[i] - y_neon[i]) < 1e-4f);
        }
    }
}

void test_kernel_ray_box_hits_and_misses() {
    std::cout << "[Test 4] Kernel C (Ray-AABB) Hit/Miss Equivalence...\n";
    AABB3D box{-1.0f, -1.0f, -1.0f, 1.0f, 1.0f, 1.0f};

    // Test Packet 1: 2 hitting rays, 2 missing rays
    RayPacket4 r4;
    // Ray 0: hits (moves along +X from x=-5, y=0, z=0)
    r4.orig_x[0] = -5.0f; r4.orig_y[0] = 0.0f; r4.orig_z[0] = 0.0f;
    r4.inv_dir_x[0] = 1.0f; r4.inv_dir_y[0] = 1e5f; r4.inv_dir_z[0] = 1e5f;
    // Ray 1: misses (y=10.0 is outside box)
    r4.orig_x[1] = -5.0f; r4.orig_y[1] = 10.0f; r4.orig_z[1] = 0.0f;
    r4.inv_dir_x[1] = 1.0f; r4.inv_dir_y[1] = 1e5f; r4.inv_dir_z[1] = 1e5f;
    // Ray 2: hits (moves along +Y from y=-5, x=0, z=0)
    r4.orig_x[2] = 0.0f; r4.orig_y[2] = -5.0f; r4.orig_z[2] = 0.0f;
    r4.inv_dir_x[2] = 1e5f; r4.inv_dir_y[2] = 1.0f; r4.inv_dir_z[2] = 1e5f;
    // Ray 3: misses (starts at x=5.0 pointing away along +X)
    r4.orig_x[3] = 5.0f; r4.orig_y[3] = 0.0f; r4.orig_z[3] = 0.0f;
    r4.inv_dir_x[3] = 1.0f; r4.inv_dir_y[3] = 1e5f; r4.inv_dir_z[3] = 1e5f;

    uint32_t mask_scalar = kernel_ray_box_4_scalar_reference(r4, box);
    uint32_t mask_opt    = kernel_ray_box_4_compiler_opt(r4, box);
    uint32_t mask_sse    = kernel_ray_box_4_sse(r4, box);

    (void)mask_scalar;
    (void)mask_opt;
    (void)mask_sse;

    assert(mask_scalar == mask_opt);
    assert(mask_scalar == mask_sse);
    assert(mask_scalar == 0x05); // Rays 0 and 2 hit (bits 0 and 2 set = 1 | 4 = 5)

    // 8-Ray Packet
    RayPacket8 r8;
    for (int i = 0; i < 4; ++i) {
        r8.orig_x[i] = r4.orig_x[i]; r8.orig_y[i] = r4.orig_y[i]; r8.orig_z[i] = r4.orig_z[i];
        r8.inv_dir_x[i] = r4.inv_dir_x[i]; r8.inv_dir_y[i] = r4.inv_dir_y[i]; r8.inv_dir_z[i] = r4.inv_dir_z[i];

        r8.orig_x[i+4] = r4.orig_x[i]; r8.orig_y[i+4] = r4.orig_y[i]; r8.orig_z[i+4] = r4.orig_z[i];
        r8.inv_dir_x[i+4] = r4.inv_dir_x[i]; r8.inv_dir_y[i+4] = r4.inv_dir_y[i]; r8.inv_dir_z[i+4] = r4.inv_dir_z[i];
    }

    uint32_t mask_avx2 = kernel_ray_box_8_avx2(r8, box);
    (void)mask_avx2;
    assert(mask_avx2 == 0x55); // 0x05 | (0x05 << 4) = 0x55
}

int main() {
    std::cout << "=== Running test_simd ===\n";

    test_simd_capabilities();
    test_kernel_dot_lengths_and_tails();
    test_kernel_axpy_lengths_and_tails();
    test_kernel_ray_box_hits_and_misses();

    std::cout << "=== All 4 SIMD unit tests PASSED cleanly ===\n";
    return 0;
}
