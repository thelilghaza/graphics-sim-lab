#ifndef PERFORMANCE_LAB_SIMD_CAPS_HPP
#define PERFORMANCE_LAB_SIMD_CAPS_HPP

#include <cstdint>
#include <string>

// ----------------------------------------------------------------------------
// Compiler & ISA Target Architecture Detection
// ----------------------------------------------------------------------------

#if defined(_M_X64) || defined(_M_AMD64) || defined(__x86_64__) || defined(_M_IX86) || defined(__i386__)
    #define PERFORMANCE_LAB_ARCH_X86 1
    #include <immintrin.h>
#elif defined(__aarch64__) || defined(_M_ARM64) || defined(__ARM_NEON) || defined(__ARM_NEON__)
    #define PERFORMANCE_LAB_ARCH_ARM 1
    #include <arm_neon.h>
#endif

#if defined(__AVX2__) || defined(PERFORMANCE_LAB_ARCH_X86)
    #define PERFORMANCE_LAB_HAS_AVX2_INTRINSICS 1
#endif

#if defined(__SSE2__) || defined(PERFORMANCE_LAB_ARCH_X86)
    #define PERFORMANCE_LAB_HAS_SSE_INTRINSICS 1
#endif

#if defined(PERFORMANCE_LAB_ARCH_ARM)
    #define PERFORMANCE_LAB_HAS_NEON_INTRINSICS 1
#endif

namespace performance_lab {

/**
 * @brief Runtime & compile-time CPU feature detection layer.
 */
class SimdCapabilities {
public:
    static bool has_sse2() {
#if defined(PERFORMANCE_LAB_HAS_SSE_INTRINSICS)
        return true;
#else
        return false;
#endif
    }

    static bool has_avx2() {
#if defined(PERFORMANCE_LAB_ARCH_X86)
        static bool avx2_supported = check_avx2_runtime();
        return avx2_supported;
#else
        return false;
#endif
    }

    static bool has_neon() {
#if defined(PERFORMANCE_LAB_HAS_NEON_INTRINSICS)
        return true;
#else
        return false;
#endif
    }

    static std::string active_isa_string() {
        std::string isa = "Scalar";
        if (has_sse2()) {
            isa += ", SSE2";
        }
        if (has_avx2()) {
            isa += ", AVX2";
        }
        if (has_neon()) {
            isa += ", NEON";
        }
        return isa;
    }

private:
    static bool check_avx2_runtime() {
#if defined(_MSC_VER) && defined(PERFORMANCE_LAB_ARCH_X86)
        int cpu_info[4] = {0};
        __cpuid(cpu_info, 0);
        int num_ids = cpu_info[0];
        if (num_ids >= 7) {
            __cpuidex(cpu_info, 7, 0);
            return (cpu_info[1] & (1 << 5)) != 0; // EBX bit 5 = AVX2
        }
        return false;
#elif (defined(__GNUC__) || defined(__clang__)) && defined(PERFORMANCE_LAB_ARCH_X86)
        return __builtin_cpu_supports("avx2");
#else
        return false;
#endif
    }
};

} // namespace performance_lab

#endif // PERFORMANCE_LAB_SIMD_CAPS_HPP
