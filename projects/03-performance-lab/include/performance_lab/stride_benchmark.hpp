#ifndef PERFORMANCE_LAB_STRIDE_BENCHMARK_HPP
#define PERFORMANCE_LAB_STRIDE_BENCHMARK_HPP

#include <cstddef>
#include <cstdint>
#include <vector>
#include <numeric>

namespace performance_lab {

/**
 * @brief Generates a deterministic floating-point buffer for stride access benchmarks.
 * 
 * Array size is (access_count * stride) elements.
 */
inline std::vector<float> generate_stride_buffer(size_t access_count, size_t stride) {
    size_t total_elements = access_count * stride;
    std::vector<float> buffer(total_elements);
    for (size_t i = 0; i < total_elements; ++i) {
        buffer[i] = static_cast<float>(i + 1) * 0.001f;
    }
    return buffer;
}

/**
 * @brief Executes a strided traversal accessing exactly access_count elements at indices (i * stride).
 */
inline float run_stride_access(const std::vector<float>& buffer, size_t access_count, size_t stride) {
    float sum = 0.0f;
    for (size_t i = 0; i < access_count; ++i) {
        sum += buffer[i * stride];
    }
    return sum;
}

} // namespace performance_lab

#endif // PERFORMANCE_LAB_STRIDE_BENCHMARK_HPP
