#ifndef PERFORMANCE_LAB_WORKINGSET_BENCHMARK_HPP
#define PERFORMANCE_LAB_WORKINGSET_BENCHMARK_HPP

#include <cstddef>
#include <cstdint>
#include <vector>

namespace performance_lab {

/**
 * @brief Generates a deterministic buffer of specified byte size (floats).
 */
inline std::vector<float> generate_workingset_buffer(size_t working_set_bytes) {
    size_t num_floats = working_set_bytes / sizeof(float);
    if (num_floats == 0) num_floats = 1;
    
    std::vector<float> buffer(num_floats);
    for (size_t i = 0; i < num_floats; ++i) {
        buffer[i] = static_cast<float>((i % 1000) + 1) * 0.001f;
    }
    return buffer;
}

/**
 * @brief Executes one complete sequential pass over the working-set buffer.
 */
inline float run_workingset_pass(const std::vector<float>& buffer) {
    float sum = 0.0f;
    size_t n = buffer.size();
    for (size_t i = 0; i < n; ++i) {
        sum += buffer[i];
    }
    return sum;
}

} // namespace performance_lab

#endif // PERFORMANCE_LAB_WORKINGSET_BENCHMARK_HPP
