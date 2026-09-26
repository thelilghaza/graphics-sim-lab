#ifndef PERFORMANCE_LAB_STATISTICS_HPP
#define PERFORMANCE_LAB_STATISTICS_HPP

#include "performance_lab/bench_types.hpp"
#include <vector>

namespace performance_lab {

/**
 * @brief Computes statistical summary (mean, median, sample stddev, min, max, throughput)
 * from timing samples in microseconds.
 * 
 * Standard Deviation Definition:
 * Uses Sample Standard Deviation (N - 1 denominator):
 * s = sqrt( sum((x_i - mean)^2) / (N - 1) ) for N > 1.
 * Returns 0.0 when N <= 1.
 * 
 * Bandwidth Definition:
 * Uses Decimal Megabytes: 1 MB = 10^6 bytes.
 */
BenchResult compute_statistics(const BenchConfig& config, const std::vector<double>& samples_us);

} // namespace performance_lab

#endif // PERFORMANCE_LAB_STATISTICS_HPP
