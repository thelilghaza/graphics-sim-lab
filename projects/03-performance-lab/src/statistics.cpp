#include "performance_lab/statistics.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>

namespace performance_lab {

BenchResult compute_statistics(const BenchConfig& config, const std::vector<double>& samples_us) {
    BenchResult result;
    result.config = config;
    result.samples_us = samples_us;

    if (samples_us.empty()) {
        return result;
    }

    std::vector<double> sorted = samples_us;
    std::sort(sorted.begin(), sorted.end());

    result.min_us = sorted.front();
    result.max_us = sorted.back();

    double sum = std::accumulate(sorted.begin(), sorted.end(), 0.0);
    result.mean_us = sum / static_cast<double>(sorted.size());

    size_t n = sorted.size();
    if (n % 2 == 1) {
        result.median_us = sorted[n / 2];
    } else {
        result.median_us = 0.5 * (sorted[n / 2 - 1] + sorted[n / 2]);
    }

    if (n > 1) {
        double variance_sum = 0.0;
        for (double val : sorted) {
            double diff = val - result.mean_us;
            variance_sum += diff * diff;
        }
        // Sample standard deviation (N - 1)
        result.stddev_us = std::sqrt(variance_sum / static_cast<double>(n - 1));
    } else {
        result.stddev_us = 0.0;
    }

    if (result.mean_us > 0.0) {
        double mean_sec = result.mean_us * 1e-6;
        if (config.total_operations > 0) {
            result.ops_per_sec = static_cast<double>(config.total_operations) / mean_sec;
        }
        if (config.total_bytes > 0) {
            // 1 MB = 10^6 bytes
            double mb = static_cast<double>(config.total_bytes) / 1e6;
            result.mb_per_sec = mb / mean_sec;
        }
    }

    // Diagnostic warning for suspiciously short benchmarks (< 0.02 us / 20 ns mean per iteration)
    if (result.mean_us < 0.02) {
        result.warning_too_short = true;
    }

    return result;
}

} // namespace performance_lab
