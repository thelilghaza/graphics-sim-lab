#pragma once

#include <cmath>
#include <cstdint>
#include <random>

namespace raylab {

class RNG {
public:
    explicit RNG(uint64_t seed = 42) : engine(seed), dist(0.0, 1.0) {}

    void set_seed(uint64_t seed) {
        engine.seed(seed);
    }

    // Returns a random double in range [0.0, 1.0)
    double next_double() {
        return dist(engine);
    }

    // Returns a random double in range [min, max)
    double next_double(double min, double max) {
        return min + (max - min) * next_double();
    }

private:
    std::mt19937_64 engine;
    std::uniform_real_distribution<double> dist;
};

// SplitMix64 deterministic 64-bit mixer for order-independent per-pixel/per-sample seeds
inline uint64_t splitmix64(uint64_t state) {
    uint64_t z = (state + 0x9e3779b97f4a7c15ULL);
    z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
    z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
    return z ^ (z >> 31);
}

inline uint64_t make_sample_seed(uint64_t global_seed, int x, int y, int sample) {
    uint64_t h = global_seed;
    h = splitmix64(h + static_cast<uint64_t>(x) * 0x27bb2ee687b0b0fdULL);
    h = splitmix64(h + static_cast<uint64_t>(y) * 0x8b51f9dd1000000dULL);
    h = splitmix64(h + static_cast<uint64_t>(sample) * 0x4898031d27bb2ee7ULL);
    return h;
}

} // namespace raylab
