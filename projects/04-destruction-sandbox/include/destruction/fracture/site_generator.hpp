#ifndef DESTRUCTION_SITE_GENERATOR_HPP
#define DESTRUCTION_SITE_GENERATOR_HPP

#include "destruction/math/vec2.hpp"
#include "destruction/math/vec3.hpp"
#include "destruction/math/math_utils.hpp"
#include "destruction/fracture/fracture_volume.hpp"
#include <vector>
#include <cstdint>
#include <cmath>

namespace destruction::fracture {

using namespace destruction::math;

// Deterministic PRNG (Xorshift32)
class DeterministicRandom {
private:
    uint32_t state_{42};

public:
    explicit DeterministicRandom(uint32_t seed = 42) : state_(seed == 0 ? 1 : seed) {}

    uint32_t next_u32() {
        uint32_t x = state_;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        state_ = x;
        return x;
    }

    // Floating-point in [0, 1)
    float next_float() {
        return static_cast<float>(next_u32()) / 4294967296.0f;
    }

    // Floating-point in [min_val, max_val]
    float range(float min_val, float max_val) {
        return min_val + next_float() * (max_val - min_val);
    }
};

class SiteGenerator {
public:
    static std::vector<Vec2> generate_2d(
        float min_x, float min_y, float max_x, float max_y,
        size_t target_count,
        float min_dist,
        uint32_t seed = 42
    ) {
        DeterministicRandom rng(seed);
        std::vector<Vec2> sites;
        sites.reserve(target_count);

        float min_dist_sq = min_dist * min_dist;
        size_t max_attempts = target_count * 100;
        size_t attempts = 0;

        while (sites.size() < target_count && attempts < max_attempts) {
            attempts++;
            Vec2 candidate(
                rng.range(min_x, max_x),
                rng.range(min_y, max_y)
            );

            bool valid = true;
            for (const auto& existing : sites) {
                if ((candidate - existing).length_sq() < min_dist_sq) {
                    valid = false;
                    break;
                }
            }

            if (valid) {
                sites.push_back(candidate);
            }
        }

        return sites;
    }

    static std::vector<Vec3> generate_3d(
        const FractureVolume& volume,
        size_t target_count,
        float min_dist,
        uint32_t seed = 42
    ) {
        DeterministicRandom rng(seed);
        std::vector<Vec3> sites;
        sites.reserve(target_count);

        float min_dist_sq = min_dist * min_dist;
        size_t max_attempts = target_count * 100;
        size_t attempts = 0;

        while (sites.size() < target_count && attempts < max_attempts) {
            attempts++;
            Vec3 candidate(
                rng.range(volume.min_pt.x, volume.max_pt.x),
                rng.range(volume.min_pt.y, volume.max_pt.y),
                rng.range(volume.min_pt.z, volume.max_pt.z)
            );

            bool valid = true;
            for (const auto& existing : sites) {
                if ((candidate - existing).length_sq() < min_dist_sq) {
                    valid = false;
                    break;
                }
            }

            if (valid) {
                sites.push_back(candidate);
            }
        }

        return sites;
    }
};

} // namespace destruction::fracture

#endif // DESTRUCTION_SITE_GENERATOR_HPP
