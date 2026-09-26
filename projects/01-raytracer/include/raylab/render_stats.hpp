#pragma once

#include <cstdint>

namespace raylab {

struct RenderStats {
    uint64_t primary_samples{0};
    uint64_t shadow_rays{0};
    uint64_t secondary_rays{0};
    uint64_t sphere_intersection_tests{0};
    uint64_t aabb_tests{0};

    uint64_t total_rays() const {
        return primary_samples + shadow_rays + secondary_rays;
    }

    void reset() {
        primary_samples = 0;
        shadow_rays = 0;
        secondary_rays = 0;
        sphere_intersection_tests = 0;
        aabb_tests = 0;
    }

    void merge(const RenderStats& other) {
        primary_samples += other.primary_samples;
        shadow_rays += other.shadow_rays;
        secondary_rays += other.secondary_rays;
        sphere_intersection_tests += other.sphere_intersection_tests;
        aabb_tests += other.aabb_tests;
    }
};

} // namespace raylab
