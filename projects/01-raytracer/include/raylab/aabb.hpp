#pragma once

#include "raylab/ray.hpp"
#include "raylab/vec3.hpp"
#include <algorithm>
#include <cmath>

namespace raylab {

class AABB {
public:
    Point3 minimum;
    Point3 maximum;

    AABB() = default;

    AABB(const Point3& min_pt, const Point3& max_pt)
        : minimum(min_pt), maximum(max_pt) {
        pad_to_minimums();
    }

    // Constructs AABB enclosing two existing AABBs
    AABB(const AABB& box0, const AABB& box1) {
        minimum = Point3(std::min(box0.minimum.x(), box1.minimum.x()),
                         std::min(box0.minimum.y(), box1.minimum.y()),
                         std::min(box0.minimum.z(), box1.minimum.z()));
        maximum = Point3(std::max(box0.maximum.x(), box1.maximum.x()),
                         std::max(box0.maximum.y(), box1.maximum.y()),
                         std::max(box0.maximum.z(), box1.maximum.z()));
        pad_to_minimums();
    }

    const Point3& min() const { return minimum; }
    const Point3& max() const { return maximum; }

    // Robust Andrew Kensler slab ray-AABB intersection test
    bool hit(const Ray& r, double ray_tmin, double ray_tmax) const {
        for (int a = 0; a < 3; ++a) {
            double invD = 1.0 / r.direction()[a];
            double t0 = (minimum[a] - r.origin()[a]) * invD;
            double t1 = (maximum[a] - r.origin()[a]) * invD;
            if (invD < 0.0) std::swap(t0, t1);
            ray_tmin = t0 > ray_tmin ? t0 : ray_tmin;
            ray_tmax = t1 < ray_tmax ? t1 : ray_tmax;
            if (ray_tmax <= ray_tmin) return false;
        }
        return true;
    }

    // Longest axis index: 0=x, 1=y, 2=z
    int longest_axis() const {
        Vec3 extent = maximum - minimum;
        if (extent.x() > extent.y() && extent.x() > extent.z()) return 0;
        if (extent.y() > extent.z()) return 1;
        return 2;
    }

private:
    void pad_to_minimums() {
        constexpr double delta = 0.0001;
        for (int i = 0; i < 3; ++i) {
            if (maximum[i] - minimum[i] < delta) {
                minimum[i] -= delta * 0.5;
                maximum[i] += delta * 0.5;
            }
        }
    }
};

} // namespace raylab
