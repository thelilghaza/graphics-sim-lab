#ifndef DESTRUCTION_AABB_HPP
#define DESTRUCTION_AABB_HPP

#include "destruction/math/vec3.hpp"
#include "destruction/math/transform.hpp"
#include "destruction/math/math_utils.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

namespace destruction::collision {

using namespace destruction::math;

constexpr float AABB_EPSILON = 1e-5f;

struct Aabb {
    Vec3 min_pt{Vec3(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max())};
    Vec3 max_pt{Vec3(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max())};

    constexpr Aabb() = default;
    constexpr Aabb(const Vec3& min_corner, const Vec3& max_corner)
        : min_pt(min_corner), max_pt(max_corner) {}

    static Aabb create_empty() {
        return Aabb();
    }

    static Aabb create_infinite() {
        float inf = 1e30f;
        return Aabb(Vec3(-inf, -inf, -inf), Vec3(inf, inf, inf));
    }

    bool is_empty() const {
        return min_pt.x > max_pt.x || min_pt.y > max_pt.y || min_pt.z > max_pt.z;
    }

    void expand(const Vec3& p) {
        min_pt.x = std::min(min_pt.x, p.x);
        min_pt.y = std::min(min_pt.y, p.y);
        min_pt.z = std::min(min_pt.z, p.z);

        max_pt.x = std::max(max_pt.x, p.x);
        max_pt.y = std::max(max_pt.y, p.y);
        max_pt.z = std::max(max_pt.z, p.z);
    }

    void expand(const Aabb& b) {
        if (b.is_empty()) return;
        expand(b.min_pt);
        expand(b.max_pt);
    }

    void fatten(float margin) {
        min_pt -= Vec3(margin, margin, margin);
        max_pt += Vec3(margin, margin, margin);
    }

    Vec3 center() const {
        return 0.5f * (min_pt + max_pt);
    }

    Vec3 extents() const {
        if (is_empty()) return Vec3::zero();
        return 0.5f * (max_pt - min_pt);
    }

    float surface_area() const {
        if (is_empty()) return 0.0f;
        Vec3 d = max_pt - min_pt;
        return 2.0f * (d.x * d.y + d.y * d.z + d.z * d.x);
    }

    bool contains(const Vec3& p, float eps = AABB_EPSILON) const {
        return (p.x >= min_pt.x - eps && p.x <= max_pt.x + eps &&
                p.y >= min_pt.y - eps && p.y <= max_pt.y + eps &&
                p.z >= min_pt.z - eps && p.z <= max_pt.z + eps);
    }

    bool overlaps(const Aabb& b, float eps = AABB_EPSILON) const {
        if (is_empty() || b.is_empty()) return false;
        if (max_pt.x + eps < b.min_pt.x || min_pt.x - eps > b.max_pt.x) return false;
        if (max_pt.y + eps < b.min_pt.y || min_pt.y - eps > b.max_pt.y) return false;
        if (max_pt.z + eps < b.min_pt.z || min_pt.z - eps > b.max_pt.z) return false;
        return true;
    }

    static Aabb merge(const Aabb& a, const Aabb& b) {
        Aabb res = a;
        res.expand(b);
        return res;
    }

    Aabb transform(const Transform& t) const {
        if (is_empty()) return Aabb::create_empty();

        Vec3 c = center();
        Vec3 e = extents();

        Vec3 world_c = t.transform_point(c);
        Mat3 r = t.orientation.to_mat3();

        Vec3 world_e(
            std::abs(r(0, 0)) * e.x + std::abs(r(0, 1)) * e.y + std::abs(r(0, 2)) * e.z,
            std::abs(r(1, 0)) * e.x + std::abs(r(1, 1)) * e.y + std::abs(r(1, 2)) * e.z,
            std::abs(r(2, 0)) * e.x + std::abs(r(2, 1)) * e.y + std::abs(r(2, 2)) * e.z
        );

        return Aabb(world_c - world_e, world_c + world_e);
    }

    bool is_valid() const {
        return min_pt.is_valid() && max_pt.is_valid();
    }
};

} // namespace destruction::collision

#endif // DESTRUCTION_AABB_HPP
