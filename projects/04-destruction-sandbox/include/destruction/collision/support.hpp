#ifndef DESTRUCTION_SUPPORT_HPP
#define DESTRUCTION_SUPPORT_HPP

#include "destruction/math/vec3.hpp"
#include "destruction/collision/collider.hpp"

namespace destruction::collision {

using namespace destruction::math;

struct MinkowskiSupportPoint {
    Vec3 v{Vec3::zero()};   // v = v_a - v_b
    Vec3 v_a{Vec3::zero()}; // Support point on Collider A (world space)
    Vec3 v_b{Vec3::zero()}; // Support point on Collider B (world space)

    constexpr MinkowskiSupportPoint() = default;
    constexpr MinkowskiSupportPoint(const Vec3& minkowski_v, const Vec3& pt_a, const Vec3& pt_b)
        : v(minkowski_v), v_a(pt_a), v_b(pt_b) {}

    bool operator==(const MinkowskiSupportPoint& rhs) const {
        return v == rhs.v && v_a == rhs.v_a && v_b == rhs.v_b;
    }

    bool operator!=(const MinkowskiSupportPoint& rhs) const {
        return !(*this == rhs);
    }
};

inline MinkowskiSupportPoint support_minkowski(
    const Collider& a,
    const Collider& b,
    const Vec3& direction
) {
    Vec3 dir_norm = direction.normalize();
    if (dir_norm.length_sq() <= 1e-8f) {
        dir_norm = Vec3::unit_x();
    }
    Vec3 pt_a = a.get_support_world(dir_norm);
    Vec3 pt_b = b.get_support_world(-dir_norm);
    return MinkowskiSupportPoint(pt_a - pt_b, pt_a, pt_b);
}

} // namespace destruction::collision

#endif // DESTRUCTION_SUPPORT_HPP
