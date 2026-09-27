#ifndef DESTRUCTION_PLANE_HPP
#define DESTRUCTION_PLANE_HPP

#include "destruction/math/vec3.hpp"
#include "destruction/math/math_utils.hpp"
#include <cmath>
#include <algorithm>

namespace destruction::fracture {

using namespace destruction::math;

constexpr float GEOM_EPSILON = 1e-5f;

enum class PointPlaneSide {
    Inside,  // dist <= GEOM_EPSILON
    Outside, // dist > GEOM_EPSILON
    OnPlane  // |dist| <= GEOM_EPSILON
};

struct Plane {
    Vec3 normal{Vec3::unit_y()};
    float d{0.0f}; // Plane equation: normal.dot(p) + d = 0

    constexpr Plane() = default;
    Plane(const Vec3& norm, float dist_val)
        : normal(norm.normalize()), d(dist_val) {}

    static Plane from_point_normal(const Vec3& point, const Vec3& norm) {
        Vec3 n = norm.normalize();
        float dist_val = -n.dot(point);
        return Plane(n, dist_val);
    }

    // Perpendicular bisector plane between site_a and site_b.
    // Inside half-space (normal.dot(p) + d <= 0) contains site_a.
    // Outward-pointing normal points towards site_b.
    static Plane bisector(const Vec3& site_a, const Vec3& site_b) {
        Vec3 mid = 0.5f * (site_a + site_b);
        Vec3 n = (site_b - site_a).normalize();
        float dist_val = -n.dot(mid);
        return Plane(n, dist_val);
    }

    float distance(const Vec3& point) const {
        return normal.dot(point) + d;
    }

    PointPlaneSide classify(const Vec3& point, float eps = GEOM_EPSILON) const {
        float dist = distance(point);
        if (std::abs(dist) <= eps) {
            return PointPlaneSide::OnPlane;
        }
        if (dist < 0.0f) {
            return PointPlaneSide::Inside;
        }
        return PointPlaneSide::Outside;
    }

    bool intersect_segment(const Vec3& a, const Vec3& b, Vec3& out_intersection, float eps = GEOM_EPSILON) const {
        float da = distance(a);
        float db = distance(b);

        float diff = db - da;
        if (std::abs(diff) <= eps) {
            return false;
        }

        float t = -da / diff;
        if (t < -eps || t > 1.0f + eps) {
            return false;
        }

        t = std::clamp(t, 0.0f, 1.0f);
        out_intersection = a + t * (b - a);
        return true;
    }

    Plane flipped() const {
        return Plane(-normal, -d);
    }
};

} // namespace destruction::fracture

#endif // DESTRUCTION_PLANE_HPP
