#ifndef DESTRUCTION_GJK_HPP
#define DESTRUCTION_GJK_HPP

#include "destruction/math/vec3.hpp"
#include "destruction/collision/collider.hpp"
#include "destruction/collision/support.hpp"
#include <vector>
#include <cmath>

namespace destruction::collision {

using namespace destruction::math;

constexpr int GJK_MAX_ITERATIONS = 64;
constexpr float GJK_EPSILON = 1e-6f;

struct Simplex {
    std::vector<MinkowskiSupportPoint> points;

    Simplex() {
        points.reserve(4);
    }

    void add(const MinkowskiSupportPoint& pt) {
        points.push_back(pt);
    }

    size_t size() const {
        return points.size();
    }

    void clear() {
        points.clear();
    }

    const MinkowskiSupportPoint& operator[](size_t idx) const {
        return points[idx];
    }
};

struct GjkResult {
    bool intersecting{false};
    Simplex simplex;
    int iterations{0};
};

class Gjk {
private:
    static bool update_simplex(Simplex& simplex, Vec3& d) {
        size_t n = simplex.size();
        if (n == 2) {
            // Line Segment AB (A is newest = points[1], B = points[0])
            Vec3 a = simplex[1].v;
            Vec3 b = simplex[0].v;
            Vec3 ab = b - a;
            Vec3 ao = -a;

            if (ab.dot(ao) > 0.0f) {
                d = ab.cross(ao).cross(ab);
                if (d.length_sq() <= GJK_EPSILON) {
                    d = Vec3(-ab.y, ab.x, 0.0f);
                    if (d.length_sq() <= GJK_EPSILON) {
                        d = Vec3(0.0f, -ab.z, ab.y);
                    }
                }
            } else {
                simplex.points = { simplex[1] };
                d = ao;
            }
            return false;
        }

        if (n == 3) {
            // Triangle ABC (A is newest = points[2], B = points[1], C = points[0])
            Vec3 a = simplex[2].v;
            Vec3 b = simplex[1].v;
            Vec3 c = simplex[0].v;

            Vec3 ab = b - a;
            Vec3 ac = c - a;
            Vec3 ao = -a;
            Vec3 abc_norm = ab.cross(ac);

            Vec3 ab_perp = ab.cross(abc_norm);
            Vec3 ac_perp = abc_norm.cross(ac);

            if (ab_perp.dot(ao) > 0.0f) {
                if (ab.dot(ao) > 0.0f) {
                    simplex.points = { simplex[1], simplex[2] };
                    d = ab.cross(ao).cross(ab);
                } else {
                    simplex.points = { simplex[2] };
                    d = ao;
                }
            } else if (ac_perp.dot(ao) > 0.0f) {
                if (ac.dot(ao) > 0.0f) {
                    simplex.points = { simplex[0], simplex[2] };
                    d = ac.cross(ao).cross(ac);
                } else {
                    simplex.points = { simplex[2] };
                    d = ao;
                }
            } else {
                if (abc_norm.dot(ao) > 0.0f) {
                    d = abc_norm;
                } else {
                    simplex.points = { simplex[0], simplex[1], simplex[2] };
                    d = -abc_norm;
                }
            }
            return false;
        }

        if (n == 4) {
            // Tetrahedron ABCD (A is newest = points[3], B = points[2], C = points[1], D = points[0])
            Vec3 a = simplex[3].v;
            Vec3 b = simplex[2].v;
            Vec3 c = simplex[1].v;
            Vec3 d_pt = simplex[0].v;

            Vec3 ab = b - a;
            Vec3 ac = c - a;
            Vec3 ad = d_pt - a;
            Vec3 ao = -a;

            Vec3 abc_norm = ab.cross(ac);
            Vec3 acd_norm = ac.cross(ad);
            Vec3 adb_norm = ad.cross(ab);

            if (abc_norm.dot(ao) > 0.0f) {
                simplex.points = { simplex[1], simplex[2], simplex[3] };
                d = abc_norm;
                return false;
            }

            if (acd_norm.dot(ao) > 0.0f) {
                simplex.points = { simplex[0], simplex[1], simplex[3] };
                d = acd_norm;
                return false;
            }

            if (adb_norm.dot(ao) > 0.0f) {
                simplex.points = { simplex[2], simplex[0], simplex[3] };
                d = adb_norm;
                return false;
            }

            // Origin is enclosed inside tetrahedron!
            return true;
        }

        return false;
    }

public:
    static GjkResult intersect(const Collider& a, const Collider& b) {
        GjkResult result;

        // Search direction pointing from A to B
        Vec3 d = b.world_transform.position - a.world_transform.position;
        if (d.length_sq() <= GJK_EPSILON) {
            d = Vec3::unit_x();
        }

        MinkowskiSupportPoint p = support_minkowski(a, b, d);
        result.simplex.add(p);
        d = -p.v;

        for (int iter = 0; iter < GJK_MAX_ITERATIONS; ++iter) {
            result.iterations++;

            if (d.length_sq() <= GJK_EPSILON) {
                d = Vec3::unit_x();
            }

            p = support_minkowski(a, b, d);

            // If new support point doesn't cross origin in search direction, no intersection
            if (p.v.dot(d) <= 0.0f) {
                result.intersecting = false;
                return result;
            }

            result.simplex.add(p);

            if (update_simplex(result.simplex, d)) {
                result.intersecting = true;
                return result;
            }
        }

        result.intersecting = false;
        return result;
    }
};

} // namespace destruction::collision

#endif // DESTRUCTION_GJK_HPP
