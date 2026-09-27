#ifndef DESTRUCTION_EPA_HPP
#define DESTRUCTION_EPA_HPP

#include "destruction/math/vec3.hpp"
#include "destruction/collision/collider.hpp"
#include "destruction/collision/support.hpp"
#include "destruction/collision/gjk.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

namespace destruction::collision {

using namespace destruction::math;

constexpr int EPA_MAX_ITERATIONS = 64;
constexpr float EPA_EPSILON = 1e-4f;

struct EpaFace {
    MinkowskiSupportPoint a;
    MinkowskiSupportPoint b;
    MinkowskiSupportPoint c;
    Vec3 normal{Vec3::unit_y()};
    float distance{0.0f};

    EpaFace(const MinkowskiSupportPoint& p_a, const MinkowskiSupportPoint& p_b, const MinkowskiSupportPoint& p_c)
        : a(p_a), b(p_b), c(p_c) {
        Vec3 n = (b.v - a.v).cross(c.v - a.v);
        float len = n.length();
        if (len > 1e-7f) {
            normal = n / len;
        } else {
            normal = Vec3::unit_y();
        }
        distance = normal.dot(a.v);
        if (distance < 0.0f) {
            normal = -normal;
            distance = -distance;
            std::swap(b, c);
        }
    }
};

struct EpaEdge {
    MinkowskiSupportPoint a;
    MinkowskiSupportPoint b;

    bool operator==(const EpaEdge& rhs) const {
        return (a == rhs.a && b == rhs.b) || (a == rhs.b && b == rhs.a);
    }
};

struct EpaResult {
    bool success{false};
    Vec3 normal{Vec3::unit_y()}; // Pointing from A to B
    float penetration_depth{0.0f};
    Vec3 point_a{Vec3::zero()};
    Vec3 point_b{Vec3::zero()};
    int iterations{0};
};

class Epa {
public:
    static EpaResult expand(const Collider& col_a, const Collider& col_b, const Simplex& gjk_simplex) {
        EpaResult result;

        if (gjk_simplex.size() < 4) {
            result.success = false;
            return result;
        }

        std::vector<EpaFace> faces;
        faces.reserve(32);

        // Initial tetrahedron faces from GJK simplex
        faces.emplace_back(gjk_simplex[3], gjk_simplex[2], gjk_simplex[1]);
        faces.emplace_back(gjk_simplex[3], gjk_simplex[1], gjk_simplex[0]);
        faces.emplace_back(gjk_simplex[3], gjk_simplex[0], gjk_simplex[2]);
        faces.emplace_back(gjk_simplex[0], gjk_simplex[1], gjk_simplex[2]);

        for (int iter = 0; iter < EPA_MAX_ITERATIONS; ++iter) {
            result.iterations++;

            // Find closest face to origin
            size_t min_face_idx = 0;
            float min_dist = faces[0].distance;

            for (size_t i = 1; i < faces.size(); ++i) {
                if (faces[i].distance < min_dist) {
                    min_dist = faces[i].distance;
                    min_face_idx = i;
                }
            }

            EpaFace closest_face = faces[min_face_idx];
            MinkowskiSupportPoint p = support_minkowski(col_a, col_b, closest_face.normal);
            float d_new = p.v.dot(closest_face.normal);

            if (d_new - min_dist <= EPA_EPSILON) {
                // Converged!
                result.success = true;
                result.normal = closest_face.normal;
                result.penetration_depth = min_dist;

                // Barycentric coordinates of origin projection on closest face
                Vec3 p0 = closest_face.a.v;
                Vec3 p1 = closest_face.b.v;
                Vec3 p2 = closest_face.c.v;
                Vec3 proj = min_dist * closest_face.normal;

                Vec3 v0 = p1 - p0, v1 = p2 - p0, v2 = proj - p0;
                float d00 = v0.dot(v0);
                float d01 = v0.dot(v1);
                float d11 = v1.dot(v1);
                float d20 = v2.dot(v0);
                float d21 = v2.dot(v1);
                float denom = d00 * d11 - d01 * d01;

                float u = 1.0f / 3.0f, v = 1.0f / 3.0f, w = 1.0f / 3.0f;
                if (std::abs(denom) > 1e-7f) {
                    v = (d11 * d20 - d01 * d21) / denom;
                    w = (d00 * d21 - d01 * d20) / denom;
                    u = 1.0f - v - w;
                }

                result.point_a = u * closest_face.a.v_a + v * closest_face.b.v_a + w * closest_face.c.v_a;
                result.point_b = u * closest_face.a.v_b + v * closest_face.b.v_b + w * closest_face.c.v_b;
                return result;
            }

            // Remove faces visible from p.v and construct horizon edges
            std::vector<EpaEdge> horizon;
            std::vector<EpaFace> next_faces;

            for (const auto& face : faces) {
                if (face.normal.dot(p.v - face.a.v) > 0.0f) {
                    // Face is visible from p.v
                    EpaEdge e1{face.a, face.b};
                    EpaEdge e2{face.b, face.c};
                    EpaEdge e3{face.c, face.a};

                    auto add_edge = [&](const EpaEdge& edge) {
                        auto it = std::find(horizon.begin(), horizon.end(), edge);
                        if (it != horizon.end()) {
                            horizon.erase(it);
                        } else {
                            horizon.push_back(edge);
                        }
                    };

                    add_edge(e1);
                    add_edge(e2);
                    add_edge(e3);
                } else {
                    next_faces.push_back(face);
                }
            }

            for (const auto& edge : horizon) {
                next_faces.emplace_back(p, edge.a, edge.b);
            }

            if (next_faces.empty()) {
                break;
            }

            faces = std::move(next_faces);
        }

        result.success = false;
        return result;
    }
};

} // namespace destruction::collision

#endif // DESTRUCTION_EPA_HPP
