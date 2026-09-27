#ifndef DESTRUCTION_SAT_HPP
#define DESTRUCTION_SAT_HPP

#include "destruction/math/vec3.hpp"
#include "destruction/collision/collider.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

namespace destruction::collision {

using namespace destruction::math;

constexpr float SAT_AXIS_EPSILON = 1e-5f;

struct SatResult {
    bool colliding{false};
    float penetration_depth{0.0f};
    Vec3 normal{Vec3::unit_y()}; // Pointing from A to B
};

class Sat {
private:
    static void project_collider(const Collider& col, const Vec3& axis, float& min_proj, float& max_proj) {
        Vec3 norm_axis = axis.normalize();
        if (col.type == ColliderType::Box) {
            Vec3 center = col.world_transform.position;
            Mat3 r = col.world_transform.orientation.to_mat3();
            Vec3 e = col.box_half_extents;

            float radius = std::abs(norm_axis.dot(r * Vec3(e.x, 0, 0))) +
                           std::abs(norm_axis.dot(r * Vec3(0, e.y, 0))) +
                           std::abs(norm_axis.dot(r * Vec3(0, 0, e.z)));
            float c_proj = center.dot(norm_axis);
            min_proj = c_proj - radius;
            max_proj = c_proj + radius;
        } else {
            min_proj = 1e30f;
            max_proj = -1e30f;
            for (const auto& v : col.mesh.vertices) {
                Vec3 world_v = col.world_transform.transform_point(v);
                float proj = world_v.dot(norm_axis);
                min_proj = std::min(min_proj, proj);
                max_proj = std::max(max_proj, proj);
            }
        }
    }

public:
    static SatResult test_collision(const Collider& col_a, const Collider& col_b) {
        SatResult result;
        std::vector<Vec3> axes;
        axes.reserve(30);

        // Add face normals of A
        Mat3 r_a = col_a.world_transform.orientation.to_mat3();
        axes.push_back(r_a * Vec3(1, 0, 0));
        axes.push_back(r_a * Vec3(0, 1, 0));
        axes.push_back(r_a * Vec3(0, 0, 1));

        if (col_a.type == ColliderType::ConvexPolyhedron) {
            for (const auto& face : col_a.mesh.faces) {
                axes.push_back(col_a.world_transform.transform_direction(face.normal));
            }
        }

        // Add face normals of B
        Mat3 r_b = col_b.world_transform.orientation.to_mat3();
        axes.push_back(r_b * Vec3(1, 0, 0));
        axes.push_back(r_b * Vec3(0, 1, 0));
        axes.push_back(r_b * Vec3(0, 0, 1));

        if (col_b.type == ColliderType::ConvexPolyhedron) {
            for (const auto& face : col_b.mesh.faces) {
                axes.push_back(col_b.world_transform.transform_direction(face.normal));
            }
        }

        // Edge cross products (A x B)
        std::vector<Vec3> edges_a = { r_a * Vec3(1,0,0), r_a * Vec3(0,1,0), r_a * Vec3(0,0,1) };
        std::vector<Vec3> edges_b = { r_b * Vec3(1,0,0), r_b * Vec3(0,1,0), r_b * Vec3(0,0,1) };

        for (const auto& ea : edges_a) {
            for (const auto& eb : edges_b) {
                Vec3 cross_axis = ea.cross(eb);
                if (cross_axis.length_sq() > SAT_AXIS_EPSILON * SAT_AXIS_EPSILON) {
                    axes.push_back(cross_axis.normalize());
                }
            }
        }

        float min_overlap = 1e30f;
        Vec3 best_axis = Vec3::unit_y();

        Vec3 center_a = col_a.world_transform.position;
        Vec3 center_b = col_b.world_transform.position;

        for (const auto& axis : axes) {
            if (axis.length_sq() <= SAT_AXIS_EPSILON * SAT_AXIS_EPSILON) continue;
            Vec3 norm_axis = axis.normalize();

            float min_a, max_a, min_b, max_b;
            project_collider(col_a, norm_axis, min_a, max_a);
            project_collider(col_b, norm_axis, min_b, max_b);

            float overlap = std::min(max_a, max_b) - std::max(min_a, min_b);
            if (overlap <= 0.0f) {
                result.colliding = false;
                return result;
            }

            if (overlap < min_overlap) {
                min_overlap = overlap;
                best_axis = norm_axis;
                // Ensure normal points from A to B
                if (best_axis.dot(center_b - center_a) < 0.0f) {
                    best_axis = -best_axis;
                }
            }
        }

        result.colliding = true;
        result.penetration_depth = min_overlap;
        result.normal = best_axis;
        return result;
    }
};

} // namespace destruction::collision

#endif // DESTRUCTION_SAT_HPP
