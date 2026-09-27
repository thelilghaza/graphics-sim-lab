#ifndef DESTRUCTION_CONTACT_MANIFOLD_HPP
#define DESTRUCTION_CONTACT_MANIFOLD_HPP

#include "destruction/math/vec3.hpp"
#include "destruction/math/math_utils.hpp"
#include <vector>
#include <cstdint>
#include <algorithm>

namespace destruction::collision {

using namespace destruction::math;

constexpr float MANIFOLD_MERGE_EPSILON = 1e-3f;

struct ContactPoint {
    Vec3 position_world{Vec3::zero()};
    Vec3 local_point_a{Vec3::zero()};
    Vec3 local_point_b{Vec3::zero()};
    float penetration_depth{0.0f};
    uint32_t feature_id{0};

    ContactPoint() = default;
    ContactPoint(const Vec3& world_pos, const Vec3& local_a, const Vec3& local_b, float depth, uint32_t feat = 0)
        : position_world(world_pos), local_point_a(local_a), local_point_b(local_b), penetration_depth(depth), feature_id(feat) {}
};

struct ContactManifold {
    uint32_t body_a_id{0};
    uint32_t body_b_id{0};
    uint32_t collider_a_id{0};
    uint32_t collider_b_id{0};

    Vec3 normal{Vec3::unit_y()}; // Pointing from A to B
    float max_penetration_depth{0.0f};
    std::vector<ContactPoint> points;

    ContactManifold() {
        points.reserve(4);
    }

    void add_point(const ContactPoint& pt, float merge_eps = MANIFOLD_MERGE_EPSILON) {
        for (const auto& existing : points) {
            if ((existing.position_world - pt.position_world).length_sq() <= merge_eps * merge_eps) {
                return;
            }
        }
        points.push_back(pt);
        max_penetration_depth = std::max(max_penetration_depth, pt.penetration_depth);
    }

    void reduce_to_max_4() {
        if (points.size() <= 4) return;

        std::vector<ContactPoint> reduced;
        reduced.reserve(4);

        // 1. Deepest penetration point
        size_t idx_deepest = 0;
        float max_depth = points[0].penetration_depth;
        for (size_t i = 1; i < points.size(); ++i) {
            if (points[i].penetration_depth > max_depth) {
                max_depth = points[i].penetration_depth;
                idx_deepest = i;
            }
        }
        reduced.push_back(points[idx_deepest]);

        // 2. Furthest from point 1
        size_t idx_furthest = 0;
        float max_dist_sq = -1.0f;
        for (size_t i = 0; i < points.size(); ++i) {
            if (i == idx_deepest) continue;
            float dist_sq = (points[i].position_world - reduced[0].position_world).length_sq();
            if (dist_sq > max_dist_sq) {
                max_dist_sq = dist_sq;
                idx_furthest = i;
            }
        }
        reduced.push_back(points[idx_furthest]);

        // 3. Maximum triangle area with points 1 and 2
        size_t idx_max_area = 0;
        float max_area = -1.0f;
        Vec3 e1 = reduced[1].position_world - reduced[0].position_world;
        for (size_t i = 0; i < points.size(); ++i) {
            if (i == idx_deepest || i == idx_furthest) continue;
            Vec3 e2 = points[i].position_world - reduced[0].position_world;
            float area = e1.cross(e2).length_sq();
            if (area > max_area) {
                max_area = area;
                idx_max_area = i;
            }
        }
        reduced.push_back(points[idx_max_area]);

        // 4. Maximum distance from triangle (1, 2, 3)
        size_t idx_fourth = 0;
        float max_val = -1.0f;
        Vec3 tri_norm = (reduced[1].position_world - reduced[0].position_world).cross(reduced[2].position_world - reduced[0].position_world);
        for (size_t i = 0; i < points.size(); ++i) {
            if (i == idx_deepest || i == idx_furthest || i == idx_max_area) continue;
            float val = std::abs((points[i].position_world - reduced[0].position_world).dot(tri_norm));
            if (val > max_val) {
                max_val = val;
                idx_fourth = i;
            }
        }
        reduced.push_back(points[idx_fourth]);

        points = std::move(reduced);
    }
};

} // namespace destruction::collision

#endif // DESTRUCTION_CONTACT_MANIFOLD_HPP
