#ifndef DESTRUCTION_COLLIDER_HPP
#define DESTRUCTION_COLLIDER_HPP

#include "destruction/math/vec3.hpp"
#include "destruction/math/transform.hpp"
#include "destruction/collision/aabb.hpp"
#include "destruction/fracture/polyhedron.hpp"
#include <cstdint>
#include <cmath>
#include <algorithm>

namespace destruction::collision {

using namespace destruction::math;
using namespace destruction::fracture;

enum class ColliderType {
    Box,
    ConvexPolyhedron
};

struct Collider {
    uint32_t id{0};
    uint32_t body_id{0};
    ColliderType type{ColliderType::Box};

    Transform local_transform{Transform::identity()};
    Transform world_transform{Transform::identity()};

    Vec3 box_half_extents{1.0f, 1.0f, 1.0f};
    ConvexPolyhedron mesh;

    Aabb local_aabb;
    Aabb world_aabb;
    bool is_static{false};

    Collider() = default;

    static Collider create_box(
        uint32_t id_val,
        uint32_t body_id_val,
        const Vec3& half_extents,
        const Transform& local_t = Transform::identity()
    ) {
        Collider col;
        col.id = id_val;
        col.body_id = body_id_val;
        col.type = ColliderType::Box;
        col.box_half_extents = half_extents;
        col.local_transform = local_t;
        col.local_aabb = Aabb(-half_extents, half_extents).transform(local_t);
        col.world_transform = local_t;
        col.world_aabb = col.local_aabb;
        return col;
    }

    static Collider create_polyhedron(
        uint32_t id_val,
        uint32_t body_id_val,
        const ConvexPolyhedron& poly,
        const Transform& local_t = Transform::identity()
    ) {
        Collider col;
        col.id = id_val;
        col.body_id = body_id_val;
        col.type = ColliderType::ConvexPolyhedron;
        col.mesh = poly;
        col.local_transform = local_t;

        Aabb bounds = Aabb::create_empty();
        for (const auto& v : poly.vertices) {
            bounds.expand(local_t.transform_point(v));
        }
        col.local_aabb = bounds;
        col.world_transform = local_t;
        col.world_aabb = col.local_aabb;
        return col;
    }

    void update_world_transform(const Transform& body_transform) {
        world_transform = body_transform.combine(local_transform);
        update_world_aabb();
    }

    void update_world_aabb() {
        if (type == ColliderType::Box) {
            world_aabb = Aabb(-box_half_extents, box_half_extents).transform(world_transform);
        } else {
            Aabb bounds = Aabb::create_empty();
            for (const auto& v : mesh.vertices) {
                bounds.expand(world_transform.transform_point(v));
            }
            world_aabb = bounds;
        }
    }

    Vec3 get_support_world(const Vec3& world_dir) const {
        // Transform direction into local space (R^T * dir)
        Vec3 local_dir = world_transform.inverse_transform_direction(world_dir);

        Vec3 local_support = Vec3::zero();

        if (type == ColliderType::Box) {
            local_support.x = (local_dir.x >= 0.0f) ? box_half_extents.x : -box_half_extents.x;
            local_support.y = (local_dir.y >= 0.0f) ? box_half_extents.y : -box_half_extents.y;
            local_support.z = (local_dir.z >= 0.0f) ? box_half_extents.z : -box_half_extents.z;
        } else { // ConvexPolyhedron
            if (!mesh.vertices.empty()) {
                float max_dot = -1e30f;
                size_t best_idx = 0;
                for (size_t i = 0; i < mesh.vertices.size(); ++i) {
                    float dot_val = mesh.vertices[i].dot(local_dir);
                    if (dot_val > max_dot) {
                        max_dot = dot_val;
                        best_idx = i;
                    }
                }
                local_support = mesh.vertices[best_idx];
            }
        }

        // Transform local support point to world space
        return world_transform.transform_point(local_support);
    }
};

} // namespace destruction::collision

#endif // DESTRUCTION_COLLIDER_HPP
