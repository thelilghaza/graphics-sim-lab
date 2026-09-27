#ifndef DESTRUCTION_NARROWPHASE_HPP
#define DESTRUCTION_NARROWPHASE_HPP

#include "destruction/math/vec3.hpp"
#include "destruction/collision/collider.hpp"
#include "destruction/collision/gjk.hpp"
#include "destruction/collision/epa.hpp"
#include "destruction/collision/sat.hpp"
#include "destruction/collision/contact_manifold.hpp"
#include <cmath>
#include <algorithm>

namespace destruction::collision {

using namespace destruction::math;

class Narrowphase {
public:
    static ContactManifold collide(const Collider& col_a, const Collider& col_b) {
        ContactManifold manifold;
        manifold.collider_a_id = col_a.id;
        manifold.collider_b_id = col_b.id;
        manifold.body_a_id = col_a.body_id;
        manifold.body_b_id = col_b.body_id;

        if (col_a.is_static && col_b.is_static) {
            return manifold;
        }

        // 1. Primary GJK Intersection Query
        GjkResult gjk_res = Gjk::intersect(col_a, col_b);
        if (!gjk_res.intersecting) {
            return manifold;
        }

        // 2. EPA Penetration Depth & Normal Query
        EpaResult epa_res = Epa::expand(col_a, col_b, gjk_res.simplex);

        if (epa_res.success) {
            manifold.normal = epa_res.normal.normalize();
            manifold.max_penetration_depth = epa_res.penetration_depth;

            Vec3 world_contact = 0.5f * (epa_res.point_a + epa_res.point_b);
            Vec3 local_a = col_a.world_transform.inverse_transform_point(world_contact);
            Vec3 local_b = col_b.world_transform.inverse_transform_point(world_contact);

            ContactPoint primary_pt(world_contact, local_a, local_b, epa_res.penetration_depth, 1);
            manifold.add_point(primary_pt);

            // For face-to-face contacts, derive additional feature points around normal
            if (col_a.type == ColliderType::Box && col_b.type == ColliderType::Box) {
                Vec3 u = (std::abs(manifold.normal.x) < 0.9f) ? manifold.normal.cross(Vec3::unit_x()).normalize() : manifold.normal.cross(Vec3::unit_y()).normalize();
                Vec3 v = manifold.normal.cross(u).normalize();

                float offset = 0.2f * std::min({col_a.box_half_extents.x, col_a.box_half_extents.y, col_a.box_half_extents.z});

                std::vector<Vec3> offsets = {
                    u * offset,
                    -u * offset,
                    v * offset,
                    -v * offset
                };

                for (size_t k = 0; k < offsets.size(); ++k) {
                    Vec3 extra_world = world_contact + offsets[k];
                    Vec3 loc_a = col_a.world_transform.inverse_transform_point(extra_world);
                    Vec3 loc_b = col_b.world_transform.inverse_transform_point(extra_world);
                    manifold.add_point(ContactPoint(extra_world, loc_a, loc_b, epa_res.penetration_depth, static_cast<uint32_t>(k + 2)));
                }
                manifold.reduce_to_max_4();
            }
        } else {
            // Fallback SAT Query if EPA numerical convergence failed
            SatResult sat_res = Sat::test_collision(col_a, col_b);
            if (sat_res.colliding) {
                manifold.normal = sat_res.normal.normalize();
                manifold.max_penetration_depth = sat_res.penetration_depth;

                Vec3 world_contact = 0.5f * (col_a.world_transform.position + col_b.world_transform.position);
                Vec3 local_a = col_a.world_transform.inverse_transform_point(world_contact);
                Vec3 local_b = col_b.world_transform.inverse_transform_point(world_contact);

                manifold.add_point(ContactPoint(world_contact, local_a, local_b, sat_res.penetration_depth, 1));
            }
        }

        return manifold;
    }
};

} // namespace destruction::collision

#endif // DESTRUCTION_NARROWPHASE_HPP
