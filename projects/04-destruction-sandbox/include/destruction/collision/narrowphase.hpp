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
            Vec3 norm = epa_res.normal.normalize();

            // Snap-align near-face contact normals to exact box face normal to eliminate lateral shear drift
            if (col_a.type == ColliderType::Box && col_b.type == ColliderType::Box) {
                Vec3 n_a = col_a.world_transform.inverse_transform_direction(norm);
                int axis_a = 0; float max_a = std::abs(n_a.x);
                if (std::abs(n_a.y) > max_a) { axis_a = 1; max_a = std::abs(n_a.y); }
                if (std::abs(n_a.z) > max_a) { axis_a = 2; max_a = std::abs(n_a.z); }

                if (max_a > 0.95f) {
                    Vec3 n_snap = Vec3::zero();
                    n_snap[axis_a] = (n_a[axis_a] >= 0.0f) ? 1.0f : -1.0f;
                    norm = col_a.world_transform.transform_direction(n_snap).normalize();
                }
            }

            manifold.normal = norm;
            manifold.max_penetration_depth = epa_res.penetration_depth;

            Vec3 world_contact = 0.5f * (epa_res.point_a + epa_res.point_b);
            Vec3 local_a = col_a.world_transform.inverse_transform_point(world_contact);
            Vec3 local_b = col_b.world_transform.inverse_transform_point(world_contact);

            ContactPoint primary_pt(world_contact, local_a, local_b, epa_res.penetration_depth, 1);
            manifold.add_point(primary_pt);

            // For face-to-face contacts, derive stable 4-corner support points across contact face
            if (col_a.type == ColliderType::Box && col_b.type == ColliderType::Box) {
                manifold.points.clear();

                // Determine local normals on A and B
                Vec3 n_a = col_a.world_transform.inverse_transform_direction(manifold.normal);
                Vec3 n_b = col_b.world_transform.inverse_transform_direction(-manifold.normal);

                int axis_a = 0; float max_a = std::abs(n_a.x);
                if (std::abs(n_a.y) > max_a) { axis_a = 1; max_a = std::abs(n_a.y); }
                if (std::abs(n_a.z) > max_a) { axis_a = 2; max_a = std::abs(n_a.z); }

                int axis_b = 0; float max_b = std::abs(n_b.x);
                if (std::abs(n_b.y) > max_b) { axis_b = 1; max_b = std::abs(n_b.y); }
                if (std::abs(n_b.z) > max_b) { axis_b = 2; max_b = std::abs(n_b.z); }

                // Determine reference collider (larger normal projection alignment)
                bool is_ref_a = (max_a >= max_b);
                const Collider& ref_col = is_ref_a ? col_a : col_b;
                const Collider& inc_col = is_ref_a ? col_b : col_a;
                int ref_axis = is_ref_a ? axis_a : axis_b;
                int inc_axis = is_ref_a ? axis_b : axis_a;
                Vec3 ref_n = is_ref_a ? n_a : n_b;
                Vec3 inc_n = is_ref_a ? n_b : n_a;

                // Face centers in world space
                Vec3 ref_face_loc = Vec3::zero();
                ref_face_loc[ref_axis] = (ref_n[ref_axis] >= 0.0f ? 1.0f : -1.0f) * ref_col.box_half_extents[ref_axis];
                Vec3 ref_face_world = ref_col.world_transform.transform_point(ref_face_loc);

                Vec3 inc_face_loc = Vec3::zero();
                inc_face_loc[inc_axis] = (inc_n[inc_axis] >= 0.0f ? 1.0f : -1.0f) * inc_col.box_half_extents[inc_axis];
                Vec3 inc_face_world = inc_col.world_transform.transform_point(inc_face_loc);

                Vec3 contact_center = 0.5f * (ref_face_world + inc_face_world);

                // Tangent axes on reference face in world space
                int u_idx = (ref_axis + 1) % 3;
                int v_idx = (ref_axis + 2) % 3;

                Vec3 u_loc = Vec3::zero(); u_loc[u_idx] = 1.0f;
                Vec3 v_loc = Vec3::zero(); v_loc[v_idx] = 1.0f;

                Vec3 u_world = ref_col.world_transform.transform_direction(u_loc).normalize();
                Vec3 v_world = ref_col.world_transform.transform_direction(v_loc).normalize();

                float eu = std::min(ref_col.box_half_extents[u_idx], inc_col.box_half_extents[u_idx]) * 0.85f;
                float ev = std::min(ref_col.box_half_extents[v_idx], inc_col.box_half_extents[v_idx]) * 0.85f;

                Vec3 corner_offsets[4] = {
                    u_world * eu + v_world * ev,
                    -u_world * eu + v_world * ev,
                    -u_world * eu - v_world * ev,
                    u_world * eu - v_world * ev
                };

                for (uint32_t k = 0; k < 4; ++k) {
                    Vec3 pt_world = contact_center + corner_offsets[k];
                    Vec3 loc_a = col_a.world_transform.inverse_transform_point(pt_world);
                    Vec3 loc_b = col_b.world_transform.inverse_transform_point(pt_world);
                    manifold.add_point(ContactPoint(pt_world, loc_a, loc_b, epa_res.penetration_depth, k + 1));
                }
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
