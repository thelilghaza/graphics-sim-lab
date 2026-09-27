#ifndef DESTRUCTION_CONTACT_CONSTRAINT_HPP
#define DESTRUCTION_CONTACT_CONSTRAINT_HPP

#include "destruction/math/vec3.hpp"
#include "destruction/math/mat3.hpp"
#include "destruction/dynamics/rigid_body.hpp"
#include "destruction/collision/contact_manifold.hpp"
#include "destruction/solver/solver_settings.hpp"
#include <cmath>
#include <algorithm>
#include <cstdint>

namespace destruction::solver {

using namespace destruction::math;
using namespace destruction::dynamics;
using namespace destruction::collision;

struct ContactConstraint {
    RigidBody* body_a{nullptr};
    RigidBody* body_b{nullptr};

    Vec3 point_world{Vec3::zero()};
    Vec3 normal{Vec3::unit_y()}; // Pointing from Body A toward Body B
    Vec3 tangent1{Vec3::zero()};
    Vec3 tangent2{Vec3::zero()};

    Vec3 r_a{Vec3::zero()};
    Vec3 r_b{Vec3::zero()};

    float penetration_depth{0.0f};
    float friction{0.3f};
    float restitution{0.0f};

    float effective_mass_n{0.0f};
    float effective_mass_t1{0.0f};
    float effective_mass_t2{0.0f};

    float restitution_bias{0.0f};
    float position_bias{0.0f};

    float accumulated_normal_impulse{0.0f};
    float accumulated_tangent_impulse_1{0.0f};
    float accumulated_tangent_impulse_2{0.0f};
    float accumulated_position_impulse{0.0f};

    uint64_t warm_start_key{0};

    ContactConstraint() = default;

    void init(
        RigidBody* b_a,
        RigidBody* b_b,
        const ContactPoint& cp,
        const Vec3& manifold_normal,
        float fric,
        float rest,
        const SolverSettings& settings,
        float dt
    ) {
        body_a = b_a;
        body_b = b_b;
        point_world = cp.position_world;
        normal = manifold_normal.normalize();
        penetration_depth = cp.penetration_depth;
        friction = fric;
        restitution = rest;

        r_a = point_world - body_a->position;
        r_b = point_world - body_b->position;

        // Construct stable orthogonal tangent basis
        Vec3 ref_axis = (std::abs(normal.x) < 0.6f) ? Vec3::unit_x() : Vec3::unit_y();
        tangent1 = (ref_axis - normal * normal.dot(ref_axis)).normalize();
        tangent2 = normal.cross(tangent1).normalize();

        float inv_m_a = body_a->is_static ? 0.0f : body_a->inv_mass;
        float inv_m_b = body_b->is_static ? 0.0f : body_b->inv_mass;

        Mat3 inv_I_a = body_a->is_static ? Mat3::zero() : body_a->get_world_inv_inertia();
        Mat3 inv_I_b = body_b->is_static ? Mat3::zero() : body_b->get_world_inv_inertia();

        // Effective normal mass
        Vec3 ra_xn = r_a.cross(normal);
        Vec3 rb_xn = r_b.cross(normal);
        Vec3 angular_term_a = (inv_I_a * ra_xn).cross(r_a);
        Vec3 angular_term_b = (inv_I_b * rb_xn).cross(r_b);
        float k_n = inv_m_a + inv_m_b + normal.dot(angular_term_a + angular_term_b);
        effective_mass_n = (k_n > 1e-12f) ? (1.0f / k_n) : 0.0f;

        // Effective tangent 1 mass
        Vec3 ra_xt1 = r_a.cross(tangent1);
        Vec3 rb_xt1 = r_b.cross(tangent1);
        Vec3 ang_t1_a = (inv_I_a * ra_xt1).cross(r_a);
        Vec3 ang_t1_b = (inv_I_b * rb_xt1).cross(r_b);
        float k_t1 = inv_m_a + inv_m_b + tangent1.dot(ang_t1_a + ang_t1_b);
        effective_mass_t1 = (k_t1 > 1e-12f) ? (1.0f / k_t1) : 0.0f;

        // Effective tangent 2 mass
        Vec3 ra_xt2 = r_a.cross(tangent2);
        Vec3 rb_xt2 = r_b.cross(tangent2);
        Vec3 ang_t2_a = (inv_I_a * ra_xt2).cross(r_a);
        Vec3 ang_t2_b = (inv_I_b * rb_xt2).cross(r_b);
        float k_t2 = inv_m_a + inv_m_b + tangent2.dot(ang_t2_a + ang_t2_b);
        effective_mass_t2 = (k_t2 > 1e-12f) ? (1.0f / k_t2) : 0.0f;

        // Initial relative velocity
        Vec3 v_a_contact = body_a->linear_velocity + body_a->angular_velocity.cross(r_a);
        Vec3 v_b_contact = body_b->linear_velocity + body_b->angular_velocity.cross(r_b);
        Vec3 v_rel = v_b_contact - v_a_contact;
        float v_normal = v_rel.dot(normal);

        // Restitution bias (applied only if closing velocity exceeds threshold)
        if (v_normal < -settings.restitution_threshold) {
            restitution_bias = -restitution * v_normal;
        } else {
            restitution_bias = 0.0f;
        }

        // Position bias for split impulse position correction
        float depth_excess = std::max(0.0f, penetration_depth - settings.penetration_slop);
        position_bias = (settings.baumgarte_beta / dt) * std::min(depth_excess, settings.max_position_correction);

        // Warm-start key construction
        uint32_t min_id = std::min(body_a->id, body_b->id);
        uint32_t max_id = std::max(body_a->id, body_b->id);
        uint32_t feat_hash = cp.feature_id;
        if (feat_hash == 0) {
            int qx = static_cast<int>(std::floor(point_world.x * 50.0f));
            int qy = static_cast<int>(std::floor(point_world.y * 50.0f));
            int qz = static_cast<int>(std::floor(point_world.z * 50.0f));
            feat_hash = static_cast<uint32_t>(qx ^ (qy * 16777619) ^ (qz * 2166136261u));
        }
        warm_start_key = (static_cast<uint64_t>(min_id) << 48) ^ (static_cast<uint64_t>(max_id) << 32) ^ feat_hash;
    }

    void apply_warm_start() {
        if (accumulated_normal_impulse == 0.0f && accumulated_tangent_impulse_1 == 0.0f && accumulated_tangent_impulse_2 == 0.0f) {
            return;
        }

        Vec3 impulse = normal * accumulated_normal_impulse + tangent1 * accumulated_tangent_impulse_1 + tangent2 * accumulated_tangent_impulse_2;

        if (!body_a->is_static && body_a->inv_mass > 0.0f) {
            body_a->linear_velocity -= impulse * body_a->inv_mass;
            body_a->angular_velocity -= body_a->get_world_inv_inertia() * r_a.cross(impulse);
        }

        if (!body_b->is_static && body_b->inv_mass > 0.0f) {
            body_b->linear_velocity += impulse * body_b->inv_mass;
            body_b->angular_velocity += body_b->get_world_inv_inertia() * r_b.cross(impulse);
        }
    }

    void solve_velocity_normal() {
        Vec3 v_a_contact = body_a->linear_velocity + body_a->angular_velocity.cross(r_a);
        Vec3 v_b_contact = body_b->linear_velocity + body_b->angular_velocity.cross(r_b);
        Vec3 v_rel = v_b_contact - v_a_contact;
        float v_normal = v_rel.dot(normal);

        float delta_jn = effective_mass_n * (restitution_bias - v_normal);

        float old_jn = accumulated_normal_impulse;
        accumulated_normal_impulse = std::max(0.0f, old_jn + delta_jn);
        float actual_delta = accumulated_normal_impulse - old_jn;

        Vec3 impulse = normal * actual_delta;

        if (!body_a->is_static && body_a->inv_mass > 0.0f) {
            body_a->linear_velocity -= impulse * body_a->inv_mass;
            body_a->angular_velocity -= body_a->get_world_inv_inertia() * r_a.cross(impulse);
        }

        if (!body_b->is_static && body_b->inv_mass > 0.0f) {
            body_b->linear_velocity += impulse * body_b->inv_mass;
            body_b->angular_velocity += body_b->get_world_inv_inertia() * r_b.cross(impulse);
        }
    }

    void solve_velocity_friction() {
        if (friction <= 0.0f || accumulated_normal_impulse <= 0.0f) return;

        Vec3 v_a_contact = body_a->linear_velocity + body_a->angular_velocity.cross(r_a);
        Vec3 v_b_contact = body_b->linear_velocity + body_b->angular_velocity.cross(r_b);
        Vec3 v_rel = v_b_contact - v_a_contact;

        float v_t1 = v_rel.dot(tangent1);
        float v_t2 = v_rel.dot(tangent2);

        float delta_jt1 = effective_mass_t1 * (-v_t1);
        float delta_jt2 = effective_mass_t2 * (-v_t2);

        float old_jt1 = accumulated_tangent_impulse_1;
        float old_jt2 = accumulated_tangent_impulse_2;

        float new_jt1 = old_jt1 + delta_jt1;
        float new_jt2 = old_jt2 + delta_jt2;

        float max_friction = friction * accumulated_normal_impulse;
        float mag_sq = new_jt1 * new_jt1 + new_jt2 * new_jt2;

        if (mag_sq > max_friction * max_friction && mag_sq > 0.0f) {
            float scale = max_friction / std::sqrt(mag_sq);
            new_jt1 *= scale;
            new_jt2 *= scale;
        }

        float actual_delta_1 = new_jt1 - old_jt1;
        float actual_delta_2 = new_jt2 - old_jt2;

        accumulated_tangent_impulse_1 = new_jt1;
        accumulated_tangent_impulse_2 = new_jt2;

        Vec3 impulse = tangent1 * actual_delta_1 + tangent2 * actual_delta_2;

        if (!body_a->is_static && body_a->inv_mass > 0.0f) {
            body_a->linear_velocity -= impulse * body_a->inv_mass;
            body_a->angular_velocity -= body_a->get_world_inv_inertia() * r_a.cross(impulse);
        }

        if (!body_b->is_static && body_b->inv_mass > 0.0f) {
            body_b->linear_velocity += impulse * body_b->inv_mass;
            body_b->angular_velocity += body_b->get_world_inv_inertia() * r_b.cross(impulse);
        }
    }

    void solve_position_split(Vec3& pseudo_v_a, Vec3& pseudo_w_a, Vec3& pseudo_v_b, Vec3& pseudo_w_b) {
        if (position_bias <= 0.0f) return;

        Vec3 v_a_ps = pseudo_v_a + pseudo_w_a.cross(r_a);
        Vec3 v_b_ps = pseudo_v_b + pseudo_w_b.cross(r_b);
        Vec3 v_rel_ps = v_b_ps - v_a_ps;
        float v_normal_ps = v_rel_ps.dot(normal);

        float delta_jp = effective_mass_n * (position_bias - v_normal_ps);

        float old_jp = accumulated_position_impulse;
        accumulated_position_impulse = std::max(0.0f, old_jp + delta_jp);
        float actual_delta = accumulated_position_impulse - old_jp;

        Vec3 impulse = normal * actual_delta;

        if (!body_a->is_static && body_a->inv_mass > 0.0f) {
            pseudo_v_a -= impulse * body_a->inv_mass;
            pseudo_w_a -= body_a->get_world_inv_inertia() * r_a.cross(impulse);
        }

        if (!body_b->is_static && body_b->inv_mass > 0.0f) {
            pseudo_v_b += impulse * body_b->inv_mass;
            pseudo_w_b += body_b->get_world_inv_inertia() * r_b.cross(impulse);
        }
    }
};

} // namespace destruction::solver

#endif // DESTRUCTION_CONTACT_CONSTRAINT_HPP
