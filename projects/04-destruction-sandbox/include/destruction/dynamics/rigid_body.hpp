#ifndef DESTRUCTION_RIGID_BODY_HPP
#define DESTRUCTION_RIGID_BODY_HPP

#include "destruction/math/vec3.hpp"
#include "destruction/math/quat.hpp"
#include "destruction/math/transform.hpp"
#include "destruction/dynamics/inertia.hpp"
#include <cstdint>

namespace destruction::dynamics {

using namespace destruction::math;

struct RigidBody {
    uint32_t id{0};
    Vec3 position{Vec3::zero()};
    Quat orientation{Quat::identity()};
    Vec3 linear_velocity{Vec3::zero()};
    Vec3 angular_velocity{Vec3::zero()};

    float mass{0.0f};
    float inv_mass{0.0f};
    InertiaTensor inertia;

    Vec3 force_accumulator{Vec3::zero()};
    Vec3 torque_accumulator{Vec3::zero()};
    bool is_static{true};

    constexpr RigidBody() = default;

    static RigidBody create_dynamic(
        uint32_t id_val,
        float mass_val,
        const InertiaTensor& inertia_val,
        const Vec3& pos = Vec3::zero(),
        const Quat& rot = Quat::identity()
    ) {
        RigidBody body;
        body.id = id_val;
        body.position = pos;
        body.orientation = rot.normalize();
        body.linear_velocity = Vec3::zero();
        body.angular_velocity = Vec3::zero();
        body.mass = mass_val;
        body.inv_mass = (mass_val > EPSILON) ? (1.0f / mass_val) : 0.0f;
        body.inertia = inertia_val;
        body.force_accumulator = Vec3::zero();
        body.torque_accumulator = Vec3::zero();
        body.is_static = (mass_val <= EPSILON);
        return body;
    }

    static RigidBody create_static(
        uint32_t id_val,
        const Vec3& pos = Vec3::zero(),
        const Quat& rot = Quat::identity()
    ) {
        RigidBody body;
        body.id = id_val;
        body.position = pos;
        body.orientation = rot.normalize();
        body.linear_velocity = Vec3::zero();
        body.angular_velocity = Vec3::zero();
        body.mass = 0.0f;
        body.inv_mass = 0.0f;
        body.inertia = InertiaTensor::static_body();
        body.force_accumulator = Vec3::zero();
        body.torque_accumulator = Vec3::zero();
        body.is_static = true;
        return body;
    }

    void apply_force(const Vec3& force) {
        if (is_static || inv_mass == 0.0f) return;
        force_accumulator += force;
    }

    void apply_force_at_point(const Vec3& force, const Vec3& world_point) {
        if (is_static || inv_mass == 0.0f) return;
        Vec3 r = world_point - position;
        apply_force(force);
        apply_torque(r.cross(force));
    }

    void apply_torque(const Vec3& torque) {
        if (is_static || inv_mass == 0.0f) return;
        torque_accumulator += torque;
    }

    void clear_accumulators() {
        force_accumulator = Vec3::zero();
        torque_accumulator = Vec3::zero();
    }

    Mat3 get_world_inv_inertia() const {
        if (is_static || inv_mass == 0.0f) return Mat3::zero();
        return inertia.get_world_inv_inertia(orientation);
    }

    Transform get_transform() const {
        return Transform(position, orientation);
    }
};

} // namespace destruction::dynamics

#endif // DESTRUCTION_RIGID_BODY_HPP
