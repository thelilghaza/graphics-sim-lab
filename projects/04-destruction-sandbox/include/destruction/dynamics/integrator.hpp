#ifndef DESTRUCTION_INTEGRATOR_HPP
#define DESTRUCTION_INTEGRATOR_HPP

#include "destruction/dynamics/rigid_body.hpp"
#include "destruction/math/vec3.hpp"
#include "destruction/math/quat.hpp"
#include "destruction/math/mat3.hpp"

namespace destruction::dynamics {

using namespace destruction::math;

class Integrator {
public:
    static void step_symplectic_euler(RigidBody& body, float dt) {
        if (body.is_static || body.inv_mass == 0.0f || dt <= 0.0f) {
            return;
        }

        // 1. Linear velocity update
        Vec3 linear_accel = body.force_accumulator * body.inv_mass;
        body.linear_velocity += linear_accel * dt;

        // 2. Position update using updated linear velocity (Symplectic Euler)
        body.position += body.linear_velocity * dt;

        // 3. Angular velocity update
        Mat3 inv_i_world = body.get_world_inv_inertia();
        Vec3 angular_accel = inv_i_world * body.torque_accumulator;
        body.angular_velocity += angular_accel * dt;

        // 4. Orientation quaternion update using updated angular velocity
        Quat omega_quat(0.0f, body.angular_velocity);
        Quat q_dot = 0.5f * (omega_quat * body.orientation);
        body.orientation = (body.orientation + q_dot * dt).normalize();
    }
};

} // namespace destruction::dynamics

#endif // DESTRUCTION_INTEGRATOR_HPP
