#ifndef DESTRUCTION_INERTIA_HPP
#define DESTRUCTION_INERTIA_HPP

#include "destruction/math/vec3.hpp"
#include "destruction/math/mat3.hpp"
#include "destruction/math/quat.hpp"
#include "destruction/math/math_utils.hpp"
#include <cassert>

namespace destruction::dynamics {

using namespace destruction::math;

struct InertiaTensor {
    Vec3 body_inertia{Vec3::zero()};
    Vec3 body_inv_inertia{Vec3::zero()};

    constexpr InertiaTensor() = default;

    constexpr InertiaTensor(const Vec3& inertia, const Vec3& inv_inertia)
        : body_inertia(inertia), body_inv_inertia(inv_inertia) {}

    static InertiaTensor static_body() {
        return InertiaTensor(Vec3::zero(), Vec3::zero());
    }

    static InertiaTensor box(float mass, float width, float height, float depth) {
        if (mass <= EPSILON || width <= EPSILON || height <= EPSILON || depth <= EPSILON) {
            return static_body();
        }
        float i_xx = (1.0f / 12.0f) * mass * (height * height + depth * depth);
        float i_yy = (1.0f / 12.0f) * mass * (width * width + depth * depth);
        float i_zz = (1.0f / 12.0f) * mass * (width * width + height * height);

        Vec3 inertia(i_xx, i_yy, i_zz);
        Vec3 inv_inertia(1.0f / i_xx, 1.0f / i_yy, 1.0f / i_zz);
        return InertiaTensor(inertia, inv_inertia);
    }

    static InertiaTensor sphere(float mass, float radius) {
        if (mass <= EPSILON || radius <= EPSILON) {
            return static_body();
        }
        float i_val = (2.0f / 5.0f) * mass * radius * radius;
        Vec3 inertia(i_val, i_val, i_val);
        Vec3 inv_inertia(1.0f / i_val, 1.0f / i_val, 1.0f / i_val);
        return InertiaTensor(inertia, inv_inertia);
    }

    Mat3 get_world_inv_inertia(const Quat& orientation) const {
        if (body_inv_inertia == Vec3::zero()) {
            return Mat3::zero();
        }
        Mat3 r = orientation.to_mat3();
        Mat3 inv_body = Mat3::diagonal(body_inv_inertia);
        return r * inv_body * r.transpose();
    }
};

} // namespace destruction::dynamics

#endif // DESTRUCTION_INERTIA_HPP
