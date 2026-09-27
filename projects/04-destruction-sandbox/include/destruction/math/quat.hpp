#ifndef DESTRUCTION_QUAT_HPP
#define DESTRUCTION_QUAT_HPP

#include "destruction/math/vec3.hpp"
#include "destruction/math/mat3.hpp"
#include "destruction/math/math_utils.hpp"
#include <cmath>
#include <cassert>

namespace destruction::math {

struct Quat {
    float w{1.0f};
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};

    constexpr Quat() = default;
    constexpr Quat(float w_val, float x_val, float y_val, float z_val)
        : w(w_val), x(x_val), y(y_val), z(z_val) {}
    constexpr Quat(float w_val, const Vec3& v)
        : w(w_val), x(v.x), y(v.y), z(v.z) {}

    static constexpr Quat identity() {
        return Quat(1.0f, 0.0f, 0.0f, 0.0f);
    }

    static Quat axis_angle(const Vec3& axis, float angle_rad) {
        Vec3 norm_axis = axis.normalize();
        if (norm_axis.length_sq() <= EPSILON) {
            return Quat::identity();
        }
        float half_angle = angle_rad * 0.5f;
        float s = std::sin(half_angle);
        return Quat(
            std::cos(half_angle),
            norm_axis.x * s,
            norm_axis.y * s,
            norm_axis.z * s
        ).normalize();
    }

    static Quat from_mat3(const Mat3& m) {
        float trace = m(0, 0) + m(1, 1) + m(2, 2);
        if (trace > 0.0f) {
            float s = 0.5f / std::sqrt(trace + 1.0f);
            return Quat(
                0.25f / s,
                (m(2, 1) - m(1, 2)) * s,
                (m(0, 2) - m(2, 0)) * s,
                (m(1, 0) - m(0, 1)) * s
            ).normalize();
        } else if ((m(0, 0) > m(1, 1)) && (m(0, 0) > m(2, 2))) {
            float s = 2.0f * std::sqrt(1.0f + m(0, 0) - m(1, 1) - m(2, 2));
            return Quat(
                (m(2, 1) - m(1, 2)) / s,
                0.25f * s,
                (m(0, 1) + m(1, 0)) / s,
                (m(0, 2) + m(2, 0)) / s
            ).normalize();
        } else if (m(1, 1) > m(2, 2)) {
            float s = 2.0f * std::sqrt(1.0f + m(1, 1) - m(0, 0) - m(2, 2));
            return Quat(
                (m(0, 2) - m(2, 0)) / s,
                (m(0, 1) + m(1, 0)) / s,
                0.25f * s,
                (m(1, 2) + m(2, 1)) / s
            ).normalize();
        } else {
            float s = 2.0f * std::sqrt(1.0f + m(2, 2) - m(0, 0) - m(1, 1));
            return Quat(
                (m(1, 0) - m(0, 1)) / s,
                (m(0, 2) + m(2, 0)) / s,
                (m(1, 2) + m(2, 1)) / s,
                0.25f * s
            ).normalize();
        }
    }

    Vec3 vec() const {
        return Vec3(x, y, z);
    }

    float length_sq() const {
        return w * w + x * x + y * y + z * z;
    }

    float length() const {
        return std::sqrt(length_sq());
    }

    Quat normalize() const {
        float len = length();
        if (len <= EPSILON) {
            return Quat::identity();
        }
        float inv = 1.0f / len;
        return Quat(w * inv, x * inv, y * inv, z * inv);
    }

    Quat conjugate() const {
        return Quat(w, -x, -y, -z);
    }

    Quat inverse() const {
        float len_sq = length_sq();
        if (len_sq <= EPSILON) {
            return Quat::identity();
        }
        float inv = 1.0f / len_sq;
        return Quat(w * inv, -x * inv, -y * inv, -z * inv);
    }

    Quat operator+(const Quat& rhs) const {
        return Quat(w + rhs.w, x + rhs.x, y + rhs.y, z + rhs.z);
    }

    Quat operator-(const Quat& rhs) const {
        return Quat(w - rhs.w, x - rhs.x, y - rhs.y, z - rhs.z);
    }

    Quat operator*(float scalar) const {
        return Quat(w * scalar, x * scalar, y * scalar, z * scalar);
    }

    Quat operator*(const Quat& rhs) const {
        return Quat(
            w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z,
            w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y,
            w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x,
            w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w
        );
    }

    Vec3 rotate(const Vec3& v) const {
        Vec3 qv(x, y, z);
        Vec3 t = 2.0f * qv.cross(v);
        return v + w * t + qv.cross(t);
    }

    Mat3 to_mat3() const {
        Quat q = normalize();
        float xx = q.x * q.x;
        float yy = q.y * q.y;
        float zz = q.z * q.z;
        float xy = q.x * q.y;
        float xz = q.x * q.z;
        float yz = q.y * q.z;
        float wx = q.w * q.x;
        float wy = q.w * q.y;
        float wz = q.w * q.z;

        return Mat3(
            1.0f - 2.0f * (yy + zz), 2.0f * (xy - wz),        2.0f * (xz + wy),
            2.0f * (xy + wz),        1.0f - 2.0f * (xx + zz), 2.0f * (yz - wx),
            2.0f * (xz - wy),        2.0f * (yz + wx),        1.0f - 2.0f * (xx + yy)
        );
    }

    bool operator==(const Quat& rhs) const {
        return is_nearly_equal(w, rhs.w) &&
               is_nearly_equal(x, rhs.x) &&
               is_nearly_equal(y, rhs.y) &&
               is_nearly_equal(z, rhs.z);
    }

    bool operator!=(const Quat& rhs) const {
        return !(*this == rhs);
    }

    bool is_valid() const {
        return is_finite(w) && is_finite(x) && is_finite(y) && is_finite(z);
    }
};

inline Quat operator*(float scalar, const Quat& q) {
    return q * scalar;
}

} // namespace destruction::math

#endif // DESTRUCTION_QUAT_HPP
