#ifndef DESTRUCTION_VEC3_HPP
#define DESTRUCTION_VEC3_HPP

#include "destruction/math/math_utils.hpp"
#include <cmath>
#include <cassert>

namespace destruction::math {

struct Vec3 {
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};

    constexpr Vec3() = default;
    constexpr Vec3(float x_val, float y_val, float z_val) : x(x_val), y(y_val), z(z_val) {}

    static constexpr Vec3 zero() { return Vec3(0.0f, 0.0f, 0.0f); }
    static constexpr Vec3 unit_x() { return Vec3(1.0f, 0.0f, 0.0f); }
    static constexpr Vec3 unit_y() { return Vec3(0.0f, 1.0f, 0.0f); }
    static constexpr Vec3 unit_z() { return Vec3(0.0f, 0.0f, 1.0f); }

    float operator[](size_t index) const {
        assert(index < 3);
        if (index == 0) return x;
        if (index == 1) return y;
        return z;
    }

    float& operator[](size_t index) {
        assert(index < 3);
        if (index == 0) return x;
        if (index == 1) return y;
        return z;
    }

    Vec3 operator+(const Vec3& rhs) const {
        return Vec3(x + rhs.x, y + rhs.y, z + rhs.z);
    }

    Vec3 operator-(const Vec3& rhs) const {
        return Vec3(x - rhs.x, y - rhs.y, z - rhs.z);
    }

    Vec3 operator*(float scalar) const {
        return Vec3(x * scalar, y * scalar, z * scalar);
    }

    Vec3 operator/(float scalar) const {
        assert(scalar != 0.0f);
        float inv = 1.0f / scalar;
        return Vec3(x * inv, y * inv, z * inv);
    }

    Vec3 operator-() const {
        return Vec3(-x, -y, -z);
    }

    Vec3& operator+=(const Vec3& rhs) {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }

    Vec3& operator-=(const Vec3& rhs) {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }

    Vec3& operator*=(float scalar) {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }

    Vec3& operator/=(float scalar) {
        assert(scalar != 0.0f);
        float inv = 1.0f / scalar;
        x *= inv;
        y *= inv;
        z *= inv;
        return *this;
    }

    bool operator==(const Vec3& rhs) const {
        return is_nearly_equal(x, rhs.x) && is_nearly_equal(y, rhs.y) && is_nearly_equal(z, rhs.z);
    }

    bool operator!=(const Vec3& rhs) const {
        return !(*this == rhs);
    }

    float dot(const Vec3& rhs) const {
        return x * rhs.x + y * rhs.y + z * rhs.z;
    }

    Vec3 cross(const Vec3& rhs) const {
        return Vec3(
            y * rhs.z - z * rhs.y,
            z * rhs.x - x * rhs.z,
            x * rhs.y - y * rhs.x
        );
    }

    float length_sq() const {
        return x * x + y * y + z * z;
    }

    float length() const {
        return std::sqrt(length_sq());
    }

    Vec3 normalize() const {
        float len = length();
        if (len <= EPSILON) {
            return Vec3::zero();
        }
        return *this / len;
    }

    bool is_valid() const {
        return is_finite(x) && is_finite(y) && is_finite(z);
    }
};

inline Vec3 operator*(float scalar, const Vec3& vec) {
    return vec * scalar;
}

} // namespace destruction::math

#endif // DESTRUCTION_VEC3_HPP
