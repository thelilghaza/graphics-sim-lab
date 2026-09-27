#ifndef DESTRUCTION_VEC4_HPP
#define DESTRUCTION_VEC4_HPP

#include "destruction/math/math_utils.hpp"
#include <cmath>
#include <cassert>

namespace destruction::math {

struct Vec4 {
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
    float w{0.0f};

    constexpr Vec4() = default;
    constexpr Vec4(float x_val, float y_val, float z_val, float w_val)
        : x(x_val), y(y_val), z(z_val), w(w_val) {}

    static constexpr Vec4 zero() { return Vec4(0.0f, 0.0f, 0.0f, 0.0f); }

    float operator[](size_t index) const {
        assert(index < 4);
        if (index == 0) return x;
        if (index == 1) return y;
        if (index == 2) return z;
        return w;
    }

    float& operator[](size_t index) {
        assert(index < 4);
        if (index == 0) return x;
        if (index == 1) return y;
        if (index == 2) return z;
        return w;
    }

    Vec4 operator+(const Vec4& rhs) const {
        return Vec4(x + rhs.x, y + rhs.y, z + rhs.z, w + rhs.w);
    }

    Vec4 operator-(const Vec4& rhs) const {
        return Vec4(x - rhs.x, y - rhs.y, z - rhs.z, w - rhs.w);
    }

    Vec4 operator*(float scalar) const {
        return Vec4(x * scalar, y * scalar, z * scalar, w * scalar);
    }

    Vec4 operator/(float scalar) const {
        assert(scalar != 0.0f);
        float inv = 1.0f / scalar;
        return Vec4(x * inv, y * inv, z * inv, w * inv);
    }

    float dot(const Vec4& rhs) const {
        return x * rhs.x + y * rhs.y + z * rhs.z + w * rhs.w;
    }

    float length_sq() const {
        return x * x + y * y + z * z + w * w;
    }

    float length() const {
        return std::sqrt(length_sq());
    }

    Vec4 normalize() const {
        float len = length();
        if (len <= EPSILON) {
            return Vec4::zero();
        }
        return *this / len;
    }

    bool is_valid() const {
        return is_finite(x) && is_finite(y) && is_finite(z) && is_finite(w);
    }
};

inline Vec4 operator*(float scalar, const Vec4& vec) {
    return vec * scalar;
}

} // namespace destruction::math

#endif // DESTRUCTION_VEC4_HPP
