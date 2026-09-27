#ifndef DESTRUCTION_VEC2_HPP
#define DESTRUCTION_VEC2_HPP

#include "destruction/math/math_utils.hpp"
#include <cmath>
#include <cassert>

namespace destruction::math {

struct Vec2 {
    float x{0.0f};
    float y{0.0f};

    constexpr Vec2() = default;
    constexpr Vec2(float x_val, float y_val) : x(x_val), y(y_val) {}

    static constexpr Vec2 zero() { return Vec2(0.0f, 0.0f); }
    static constexpr Vec2 unit_x() { return Vec2(1.0f, 0.0f); }
    static constexpr Vec2 unit_y() { return Vec2(0.0f, 1.0f); }

    float operator[](size_t index) const {
        assert(index < 2);
        return (index == 0) ? x : y;
    }

    float& operator[](size_t index) {
        assert(index < 2);
        return (index == 0) ? x : y;
    }

    Vec2 operator+(const Vec2& rhs) const {
        return Vec2(x + rhs.x, y + rhs.y);
    }

    Vec2 operator-(const Vec2& rhs) const {
        return Vec2(x - rhs.x, y - rhs.y);
    }

    Vec2 operator*(float scalar) const {
        return Vec2(x * scalar, y * scalar);
    }

    Vec2 operator/(float scalar) const {
        assert(scalar != 0.0f);
        float inv = 1.0f / scalar;
        return Vec2(x * inv, y * inv);
    }

    Vec2 operator-() const {
        return Vec2(-x, -y);
    }

    Vec2& operator+=(const Vec2& rhs) {
        x += rhs.x;
        y += rhs.y;
        return *this;
    }

    Vec2& operator-=(const Vec2& rhs) {
        x -= rhs.x;
        y -= rhs.y;
        return *this;
    }

    Vec2& operator*=(float scalar) {
        x *= scalar;
        y *= scalar;
        return *this;
    }

    Vec2& operator/=(float scalar) {
        assert(scalar != 0.0f);
        float inv = 1.0f / scalar;
        x *= inv;
        y *= inv;
        return *this;
    }

    bool operator==(const Vec2& rhs) const {
        return is_nearly_equal(x, rhs.x) && is_nearly_equal(y, rhs.y);
    }

    bool operator!=(const Vec2& rhs) const {
        return !(*this == rhs);
    }

    float dot(const Vec2& rhs) const {
        return x * rhs.x + y * rhs.y;
    }

    float cross(const Vec2& rhs) const {
        return x * rhs.y - y * rhs.x;
    }

    float length_sq() const {
        return x * x + y * y;
    }

    float length() const {
        return std::sqrt(length_sq());
    }

    Vec2 normalize() const {
        float len = length();
        if (len <= EPSILON) {
            return Vec2::zero();
        }
        return *this / len;
    }

    bool is_valid() const {
        return is_finite(x) && is_finite(y);
    }
};

inline Vec2 operator*(float scalar, const Vec2& vec) {
    return vec * scalar;
}

} // namespace destruction::math

#endif // DESTRUCTION_VEC2_HPP
