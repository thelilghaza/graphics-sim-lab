#ifndef DESTRUCTION_MAT3_HPP
#define DESTRUCTION_MAT3_HPP

#include "destruction/math/vec3.hpp"
#include "destruction/math/math_utils.hpp"
#include <cmath>
#include <cassert>

namespace destruction::math {

struct Mat3 {
    // Row-major storage: m[row * 3 + col]
    float m[9]{
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f
    };

    constexpr Mat3() = default;

    constexpr Mat3(
        float m00, float m01, float m02,
        float m10, float m11, float m12,
        float m20, float m21, float m22
    ) : m{m00, m01, m02, m10, m11, m12, m20, m21, m22} {}

    static constexpr Mat3 identity() {
        return Mat3(
            1.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 1.0f
        );
    }

    static constexpr Mat3 zero() {
        return Mat3(
            0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f
        );
    }

    static constexpr Mat3 diagonal(float d0, float d1, float d2) {
        return Mat3(
            d0,   0.0f, 0.0f,
            0.0f, d1,   0.0f,
            0.0f, 0.0f, d2
        );
    }

    static constexpr Mat3 diagonal(const Vec3& diag) {
        return diagonal(diag.x, diag.y, diag.z);
    }

    static Mat3 rotation_x(float angle_rad) {
        float c = std::cos(angle_rad);
        float s = std::sin(angle_rad);
        return Mat3(
            1.0f, 0.0f, 0.0f,
            0.0f, c,    -s,
            0.0f, s,    c
        );
    }

    static Mat3 rotation_y(float angle_rad) {
        float c = std::cos(angle_rad);
        float s = std::sin(angle_rad);
        return Mat3(
            c,    0.0f, s,
            0.0f, 1.0f, 0.0f,
            -s,   0.0f, c
        );
    }

    static Mat3 rotation_z(float angle_rad) {
        float c = std::cos(angle_rad);
        float s = std::sin(angle_rad);
        return Mat3(
            c,    -s,   0.0f,
            s,    c,    0.0f,
            0.0f, 0.0f, 1.0f
        );
    }

    float operator()(size_t row, size_t col) const {
        assert(row < 3 && col < 3);
        return m[row * 3 + col];
    }

    float& operator()(size_t row, size_t col) {
        assert(row < 3 && col < 3);
        return m[row * 3 + col];
    }

    Mat3 operator+(const Mat3& rhs) const {
        Mat3 res;
        for (size_t i = 0; i < 9; ++i) {
            res.m[i] = m[i] + rhs.m[i];
        }
        return res;
    }

    Mat3 operator-(const Mat3& rhs) const {
        Mat3 res;
        for (size_t i = 0; i < 9; ++i) {
            res.m[i] = m[i] - rhs.m[i];
        }
        return res;
    }

    Mat3 operator*(float scalar) const {
        Mat3 res;
        for (size_t i = 0; i < 9; ++i) {
            res.m[i] = m[i] * scalar;
        }
        return res;
    }

    Mat3 operator*(const Mat3& rhs) const {
        Mat3 res = Mat3::zero();
        for (size_t i = 0; i < 3; ++i) {
            for (size_t j = 0; j < 3; ++j) {
                float sum = 0.0f;
                for (size_t k = 0; k < 3; ++k) {
                    sum += (*this)(i, k) * rhs(k, j);
                }
                res(i, j) = sum;
            }
        }
        return res;
    }

    Vec3 operator*(const Vec3& rhs) const {
        return Vec3(
            m[0] * rhs.x + m[1] * rhs.y + m[2] * rhs.z,
            m[3] * rhs.x + m[4] * rhs.y + m[5] * rhs.z,
            m[6] * rhs.x + m[7] * rhs.y + m[8] * rhs.z
        );
    }

    Mat3 transpose() const {
        return Mat3(
            m[0], m[3], m[6],
            m[1], m[4], m[7],
            m[2], m[5], m[8]
        );
    }

    float determinant() const {
        return m[0] * (m[4] * m[8] - m[5] * m[7]) -
               m[1] * (m[3] * m[8] - m[5] * m[6]) +
               m[2] * (m[3] * m[7] - m[4] * m[6]);
    }

    Mat3 inverse(bool* success = nullptr) const {
        float det = determinant();
        if (std::abs(det) < EPSILON) {
            if (success) *success = false;
            return Mat3::zero();
        }

        float inv_det = 1.0f / det;
        Mat3 inv;
        inv(0, 0) = (m[4] * m[8] - m[5] * m[7]) * inv_det;
        inv(0, 1) = (m[2] * m[7] - m[1] * m[8]) * inv_det;
        inv(0, 2) = (m[1] * m[5] - m[2] * m[4]) * inv_det;

        inv(1, 0) = (m[5] * m[6] - m[3] * m[8]) * inv_det;
        inv(1, 1) = (m[0] * m[8] - m[2] * m[6]) * inv_det;
        inv(1, 2) = (m[2] * m[3] - m[0] * m[5]) * inv_det;

        inv(2, 0) = (m[3] * m[7] - m[4] * m[6]) * inv_det;
        inv(2, 1) = (m[1] * m[6] - m[0] * m[7]) * inv_det;
        inv(2, 2) = (m[0] * m[4] - m[1] * m[3]) * inv_det;

        if (success) *success = true;
        return inv;
    }

    bool operator==(const Mat3& rhs) const {
        for (size_t i = 0; i < 9; ++i) {
            if (!is_nearly_equal(m[i], rhs.m[i])) return false;
        }
        return true;
    }

    bool operator!=(const Mat3& rhs) const {
        return !(*this == rhs);
    }

    bool is_valid() const {
        for (size_t i = 0; i < 9; ++i) {
            if (!is_finite(m[i])) return false;
        }
        return true;
    }
};

inline Mat3 operator*(float scalar, const Mat3& mat) {
    return mat * scalar;
}

} // namespace destruction::math

#endif // DESTRUCTION_MAT3_HPP
