#pragma once

#include "raylab/rng.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace raylab {

class Vec3 {
public:
    double e[3];

    constexpr Vec3() : e{0.0, 0.0, 0.0} {}
    constexpr Vec3(double e0, double e1, double e2) : e{e0, e1, e2} {}

    constexpr double x() const { return e[0]; }
    constexpr double y() const { return e[1]; }
    constexpr double z() const { return e[2]; }

    constexpr Vec3 operator-() const { return Vec3(-e[0], -e[1], -e[2]); }
    constexpr double operator[](int i) const { return e[i]; }
    constexpr double& operator[](int i) { return e[i]; }

    Vec3& operator+=(const Vec3& v) {
        e[0] += v.e[0];
        e[1] += v.e[1];
        e[2] += v.e[2];
        return *this;
    }

    Vec3& operator*=(double t) {
        e[0] *= t;
        e[1] *= t;
        e[2] *= t;
        return *this;
    }

    Vec3& operator/=(double t) {
        return *this *= 1.0 / t;
    }

    double length_squared() const {
        return e[0] * e[0] + e[1] * e[1] + e[2] * e[2];
    }

    double length() const {
        return std::sqrt(length_squared());
    }

    bool near_zero() const {
        constexpr double s = 1e-8;
        return (std::abs(e[0]) < s) && (std::abs(e[1]) < s) && (std::abs(e[2]) < s);
    }
};

// Type aliases for geometric clarity
using Point3 = Vec3;
using Color = Vec3;

// Vector Utility Functions
inline std::ostream& operator<<(std::ostream& out, const Vec3& v) {
    return out << v.e[0] << ' ' << v.e[1] << ' ' << v.e[2];
}

inline constexpr Vec3 operator+(const Vec3& u, const Vec3& v) {
    return Vec3(u.e[0] + v.e[0], u.e[1] + v.e[1], u.e[2] + v.e[2]);
}

inline constexpr Vec3 operator-(const Vec3& u, const Vec3& v) {
    return Vec3(u.e[0] - v.e[0], u.e[1] - v.e[1], u.e[2] - v.e[2]);
}

inline constexpr Vec3 operator*(const Vec3& u, const Vec3& v) {
    return Vec3(u.e[0] * v.e[0], u.e[1] * v.e[1], u.e[2] * v.e[2]);
}

inline constexpr Vec3 operator*(double t, const Vec3& v) {
    return Vec3(t * v.e[0], t * v.e[1], t * v.e[2]);
}

inline constexpr Vec3 operator*(const Vec3& v, double t) {
    return t * v;
}

inline constexpr Vec3 operator/(const Vec3& v, double t) {
    return (1.0 / t) * v;
}

inline constexpr double dot(const Vec3& u, const Vec3& v) {
    return u.e[0] * v.e[0] + u.e[1] * v.e[1] + u.e[2] * v.e[2];
}

inline constexpr Vec3 cross(const Vec3& u, const Vec3& v) {
    return Vec3(u.e[1] * v.e[2] - u.e[2] * v.e[1],
                u.e[2] * v.e[0] - u.e[0] * v.e[2],
                u.e[0] * v.e[1] - u.e[1] * v.e[0]);
}

inline Vec3 unit_vector(const Vec3& v) {
    return v / v.length();
}

inline Vec3 reflect(const Vec3& v, const Vec3& n) {
    return v - 2.0 * dot(v, n) * n;
}

inline Vec3 refract(const Vec3& uv, const Vec3& n, double etai_over_etat) {
    auto cos_theta = std::min(dot(-uv, n), 1.0);
    Vec3 r_out_perp = etai_over_etat * (uv + cos_theta * n);
    double r_out_parallel_sq = std::abs(1.0 - r_out_perp.length_squared());
    Vec3 r_out_parallel = -std::sqrt(r_out_parallel_sq) * n;
    return r_out_perp + r_out_parallel;
}

inline Vec3 random_unit_vector(RNG& rng) {
    for (int i = 0; i < 100; ++i) {
        Vec3 p(rng.next_double(-1.0, 1.0), rng.next_double(-1.0, 1.0), rng.next_double(-1.0, 1.0));
        double lensq = p.length_squared();
        if (1e-160 < lensq && lensq <= 1.0) {
            return p / std::sqrt(lensq);
        }
    }
    return Vec3(0.0, 1.0, 0.0);
}

} // namespace raylab
