#pragma once

#include <cmath>
#include <cstddef>

namespace voxel_lab {

struct Vec3 {
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};

    constexpr Vec3() = default;
    constexpr Vec3(float vx, float vy, float vz) : x(vx), y(vy), z(vz) {}

    constexpr Vec3 operator+(const Vec3& o) const noexcept { return {x + o.x, y + o.y, z + o.z}; }
    constexpr Vec3 operator-(const Vec3& o) const noexcept { return {x - o.x, y - o.y, z - o.z}; }
    constexpr Vec3 operator*(float s) const noexcept { return {x * s, y * s, z * s}; }
    constexpr Vec3 operator/(float s) const noexcept { return {x / s, y / s, z / s}; }

    constexpr Vec3& operator+=(const Vec3& o) noexcept { x += o.x; y += o.y; z += o.z; return *this; }
    constexpr Vec3& operator-=(const Vec3& o) noexcept { x -= o.x; y -= o.y; z -= o.z; return *this; }
    constexpr Vec3& operator*=(float s) noexcept { x *= s; y *= s; z *= s; return *this; }

    constexpr bool operator==(const Vec3& o) const noexcept { return x == o.x && y == o.y && z == o.z; }
    constexpr bool operator!=(const Vec3& o) const noexcept { return !(*this == o); }
};

inline constexpr float dot(const Vec3& a, const Vec3& b) noexcept {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline constexpr Vec3 cross(const Vec3& a, const Vec3& b) noexcept {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

inline float length(const Vec3& v) noexcept {
    return std::sqrt(dot(v, v));
}

inline Vec3 normalize(const Vec3& v) noexcept {
    float len = length(v);
    if (len > 0.00001f) {
        return v * (1.0f / len);
    }
    return {0.0f, 0.0f, 0.0f};
}

// 4x4 Matrix stored in column-major order (OpenGL standard)
struct Mat4 {
    float m[16]{};

    static constexpr Mat4 identity() noexcept {
        Mat4 r{};
        r.m[0] = 1.0f;  r.m[5] = 1.0f;  r.m[10] = 1.0f; r.m[15] = 1.0f;
        return r;
    }

    static Mat4 perspective(float fov_radians, float aspect, float z_near, float z_far) noexcept {
        Mat4 r{};
        float tan_half = std::tan(fov_radians * 0.5f);
        r.m[0] = 1.0f / (aspect * tan_half);
        r.m[5] = 1.0f / tan_half;
        r.m[10] = -(z_far + z_near) / (z_far - z_near);
        r.m[11] = -1.0f;
        r.m[14] = -(2.0f * z_far * z_near) / (z_far - z_near);
        return r;
    }

    static Mat4 look_at(const Vec3& eye, const Vec3& target, const Vec3& up) noexcept {
        Vec3 f = normalize(target - eye);
        Vec3 s = normalize(cross(f, up));
        Vec3 u = cross(s, f);

        Mat4 r = identity();
        r.m[0] = s.x;
        r.m[4] = s.y;
        r.m[8] = s.z;
        r.m[1] = u.x;
        r.m[5] = u.y;
        r.m[9] = u.z;
        r.m[2] = -f.x;
        r.m[6] = -f.y;
        r.m[10] = -f.z;
        r.m[12] = -dot(s, eye);
        r.m[13] = -dot(u, eye);
        r.m[14] = dot(f, eye);
        return r;
    }

    static Mat4 translate(const Vec3& t) noexcept {
        Mat4 r = identity();
        r.m[12] = t.x;
        r.m[13] = t.y;
        r.m[14] = t.z;
        return r;
    }

    const float* data() const noexcept { return m; }
};

} // namespace voxel_lab
