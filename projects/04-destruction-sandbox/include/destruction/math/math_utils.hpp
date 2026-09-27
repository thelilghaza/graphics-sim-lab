#ifndef DESTRUCTION_MATH_UTILS_HPP
#define DESTRUCTION_MATH_UTILS_HPP

#include <cmath>
#include <limits>

namespace destruction::math {

constexpr float PI = 3.14159265358979323846f;
constexpr float TWO_PI = 6.28318530717958647692f;
constexpr float HALF_PI = 1.57079632679489661923f;
constexpr float DEG_TO_RAD = PI / 180.0f;
constexpr float RAD_TO_DEG = 180.0f / PI;
constexpr float EPSILON = 1e-6f;

inline bool is_finite(float val) {
    return std::isfinite(val);
}

inline bool is_nearly_zero(float val, float tolerance = EPSILON) {
    return std::abs(val) <= tolerance;
}

inline bool is_nearly_equal(float a, float b, float tolerance = EPSILON) {
    return std::abs(a - b) <= tolerance;
}

inline float radians(float degrees) {
    return degrees * DEG_TO_RAD;
}

inline float degrees(float rad) {
    return rad * RAD_TO_DEG;
}

} // namespace destruction::math

#endif // DESTRUCTION_MATH_UTILS_HPP
