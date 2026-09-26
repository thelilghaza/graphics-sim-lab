#pragma once

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

namespace raylab::test {

inline int g_failures = 0;

inline void check(bool condition, const std::string& expr, const char* file, int line) {
    if (!condition) {
        std::cerr << "FAILED: " << expr << " at " << file << ":" << line << "\n";
        g_failures++;
    }
}

inline void check_near(double a, double b, double eps, const std::string& msg, const char* file, int line) {
    if (std::abs(a - b) > eps) {
        std::cerr << "FAILED: " << msg << " (" << a << " vs " << b << ", delta: " << std::abs(a - b)
                  << " > " << eps << ") at " << file << ":" << line << "\n";
        g_failures++;
    }
}

} // namespace raylab::test

#define TEST_ASSERT(cond) ::raylab::test::check((cond), #cond, __FILE__, __LINE__)
#define TEST_ASSERT_NEAR(a, b, eps) ::raylab::test::check_near((a), (b), (eps), #a " approx " #b, __FILE__, __LINE__)
