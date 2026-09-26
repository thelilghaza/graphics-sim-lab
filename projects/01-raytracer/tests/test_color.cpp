#include "raylab/ppm_writer.hpp"
#include "raylab/vec3.hpp"
#include "test_harness.hpp"
#include <algorithm>
#include <cmath>

int main() {
    using namespace raylab;

    constexpr double eps = 1e-6;

    // Linear color input
    Color c_over(1.5, -0.2, 0.25);

    // Pipeline test: clamp -> sqrt -> 255.999 * val -> int
    double r_clamp = std::clamp(c_over.x(), 0.0, 1.0); // 1.0
    double g_clamp = std::clamp(c_over.y(), 0.0, 1.0); // 0.0
    double b_clamp = std::clamp(c_over.z(), 0.0, 1.0); // 0.25

    TEST_ASSERT_NEAR(r_clamp, 1.0, eps);
    TEST_ASSERT_NEAR(g_clamp, 0.0, eps);
    TEST_ASSERT_NEAR(b_clamp, 0.25, eps);

    double r_gamma = std::sqrt(r_clamp); // 1.0
    double g_gamma = std::sqrt(g_clamp); // 0.0
    double b_gamma = std::sqrt(b_clamp); // 0.5

    TEST_ASSERT_NEAR(r_gamma, 1.0, eps);
    TEST_ASSERT_NEAR(g_gamma, 0.0, eps);
    TEST_ASSERT_NEAR(b_gamma, 0.5, eps);

    int r_int = static_cast<int>(255.999 * r_gamma); // 255
    int g_int = static_cast<int>(255.999 * g_gamma); // 0
    int b_int = static_cast<int>(255.999 * b_gamma); // 127

    TEST_ASSERT(r_int == 255);
    TEST_ASSERT(g_int == 0);
    TEST_ASSERT(b_int == 127);

    return raylab::test::g_failures == 0 ? 0 : 1;
}
