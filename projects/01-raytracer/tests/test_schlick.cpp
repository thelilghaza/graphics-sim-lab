#include "raylab/material.hpp"
#include "test_harness.hpp"

int main() {
    using namespace raylab;

    constexpr double eps = 1e-5;

    double ref_idx = 1.0 / 1.5; // air to glass

    // 1. Normal incidence (cosine = 1.0)
    // r0 = (1 - 1/1.5)^2 / (1 + 1/1.5)^2 = (0.333/1.666)^2 = (0.2)^2 = 0.04
    double R_normal = Dielectric::schlick_reflectance(1.0, ref_idx);
    TEST_ASSERT_NEAR(R_normal, 0.04, eps);

    // 2. Grazing incidence (cosine = 0.0)
    // R(0) = r0 + (1 - r0) * (1 - 0)^5 = 1.0
    double R_grazing = Dielectric::schlick_reflectance(0.0, ref_idx);
    TEST_ASSERT_NEAR(R_grazing, 1.0, eps);

    // 3. Intermediate angle (cosine = 0.5, 60 degrees)
    // R(0.5) = 0.04 + (0.96) * (0.5)^5 = 0.04 + 0.96 * 0.03125 = 0.07
    double R_inter = Dielectric::schlick_reflectance(0.5, ref_idx);
    TEST_ASSERT_NEAR(R_inter, 0.07, eps);

    return raylab::test::g_failures == 0 ? 0 : 1;
}
