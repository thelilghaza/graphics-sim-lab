#include "raylab/material.hpp"
#include "test_harness.hpp"

int main() {
    using namespace raylab;

    constexpr double eps = 1e-7;

    Color albedo(0.7, 0.4, 0.1);
    Lambertian mat(albedo);

    TEST_ASSERT_NEAR(mat.albedo().x(), 0.7, eps);
    TEST_ASSERT_NEAR(mat.albedo().y(), 0.4, eps);
    TEST_ASSERT_NEAR(mat.albedo().z(), 0.1, eps);

    return raylab::test::g_failures == 0 ? 0 : 1;
}
