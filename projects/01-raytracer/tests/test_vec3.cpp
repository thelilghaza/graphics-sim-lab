#include "raylab/vec3.hpp"
#include "test_harness.hpp"

int main() {
    using namespace raylab;

    constexpr double eps = 1e-7;

    // Construction and accessors
    Vec3 v1(1.0, 2.0, 3.0);
    TEST_ASSERT_NEAR(v1.x(), 1.0, eps);
    TEST_ASSERT_NEAR(v1.y(), 2.0, eps);
    TEST_ASSERT_NEAR(v1.z(), 3.0, eps);

    // Unary negation
    Vec3 neg = -v1;
    TEST_ASSERT_NEAR(neg.x(), -1.0, eps);
    TEST_ASSERT_NEAR(neg.y(), -2.0, eps);
    TEST_ASSERT_NEAR(neg.z(), -3.0, eps);

    // Addition & Subtraction
    Vec3 v2(4.0, 5.0, 6.0);
    Vec3 add = v1 + v2;
    TEST_ASSERT_NEAR(add.x(), 5.0, eps);
    TEST_ASSERT_NEAR(add.y(), 7.0, eps);
    TEST_ASSERT_NEAR(add.z(), 9.0, eps);

    Vec3 sub = v2 - v1;
    TEST_ASSERT_NEAR(sub.x(), 3.0, eps);
    TEST_ASSERT_NEAR(sub.y(), 3.0, eps);
    TEST_ASSERT_NEAR(sub.z(), 3.0, eps);

    // Scalar Multiplication & Division
    Vec3 mult = v1 * 2.5;
    TEST_ASSERT_NEAR(mult.x(), 2.5, eps);
    TEST_ASSERT_NEAR(mult.y(), 5.0, eps);
    TEST_ASSERT_NEAR(mult.z(), 7.5, eps);

    Vec3 div = v2 / 2.0;
    TEST_ASSERT_NEAR(div.x(), 2.0, eps);
    TEST_ASSERT_NEAR(div.y(), 2.5, eps);
    TEST_ASSERT_NEAR(div.z(), 3.0, eps);

    // Length & Length Squared
    Vec3 v3(3.0, 4.0, 0.0);
    TEST_ASSERT_NEAR(v3.length_squared(), 25.0, eps);
    TEST_ASSERT_NEAR(v3.length(), 5.0, eps);

    // Normalization
    Vec3 u = unit_vector(v3);
    TEST_ASSERT_NEAR(u.length(), 1.0, eps);
    TEST_ASSERT_NEAR(u.x(), 0.6, eps);
    TEST_ASSERT_NEAR(u.y(), 0.8, eps);
    TEST_ASSERT_NEAR(u.z(), 0.0, eps);

    // Dot product
    double d = dot(v1, v2); // 1*4 + 2*5 + 3*6 = 4 + 10 + 18 = 32
    TEST_ASSERT_NEAR(d, 32.0, eps);

    // Cross product
    Vec3 i(1.0, 0.0, 0.0);
    Vec3 j(0.0, 1.0, 0.0);
    Vec3 k = cross(i, j);
    TEST_ASSERT_NEAR(k.x(), 0.0, eps);
    TEST_ASSERT_NEAR(k.y(), 0.0, eps);
    TEST_ASSERT_NEAR(k.z(), 1.0, eps);

    return raylab::test::g_failures == 0 ? 0 : 1;
}
