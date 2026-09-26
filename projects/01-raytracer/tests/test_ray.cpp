#include "raylab/ray.hpp"
#include "test_harness.hpp"

int main() {
    using namespace raylab;

    constexpr double eps = 1e-7;

    Point3 origin(1.0, 2.0, 3.0);
    Vec3 direction(0.0, 1.0, 0.0);
    Ray r(origin, direction);

    TEST_ASSERT_NEAR(r.origin().x(), 1.0, eps);
    TEST_ASSERT_NEAR(r.origin().y(), 2.0, eps);
    TEST_ASSERT_NEAR(r.origin().z(), 3.0, eps);

    TEST_ASSERT_NEAR(r.direction().x(), 0.0, eps);
    TEST_ASSERT_NEAR(r.direction().y(), 1.0, eps);
    TEST_ASSERT_NEAR(r.direction().z(), 0.0, eps);

    Point3 p0 = r.at(0.0);
    TEST_ASSERT_NEAR(p0.x(), 1.0, eps);
    TEST_ASSERT_NEAR(p0.y(), 2.0, eps);
    TEST_ASSERT_NEAR(p0.z(), 3.0, eps);

    Point3 p5 = r.at(5.0);
    TEST_ASSERT_NEAR(p5.x(), 1.0, eps);
    TEST_ASSERT_NEAR(p5.y(), 7.0, eps);
    TEST_ASSERT_NEAR(p5.z(), 3.0, eps);

    return raylab::test::g_failures == 0 ? 0 : 1;
}
