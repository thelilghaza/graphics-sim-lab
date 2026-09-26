#include "raylab/camera.hpp"
#include "test_harness.hpp"

int main() {
    using namespace raylab;

    constexpr double eps = 1e-4;

    // Milestone 1-3 backward compatibility pinhole constructor
    Camera cam(400, 200, 1.0, 2.0);

    // Origin check
    TEST_ASSERT_NEAR(cam.center().x(), 0.0, eps);
    TEST_ASSERT_NEAR(cam.center().y(), 0.0, eps);
    TEST_ASSERT_NEAR(cam.center().z(), 0.0, eps);

    // Center ray
    Ray center_ray = cam.get_ray(200, 100);
    Vec3 unit_dir = unit_vector(center_ray.direction());
    TEST_ASSERT_NEAR(unit_dir.z(), -1.0, 1e-3);

    // Top-left pixel ray (0, 0)
    Ray top_left_ray = cam.get_ray(0, 0);
    TEST_ASSERT(top_left_ray.direction().x() < 0.0);
    TEST_ASSERT(top_left_ray.direction().y() > 0.0);
    TEST_ASSERT(top_left_ray.direction().z() < 0.0);

    // Full orientation constructor matching defaults
    Camera cam_orient(Point3(0, 0, 0), Point3(0, 0, -1), Vec3(0, 1, 0), 90.0, 400, 200, 0.0, 1.0);
    Ray r_orient = cam_orient.get_ray(200, 100);
    TEST_ASSERT_NEAR(r_orient.direction().z(), -1.0, 1e-3);

    return raylab::test::g_failures == 0 ? 0 : 1;
}
