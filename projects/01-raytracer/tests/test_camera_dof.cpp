#include "raylab/camera.hpp"
#include "raylab/rng.hpp"
#include "test_harness.hpp"

int main() {
    using namespace raylab;

    constexpr double eps = 1e-6;

    // 1. Aperture = 0 pinhole regression check
    Camera pinhole(Point3(0, 0, 0), Point3(0, 0, -1), Vec3(0, 1, 0), 90.0, 400, 200, 0.0, 1.0);
    TEST_ASSERT_NEAR(pinhole.aperture(), 0.0, eps);
    TEST_ASSERT_NEAR(pinhole.lens_radius(), 0.0, eps);

    RNG rng1(42);
    Ray r_pinhole = pinhole.get_ray(200, 100, 0.5, 0.5, rng1);

    // Pinhole ray origin MUST be exactly camera_center
    TEST_ASSERT_NEAR(r_pinhole.origin().x(), 0.0, eps);
    TEST_ASSERT_NEAR(r_pinhole.origin().y(), 0.0, eps);
    TEST_ASSERT_NEAR(r_pinhole.origin().z(), 0.0, eps);

    // 2. Aperture = 0.4 thin-lens DOF check
    Camera dof_cam(Point3(0, 0, 0), Point3(0, 0, -1), Vec3(0, 1, 0), 90.0, 400, 200, 0.4, 2.0);
    TEST_ASSERT_NEAR(dof_cam.aperture(), 0.4, eps);
    TEST_ASSERT_NEAR(dof_cam.lens_radius(), 0.2, eps);
    TEST_ASSERT_NEAR(dof_cam.focus_distance(), 2.0, eps);

    RNG rng2(42);
    Ray r_dof = dof_cam.get_ray(200, 100, 0.5, 0.5, rng2);

    // Ray origin for thin-lens camera MUST be offset on the lens disk (radius <= 0.2)
    double lens_dist = (r_dof.origin() - dof_cam.center()).length();
    TEST_ASSERT(lens_dist <= 0.2);
    TEST_ASSERT(lens_dist > 0.0);

    return raylab::test::g_failures == 0 ? 0 : 1;
}
