#include "raylab/material.hpp"
#include "raylab/rng.hpp"
#include "raylab/vec3.hpp"
#include "test_harness.hpp"

int main() {
    using namespace raylab;

    constexpr double eps = 1e-5;

    // 1. Perpendicular incidence refraction: ray going straight down (0, -1, 0)
    Vec3 uv(0.0, -1.0, 0.0);
    Vec3 n(0.0, 1.0, 0.0);
    Vec3 refr = refract(uv, n, 1.0 / 1.5);
    TEST_ASSERT_NEAR(refr.x(), 0.0, eps);
    TEST_ASSERT_NEAR(refr.y(), -1.0, eps);
    TEST_ASSERT_NEAR(refr.z(), 0.0, eps);

    // 2. Oblique incidence Snell's Law test: air -> glass (eta = 1.0 / 1.5)
    // Angle of incidence = 45 deg => sin(45) = 0.7071
    // sin(theta_t) = (1.0 / 1.5) * sin(45) = 0.4714
    Vec3 uv_45 = unit_vector(Vec3(1.0, -1.0, 0.0));
    Vec3 refr_45 = refract(uv_45, n, 1.0 / 1.5);
    TEST_ASSERT_NEAR(refr_45.x(), 0.4714045, 1e-4);

    // 3. Dielectric entering vs exiting scatter test
    Dielectric glass(1.5);
    ScatterRecord srec;
    RNG rng(42);

    // Entering (front_face = true)
    HitRecord rec_enter;
    rec_enter.p = Point3(0, 0, 0);
    rec_enter.normal = Vec3(0, 1, 0);
    rec_enter.front_face = true;
    Ray r_enter(Point3(0, 1, 0), unit_vector(Vec3(0, -1, 0)));

    TEST_ASSERT(glass.scatter(r_enter, rec_enter, srec, rng));
    TEST_ASSERT(srec.is_specular);

    // 4. Total Internal Reflection test: glass -> air (eta = 1.5), angle > critical angle (~41.8 deg)
    // Ray inside glass hitting boundary at 60 deg angle of incidence
    // sin(60) = 0.866 => eta * sin(60) = 1.5 * 0.866 = 1.299 > 1.0 => Total Internal Reflection!
    HitRecord rec_exit;
    rec_exit.p = Point3(0, 0, 0);
    rec_exit.normal = Vec3(0, -1, 0); // normal pointing into glass
    rec_exit.front_face = false;     // ray is inside glass

    Vec3 dir_tir = unit_vector(Vec3(std::sqrt(3.0) / 2.0, -0.5, 0.0)); // 60 deg relative to normal (0, -1, 0)
    Ray r_tir(Point3(0, 0.5, 0), dir_tir);

    TEST_ASSERT(glass.scatter(r_tir, rec_exit, srec, rng));
    // Due to TIR, scattered ray MUST be reflected!
    TEST_ASSERT(srec.scattered_ray.direction().y() > 0.0);

    return raylab::test::g_failures == 0 ? 0 : 1;
}
