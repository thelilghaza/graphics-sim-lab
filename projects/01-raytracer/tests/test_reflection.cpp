#include "raylab/material.hpp"
#include "raylab/rng.hpp"
#include "raylab/vec3.hpp"
#include "test_harness.hpp"

int main() {
    using namespace raylab;

    constexpr double eps = 1e-6;

    // 1. Vector reflection math test
    Vec3 v_in(1.0, -1.0, 0.0);
    Vec3 n(0.0, 1.0, 0.0);
    Vec3 refl = reflect(v_in, n);
    TEST_ASSERT_NEAR(refl.x(), 1.0, eps);
    TEST_ASSERT_NEAR(refl.y(), 1.0, eps);
    TEST_ASSERT_NEAR(refl.z(), 0.0, eps);

    // 2. Perfect mirror Metal (fuzz = 0)
    Metal mirror(Color(0.9, 0.9, 0.9), 0.0);
    TEST_ASSERT_NEAR(mirror.fuzz(), 0.0, eps);

    HitRecord rec;
    rec.p = Point3(0, 0, 0);
    rec.normal = Vec3(0, 1, 0);
    rec.front_face = true;

    Ray r_incoming(Point3(0, 1, 0), unit_vector(Vec3(1, -1, 0)));
    ScatterRecord srec;
    RNG rng(42);

    TEST_ASSERT(mirror.scatter(r_incoming, rec, srec, rng));
    TEST_ASSERT(srec.is_specular);
    TEST_ASSERT_NEAR(srec.scattered_ray.direction().x(), 1.0 / std::sqrt(2.0), 1e-4);
    TEST_ASSERT_NEAR(srec.scattered_ray.direction().y(), 1.0 / std::sqrt(2.0), 1e-4);

    // 3. Rough Metal (fuzz > 0)
    Metal rough(Color(0.9, 0.9, 0.9), 0.25);
    ScatterRecord srec_rough;
    RNG rng_rough(42);
    TEST_ASSERT(rough.scatter(r_incoming, rec, srec_rough, rng_rough));
    // Perturbed direction differs from exact mirror direction
    TEST_ASSERT(srec_rough.scattered_ray.direction().x() != srec.scattered_ray.direction().x());

    // 4. Out-of-bounds fuzz clamping test
    Metal clamped_metal(Color(1, 1, 1), 2.5);
    TEST_ASSERT_NEAR(clamped_metal.fuzz(), 1.0, eps);

    return raylab::test::g_failures == 0 ? 0 : 1;
}
