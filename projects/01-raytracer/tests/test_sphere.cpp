#include "raylab/material.hpp"
#include "raylab/sphere.hpp"
#include "test_harness.hpp"
#include <memory>

int main() {
    using namespace raylab;

    constexpr double eps = 1e-7;

    auto mat = std::make_shared<Lambertian>(Color(0.8, 0.3, 0.3));
    Sphere s(Point3(0.0, 0.0, -5.0), 1.0, mat);
    HitRecord rec;

    // 1. Obvious Hit (head-on)
    Ray ray_hit(Point3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, -1.0));
    TEST_ASSERT(s.hit(ray_hit, 0.001, 100.0, rec));
    TEST_ASSERT_NEAR(rec.t, 4.0, eps);
    TEST_ASSERT_NEAR(rec.p.z(), -4.0, eps);
    TEST_ASSERT_NEAR(rec.normal.z(), 1.0, eps);
    TEST_ASSERT(rec.front_face);
    TEST_ASSERT(rec.mat_ptr != nullptr);
    TEST_ASSERT_NEAR(rec.mat_ptr->albedo().x(), 0.8, eps);

    // 2. Obvious Miss
    Ray ray_miss(Point3(0.0, 0.0, 0.0), Vec3(1.0, 1.0, 0.0));
    TEST_ASSERT(!s.hit(ray_miss, 0.001, 100.0, rec));

    // 3. Tangent Hit
    Ray ray_tangent(Point3(0.0, 1.0, 0.0), Vec3(0.0, 0.0, -1.0));
    TEST_ASSERT(s.hit(ray_tangent, 0.001, 100.0, rec));
    TEST_ASSERT_NEAR(rec.t, 5.0, eps);

    // 4. Interval Clipping
    TEST_ASSERT(!s.hit(ray_hit, 0.001, 3.5, rec));

    return raylab::test::g_failures == 0 ? 0 : 1;
}
