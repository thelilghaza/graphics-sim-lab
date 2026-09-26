#include "raylab/hittable_list.hpp"
#include "raylab/light.hpp"
#include "raylab/material.hpp"
#include "raylab/raytracer.hpp"
#include "raylab/rng.hpp"
#include "raylab/sphere.hpp"
#include "test_harness.hpp"
#include <memory>

int main() {
    using namespace raylab;

    constexpr double eps = 1e-4;

    auto white_mat = std::make_shared<Lambertian>(Color(1.0, 1.0, 1.0));

    HittableList world;
    auto target_sphere = std::make_shared<Sphere>(Point3(0.0, 0.0, -10.0), 3.0, white_mat);
    auto blocker_sphere = std::make_shared<Sphere>(Point3(0.0, 0.0, -4.0), 1.0, white_mat);

    world.add(target_sphere);
    world.add(blocker_sphere);

    PointLight light(Point3(0.0, 0.0, 0.0), Color(1.0, 1.0, 1.0), 1.0);

    // Unoccluded ray (max_depth = 1 for direct light test)
    Ray r_unoccluded(Point3(2.5, 0.0, 0.0), Vec3(0.0, 0.0, -1.0));
    RNG rng1(42);
    Color c_unoccluded = ray_color(r_unoccluded, world, light, 0, 1, rng1);
    TEST_ASSERT(c_unoccluded.x() > 0.3);

    // Occluded ray (max_depth = 1)
    Ray r_shadowed(Point3(0.0, 0.0, -6.0), Vec3(0.0, 0.0, -1.0));
    RNG rng2(42);
    Color c_shadowed = ray_color(r_shadowed, world, light, 0, 1, rng2);

    // Direct lighting in shadow receives ambient light only (0.15)
    TEST_ASSERT_NEAR(c_shadowed.x(), 0.15, eps);
    TEST_ASSERT_NEAR(c_shadowed.y(), 0.15, eps);
    TEST_ASSERT_NEAR(c_shadowed.z(), 0.15, eps);

    return raylab::test::g_failures == 0 ? 0 : 1;
}
