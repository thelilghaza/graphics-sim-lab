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

    auto mat = std::make_shared<Lambertian>(Color(1.0, 1.0, 1.0)); // Pure white
    HittableList world;
    world.add(std::make_shared<Sphere>(Point3(0.0, 0.0, -5.0), 1.0, mat));

    PointLight light_front(Point3(0.0, 0.0, 0.0), Color(1.0, 1.0, 1.0), 1.0);
    Ray r_front(Point3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, -1.0));

    RNG rng(42);
    // Use max_depth = 1 to test direct lighting in isolation (0 secondary bounces)
    Color c_front = ray_color(r_front, world, light_front, 0, 1, rng);

    // Front surface faces light => direct (1.0) + ambient (0.15) = 1.15
    TEST_ASSERT_NEAR(c_front.x(), 1.15, eps);
    TEST_ASSERT_NEAR(c_front.y(), 1.15, eps);
    TEST_ASSERT_NEAR(c_front.z(), 1.15, eps);

    // Light behind sphere
    PointLight light_behind(Point3(0.0, 0.0, -10.0), Color(1.0, 1.0, 1.0), 1.0);
    RNG rng2(42);
    Color c_back = ray_color(r_front, world, light_behind, 0, 1, rng2);

    // Front surface faces away from light => direct is 0, ambient = 0.15
    TEST_ASSERT_NEAR(c_back.x(), 0.15, eps);
    TEST_ASSERT_NEAR(c_back.y(), 0.15, eps);
    TEST_ASSERT_NEAR(c_back.z(), 0.15, eps);

    return raylab::test::g_failures == 0 ? 0 : 1;
}
