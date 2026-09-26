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

    // Two parallel mirrors facing each other
    auto mirror = std::make_shared<Metal>(Color(1.0, 1.0, 1.0), 0.0);
    HittableList world;
    world.add(std::make_shared<Sphere>(Point3(0.0, 0.0, -2.0), 1.0, mirror));
    world.add(std::make_shared<Sphere>(Point3(0.0, 0.0, 2.0), 1.0, mirror));

    PointLight light(Point3(0.0, 5.0, 0.0), Color(1.0, 1.0, 1.0), 1.0);
    Ray r_pingpong(Point3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, -1.0));
    RNG rng(42);

    // 1. Max depth = 1 terminates after primary ray
    Color c_depth1 = ray_color(r_pingpong, world, light, 0, 1, rng);

    // 2. Max depth = 5 terminates cleanly without stack overflow or infinite loop
    Color c_depth5 = ray_color(r_pingpong, world, light, 0, 5, rng);

    // 3. Max depth = 10 terminates cleanly
    Color c_depth10 = ray_color(r_pingpong, world, light, 0, 10, rng);

    // Verify recursion depth changes contribution (returns 0 color at terminal depth)
    TEST_ASSERT(c_depth1.x() == 0.0 || c_depth5.x() != c_depth1.x());

    // 4. Robustness check for near-zero scatter direction in Lambertian
    Lambertian lamb(Color(0.5, 0.5, 0.5));
    HitRecord rec;
    rec.p = Point3(0, 0, 0);
    rec.normal = Vec3(0, 1, 0);
    rec.front_face = true;
    ScatterRecord srec;

    // We force a test of Lambertian scatter
    TEST_ASSERT(lamb.scatter(r_pingpong, rec, srec, rng));
    TEST_ASSERT(!srec.scattered_ray.direction().near_zero());

    return raylab::test::g_failures == 0 ? 0 : 1;
}
