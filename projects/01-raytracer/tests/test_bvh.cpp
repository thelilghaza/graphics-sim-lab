#include "raylab/bvh_node.hpp"
#include "raylab/camera.hpp"
#include "raylab/hittable_list.hpp"
#include "raylab/material.hpp"
#include "raylab/ray.hpp"
#include "raylab/scene_builder.hpp"
#include "raylab/sphere.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <memory>

using namespace raylab;

inline void assert_hits_equal(const HitRecord& rec_naive, const HitRecord& rec_bvh) {
    constexpr double eps = 1e-9;
    (void)eps;
    (void)rec_naive;
    (void)rec_bvh;
    assert(std::abs(rec_naive.t - rec_bvh.t) < eps);
    assert((rec_naive.p - rec_bvh.p).length() < eps);
    assert((rec_naive.normal - rec_bvh.normal).length() < eps);
    assert(rec_naive.front_face == rec_bvh.front_face);
    assert(rec_naive.mat_ptr == rec_bvh.mat_ptr);
}

void test_bvh_construction_and_equivalence() {
    HittableList world;
    auto mat1 = std::make_shared<Lambertian>(Color(0.8, 0.3, 0.3));
    auto mat2 = std::make_shared<Metal>(Color(0.8, 0.8, 0.8), 0.1);
    auto mat3 = std::make_shared<Dielectric>(1.5);

    world.add(std::make_shared<Sphere>(Point3(0, 0, -1), 0.5, mat1));
    world.add(std::make_shared<Sphere>(Point3(0, -100.5, -1), 100, mat2));
    world.add(std::make_shared<Sphere>(Point3(1, 0, -1), 0.5, mat3));
    world.add(std::make_shared<Sphere>(Point3(-1, 0, -1), 0.5, mat1));

    BVHNode bvh_world(world);

    // Test a grid of rays
    for (int y = -10; y <= 10; ++y) {
        for (int x = -10; x <= 10; ++x) {
            Ray r(Point3(0, 0, 0), Vec3(x * 0.1, y * 0.1, -1));
            HitRecord rec_naive, rec_bvh;
            RenderStats stats_naive, stats_bvh;

            bool hit_naive = world.hit(r, 0.001, 1000.0, rec_naive, &stats_naive);
            bool hit_bvh = bvh_world.hit(r, 0.001, 1000.0, rec_bvh, &stats_bvh);
            (void)hit_bvh;

            assert(hit_naive == hit_bvh);
            if (hit_naive) {
                assert_hits_equal(rec_naive, rec_bvh);
            }
        }
    }
    std::cout << "[PASS] test_bvh_construction_and_equivalence\n";
}

void test_bvh_procedural_scene_equivalence() {
    SceneData scene = create_procedural_random_scene(42);
    BVHNode bvh_world(scene.world);
    Camera camera(Point3(13, 2, 3), Point3(0, 0, 0), Vec3(0, 1, 0), 20.0, 400, 225);

    // Test rays across scene using scene camera
    for (int j = 0; j < 225; j += 20) {
        for (int i = 0; i < 400; i += 40) {
            Ray r = camera.get_ray(i, j);
            HitRecord rec_naive, rec_bvh;
            RenderStats stats_naive, stats_bvh;

            bool hit_naive = scene.world.hit(r, 0.001, 1000.0, rec_naive, &stats_naive);
            bool hit_bvh = bvh_world.hit(r, 0.001, 1000.0, rec_bvh, &stats_bvh);
            (void)hit_bvh;

            assert(hit_naive == hit_bvh);
            if (hit_naive) {
                assert_hits_equal(rec_naive, rec_bvh);
                // Verify BVH performs fewer sphere tests than linear traversal
                assert(stats_bvh.sphere_intersection_tests <= stats_naive.sphere_intersection_tests);
            }
        }
    }
    std::cout << "[PASS] test_bvh_procedural_scene_equivalence\n";
}

int main() {
    test_bvh_construction_and_equivalence();
    test_bvh_procedural_scene_equivalence();
    std::cout << "All BVH tests passed!\n";
    return 0;
}
