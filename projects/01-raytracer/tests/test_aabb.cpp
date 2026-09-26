#include "raylab/aabb.hpp"
#include "raylab/ray.hpp"
#include <cassert>
#include <iostream>

using namespace raylab;

void test_aabb_obvious_hit() {
    AABB box(Point3(-1, -1, -1), Point3(1, 1, 1));
    Ray r(Point3(0, 0, -5), Vec3(0, 0, 1));
    assert(box.hit(r, 0.001, 1000.0) == true);
    std::cout << "[PASS] test_aabb_obvious_hit\n";
}

void test_aabb_obvious_miss() {
    AABB box(Point3(-1, -1, -1), Point3(1, 1, 1));
    Ray r(Point3(0, 5, -5), Vec3(0, 0, 1));
    assert(box.hit(r, 0.001, 1000.0) == false);
    std::cout << "[PASS] test_aabb_obvious_miss\n";
}

void test_aabb_inside_box() {
    AABB box(Point3(-1, -1, -1), Point3(1, 1, 1));
    Ray r(Point3(0, 0, 0), Vec3(1, 1, 1));
    assert(box.hit(r, 0.001, 1000.0) == true);
    std::cout << "[PASS] test_aabb_inside_box\n";
}

void test_aabb_negative_direction() {
    AABB box(Point3(-1, -1, -1), Point3(1, 1, 1));
    Ray r(Point3(0, 0, 5), Vec3(0, 0, -1));
    assert(box.hit(r, 0.001, 1000.0) == true);
    std::cout << "[PASS] test_aabb_negative_direction\n";
}

void test_aabb_parallel_ray_hit() {
    AABB box(Point3(-1, -1, -1), Point3(1, 1, 1));
    Ray r(Point3(0, 0.5, -5), Vec3(0, 0, 1)); // parallel to Z axis
    assert(box.hit(r, 0.001, 1000.0) == true);
    std::cout << "[PASS] test_aabb_parallel_ray_hit\n";
}

void test_aabb_parallel_ray_miss() {
    AABB box(Point3(-1, -1, -1), Point3(1, 1, 1));
    Ray r(Point3(0, 2.0, -5), Vec3(0, 0, 1)); // parallel to Z axis outside Y bounds
    assert(box.hit(r, 0.001, 1000.0) == false);
    std::cout << "[PASS] test_aabb_parallel_ray_miss\n";
}

void test_aabb_t_interval_clipping() {
    AABB box(Point3(-1, -1, -1), Point3(1, 1, 1));
    Ray r(Point3(0, 0, -5), Vec3(0, 0, 1));
    // Box is between z=-1 (t=4) and z=1 (t=6)
    // If tmax is 3.0, it should miss
    assert(box.hit(r, 0.001, 3.0) == false);
    // If tmin is 7.0, it should miss
    assert(box.hit(r, 7.0, 10.0) == false);
    // Valid interval covering t=4..6
    assert(box.hit(r, 3.0, 5.0) == true);
    std::cout << "[PASS] test_aabb_t_interval_clipping\n";
}

void test_aabb_union() {
    AABB box1(Point3(-1, -1, -1), Point3(0, 0, 0));
    AABB box2(Point3(0, 0, 0), Point3(2, 2, 2));
    AABB combined(box1, box2);
    assert(combined.min().x() == -1 && combined.min().y() == -1 && combined.min().z() == -1);
    assert(combined.max().x() == 2 && combined.max().y() == 2 && combined.max().z() == 2);
    assert(combined.longest_axis() == 0 || combined.longest_axis() == 1 || combined.longest_axis() == 2);
    std::cout << "[PASS] test_aabb_union\n";
}

int main() {
    test_aabb_obvious_hit();
    test_aabb_obvious_miss();
    test_aabb_inside_box();
    test_aabb_negative_direction();
    test_aabb_parallel_ray_hit();
    test_aabb_parallel_ray_miss();
    test_aabb_t_interval_clipping();
    test_aabb_union();
    std::cout << "All AABB tests passed!\n";
    return 0;
}
