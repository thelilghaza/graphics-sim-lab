#pragma once

#include "raylab/scene_builder.hpp"

namespace raylab {

inline SceneData create_demo_scene() {
    SceneBuilder builder;

    // Materials
    auto ground_mat = std::make_shared<Lambertian>(Color(0.4, 0.7, 0.3));   // Soft green ground
    auto center_mat = std::make_shared<Lambertian>(Color(0.8, 0.2, 0.2));   // Terracotta red Lambertian
    auto glass_mat  = std::make_shared<Dielectric>(1.5);                    // Glass dielectric (IOR 1.5)
    auto mirror_mat = std::make_shared<Metal>(Color(0.85, 0.85, 0.9), 0.0); // Perfect mirror metal (fuzz = 0)
    auto rough_mat  = std::make_shared<Metal>(Color(0.9, 0.6, 0.2), 0.25);  // Brushed copper metal (fuzz = 0.25)

    // Spheres
    builder.add_sphere(Point3(0.0, -100.5, -1.0), 100.0, ground_mat)
           .add_sphere(Point3(0.0, 0.0, -1.2), 0.5, center_mat)
           .add_sphere(Point3(-1.1, 0.0, -1.0), 0.5, glass_mat)
           .add_sphere(Point3(1.1, 0.0, -1.0), 0.5, mirror_mat)
           .add_sphere(Point3(0.0, 0.65, -2.0), 0.45, rough_mat)
           .set_light(Point3(3.0, 5.0, 2.0), Color(1.0, 1.0, 1.0), 1.0);

    return builder.build();
}

} // namespace raylab
