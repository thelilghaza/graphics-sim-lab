#pragma once

#include "raylab/hittable_list.hpp"
#include "raylab/light.hpp"
#include "raylab/material.hpp"
#include "raylab/sphere.hpp"

#include <memory>

namespace raylab {

struct SceneData {
    HittableList world;
    PointLight light{Point3(3.0, 5.0, 2.0), Color(1.0, 1.0, 1.0), 1.0};
};

class SceneBuilder {
public:
    SceneBuilder() = default;

    SceneBuilder& add_sphere(const Point3& center, double radius, std::shared_ptr<Material> mat) {
        data.world.add(std::make_shared<Sphere>(center, radius, mat));
        return *this;
    }

    SceneBuilder& add_diffuse(const Point3& center, double radius, const Color& albedo) {
        return add_sphere(center, radius, std::make_shared<Lambertian>(albedo));
    }

    SceneBuilder& add_metal(const Point3& center, double radius, const Color& albedo, double fuzz) {
        return add_sphere(center, radius, std::make_shared<Metal>(albedo, fuzz));
    }

    SceneBuilder& add_dielectric(const Point3& center, double radius, double ior) {
        return add_sphere(center, radius, std::make_shared<Dielectric>(ior));
    }

    SceneBuilder& set_light(const Point3& pos, const Color& col = Color(1, 1, 1), double intensity = 1.0) {
        data.light = PointLight(pos, col, intensity);
        return *this;
    }

    SceneData build() const { return data; }

private:
    SceneData data;
};

// Deterministic procedural scene generator
inline SceneData create_procedural_random_scene(uint64_t seed) {
    RNG rng(seed);
    SceneBuilder builder;

    // Ground sphere
    builder.add_diffuse(Point3(0.0, -1000.0, 0.0), 1000.0, Color(0.5, 0.5, 0.5));

    // Three large central reference spheres
    builder.add_dielectric(Point3(0.0, 1.0, 0.0), 1.0, 1.5);
    builder.add_diffuse(Point3(-4.0, 1.0, 0.0), 1.0, Color(0.8, 0.2, 0.2));
    builder.add_metal(Point3(4.0, 1.0, 0.0), 1.0, Color(0.7, 0.6, 0.5), 0.0);

    // Procedural small spheres grid
    for (int a = -7; a < 7; ++a) {
        for (int b = -7; b < 7; ++b) {
            double choose_mat = rng.next_double();
            Point3 center(static_cast<double>(a) + 0.9 * rng.next_double(),
                          0.2,
                          static_cast<double>(b) + 0.9 * rng.next_double());

            // Avoid collision with large reference spheres
            if ((center - Point3(4.0, 0.2, 0.0)).length() > 0.9 &&
                (center - Point3(0.0, 0.2, 0.0)).length() > 0.9 &&
                (center - Point3(-4.0, 0.2, 0.0)).length() > 0.9) {
                if (choose_mat < 0.75) {
                    // Diffuse
                    Color albedo(rng.next_double() * rng.next_double(),
                                 rng.next_double() * rng.next_double(),
                                 rng.next_double() * rng.next_double());
                    builder.add_diffuse(center, 0.2, albedo);
                } else if (choose_mat < 0.90) {
                    // Metal
                    Color albedo(rng.next_double(0.5, 1.0),
                                 rng.next_double(0.5, 1.0),
                                 rng.next_double(0.5, 1.0));
                    double fuzz = rng.next_double(0.0, 0.5);
                    builder.add_metal(center, 0.2, albedo, fuzz);
                } else {
                    // Glass
                    builder.add_dielectric(center, 0.2, 1.5);
                }
            }
        }
    }

    builder.set_light(Point3(12.0, 10.0, 8.0), Color(1.0, 1.0, 1.0), 1.2);
    return builder.build();
}

} // namespace raylab
