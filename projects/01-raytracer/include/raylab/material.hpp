#pragma once

#include "raylab/color.hpp"
#include "raylab/hit_record.hpp"
#include "raylab/ray.hpp"
#include "raylab/rng.hpp"
#include "raylab/vec3.hpp"

#include <algorithm>
#include <cmath>

namespace raylab {

constexpr double SCATTER_EPSILON = 0.001;

struct ScatterRecord {
    Color attenuation{1.0, 1.0, 1.0};
    Ray scattered_ray;
    bool is_specular{false};
};

class Material {
public:
    virtual ~Material() = default;
    virtual Color albedo() const = 0;
    virtual bool scatter(const Ray& r_in,
                         const HitRecord& rec,
                         ScatterRecord& srec,
                         RNG& rng) const = 0;
};

// Lambertian Diffuse Material
class Lambertian : public Material {
public:
    explicit Lambertian(const Color& a) : albedo_color(a) {}

    Color albedo() const override { return albedo_color; }

    bool scatter(const Ray& /*r_in*/,
                 const HitRecord& rec,
                 ScatterRecord& srec,
                 RNG& rng) const override {
        Vec3 scatter_direction = rec.normal + random_unit_vector(rng);

        // Catch degenerate scatter direction opposing normal
        if (scatter_direction.near_zero()) {
            scatter_direction = rec.normal;
        } else {
            scatter_direction = unit_vector(scatter_direction);
        }

        srec.scattered_ray = Ray(rec.p + rec.normal * SCATTER_EPSILON, scatter_direction);
        srec.attenuation = albedo_color;
        srec.is_specular = false;
        return true;
    }

private:
    Color albedo_color;
};

// Metallic Reflective Material
class Metal : public Material {
public:
    Metal(const Color& a, double fuzz)
        : albedo_color(a), fuzz_factor(std::clamp(fuzz, 0.0, 1.0)) {}

    Color albedo() const override { return albedo_color; }
    double fuzz() const { return fuzz_factor; }

    bool scatter(const Ray& r_in,
                 const HitRecord& rec,
                 ScatterRecord& srec,
                 RNG& rng) const override {
        Vec3 reflected = reflect(unit_vector(r_in.direction()), rec.normal);
        if (fuzz_factor > 0.0) {
            reflected += fuzz_factor * random_unit_vector(rng);
        }
        Vec3 scatter_direction = unit_vector(reflected);

        // Only valid if reflected ray leaves surface facing outward
        if (dot(scatter_direction, rec.normal) <= 0.0) {
            return false;
        }

        srec.scattered_ray = Ray(rec.p + rec.normal * SCATTER_EPSILON, scatter_direction);
        srec.attenuation = albedo_color;
        srec.is_specular = true;
        return true;
    }

private:
    Color albedo_color;
    double fuzz_factor;
};

// Dielectric Glass Material
class Dielectric : public Material {
public:
    explicit Dielectric(double index_of_refraction)
        : ior(std::max(1.0, index_of_refraction)) {}

    Color albedo() const override { return Color(1.0, 1.0, 1.0); }
    double refraction_index() const { return ior; }

    bool scatter(const Ray& r_in,
                 const HitRecord& rec,
                 ScatterRecord& srec,
                 RNG& rng) const override {
        srec.attenuation = Color(1.0, 1.0, 1.0);
        srec.is_specular = true;

        // Relative index of refraction:
        // Ray entering glass from air: 1.0 / ior
        // Ray exiting glass into air:  ior / 1.0
        double refraction_ratio = rec.front_face ? (1.0 / ior) : ior;

        Vec3 unit_direction = unit_vector(r_in.direction());
        double cos_theta = std::min(dot(-unit_direction, rec.normal), 1.0);
        double sin_theta = std::sqrt(std::max(0.0, 1.0 - cos_theta * cos_theta));

        // Total Internal Reflection test
        bool cannot_refract = refraction_ratio * sin_theta > 1.0;

        Vec3 direction;
        Point3 origin_offset;

        if (cannot_refract || schlick_reflectance(cos_theta, refraction_ratio) > rng.next_double()) {
            // Reflect
            direction = reflect(unit_direction, rec.normal);
            origin_offset = rec.p + rec.normal * SCATTER_EPSILON;
        } else {
            // Refract
            direction = refract(unit_direction, rec.normal, refraction_ratio);
            origin_offset = rec.p - rec.normal * SCATTER_EPSILON;
        }

        srec.scattered_ray = Ray(origin_offset, unit_vector(direction));
        return true;
    }

    static double schlick_reflectance(double cosine, double ref_idx) {
        auto r0 = (1.0 - ref_idx) / (1.0 + ref_idx);
        r0 = r0 * r0;
        return r0 + (1.0 - r0) * std::pow((1.0 - cosine), 5);
    }

private:
    double ior;
};

} // namespace raylab
