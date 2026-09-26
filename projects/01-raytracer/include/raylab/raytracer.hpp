#pragma once

#include "raylab/color.hpp"
#include "raylab/hittable.hpp"
#include "raylab/light.hpp"
#include "raylab/material.hpp"
#include "raylab/ray.hpp"
#include "raylab/render_stats.hpp"
#include "raylab/rng.hpp"

namespace raylab {

constexpr double SHADOW_EPSILON = 0.001;

// Direct lighting evaluation for Lambertian / Diffuse surfaces
inline Color evaluate_direct_light(const HitRecord& rec,
                                    const Hittable& world,
                                    const PointLight& light,
                                    RenderStats* stats = nullptr) {
    Color albedo = rec.mat_ptr ? rec.mat_ptr->albedo() : Color(0.5, 0.5, 0.5);
    Color ambient = 0.15 * albedo;

    Vec3 light_vec = light.position() - rec.p;
    double light_distance = light_vec.length();
    Vec3 L = unit_vector(light_vec);

    double cos_theta = dot(rec.normal, L);
    if (cos_theta <= 0.0) {
        return ambient;
    }

    if (stats) {
        stats->shadow_rays++;
    }

    Ray shadow_ray(rec.p + rec.normal * SHADOW_EPSILON, L);
    HitRecord shadow_rec;
    bool in_shadow = world.hit(shadow_ray, SHADOW_EPSILON, light_distance - SHADOW_EPSILON, shadow_rec, stats);

    if (in_shadow) {
        return ambient;
    }

    Color direct_diffuse = (albedo * light.color()) * (light.intensity() * cos_theta);
    return ambient + direct_diffuse;
}

// Recursive ray color evaluation pipeline with max-depth limit and ray counting statistics
inline Color ray_color(const Ray& r,
                        const Hittable& world,
                        const PointLight& light,
                        int depth,
                        int max_depth,
                        RNG& rng,
                        RenderStats* stats = nullptr) {
    // Record ray counts explicitly
    if (stats) {
        if (depth == 0) {
            stats->primary_samples++;
        } else {
            stats->secondary_rays++;
        }
    }

    // Terminal condition: stop recursion if max depth reached
    if (depth >= max_depth) {
        return Color(0.0, 0.0, 0.0);
    }

    HitRecord rec;
    if (world.hit(r, 0.001, 1e9, rec, stats)) {
        ScatterRecord srec;
        if (rec.mat_ptr && rec.mat_ptr->scatter(r, rec, srec, rng)) {
            if (srec.is_specular) {
                // Specular reflection/refraction: purely recursive bounce
                return srec.attenuation * ray_color(srec.scattered_ray, world, light, depth + 1, max_depth, rng, stats);
            } else {
                // Lambertian: Hybrid direct point-light + recursive secondary bounce
                Color direct_light = evaluate_direct_light(rec, world, light, stats);
                Color indirect_light(0.0, 0.0, 0.0);
                if (depth + 1 < max_depth) {
                    indirect_light = srec.attenuation * ray_color(srec.scattered_ray, world, light, depth + 1, max_depth, rng, stats);
                }
                return direct_light + indirect_light;
            }
        }
        return evaluate_direct_light(rec, world, light, stats);
    }

    // Sky gradient background for missed rays
    Vec3 unit_direction = unit_vector(r.direction());
    auto a = 0.5 * (unit_direction.y() + 1.0);
    return (1.0 - a) * Color(1.0, 1.0, 1.0) + a * Color(0.5, 0.7, 1.0);
}

} // namespace raylab
