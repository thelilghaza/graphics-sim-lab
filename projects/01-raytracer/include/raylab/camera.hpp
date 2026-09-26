#pragma once

#include "raylab/ray.hpp"
#include "raylab/rng.hpp"
#include "raylab/vec3.hpp"

namespace raylab {

class Camera {
public:
    // Full orientation & thin-lens depth of field constructor
    // NOTE: aperture represents lens diameter.
    Camera(Point3 lookfrom,
           Point3 lookat,
           Vec3 vup,
           double vfov_degrees,
           int image_width,
           int image_height,
           double aperture = 0.0,
           double focus_dist = 1.0);

    // Backward-compatible pinhole camera constructor matching Milestone 1-3
    Camera(int image_width,
           int image_height,
           double focal_length = 1.0,
           double viewport_height = 2.0);

    // Generates pinhole camera ray for pixel coordinate (i, j)
    Ray get_ray(int i, int j, double u_offset = 0.5, double v_offset = 0.5) const;

    // Generates camera ray with optional thin-lens lens disk sampling (if aperture > 0)
    Ray get_ray(int i, int j, double u_offset, double v_offset, RNG& rng) const;

    Point3 center() const { return camera_center; }
    Vec3 u_basis() const { return u; }
    Vec3 v_basis() const { return v; }
    Vec3 w_basis() const { return w; }
    double aperture() const { return lens_aperture; }
    double lens_radius() const { return radius; }
    double focus_distance() const { return focus_distance_val; }

private:
    Point3 camera_center{0.0, 0.0, 0.0};
    Point3 pixel00_loc;
    Vec3 pixel_delta_u;
    Vec3 pixel_delta_v;
    Vec3 u, v, w;
    double lens_aperture{0.0};
    double radius{0.0};
    double focus_distance_val{1.0};

    static Vec3 random_in_unit_disk(RNG& rng);
};

} // namespace raylab
