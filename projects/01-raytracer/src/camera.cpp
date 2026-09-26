#include "raylab/camera.hpp"
#include <cmath>

namespace raylab {

constexpr double PI = 3.14159265358979323846;

inline double degrees_to_radians(double degrees) {
    return degrees * PI / 180.0;
}

Vec3 Camera::random_in_unit_disk(RNG& rng) {
    for (int i = 0; i < 100; ++i) {
        Vec3 p(rng.next_double(-1.0, 1.0), rng.next_double(-1.0, 1.0), 0.0);
        if (p.length_squared() <= 1.0) {
            return p;
        }
    }
    return Vec3(0.0, 0.0, 0.0);
}

Camera::Camera(Point3 lookfrom,
               Point3 lookat,
               Vec3 vup,
               double vfov_degrees,
               int image_width,
               int image_height,
               double aperture,
               double focus_dist)
    : camera_center(lookfrom),
      lens_aperture(aperture),
      radius(aperture * 0.5),
      focus_distance_val(focus_dist) {

    double aspect_ratio = static_cast<double>(image_width) / static_cast<double>(image_height);
    double theta = degrees_to_radians(vfov_degrees);
    double h = std::tan(theta / 2.0);
    double viewport_height = 2.0 * h * focus_dist;
    double viewport_width = viewport_height * aspect_ratio;

    // Calculate camera orthonormal basis vectors
    w = unit_vector(lookfrom - lookat);
    u = unit_vector(cross(vup, w));
    v = cross(w, u);

    // Viewport edge vectors
    Vec3 viewport_u = viewport_width * u;
    Vec3 viewport_v = -viewport_height * v; // -v because pixel 0 is top

    // Pixel delta vectors
    pixel_delta_u = viewport_u / static_cast<double>(image_width);
    pixel_delta_v = viewport_v / static_cast<double>(image_height);

    // Upper left pixel center location
    Point3 viewport_upper_left = camera_center
                                 - (focus_dist * w)
                                 - viewport_u / 2.0
                                 - viewport_v / 2.0;
    pixel00_loc = viewport_upper_left + 0.5 * (pixel_delta_u + pixel_delta_v);
}

Camera::Camera(int image_width, int image_height, double focal_length, double viewport_height)
    : Camera(Point3(0.0, 0.0, 0.0),
             Point3(0.0, 0.0, -focal_length),
             Vec3(0.0, 1.0, 0.0),
             90.0,
             image_width,
             image_height,
             0.0,
             focal_length) {
    // Override exact viewport height for backward compatibility if specified
    double aspect_ratio = static_cast<double>(image_width) / static_cast<double>(image_height);
    double v_width = viewport_height * aspect_ratio;

    w = Vec3(0.0, 0.0, 1.0);
    u = Vec3(1.0, 0.0, 0.0);
    v = Vec3(0.0, 1.0, 0.0);

    Vec3 vp_u(v_width, 0.0, 0.0);
    Vec3 vp_v(0.0, -viewport_height, 0.0);

    pixel_delta_u = vp_u / static_cast<double>(image_width);
    pixel_delta_v = vp_v / static_cast<double>(image_height);

    Point3 vp_upper_left = camera_center
                           - Vec3(0.0, 0.0, focal_length)
                           - vp_u / 2.0
                           - vp_v / 2.0;
    pixel00_loc = vp_upper_left + 0.5 * (pixel_delta_u + pixel_delta_v);
}

Ray Camera::get_ray(int i, int j, double u_offset, double v_offset) const {
    Point3 pixel_sample = pixel00_loc
                          + ((static_cast<double>(i) + u_offset - 0.5) * pixel_delta_u)
                          + ((static_cast<double>(j) + v_offset - 0.5) * pixel_delta_v);
    Vec3 ray_direction = pixel_sample - camera_center;
    return Ray(camera_center, unit_vector(ray_direction));
}

Ray Camera::get_ray(int i, int j, double u_offset, double v_offset, RNG& rng) const {
    Point3 pixel_sample = pixel00_loc
                          + ((static_cast<double>(i) + u_offset - 0.5) * pixel_delta_u)
                          + ((static_cast<double>(j) + v_offset - 0.5) * pixel_delta_v);

    // Exact pinhole path when aperture == 0.0
    if (radius <= 0.0) {
        Vec3 ray_direction = pixel_sample - camera_center;
        return Ray(camera_center, unit_vector(ray_direction));
    }

    // Thin-lens DOF path
    Vec3 rd = random_in_unit_disk(rng);
    Vec3 lens_offset = u * (rd.x() * radius) + v * (rd.y() * radius);
    Point3 ray_origin = camera_center + lens_offset;
    Vec3 ray_direction = pixel_sample - ray_origin;
    return Ray(ray_origin, unit_vector(ray_direction));
}

} // namespace raylab
