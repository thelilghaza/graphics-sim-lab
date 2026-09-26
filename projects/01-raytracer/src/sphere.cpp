#include "raylab/sphere.hpp"
#include <cmath>

namespace raylab {

bool Sphere::hit(const Ray& r, double ray_tmin, double ray_tmax, HitRecord& rec, RenderStats* stats) const {
    if (stats) {
        stats->sphere_intersection_tests++;
    }

    Vec3 oc = r.origin() - center_point;
    double a = r.direction().length_squared();
    double half_b = dot(oc, r.direction());
    double c = oc.length_squared() - rad * rad;

    double discriminant = half_b * half_b - a * c;
    if (discriminant < 0.0) {
        return false;
    }

    double sqrtd = std::sqrt(discriminant);

    // Find the nearest root that lies in the acceptable range.
    double root = (-half_b - sqrtd) / a;
    if (root <= ray_tmin || ray_tmax <= root) {
        root = (-half_b + sqrtd) / a;
        if (root <= ray_tmin || ray_tmax <= root) {
            return false;
        }
    }

    rec.t = root;
    rec.p = r.at(rec.t);
    Vec3 outward_normal = (rec.p - center_point) / rad;
    rec.set_face_normal(r, outward_normal);
    rec.mat_ptr = mat_ptr;

    return true;
}

} // namespace raylab
