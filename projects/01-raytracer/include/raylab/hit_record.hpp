#pragma once

#include "raylab/ray.hpp"
#include "raylab/vec3.hpp"
#include <memory>

namespace raylab {

class Material; // Forward declaration

struct HitRecord {
    Point3 p;
    Vec3 normal;
    double t{0.0};
    bool front_face{true};
    std::shared_ptr<Material> mat_ptr{nullptr};

    // Sets the hit record normal vector to point outward relative to ray.
    // NOTE: outward_normal is assumed to have unit length.
    void set_face_normal(const Ray& r, const Vec3& outward_normal) {
        front_face = dot(r.direction(), outward_normal) < 0.0;
        normal = front_face ? outward_normal : -outward_normal;
    }
};

} // namespace raylab
