#pragma once

#include "raylab/hittable.hpp"
#include "raylab/material.hpp"
#include "raylab/vec3.hpp"
#include <memory>

namespace raylab {

class Sphere : public Hittable {
public:
    Sphere(const Point3& center, double radius, std::shared_ptr<Material> mat = nullptr)
        : center_point(center), rad(std::max(0.0, radius)), mat_ptr(mat) {}

    bool hit(const Ray& r, double ray_tmin, double ray_tmax, HitRecord& rec, RenderStats* stats = nullptr) const override;
    bool bounding_box(AABB& output_box) const override {
        output_box = AABB(center_point - Vec3(rad, rad, rad), center_point + Vec3(rad, rad, rad));
        return true;
    }

    const Point3& center() const { return center_point; }
    double radius() const { return rad; }
    std::shared_ptr<Material> material() const { return mat_ptr; }

private:
    Point3 center_point;
    double rad;
    std::shared_ptr<Material> mat_ptr;
};

} // namespace raylab
