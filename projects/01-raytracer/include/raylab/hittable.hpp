#pragma once

#include "raylab/aabb.hpp"
#include "raylab/hit_record.hpp"
#include "raylab/ray.hpp"
#include "raylab/render_stats.hpp"

namespace raylab {

class Hittable {
public:
    virtual ~Hittable() = default;

    virtual bool hit(const Ray& r, double ray_tmin, double ray_tmax, HitRecord& rec, RenderStats* stats = nullptr) const = 0;
    virtual bool bounding_box(AABB& output_box) const = 0;
};

} // namespace raylab
