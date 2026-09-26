#pragma once

#include "raylab/hittable.hpp"

#include <memory>
#include <vector>

namespace raylab {

class HittableList : public Hittable {
public:
    std::vector<std::shared_ptr<Hittable>> objects;

    HittableList() = default;
    explicit HittableList(std::shared_ptr<Hittable> object) { add(object); }

    void clear() { objects.clear(); }
    void add(std::shared_ptr<Hittable> object) { objects.push_back(object); }

    bool hit(const Ray& r, double ray_tmin, double ray_tmax, HitRecord& rec, RenderStats* stats = nullptr) const override;
    bool bounding_box(AABB& output_box) const override;
};

} // namespace raylab
