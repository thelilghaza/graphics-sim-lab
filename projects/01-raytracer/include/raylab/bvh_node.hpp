#pragma once

#include "raylab/aabb.hpp"
#include "raylab/hittable.hpp"
#include "raylab/hittable_list.hpp"

#include <algorithm>
#include <memory>
#include <vector>

namespace raylab {

class BVHNode : public Hittable {
public:
    BVHNode() = default;

    explicit BVHNode(const HittableList& list)
        : BVHNode(list.objects, 0, list.objects.size()) {}

    BVHNode(const std::vector<std::shared_ptr<Hittable>>& src_objects, size_t start, size_t end);

    bool hit(const Ray& r, double ray_tmin, double ray_tmax, HitRecord& rec, RenderStats* stats = nullptr) const override;
    bool bounding_box(AABB& output_box) const override;

    const std::shared_ptr<Hittable>& left_child() const { return left; }
    const std::shared_ptr<Hittable>& right_child() const { return right; }
    const AABB& box_bounds() const { return box; }

private:
    std::shared_ptr<Hittable> left;
    std::shared_ptr<Hittable> right;
    AABB box;
};

} // namespace raylab
