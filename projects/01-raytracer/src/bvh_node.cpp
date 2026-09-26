#include "raylab/bvh_node.hpp"
#include <algorithm>
#include <iostream>
#include <stdexcept>

namespace raylab {

BVHNode::BVHNode(const std::vector<std::shared_ptr<Hittable>>& src_objects, size_t start, size_t end) {
    auto objects = src_objects; // Local copy for sorting at current recursion level

    // Compute bounding box for current span to pick longest axis
    AABB span_box;
    bool first = true;
    for (size_t i = start; i < end; ++i) {
        AABB temp_box;
        if (!objects[i]->bounding_box(temp_box)) {
            std::cerr << "Warning: No bounding box in BVHNode constructor.\n";
        } else {
            span_box = first ? temp_box : AABB(span_box, temp_box);
            first = false;
        }
    }

    int axis = span_box.longest_axis();

    auto box_compare = [axis](const std::shared_ptr<Hittable>& a, const std::shared_ptr<Hittable>& b) {
        AABB box_a, box_b;
        if (!a->bounding_box(box_a) || !b->bounding_box(box_b)) {
            return false;
        }
        double center_a = (box_a.min()[axis] + box_a.max()[axis]) * 0.5;
        double center_b = (box_b.min()[axis] + box_b.max()[axis]) * 0.5;
        return center_a < center_b;
    };

    size_t object_span = end - start;

    if (object_span == 1) {
        left = right = objects[start];
    } else if (object_span == 2) {
        if (box_compare(objects[start], objects[start + 1])) {
            left = objects[start];
            right = objects[start + 1];
        } else {
            left = objects[start + 1];
            right = objects[start];
        }
    } else {
        std::sort(objects.begin() + start, objects.begin() + end, box_compare);
        auto mid = start + object_span / 2;
        left = std::make_shared<BVHNode>(objects, start, mid);
        right = std::make_shared<BVHNode>(objects, mid, end);
    }

    AABB box_left, box_right;
    if (!left->bounding_box(box_left) || !right->bounding_box(box_right)) {
        std::cerr << "Warning: No bounding box in BVHNode constructor.\n";
    }

    box = AABB(box_left, box_right);
}

bool BVHNode::hit(const Ray& r, double ray_tmin, double ray_tmax, HitRecord& rec, RenderStats* stats) const {
    if (stats) {
        stats->aabb_tests++;
    }

    if (!box.hit(r, ray_tmin, ray_tmax)) {
        return false;
    }

    bool hit_left = left->hit(r, ray_tmin, ray_tmax, rec, stats);
    bool hit_right = right->hit(r, ray_tmin, hit_left ? rec.t : ray_tmax, rec, stats);

    return hit_left || hit_right;
}

bool BVHNode::bounding_box(AABB& output_box) const {
    output_box = box;
    return true;
}

} // namespace raylab
