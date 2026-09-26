#include "raylab/hittable_list.hpp"

namespace raylab {

bool HittableList::hit(const Ray& r, double ray_tmin, double ray_tmax, HitRecord& rec, RenderStats* stats) const {
    HitRecord temp_rec;
    bool hit_anything = false;
    double closest_so_far = ray_tmax;

    for (const auto& object : objects) {
        if (object->hit(r, ray_tmin, closest_so_far, temp_rec, stats)) {
            hit_anything = true;
            closest_so_far = temp_rec.t;
            rec = temp_rec;
        }
    }

    return hit_anything;
}

bool HittableList::bounding_box(AABB& output_box) const {
    if (objects.empty()) return false;

    AABB temp_box;
    bool first_box = true;

    for (const auto& object : objects) {
        if (!object->bounding_box(temp_box)) return false;
        output_box = first_box ? temp_box : AABB(output_box, temp_box);
        first_box = false;
    }

    return true;
}

} // namespace raylab
