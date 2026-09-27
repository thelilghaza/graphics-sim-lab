#ifndef DESTRUCTION_FRACTURE_VOLUME_HPP
#define DESTRUCTION_FRACTURE_VOLUME_HPP

#include "destruction/math/vec3.hpp"
#include "destruction/math/math_utils.hpp"
#include "destruction/fracture/polyhedron.hpp"
#include <cmath>

namespace destruction::fracture {

using namespace destruction::math;

struct FractureVolume {
    Vec3 min_pt{-1.0f, -1.0f, -1.0f};
    Vec3 max_pt{1.0f, 1.0f, 1.0f};
    float density{1.0f}; // kg/m^3

    constexpr FractureVolume() = default;
    constexpr FractureVolume(const Vec3& min_corner, const Vec3& max_corner, float density_val = 1.0f)
        : min_pt(min_corner), max_pt(max_corner), density(density_val) {}

    static constexpr FractureVolume unit_box(float density_val = 1.0f) {
        return FractureVolume(Vec3(-1.0f, -1.0f, -1.0f), Vec3(1.0f, 1.0f, 1.0f), density_val);
    }

    Vec3 dimensions() const {
        return Vec3(
            std::abs(max_pt.x - min_pt.x),
            std::abs(max_pt.y - min_pt.y),
            std::abs(max_pt.z - min_pt.z)
        );
    }

    float volume() const {
        Vec3 dim = dimensions();
        return dim.x * dim.y * dim.z;
    }

    Vec3 centroid() const {
        return 0.5f * (min_pt + max_pt);
    }

    float mass() const {
        return volume() * density;
    }

    ConvexPolyhedron to_polyhedron() const {
        return ConvexPolyhedron::create_box(min_pt, max_pt);
    }

    bool contains(const Vec3& p, float eps = GEOM_EPSILON) const {
        return (p.x >= min_pt.x - eps && p.x <= max_pt.x + eps &&
                p.y >= min_pt.y - eps && p.y <= max_pt.y + eps &&
                p.z >= min_pt.z - eps && p.z <= max_pt.z + eps);
    }
};

} // namespace destruction::fracture

#endif // DESTRUCTION_FRACTURE_VOLUME_HPP
