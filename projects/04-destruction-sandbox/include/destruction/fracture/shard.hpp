#ifndef DESTRUCTION_SHARD_HPP
#define DESTRUCTION_SHARD_HPP

#include "destruction/math/vec3.hpp"
#include "destruction/fracture/polyhedron.hpp"
#include <cstdint>

namespace destruction::fracture {

using namespace destruction::math;

struct Shard {
    uint32_t id{0};
    Vec3 site{Vec3::zero()};
    ConvexPolyhedron mesh;
    float volume{0.0f};
    Vec3 centroid{Vec3::zero()};
    float mass{0.0f};
    bool is_valid{false};

    Shard() = default;

    Shard(
        uint32_t shard_id,
        const Vec3& site_pos,
        ConvexPolyhedron poly,
        float density = 1.0f
    ) : id(shard_id), site(site_pos), mesh(std::move(poly)) {
        volume = mesh.volume();
        centroid = mesh.centroid();
        mass = volume * density;
        is_valid = (volume > GEOM_EPSILON && mesh.vertices.size() >= 4 && mesh.faces.size() >= 4);
    }
};

} // namespace destruction::fracture

#endif // DESTRUCTION_SHARD_HPP
