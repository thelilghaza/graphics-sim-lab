#ifndef DESTRUCTION_VORONOI3D_HPP
#define DESTRUCTION_VORONOI3D_HPP

#include "destruction/math/vec3.hpp"
#include "destruction/math/math_utils.hpp"
#include "destruction/fracture/plane.hpp"
#include "destruction/fracture/polyhedron.hpp"
#include "destruction/fracture/clipper3d.hpp"
#include "destruction/fracture/fracture_volume.hpp"
#include "destruction/fracture/shard.hpp"
#include "destruction/fracture/mesh_validator.hpp"
#include <vector>
#include <cmath>

namespace destruction::fracture {

using namespace destruction::math;

struct VoronoiPartitionSummary {
    size_t requested_site_count{0};
    size_t valid_shard_count{0};
    float source_volume{0.0f};
    float sum_shard_volume{0.0f};
    float relative_volume_error{0.0f};
    float source_mass{0.0f};
    float sum_shard_mass{0.0f};
    float relative_mass_error{0.0f};
};

class Voronoi3D {
public:
    static std::vector<Shard> compute_partition(
        const FractureVolume& source_vol,
        const std::vector<Vec3>& sites,
        float eps = GEOM_EPSILON
    ) {
        ConvexPolyhedron source_poly = source_vol.to_polyhedron();
        std::vector<Shard> shards;
        shards.reserve(sites.size());

        for (size_t i = 0; i < sites.size(); ++i) {
            const Vec3& site_i = sites[i];
            ConvexPolyhedron current_poly = source_poly;

            for (size_t j = 0; j < sites.size(); ++j) {
                if (i == j) continue;
                const Vec3& site_j = sites[j];

                if ((site_j - site_i).length_sq() <= eps * eps) continue;

                Plane bisector = Plane::bisector(site_i, site_j);
                current_poly = Clipper3D::clip_halfspace(current_poly, bisector, static_cast<int>(j), eps);

                if (current_poly.vertices.size() < 4 || current_poly.faces.size() < 4) {
                    break;
                }
            }

            Shard shard(static_cast<uint32_t>(i), site_i, current_poly, source_vol.density);
            shards.push_back(std::move(shard));
        }

        return shards;
    }

    static VoronoiPartitionSummary evaluate_summary(
        const FractureVolume& source_vol,
        const std::vector<Shard>& shards
    ) {
        VoronoiPartitionSummary summary;
        summary.requested_site_count = shards.size();
        summary.source_volume = source_vol.volume();
        summary.source_mass = source_vol.mass();

        for (const auto& shard : shards) {
            if (shard.is_valid) {
                summary.valid_shard_count++;
                summary.sum_shard_volume += shard.volume;
                summary.sum_shard_mass += shard.mass;
            }
        }

        if (summary.source_volume > GEOM_EPSILON) {
            summary.relative_volume_error = std::abs(summary.sum_shard_volume - summary.source_volume) / summary.source_volume;
        }

        if (summary.source_mass > GEOM_EPSILON) {
            summary.relative_mass_error = std::abs(summary.sum_shard_mass - summary.source_mass) / summary.source_mass;
        }

        return summary;
    }
};

} // namespace destruction::fracture

#endif // DESTRUCTION_VORONOI3D_HPP
