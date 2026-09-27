#ifndef DESTRUCTION_VORONOI2D_HPP
#define DESTRUCTION_VORONOI2D_HPP

#include "destruction/math/vec2.hpp"
#include "destruction/fracture/polygon2d.hpp"
#include "destruction/math/math_utils.hpp"
#include <cstdint>
#include <vector>
#include <cmath>

namespace destruction::fracture {

using namespace destruction::math;

struct VoronoiCell2D {
    uint32_t site_id{0};
    Vec2 site{Vec2::zero()};
    Polygon2D polygon;
    float area{0.0f};
    Vec2 centroid{Vec2::zero()};
    bool is_valid{false};
};

class Voronoi2D {
public:
    static std::vector<VoronoiCell2D> compute_partition(
        float min_x, float min_y, float max_x, float max_y,
        const std::vector<Vec2>& sites,
        float eps = GEOM_EPSILON
    ) {
        Polygon2D source_rect = Polygon2D::create_rectangle(min_x, min_y, max_x, max_y);
        std::vector<VoronoiCell2D> cells;
        cells.reserve(sites.size());

        for (size_t i = 0; i < sites.size(); ++i) {
            const Vec2& site_i = sites[i];
            Polygon2D current_poly = source_rect;

            for (size_t j = 0; j < sites.size(); ++j) {
                if (i == j) continue;
                const Vec2& site_j = sites[j];

                Vec2 diff = site_j - site_i;
                float dist_sq = diff.length_sq();
                if (dist_sq <= eps * eps) continue;

                Vec2 n = diff.normalize();
                Vec2 mid = 0.5f * (site_i + site_j);
                float d = -n.dot(mid);

                current_poly = current_poly.clip_by_line(n, d, eps);
                if (current_poly.is_empty()) break;
            }

            VoronoiCell2D cell;
            cell.site_id = static_cast<uint32_t>(i);
            cell.site = site_i;
            cell.polygon = current_poly;
            cell.area = current_poly.area();
            cell.centroid = current_poly.centroid();
            cell.is_valid = (cell.area > eps && current_poly.vertex_count() >= 3);
            cells.push_back(cell);
        }

        return cells;
    }
};

} // namespace destruction::fracture

#endif // DESTRUCTION_VORONOI2D_HPP
