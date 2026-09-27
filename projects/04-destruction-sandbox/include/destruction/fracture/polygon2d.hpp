#ifndef DESTRUCTION_POLYGON2D_HPP
#define DESTRUCTION_POLYGON2D_HPP

#include "destruction/math/vec2.hpp"
#include "destruction/math/math_utils.hpp"
#include "destruction/fracture/plane.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

namespace destruction::fracture {

using namespace destruction::math;

struct Polygon2D {
    std::vector<Vec2> vertices;

    Polygon2D() = default;
    explicit Polygon2D(std::vector<Vec2> verts) : vertices(std::move(verts)) {}

    static Polygon2D create_rectangle(float min_x, float min_y, float max_x, float max_y) {
        std::vector<Vec2> verts = {
            Vec2(min_x, min_y),
            Vec2(max_x, min_y),
            Vec2(max_x, max_y),
            Vec2(min_x, max_y)
        };
        return Polygon2D(verts);
    }

    size_t vertex_count() const {
        return vertices.size();
    }

    bool is_empty() const {
        return vertices.size() < 3;
    }

    float area() const {
        if (vertices.size() < 3) return 0.0f;
        float sum = 0.0f;
        size_t n = vertices.size();
        for (size_t i = 0; i < n; ++i) {
            const Vec2& v1 = vertices[i];
            const Vec2& v2 = vertices[(i + 1) % n];
            sum += (v1.x * v2.y - v2.x * v1.y);
        }
        return std::abs(0.5f * sum);
    }

    Vec2 centroid() const {
        if (vertices.size() < 3) return Vec2::zero();
        float a = 0.0f;
        float cx = 0.0f;
        float cy = 0.0f;
        size_t n = vertices.size();

        for (size_t i = 0; i < n; ++i) {
            const Vec2& v1 = vertices[i];
            const Vec2& v2 = vertices[(i + 1) % n];
            float cross = (v1.x * v2.y - v2.x * v1.y);
            a += cross;
            cx += (v1.x + v2.x) * cross;
            cy += (v1.y + v2.y) * cross;
        }

        a *= 0.5f;
        if (std::abs(a) <= GEOM_EPSILON) {
            Vec2 avg = Vec2::zero();
            for (const auto& v : vertices) avg += v;
            return avg / static_cast<float>(n);
        }

        float inv = 1.0f / (6.0f * a);
        return Vec2(cx * inv, cy * inv);
    }

    void deduplicate_vertices(float eps = GEOM_EPSILON) {
        if (vertices.size() < 2) return;
        std::vector<Vec2> clean;
        for (const auto& v : vertices) {
            bool exists = false;
            for (const auto& c : clean) {
                if ((v - c).length_sq() <= eps * eps) {
                    exists = true;
                    break;
                }
            }
            if (!exists) {
                clean.push_back(v);
            }
        }
        vertices = std::move(clean);
    }

    // Clip polygon against 2D half-plane: n.x * x + n.y * y + d <= 0
    Polygon2D clip_by_line(const Vec2& n_norm, float d, float eps = GEOM_EPSILON) const {
        if (vertices.empty()) return Polygon2D();

        auto dist = [&](const Vec2& p) {
            return n_norm.dot(p) + d;
        };

        std::vector<Vec2> output;
        size_t n = vertices.size();

        for (size_t i = 0; i < n; ++i) {
            const Vec2& curr = vertices[i];
            const Vec2& next = vertices[(i + 1) % n];

            float d_curr = dist(curr);
            float d_next = dist(next);

            bool curr_in = (d_curr <= eps);
            bool next_in = (d_next <= eps);

            if (curr_in) {
                output.push_back(curr);
            }

            if (curr_in != next_in) {
                float diff = d_next - d_curr;
                if (std::abs(diff) > eps) {
                    float t = -d_curr / diff;
                    t = std::clamp(t, 0.0f, 1.0f);
                    Vec2 inter = curr + t * (next - curr);
                    output.push_back(inter);
                }
            }
        }

        Polygon2D result(output);
        result.deduplicate_vertices(eps);
        return result;
    }
};

} // namespace destruction::fracture

#endif // DESTRUCTION_POLYGON2D_HPP
