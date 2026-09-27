#ifndef DESTRUCTION_CLIPPER3D_HPP
#define DESTRUCTION_CLIPPER3D_HPP

#include "destruction/math/vec3.hpp"
#include "destruction/math/math_utils.hpp"
#include "destruction/fracture/plane.hpp"
#include "destruction/fracture/polyhedron.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

namespace destruction::fracture {

using namespace destruction::math;

class Clipper3D {
public:
    static ConvexPolyhedron clip_halfspace(
        const ConvexPolyhedron& poly,
        const Plane& plane,
        int neighbor_site_id = -1,
        float eps = GEOM_EPSILON
    ) {
        if (poly.vertices.empty() || poly.faces.empty()) {
            return ConvexPolyhedron();
        }

        std::vector<Vec3> new_vertices;
        std::vector<Face> new_faces;
        std::vector<Vec3> cap_points;

        auto get_or_add_vertex = [&](const Vec3& p) -> uint32_t {
            for (size_t i = 0; i < new_vertices.size(); ++i) {
                if ((new_vertices[i] - p).length_sq() <= eps * eps) {
                    return static_cast<uint32_t>(i);
                }
            }
            new_vertices.push_back(p);
            return static_cast<uint32_t>(new_vertices.size() - 1);
        };

        // 1. Clip existing faces against plane
        for (const auto& face : poly.faces) {
            size_t n_verts = face.vertex_indices.size();
            if (n_verts < 3) continue;

            std::vector<uint32_t> face_new_indices;

            for (size_t i = 0; i < n_verts; ++i) {
                uint32_t idx_a = face.vertex_indices[i];
                uint32_t idx_b = face.vertex_indices[(i + 1) % n_verts];

                const Vec3& va = poly.vertices[idx_a];
                const Vec3& vb = poly.vertices[idx_b];

                PointPlaneSide side_a = plane.classify(va, eps);
                PointPlaneSide side_b = plane.classify(vb, eps);

                bool a_in = (side_a != PointPlaneSide::Outside);
                bool b_in = (side_b != PointPlaneSide::Outside);

                if (a_in) {
                    face_new_indices.push_back(get_or_add_vertex(va));
                }

                if (a_in != b_in) {
                    Vec3 inter;
                    if (plane.intersect_segment(va, vb, inter, eps)) {
                        uint32_t inter_idx = get_or_add_vertex(inter);
                        face_new_indices.push_back(inter_idx);
                        cap_points.push_back(inter);
                    }
                }
            }

            // Clean adjacent duplicates in face
            std::vector<uint32_t> clean_indices;
            for (uint32_t idx : face_new_indices) {
                if (clean_indices.empty() || clean_indices.back() != idx) {
                    clean_indices.push_back(idx);
                }
            }
            if (clean_indices.size() > 1 && clean_indices.front() == clean_indices.back()) {
                clean_indices.pop_back();
            }

            if (clean_indices.size() >= 3) {
                new_faces.emplace_back(clean_indices, face.normal, face.plane_d, face.neighbor_site_id);
            }
        }

        // 2. Cap-Face Construction for the clipping plane
        // Deduplicate cap points
        std::vector<Vec3> unique_cap;
        for (const auto& p : cap_points) {
            bool exists = false;
            for (const auto& u : unique_cap) {
                if ((p - u).length_sq() <= eps * eps) {
                    exists = true;
                    break;
                }
            }
            if (!exists) {
                unique_cap.push_back(p);
            }
        }

        if (unique_cap.size() >= 3) {
            Vec3 cap_center = Vec3::zero();
            for (const auto& p : unique_cap) cap_center += p;
            cap_center /= static_cast<float>(unique_cap.size());

            // Stable orthonormal basis (u, v) on the clipping plane
            Vec3 n = plane.normal.normalize();
            Vec3 u_axis;
            if (std::abs(n.x) < 0.9f) {
                u_axis = n.cross(Vec3::unit_x()).normalize();
            } else {
                u_axis = n.cross(Vec3::unit_y()).normalize();
            }
            Vec3 v_axis = n.cross(u_axis).normalize();

            struct CapVert {
                uint32_t idx;
                float angle;
            };

            std::vector<CapVert> sorted_cap;
            for (const auto& p : unique_cap) {
                uint32_t idx = get_or_add_vertex(p);
                Vec3 rel = p - cap_center;
                float u_coord = rel.dot(u_axis);
                float v_coord = rel.dot(v_axis);
                float angle = std::atan2(v_coord, u_coord);
                sorted_cap.push_back({idx, angle});
            }

            std::sort(sorted_cap.begin(), sorted_cap.end(), [](const CapVert& a, const CapVert& b) {
                return a.angle < b.angle;
            });

            std::vector<uint32_t> cap_indices;
            for (const auto& cv : sorted_cap) {
                if (cap_indices.empty() || cap_indices.back() != cv.idx) {
                    cap_indices.push_back(cv.idx);
                }
            }
            if (cap_indices.size() > 1 && cap_indices.front() == cap_indices.back()) {
                cap_indices.pop_back();
            }

            if (cap_indices.size() >= 3) {
                // Plane normal n points OUT of the kept region half-space
                new_faces.emplace_back(cap_indices, n, plane.d, neighbor_site_id);
            }
        }

        ConvexPolyhedron result;
        result.vertices = std::move(new_vertices);
        result.faces = std::move(new_faces);
        result.weld_vertices(eps);

        return result;
    }
};

} // namespace destruction::fracture

#endif // DESTRUCTION_CLIPPER3D_HPP
