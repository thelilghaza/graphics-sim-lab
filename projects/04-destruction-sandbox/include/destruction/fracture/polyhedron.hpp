#ifndef DESTRUCTION_POLYHEDRON_HPP
#define DESTRUCTION_POLYHEDRON_HPP

#include "destruction/math/vec3.hpp"
#include "destruction/math/math_utils.hpp"
#include "destruction/fracture/plane.hpp"
#include <vector>
#include <cmath>
#include <algorithm>
#include <cstdint>

namespace destruction::fracture {

using namespace destruction::math;

struct Face {
    std::vector<uint32_t> vertex_indices;
    Vec3 normal{Vec3::unit_y()};
    float plane_d{0.0f};
    int neighbor_site_id{-1};

    Face() = default;
    Face(std::vector<uint32_t> indices, const Vec3& norm, float d_val, int neighbor_id = -1)
        : vertex_indices(std::move(indices)), normal(norm.normalize()), plane_d(d_val), neighbor_site_id(neighbor_id) {}
};

struct ConvexPolyhedron {
    std::vector<Vec3> vertices;
    std::vector<Face> faces;

    ConvexPolyhedron() = default;

    static ConvexPolyhedron create_box(const Vec3& min_pt, const Vec3& max_pt) {
        ConvexPolyhedron poly;
        poly.vertices = {
            Vec3(min_pt.x, min_pt.y, min_pt.z), // 0
            Vec3(max_pt.x, min_pt.y, min_pt.z), // 1
            Vec3(max_pt.x, max_pt.y, min_pt.z), // 2
            Vec3(min_pt.x, max_pt.y, min_pt.z), // 3
            Vec3(min_pt.x, min_pt.y, max_pt.z), // 4
            Vec3(max_pt.x, min_pt.y, max_pt.z), // 5
            Vec3(max_pt.x, max_pt.y, max_pt.z), // 6
            Vec3(min_pt.x, max_pt.y, max_pt.z)  // 7
        };

        // Outward-pointing face normals
        // -Z face
        poly.faces.emplace_back(std::vector<uint32_t>{0, 3, 2, 1}, Vec3(0, 0, -1), min_pt.z);
        // +Z face
        poly.faces.emplace_back(std::vector<uint32_t>{4, 5, 6, 7}, Vec3(0, 0, 1), -max_pt.z);
        // -X face
        poly.faces.emplace_back(std::vector<uint32_t>{0, 4, 7, 3}, Vec3(-1, 0, 0), min_pt.x);
        // +X face
        poly.faces.emplace_back(std::vector<uint32_t>{1, 2, 6, 5}, Vec3(1, 0, 0), -max_pt.x);
        // -Y face
        poly.faces.emplace_back(std::vector<uint32_t>{0, 1, 5, 4}, Vec3(0, -1, 0), min_pt.y);
        // +Y face
        poly.faces.emplace_back(std::vector<uint32_t>{3, 7, 6, 2}, Vec3(0, 1, 0), -max_pt.y);

        return poly;
    }

    Vec3 calculate_vertex_average() const {
        if (vertices.empty()) return Vec3::zero();
        Vec3 sum = Vec3::zero();
        for (const auto& v : vertices) sum += v;
        return sum / static_cast<float>(vertices.size());
    }

    float volume() const {
        if (vertices.size() < 4 || faces.size() < 4) return 0.0f;
        Vec3 ref = calculate_vertex_average();
        float total_vol = 0.0f;

        for (const auto& face : faces) {
            if (face.vertex_indices.size() < 3) continue;
            const Vec3& v0 = vertices[face.vertex_indices[0]];

            for (size_t k = 1; k + 1 < face.vertex_indices.size(); ++k) {
                const Vec3& vk = vertices[face.vertex_indices[k]];
                const Vec3& vk1 = vertices[face.vertex_indices[k + 1]];

                // Signed volume of tetrahedron (ref, v0, vk, vk1)
                Vec3 a = v0 - ref;
                Vec3 b = vk - ref;
                Vec3 c = vk1 - ref;

                float tet_vol = (1.0f / 6.0f) * a.dot(b.cross(c));
                total_vol += tet_vol;
            }
        }
        return std::abs(total_vol);
    }

    Vec3 centroid() const {
        if (vertices.size() < 4 || faces.size() < 4) return calculate_vertex_average();
        Vec3 ref = calculate_vertex_average();
        float total_vol = 0.0f;
        Vec3 weighted_centroid_sum = Vec3::zero();

        for (const auto& face : faces) {
            if (face.vertex_indices.size() < 3) continue;
            const Vec3& v0 = vertices[face.vertex_indices[0]];

            for (size_t k = 1; k + 1 < face.vertex_indices.size(); ++k) {
                const Vec3& vk = vertices[face.vertex_indices[k]];
                const Vec3& vk1 = vertices[face.vertex_indices[k + 1]];

                Vec3 a = v0 - ref;
                Vec3 b = vk - ref;
                Vec3 c = vk1 - ref;

                float tet_vol = (1.0f / 6.0f) * a.dot(b.cross(c));
                Vec3 tet_centroid = 0.25f * (ref + v0 + vk + vk1);

                total_vol += tet_vol;
                weighted_centroid_sum += tet_vol * tet_centroid;
            }
        }

        if (std::abs(total_vol) <= GEOM_EPSILON) {
            return ref;
        }

        return weighted_centroid_sum / total_vol;
    }

    void weld_vertices(float eps = GEOM_EPSILON) {
        if (vertices.empty()) return;

        std::vector<Vec3> unique_verts;
        std::vector<uint32_t> remap(vertices.size());

        for (size_t i = 0; i < vertices.size(); ++i) {
            const Vec3& v = vertices[i];
            int match_idx = -1;
            for (size_t u = 0; u < unique_verts.size(); ++u) {
                if ((v - unique_verts[u]).length_sq() <= eps * eps) {
                    match_idx = static_cast<int>(u);
                    break;
                }
            }
            if (match_idx >= 0) {
                remap[i] = static_cast<uint32_t>(match_idx);
            } else {
                remap[i] = static_cast<uint32_t>(unique_verts.size());
                unique_verts.push_back(v);
            }
        }

        vertices = std::move(unique_verts);

        std::vector<Face> clean_faces;
        for (auto& face : faces) {
            std::vector<uint32_t> new_indices;
            for (uint32_t old_idx : face.vertex_indices) {
                uint32_t new_idx = remap[old_idx];
                if (new_indices.empty() || new_indices.back() != new_idx) {
                    new_indices.push_back(new_idx);
                }
            }
            // Remove wrap-around duplicate
            if (new_indices.size() > 1 && new_indices.front() == new_indices.back()) {
                new_indices.pop_back();
            }

            if (new_indices.size() >= 3) {
                face.vertex_indices = std::move(new_indices);
                clean_faces.push_back(face);
            }
        }

        faces = std::move(clean_faces);
    }
};

} // namespace destruction::fracture

#endif // DESTRUCTION_POLYHEDRON_HPP
