#ifndef DESTRUCTION_MESH_VALIDATOR_HPP
#define DESTRUCTION_MESH_VALIDATOR_HPP

#include "destruction/math/vec3.hpp"
#include "destruction/math/math_utils.hpp"
#include "destruction/fracture/polyhedron.hpp"
#include <map>
#include <set>
#include <utility>
#include <cmath>

namespace destruction::fracture {

using namespace destruction::math;

struct MeshValidationResult {
    bool is_valid{true};
    bool is_closed_manifold{true};
    bool has_duplicate_vertices{false};
    bool has_unreferenced_vertices{false};
    bool has_zero_area_faces{false};
    size_t boundary_edge_count{0};
    size_t invalid_edge_count{0};
};

class MeshValidator {
public:
    static MeshValidationResult validate(const ConvexPolyhedron& poly, float eps = GEOM_EPSILON) {
        MeshValidationResult result;

        // 1. Structural Checks
        if (poly.vertices.size() < 4 || poly.faces.size() < 4) {
            result.is_valid = false;
            return result;
        }

        // Check vertex finiteness & index bounds
        for (const auto& v : poly.vertices) {
            if (!v.is_valid()) {
                result.is_valid = false;
                return result;
            }
        }

        std::set<uint32_t> referenced_indices;

        for (const auto& face : poly.faces) {
            if (face.vertex_indices.size() < 3) {
                result.has_zero_area_faces = true;
                result.is_valid = false;
            }
            for (uint32_t idx : face.vertex_indices) {
                if (idx >= poly.vertices.size()) {
                    result.is_valid = false;
                }
                referenced_indices.insert(idx);
            }
        }

        if (referenced_indices.size() < poly.vertices.size()) {
            result.has_unreferenced_vertices = true;
        }

        // 2. Geometric Duplicate Vertices
        for (size_t i = 0; i < poly.vertices.size(); ++i) {
            for (size_t j = i + 1; j < poly.vertices.size(); ++j) {
                if ((poly.vertices[i] - poly.vertices[j]).length_sq() <= eps * eps) {
                    result.has_duplicate_vertices = true;
                    break;
                }
            }
        }

        // 3. Topological Closed Manifold Edge Counting
        using EdgeKey = std::pair<uint32_t, uint32_t>;
        std::map<EdgeKey, size_t> edge_use_count;

        for (const auto& face : poly.faces) {
            size_t n = face.vertex_indices.size();
            for (size_t i = 0; i < n; ++i) {
                uint32_t a = face.vertex_indices[i];
                uint32_t b = face.vertex_indices[(i + 1) % n];
                if (a > b) std::swap(a, b);
                edge_use_count[EdgeKey(a, b)]++;
            }
        }

        for (const auto& [edge, count] : edge_use_count) {
            if (count != 2) {
                result.is_closed_manifold = false;
                if (count == 1) {
                    result.boundary_edge_count++;
                } else {
                    result.invalid_edge_count++;
                }
            }
        }

        if (poly.volume() <= eps) {
            result.is_valid = false;
        }

        result.is_valid = result.is_valid && result.is_closed_manifold && !result.has_duplicate_vertices;
        return result;
    }
};

} // namespace destruction::fracture

#endif // DESTRUCTION_MESH_VALIDATOR_HPP
