#ifndef DESTRUCTION_STRUCTURAL_GRAPH_HPP
#define DESTRUCTION_STRUCTURAL_GRAPH_HPP

#include "destruction/math/vec3.hpp"
#include "destruction/dynamics/rigid_body.hpp"
#include "destruction/collision/contact_manifold.hpp"
#include "destruction/graph/material_params.hpp"
#include <vector>
#include <queue>
#include <map>
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace destruction::graph {

using namespace destruction::math;
using namespace destruction::dynamics;
using namespace destruction::collision;

struct StructuralNode {
    uint32_t body_id{0};
    float mass{0.0f};
    Vec3 position{Vec3::zero()};
    bool is_static{false};
    bool is_anchored{false}; // Static or ground-resting node
    float total_load{0.0f};
    bool is_supported{true};
};

struct SupportEdge {
    uint32_t node_a_id{0}; // Upper node receiving support
    uint32_t node_b_id{0}; // Lower node providing support reaction
    Vec3 contact_normal{Vec3::unit_y()}; // Pointing from A to B (so reaction on A is -normal)
    float contact_area{0.01f};
    float capacity{1000.0f};
    float transmitted_load{0.0f};
    bool is_broken{false};
};

class StructuralGraph {
private:
    std::vector<StructuralNode> nodes_;
    std::vector<SupportEdge> edges_;
    MaterialParams material_;
    float support_angle_threshold_{0.3f}; // Minimum upward y component of support reaction
    float ground_y_threshold_{0.05f};     // Threshold position for ground-anchored shards

public:
    StructuralGraph() = default;

    explicit StructuralGraph(const MaterialParams& material)
        : material_(material) {}

    void set_material_params(const MaterialParams& params) {
        material_ = params;
    }

    const MaterialParams& get_material_params() const {
        return material_;
    }

    void set_support_angle_threshold(float thresh) {
        support_angle_threshold_ = thresh;
    }

    void set_ground_y_threshold(float thresh) {
        ground_y_threshold_ = thresh;
    }

    const std::vector<StructuralNode>& get_nodes() const { return nodes_; }
    const std::vector<SupportEdge>& get_edges() const { return edges_; }

    size_t supported_count() const {
        size_t count = 0;
        for (const auto& n : nodes_) {
            if (n.is_supported) count++;
        }
        return count;
    }

    size_t unsupported_count() const {
        size_t count = 0;
        for (const auto& n : nodes_) {
            if (!n.is_supported) count++;
        }
        return count;
    }

    size_t broken_edge_count() const {
        size_t count = 0;
        for (const auto& e : edges_) {
            if (e.is_broken) count++;
        }
        return count;
    }

    void build_from_world_and_contacts(
        const std::vector<RigidBody>& bodies,
        const std::vector<ContactManifold>& manifolds
    ) {
        nodes_.clear();
        edges_.clear();

        // 1. Build Nodes
        for (const auto& body : bodies) {
            StructuralNode node;
            node.body_id = body.id;
            node.mass = body.mass;
            node.position = body.position;
            node.is_static = body.is_static;
            node.is_anchored = body.is_static || (body.position.y <= ground_y_threshold_);
            node.total_load = 0.0f;
            node.is_supported = node.is_anchored;
            nodes_.push_back(node);
        }

        // Sort nodes deterministically by body_id
        std::sort(nodes_.begin(), nodes_.end(), [](const StructuralNode& a, const StructuralNode& b) {
            return a.body_id < b.body_id;
        });

        // Fast node lookup index
        std::map<uint32_t, size_t> node_map;
        for (size_t i = 0; i < nodes_.size(); ++i) {
            node_map[nodes_[i].body_id] = i;
        }

        // 2. Build Support Edges from Eligible Contacts
        for (const auto& manifold : manifolds) {
            auto it_a = node_map.find(manifold.body_a_id);
            auto it_b = node_map.find(manifold.body_b_id);
            if (it_a == node_map.end() || it_b == node_map.end()) continue;

            // manifold.normal points from A to B.
            // Reaction on A is -manifold.normal, reaction on B is +manifold.normal.
            Vec3 normal = manifold.normal.normalize();
            float reaction_on_a_y = -normal.y;
            float reaction_on_b_y = normal.y;

            uint32_t upper_id = 0;
            uint32_t lower_id = 0;

            if (reaction_on_a_y > support_angle_threshold_) {
                // B supports A
                upper_id = manifold.body_a_id;
                lower_id = manifold.body_b_id;
            } else if (reaction_on_b_y > support_angle_threshold_) {
                // A supports B
                upper_id = manifold.body_b_id;
                lower_id = manifold.body_a_id;
            } else {
                // Sideways contact does not constitute a vertical support edge
                continue;
            }

            // Estimate effective support contact patch area
            float patch_area = static_cast<float>(manifold.points.size()) * 0.01f;
            if (manifold.points.size() > 1) {
                // Compute convex bounding box area of contact points in local tangent plane
                float min_x = manifold.points[0].position_world.x;
                float max_x = min_x;
                float min_z = manifold.points[0].position_world.z;
                float max_z = min_z;
                for (size_t k = 1; k < manifold.points.size(); ++k) {
                    min_x = std::min(min_x, manifold.points[k].position_world.x);
                    max_x = std::max(max_x, manifold.points[k].position_world.x);
                    min_z = std::min(min_z, manifold.points[k].position_world.z);
                    max_z = std::max(max_z, manifold.points[k].position_world.z);
                }
                float area_calc = (max_x - min_x + 0.02f) * (max_z - min_z + 0.02f);
                patch_area = std::max(patch_area, area_calc);
            }

            float capacity = patch_area * material_.tensile_strength * material_.capacity_multiplier;

            // Check if an edge between upper and lower already exists
            bool found_existing = false;
            for (auto& edge : edges_) {
                if (edge.node_a_id == upper_id && edge.node_b_id == lower_id) {
                    edge.contact_area += patch_area;
                    edge.capacity += capacity;
                    found_existing = true;
                    break;
                }
            }

            if (!found_existing) {
                SupportEdge edge;
                edge.node_a_id = upper_id;
                edge.node_b_id = lower_id;
                edge.contact_normal = normal;
                edge.contact_area = patch_area;
                edge.capacity = capacity;
                edge.transmitted_load = 0.0f;
                edge.is_broken = false;
                edges_.push_back(edge);
            }
        }

        // Sort edges deterministically by (node_a_id, node_b_id)
        std::sort(edges_.begin(), edges_.end(), [](const SupportEdge& e1, const SupportEdge& e2) {
            if (e1.node_a_id != e2.node_a_id) return e1.node_a_id < e2.node_a_id;
            return e1.node_b_id < e2.node_b_id;
        });
    }

    void evaluate_load_and_connectivity(const Vec3& gravity) {
        if (nodes_.empty()) return;

        float g_mag = gravity.length();

        // Initialize node base loads from mass * g
        for (auto& node : nodes_) {
            node.total_load = node.mass * g_mag;
        }

        for (auto& edge : edges_) {
            edge.transmitted_load = 0.0f;
            edge.is_broken = false;
        }

        // 1. Iterative Load Propagation Sweep
        constexpr int max_sweeps = 50;
        for (int sweep = 0; sweep < max_sweeps; ++sweep) {
            bool any_new_broken = false;

            // Clear node loads back to intrinsic weight
            for (auto& node : nodes_) {
                node.total_load = node.mass * g_mag;
            }

            // Propagate load from upper nodes down through active support edges
            for (auto& node : nodes_) {
                if (node.is_static) continue;

                // Find active outgoing support edges where this node is node_a (upper)
                std::vector<size_t> active_edge_indices;
                for (size_t i = 0; i < edges_.size(); ++i) {
                    if (edges_[i].node_a_id == node.body_id && !edges_[i].is_broken) {
                        active_edge_indices.push_back(i);
                    }
                }

                if (active_edge_indices.empty()) continue;

                float load_per_edge = node.total_load / static_cast<float>(active_edge_indices.size());
                for (size_t idx : active_edge_indices) {
                    edges_[idx].transmitted_load = load_per_edge;

                    // Add load to support node_b
                    uint32_t b_id = edges_[idx].node_b_id;
                    for (auto& target_node : nodes_) {
                        if (target_node.body_id == b_id) {
                            target_node.total_load += load_per_edge;
                            break;
                        }
                    }
                }
            }

            // Evaluate capacity failure for all active edges
            for (auto& edge : edges_) {
                if (!edge.is_broken && edge.transmitted_load > edge.capacity) {
                    edge.is_broken = true;
                    any_new_broken = true;
                }
            }

            if (!any_new_broken) break;
        }

        // 2. Recompute Connectivity from Anchored/Root Nodes
        for (auto& node : nodes_) {
            node.is_supported = node.is_anchored;
        }

        std::queue<uint32_t> bfs_queue;
        for (const auto& node : nodes_) {
            if (node.is_anchored) {
                bfs_queue.push(node.body_id);
            }
        }

        while (!bfs_queue.empty()) {
            uint32_t root_id = bfs_queue.front();
            bfs_queue.pop();

            // Find all active edges where root_id acts as support provider (node_b_id)
            for (const auto& edge : edges_) {
                if (edge.node_b_id == root_id && !edge.is_broken) {
                    uint32_t upper_id = edge.node_a_id;
                    for (auto& upper_node : nodes_) {
                        if (upper_node.body_id == upper_id && !upper_node.is_supported) {
                            upper_node.is_supported = true;
                            bfs_queue.push(upper_id);
                        }
                    }
                }
            }
        }
    }
};

} // namespace destruction::graph

#endif // DESTRUCTION_STRUCTURAL_GRAPH_HPP
