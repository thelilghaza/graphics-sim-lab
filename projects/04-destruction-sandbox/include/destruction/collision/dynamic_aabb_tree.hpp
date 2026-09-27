#ifndef DESTRUCTION_DYNAMIC_AABB_TREE_HPP
#define DESTRUCTION_DYNAMIC_AABB_TREE_HPP

#include "destruction/collision/aabb.hpp"
#include <vector>
#include <cstdint>
#include <algorithm>

namespace destruction::collision {

struct BroadphasePair {
    uint32_t collider_a{0};
    uint32_t collider_b{0};

    bool operator==(const BroadphasePair& rhs) const {
        return collider_a == rhs.collider_a && collider_b == rhs.collider_b;
    }

    bool operator<(const BroadphasePair& rhs) const {
        if (collider_a != rhs.collider_a) return collider_a < rhs.collider_a;
        return collider_b < rhs.collider_b;
    }
};

struct Node {
    Aabb aabb;
    int parent{-1};
    int left{-1};
    int right{-1};
    int height{0};
    uint32_t collider_id{0};

    bool is_leaf() const {
        return left == -1 && right == -1;
    }
};

class DynamicAabbTree {
private:
    std::vector<Node> nodes_;
    int root_{-1};
    int free_head_{-1};
    size_t node_count_{0};

    int allocate_node() {
        if (free_head_ != -1) {
            int node_idx = free_head_;
            free_head_ = nodes_[node_idx].parent;
            nodes_[node_idx] = Node();
            node_count_++;
            return node_idx;
        }
        nodes_.emplace_back();
        node_count_++;
        return static_cast<int>(nodes_.size() - 1);
    }

    void free_node(int node_idx) {
        nodes_[node_idx].parent = free_head_;
        nodes_[node_idx].left = -1;
        nodes_[node_idx].right = -1;
        free_head_ = node_idx;
        node_count_--;
    }

    void refit_ancestors(int node_idx) {
        while (node_idx != -1) {
            int left = nodes_[node_idx].left;
            int right = nodes_[node_idx].right;

            if (left != -1 && right != -1) {
                nodes_[node_idx].aabb = Aabb::merge(nodes_[left].aabb, nodes_[right].aabb);
                nodes_[node_idx].height = 1 + std::max(nodes_[left].height, nodes_[right].height);
            }

            node_idx = nodes_[node_idx].parent;
        }
    }

    void query_pair_recursive(int na, int nb, std::vector<BroadphasePair>& out_pairs) const {
        if (na == -1 || nb == -1) return;
        if (!nodes_[na].aabb.overlaps(nodes_[nb].aabb)) return;

        bool is_leaf_a = nodes_[na].is_leaf();
        bool is_leaf_b = nodes_[nb].is_leaf();

        if (is_leaf_a && is_leaf_b) {
            uint32_t ca = nodes_[na].collider_id;
            uint32_t cb = nodes_[nb].collider_id;
            if (ca != cb) {
                if (ca > cb) std::swap(ca, cb);
                out_pairs.push_back({ca, cb});
            }
        } else if (is_leaf_a) {
            query_pair_recursive(na, nodes_[nb].left, out_pairs);
            query_pair_recursive(na, nodes_[nb].right, out_pairs);
        } else if (is_leaf_b) {
            query_pair_recursive(nodes_[na].left, nb, out_pairs);
            query_pair_recursive(nodes_[na].right, nb, out_pairs);
        } else {
            query_pair_recursive(nodes_[na].left, nb, out_pairs);
            query_pair_recursive(nodes_[na].right, nb, out_pairs);
        }
    }

    void generate_pairs_recursive(int node, std::vector<BroadphasePair>& out_pairs) const {
        if (node == -1 || nodes_[node].is_leaf()) return;

        int left = nodes_[node].left;
        int right = nodes_[node].right;

        generate_pairs_recursive(left, out_pairs);
        generate_pairs_recursive(right, out_pairs);
        query_pair_recursive(left, right, out_pairs);
    }

public:
    DynamicAabbTree() = default;

    int get_root() const { return root_; }
    size_t size() const { return node_count_; }

    int insert_leaf(uint32_t collider_id, const Aabb& aabb) {
        int leaf = allocate_node();
        nodes_[leaf].aabb = aabb;
        nodes_[leaf].collider_id = collider_id;
        nodes_[leaf].height = 0;

        if (root_ == -1) {
            root_ = leaf;
            return leaf;
        }

        // Find best sibling using surface-area cost heuristic
        int sibling = root_;
        while (!nodes_[sibling].is_leaf()) {
            int left = nodes_[sibling].left;
            int right = nodes_[sibling].right;

            float area = nodes_[sibling].aabb.surface_area();
            Aabb combined = Aabb::merge(nodes_[sibling].aabb, aabb);
            float combined_area = combined.surface_area();

            float cost = 2.0f * combined_area;
            float inheritance_cost = 2.0f * (combined_area - area);

            float cost_left = inheritance_cost + (Aabb::merge(nodes_[left].aabb, aabb).surface_area() - nodes_[left].aabb.surface_area());
            float cost_right = inheritance_cost + (Aabb::merge(nodes_[right].aabb, aabb).surface_area() - nodes_[right].aabb.surface_area());

            if (cost < cost_left && cost < cost_right) {
                break;
            }

            if (cost_left < cost_right) {
                sibling = left;
            } else {
                sibling = right;
            }
        }

        // Create new parent
        int old_parent = nodes_[sibling].parent;
        int new_parent = allocate_node();

        nodes_[new_parent].parent = old_parent;
        nodes_[new_parent].aabb = Aabb::merge(aabb, nodes_[sibling].aabb);
        nodes_[new_parent].height = nodes_[sibling].height + 1;

        if (old_parent != -1) {
            if (nodes_[old_parent].left == sibling) {
                nodes_[old_parent].left = new_parent;
            } else {
                nodes_[old_parent].right = new_parent;
            }
            nodes_[new_parent].left = sibling;
            nodes_[new_parent].right = leaf;
            nodes_[sibling].parent = new_parent;
            nodes_[leaf].parent = new_parent;
        } else {
            nodes_[new_parent].left = sibling;
            nodes_[new_parent].right = leaf;
            nodes_[sibling].parent = new_parent;
            nodes_[leaf].parent = new_parent;
            root_ = new_parent;
        }

        refit_ancestors(nodes_[leaf].parent);
        return leaf;
    }

    void remove_leaf(int leaf) {
        if (leaf == -1) return;

        if (leaf == root_) {
            root_ = -1;
            free_node(leaf);
            return;
        }

        int parent = nodes_[leaf].parent;
        int grand_parent = nodes_[parent].parent;
        int sibling = (nodes_[parent].left == leaf) ? nodes_[parent].right : nodes_[parent].left;

        if (grand_parent != -1) {
            if (nodes_[grand_parent].left == parent) {
                nodes_[grand_parent].left = sibling;
            } else {
                nodes_[grand_parent].right = sibling;
            }
            nodes_[sibling].parent = grand_parent;
            free_node(parent);
            free_node(leaf);

            refit_ancestors(grand_parent);
        } else {
            root_ = sibling;
            nodes_[sibling].parent = -1;
            free_node(parent);
            free_node(leaf);
        }
    }

    void update_leaf(int leaf, const Aabb& new_aabb) {
        if (leaf == -1) return;
        if (nodes_[leaf].aabb.contains(new_aabb.min_pt) && nodes_[leaf].aabb.contains(new_aabb.max_pt)) {
            return;
        }
        uint32_t cid = nodes_[leaf].collider_id;
        remove_leaf(leaf);

        Aabb fattened = new_aabb;
        fattened.fatten(0.1f);
        insert_leaf(cid, fattened);
    }

    void query_aabb(const Aabb& query_box, std::vector<uint32_t>& out_collider_ids) const {
        if (root_ == -1) return;

        std::vector<int> stack;
        stack.push_back(root_);

        while (!stack.empty()) {
            int curr = stack.back();
            stack.pop_back();

            if (nodes_[curr].aabb.overlaps(query_box)) {
                if (nodes_[curr].is_leaf()) {
                    out_collider_ids.push_back(nodes_[curr].collider_id);
                } else {
                    if (nodes_[curr].left != -1) stack.push_back(nodes_[curr].left);
                    if (nodes_[curr].right != -1) stack.push_back(nodes_[curr].right);
                }
            }
        }
    }

    void generate_candidate_pairs(std::vector<BroadphasePair>& out_pairs) const {
        out_pairs.clear();
        generate_pairs_recursive(root_, out_pairs);

        // Deduplicate pairs
        std::sort(out_pairs.begin(), out_pairs.end());
        out_pairs.erase(std::unique(out_pairs.begin(), out_pairs.end()), out_pairs.end());
    }
};

} // namespace destruction::collision

#endif // DESTRUCTION_DYNAMIC_AABB_TREE_HPP
