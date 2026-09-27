#ifndef DESTRUCTION_PHYSICS_WORLD_HPP
#define DESTRUCTION_PHYSICS_WORLD_HPP

#include "destruction/dynamics/rigid_body.hpp"
#include "destruction/dynamics/integrator.hpp"
#include "destruction/math/vec3.hpp"
#include "destruction/collision/collider.hpp"
#include "destruction/collision/dynamic_aabb_tree.hpp"
#include "destruction/collision/narrowphase.hpp"
#include "destruction/collision/contact_manifold.hpp"
#include "destruction/solver/sequential_impulse_solver.hpp"
#include "destruction/graph/structural_graph.hpp"

#include <vector>
#include <map>
#include <cstdint>
#include <algorithm>

namespace destruction::dynamics {

using namespace destruction::math;
using namespace destruction::collision;
using namespace destruction::solver;
using namespace destruction::graph;

class PhysicsWorld {
private:
    std::vector<RigidBody> bodies_;
    std::vector<Collider> colliders_;
    DynamicAabbTree broadphase_;
    SequentialImpulseSolver solver_;
    StructuralGraph graph_;
    std::vector<ContactManifold> active_manifolds_;

    Vec3 gravity_{0.0f, -9.81f, 0.0f};
    bool solver_enabled_{true};
    bool graph_enabled_{true};

public:
    PhysicsWorld() = default;

    void set_gravity(const Vec3& g) {
        gravity_ = g;
    }

    Vec3 get_gravity() const {
        return gravity_;
    }

    void set_solver_enabled(bool enabled) {
        solver_enabled_ = enabled;
    }

    bool is_solver_enabled() const {
        return solver_enabled_;
    }

    void set_graph_enabled(bool enabled) {
        graph_enabled_ = enabled;
    }

    bool is_graph_enabled() const {
        return graph_enabled_;
    }

    void set_solver_settings(const SolverSettings& settings) {
        solver_.set_settings(settings);
    }

    SequentialImpulseSolver& get_solver() {
        return solver_;
    }

    const SequentialImpulseSolver& get_solver() const {
        return solver_;
    }

    StructuralGraph& get_graph() {
        return graph_;
    }

    const StructuralGraph& get_graph() const {
        return graph_;
    }

    const std::vector<ContactManifold>& get_active_manifolds() const {
        return active_manifolds_;
    }

    uint32_t add_body(const RigidBody& body) {
        bodies_.push_back(body);
        return body.id;
    }

    uint32_t add_collider(const Collider& collider) {
        colliders_.push_back(collider);
        return collider.id;
    }

    size_t body_count() const {
        return bodies_.size();
    }

    size_t collider_count() const {
        return colliders_.size();
    }

    RigidBody* get_body(uint32_t id) {
        for (auto& body : bodies_) {
            if (body.id == id) {
                return &body;
            }
        }
        return nullptr;
    }

    const RigidBody* get_body(uint32_t id) const {
        for (const auto& body : bodies_) {
            if (body.id == id) {
                return &body;
            }
        }
        return nullptr;
    }

    Collider* get_collider(uint32_t id) {
        for (auto& col : colliders_) {
            if (col.id == id) {
                return &col;
            }
        }
        return nullptr;
    }

    const Collider* get_collider(uint32_t id) const {
        for (const auto& col : colliders_) {
            if (col.id == id) {
                return &col;
            }
        }
        return nullptr;
    }

    std::vector<RigidBody>& get_bodies() {
        return bodies_;
    }

    const std::vector<RigidBody>& get_bodies() const {
        return bodies_;
    }

    std::vector<Collider>& get_colliders() {
        return colliders_;
    }

    const std::vector<Collider>& get_colliders() const {
        return colliders_;
    }

    void clear() {
        bodies_.clear();
        colliders_.clear();
        broadphase_ = DynamicAabbTree();
        active_manifolds_.clear();
    }

    // Preservation method for Milestone 1 math/dynamics compatibility
    void step(float dt) {
        if (dt <= 0.0f) return;

        for (auto& body : bodies_) {
            if (!body.is_static && body.inv_mass > 0.0f) {
                body.apply_force(gravity_ * body.mass);
            }
            Integrator::step_symplectic_euler(body, dt);
            body.clear_accumulators();
        }
    }

    // Full Milestone 4 pipeline simulation step
    void step_full(float dt) {
        if (dt <= 0.0f) return;

        // 1. Apply gravity and integrate velocities from forces
        for (auto& body : bodies_) {
            if (!body.is_static && body.inv_mass > 0.0f) {
                body.apply_force(gravity_ * body.mass);
            }
            if (!body.is_static && body.inv_mass > 0.0f) {
                body.linear_velocity += (body.force_accumulator * body.inv_mass) * dt;
                body.angular_velocity += (body.get_world_inv_inertia() * body.torque_accumulator) * dt;
            }
            body.clear_accumulators();
        }

        // Fast body map lookup
        std::map<uint32_t, RigidBody*> body_map;
        for (auto& body : bodies_) {
            body_map[body.id] = &body;
        }

        // 2. Update colliders world transforms from associated bodies
        for (auto& col : colliders_) {
            auto it = body_map.find(col.body_id);
            if (it != body_map.end()) {
                col.world_transform = it->second->get_transform();
            }
        }

        // 3. Update broadphase dynamic AABB tree
        broadphase_ = DynamicAabbTree();
        for (auto& col : colliders_) {
            col.update_world_aabb();
            broadphase_.insert_leaf(col.id, col.world_aabb);
        }

        // 4. Generate candidate pairs & run narrowphase
        std::vector<BroadphasePair> candidate_pairs;
        broadphase_.generate_candidate_pairs(candidate_pairs);

        active_manifolds_.clear();
        for (const auto& pair : candidate_pairs) {
            const Collider* col_a = get_collider(pair.collider_a);
            const Collider* col_b = get_collider(pair.collider_b);
            if (!col_a || !col_b) continue;

            ContactManifold manifold = Narrowphase::collide(*col_a, *col_b);
            if (!manifold.points.empty()) {
                active_manifolds_.push_back(manifold);
            }
        }

        // 5. Run Sequential Impulse Solver if enabled
        if (solver_enabled_) {
            solver_.solve(bodies_, active_manifolds_, dt);
        } else {
            // Unconstrained physical integration fallback
            for (auto& body : bodies_) {
                if (body.is_static) continue;
                body.position += body.linear_velocity * dt;
                Vec3 w = body.angular_velocity;
                Quat dq(0.0f, w.x, w.y, w.z);
                Quat q_new = body.orientation + (dq * body.orientation) * (0.5f * dt);
                body.orientation = q_new.normalize();
            }
        }

        // Re-update colliders world transforms after physical solver position update
        for (auto& col : colliders_) {
            auto it = body_map.find(col.body_id);
            if (it != body_map.end()) {
                col.world_transform = it->second->get_transform();
            }
        }

        // 6. Update Structural Connectivity Graph if enabled
        if (graph_enabled_) {
            graph_.build_from_world_and_contacts(bodies_, active_manifolds_);
            graph_.evaluate_load_and_connectivity(gravity_);
        }
    }
};

} // namespace destruction::dynamics

#endif // DESTRUCTION_PHYSICS_WORLD_HPP
