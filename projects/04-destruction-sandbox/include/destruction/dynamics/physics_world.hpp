#ifndef DESTRUCTION_PHYSICS_WORLD_HPP
#define DESTRUCTION_PHYSICS_WORLD_HPP

#include "destruction/dynamics/rigid_body.hpp"
#include "destruction/dynamics/integrator.hpp"
#include "destruction/math/vec3.hpp"
#include <vector>
#include <cstdint>
#include <algorithm>

namespace destruction::dynamics {

using namespace destruction::math;

class PhysicsWorld {
private:
    std::vector<RigidBody> bodies_;
    Vec3 gravity_{0.0f, -9.81f, 0.0f};

public:
    PhysicsWorld() = default;

    void set_gravity(const Vec3& g) {
        gravity_ = g;
    }

    Vec3 get_gravity() const {
        return gravity_;
    }

    uint32_t add_body(const RigidBody& body) {
        bodies_.push_back(body);
        return body.id;
    }

    size_t body_count() const {
        return bodies_.size();
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

    std::vector<RigidBody>& get_bodies() {
        return bodies_;
    }

    const std::vector<RigidBody>& get_bodies() const {
        return bodies_;
    }

    void clear() {
        bodies_.clear();
    }

    void step(float dt) {
        if (dt <= 0.0f) return;

        // Deterministic single-threaded step loop
        for (auto& body : bodies_) {
            if (!body.is_static && body.inv_mass > 0.0f) {
                body.apply_force(gravity_ * body.mass);
            }
            Integrator::step_symplectic_euler(body, dt);
            body.clear_accumulators();
        }
    }
};

} // namespace destruction::dynamics

#endif // DESTRUCTION_PHYSICS_WORLD_HPP
