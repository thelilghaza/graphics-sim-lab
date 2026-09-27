#ifndef DESTRUCTION_SEQUENTIAL_IMPULSE_SOLVER_HPP
#define DESTRUCTION_SEQUENTIAL_IMPULSE_SOLVER_HPP

#include "destruction/dynamics/rigid_body.hpp"
#include "destruction/collision/contact_manifold.hpp"
#include "destruction/solver/solver_settings.hpp"
#include "destruction/solver/contact_constraint.hpp"
#include "destruction/solver/warm_start_cache.hpp"
#include <vector>
#include <map>
#include <algorithm>
#include <cmath>

namespace destruction::solver {

using namespace destruction::math;
using namespace destruction::dynamics;
using namespace destruction::collision;

class SequentialImpulseSolver {
private:
    SolverSettings settings_;
    WarmStartCache warm_start_cache_;

public:
    SequentialImpulseSolver() = default;

    explicit SequentialImpulseSolver(const SolverSettings& settings)
        : settings_(settings) {}

    void set_settings(const SolverSettings& settings) {
        settings_ = settings;
    }

    const SolverSettings& get_settings() const {
        return settings_;
    }

    WarmStartCache& get_warm_start_cache() {
        return warm_start_cache_;
    }

    const WarmStartCache& get_warm_start_cache() const {
        return warm_start_cache_;
    }

    void solve(
        std::vector<RigidBody>& bodies,
        std::vector<ContactManifold>& manifolds,
        float dt
    ) {
        if (dt <= 0.0f) return;

        // 1. Sort manifolds deterministically
        std::sort(manifolds.begin(), manifolds.end(), [](const ContactManifold& m1, const ContactManifold& m2) {
            uint32_t min1 = std::min(m1.body_a_id, m1.body_b_id);
            uint32_t max1 = std::max(m1.body_a_id, m1.body_b_id);
            uint32_t min2 = std::min(m2.body_a_id, m2.body_b_id);
            uint32_t max2 = std::max(m2.body_a_id, m2.body_b_id);
            if (min1 != min2) return min1 < min2;
            return max1 < max2;
        });

        // Fast lookup map for body pointer by ID
        std::map<uint32_t, RigidBody*> body_map;
        for (auto& body : bodies) {
            body_map[body.id] = &body;
        }

        // 2. Build contact constraints
        std::vector<ContactConstraint> constraints;
        for (const auto& manifold : manifolds) {
            auto it_a = body_map.find(manifold.body_a_id);
            auto it_b = body_map.find(manifold.body_b_id);
            if (it_a == body_map.end() || it_b == body_map.end()) continue;

            RigidBody* body_a = it_a->second;
            RigidBody* body_b = it_b->second;

            if (body_a->is_static && body_b->is_static) continue;

            for (const auto& cp : manifold.points) {
                ContactConstraint constraint;
                constraint.init(
                    body_a,
                    body_b,
                    cp,
                    manifold.normal,
                    settings_.default_friction,
                    settings_.default_restitution,
                    settings_,
                    dt
                );

                // Warm start lookup
                float cached_n = 0.0f, cached_t1 = 0.0f, cached_t2 = 0.0f;
                if (warm_start_cache_.find(constraint.warm_start_key, cached_n, cached_t1, cached_t2)) {
                    constraint.accumulated_normal_impulse = cached_n;
                    constraint.accumulated_tangent_impulse_1 = cached_t1;
                    constraint.accumulated_tangent_impulse_2 = cached_t2;
                    constraint.apply_warm_start();
                }

                constraints.push_back(constraint);
            }
        }

        // 3. Iterative Gauss-Seidel Velocity Solver
        for (int iter = 0; iter < settings_.velocity_iterations; ++iter) {
            for (auto& constraint : constraints) {
                constraint.solve_velocity_normal();
            }
            for (auto& constraint : constraints) {
                constraint.solve_velocity_friction();
            }
        }

        // Cap linear & angular velocity bounds if needed for stability
        for (auto& body : bodies) {
            if (body.is_static) continue;
            float w_mag = body.angular_velocity.length();
            if (w_mag > settings_.max_angular_velocity && w_mag > 0.0f) {
                body.angular_velocity *= (settings_.max_angular_velocity / w_mag);
            }
        }

        // 4. Position Integration (Physical state update from velocity)
        for (auto& body : bodies) {
            if (body.is_static) continue;

            body.position += body.linear_velocity * dt;

            Vec3 w = body.angular_velocity;
            Quat dq(0.0f, w.x, w.y, w.z);
            Quat q_new = body.orientation + (dq * body.orientation) * (0.5f * dt);
            body.orientation = q_new.normalize();
        }

        // 5. Position Stabilization via Split Impulses (Pseudo-velocities)
        if (settings_.position_iterations > 0) {
            std::map<uint32_t, Vec3> pseudo_v;
            std::map<uint32_t, Vec3> pseudo_w;

            for (const auto& body : bodies) {
                pseudo_v[body.id] = Vec3::zero();
                pseudo_w[body.id] = Vec3::zero();
            }

            for (int iter = 0; iter < settings_.position_iterations; ++iter) {
                for (auto& constraint : constraints) {
                    Vec3& pv_a = pseudo_v[constraint.body_a->id];
                    Vec3& pw_a = pseudo_w[constraint.body_a->id];
                    Vec3& pv_b = pseudo_v[constraint.body_b->id];
                    Vec3& pw_b = pseudo_w[constraint.body_b->id];

                    constraint.solve_position_split(pv_a, pw_a, pv_b, pw_b);
                }
            }

            // Apply pseudo-velocity adjustments to positions & orientations
            for (auto& body : bodies) {
                if (body.is_static) continue;

                Vec3 pv = pseudo_v[body.id];
                Vec3 pw = pseudo_w[body.id];

                body.position += pv * dt;

                if (pw.length_sq() > 1e-10f) {
                    Quat dpq(0.0f, pw.x, pw.y, pw.z);
                    Quat q_corr = body.orientation + (dpq * body.orientation) * (0.5f * dt);
                    body.orientation = q_corr.normalize();
                }
            }
        }

        // 6. Write back accumulated impulses to warm start cache and age cache
        for (const auto& constraint : constraints) {
            warm_start_cache_.insert(
                constraint.warm_start_key,
                constraint.accumulated_normal_impulse,
                constraint.accumulated_tangent_impulse_1,
                constraint.accumulated_tangent_impulse_2
            );
        }
        warm_start_cache_.age_and_prune(2);
    }
};

} // namespace destruction::solver

#endif // DESTRUCTION_SEQUENTIAL_IMPULSE_SOLVER_HPP
