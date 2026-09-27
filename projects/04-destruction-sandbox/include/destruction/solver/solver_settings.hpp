#ifndef DESTRUCTION_SOLVER_SETTINGS_HPP
#define DESTRUCTION_SOLVER_SETTINGS_HPP

namespace destruction::solver {

struct SolverSettings {
    int velocity_iterations{10};
    int position_iterations{5};

    float penetration_slop{0.005f};     // 5mm allowed penetration slop before position stabilization
    float baumgarte_beta{0.2f};          // Stabilization coefficient (20%)
    float restitution_threshold{0.5f};   // Velocity threshold (m/s) below which contacts are inelastic
    float default_friction{0.3f};        // Coefficient of Coulomb friction
    float default_restitution{0.2f};     // Coefficient of restitution
    float impulse_epsilon{1e-6f};        // Convergence threshold for solver impulses
    float max_angular_velocity{50.0f};   // Angular velocity clamp (rad/s)
    float max_position_correction{0.2f}; // Maximum position adjustment per frame (m)
};

} // namespace destruction::solver

#endif // DESTRUCTION_SOLVER_SETTINGS_HPP
