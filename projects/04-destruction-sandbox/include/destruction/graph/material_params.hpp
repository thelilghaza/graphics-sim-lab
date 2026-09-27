#ifndef DESTRUCTION_MATERIAL_PARAMS_HPP
#define DESTRUCTION_MATERIAL_PARAMS_HPP

namespace destruction::graph {

struct MaterialParams {
    float density{2400.0f};           // kg/m^3 (concrete/stone material density)
    float restitution{0.2f};          // Coefficient of restitution
    float friction{0.4f};             // Coefficient of friction
    float tensile_strength{1.0e5f};   // Pa (structural tensile capacity)
    float shear_strength{1.0e5f};     // Pa (structural shear capacity)
    float capacity_multiplier{10.0f}; // Material strength multiplier for support edges
};

} // namespace destruction::graph

#endif // DESTRUCTION_MATERIAL_PARAMS_HPP
