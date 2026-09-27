#ifndef DESTRUCTION_RENDERER_HPP
#define DESTRUCTION_RENDERER_HPP

#include "destruction/render/gl_loader.hpp"
#include "destruction/render/shader.hpp"
#include "destruction/render/camera.hpp"
#include "destruction/render/gl_mesh.hpp"
#include "destruction/dynamics/physics_world.hpp"
#include "destruction/collision/collider.hpp"
#include "destruction/collision/aabb.hpp"
#include "destruction/graph/structural_graph.hpp"

#include <unordered_map>
#include <memory>
#include <vector>
#include <array>

namespace destruction::render {

using namespace destruction::dynamics;
using namespace destruction::collision;
using namespace destruction::graph;

struct RenderFlags {
    bool wireframe{false};
    bool show_aabbs{false};
    bool show_contacts{true};
    bool show_support_graph{true};
    bool show_centers_of_mass{false};
};

class Renderer {
private:
    std::unique_ptr<Shader> lit_shader_;
    std::unique_ptr<Shader> flat_shader_;

    std::unordered_map<uint32_t, GLMesh> mesh_cache_;
    GLMesh line_mesh_;

    static std::array<float, 16> transform_to_mat4(const Transform& t) {
        Mat3 r = t.orientation.to_mat3();
        std::array<float, 16> m{};

        // Column 0
        m[0] = r(0, 0); m[1] = r(1, 0); m[2] = r(2, 0); m[3] = 0.0f;
        // Column 1
        m[4] = r(0, 1); m[5] = r(1, 1); m[6] = r(2, 1); m[7] = 0.0f;
        // Column 2
        m[8] = r(0, 2); m[9] = r(1, 2); m[10] = r(2, 2); m[11] = 0.0f;
        // Column 3
        m[12] = t.position.x; m[13] = t.position.y; m[14] = t.position.z; m[15] = 1.0f;

        return m;
    }

    static std::array<float, 16> identity_mat4() {
        std::array<float, 16> m{};
        m[0] = 1.0f; m[5] = 1.0f; m[10] = 1.0f; m[15] = 1.0f;
        return m;
    }

public:
    Renderer() = default;

    bool init() {
        const char* lit_vs = R"(#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec4 aColor;

out vec3 vNormal;
out vec3 vFragPos;
out vec4 vColor;

uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProj;

void main() {
    vec4 worldPos = uModel * vec4(aPos, 1.0);
    vFragPos = worldPos.xyz;
    vNormal = mat3(uModel) * aNormal;
    vColor = aColor;
    gl_Position = uProj * uView * worldPos;
}
)";

        const char* lit_fs = R"(#version 330 core
in vec3 vNormal;
in vec3 vFragPos;
in vec4 vColor;

out vec4 FragColor;

uniform vec3 uLightDir;
uniform vec3 uViewPos;

void main() {
    vec3 norm = normalize(vNormal);
    vec3 lightDir = normalize(uLightDir);

    // Ambient
    float ambientStrength = 0.3;
    vec3 ambient = ambientStrength * vec3(1.0, 1.0, 1.0);

    // Diffuse
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * vec3(0.9, 0.9, 0.85);

    // Specular
    float specularStrength = 0.2;
    vec3 viewDir = normalize(uViewPos - vFragPos);
    vec3 reflectDir = reflect(-lightDir, norm);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 16.0);
    vec3 specular = specularStrength * spec * vec3(1.0, 1.0, 1.0);

    vec3 result = (ambient + diffuse + specular) * vColor.rgb;
    FragColor = vec4(result, vColor.a);
}
)";

        const char* flat_vs = R"(#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec4 aColor;

out vec4 vColor;

uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProj;
uniform vec4 uColorOverride;

void main() {
    if (uColorOverride.a > 0.0) {
        vColor = uColorOverride;
    } else {
        vColor = aColor;
    }
    gl_Position = uProj * uView * (uModel * vec4(aPos, 1.0));
}
)";

        const char* flat_fs = R"(#version 330 core
in vec4 vColor;
out vec4 FragColor;

void main() {
    FragColor = vColor;
}
)";

        lit_shader_ = std::make_unique<Shader>(lit_vs, lit_fs);
        flat_shader_ = std::make_unique<Shader>(flat_vs, flat_fs);

        if (!lit_shader_->is_valid() || !flat_shader_->is_valid()) {
            std::cerr << "[Renderer Error] Failed to compile shaders.\n";
            return false;
        }

        return true;
    }

    void clear_cache() {
        mesh_cache_.clear();
    }

    void render_scene(
        const PhysicsWorld& world,
        const Camera& camera,
        float aspect_ratio,
        const RenderFlags& flags
    ) {
        if (!lit_shader_ || !flat_shader_) return;

        std::array<float, 16> view = camera.get_view_matrix();
        std::array<float, 16> proj = camera.get_projection_matrix(aspect_ratio);

        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);

        // 1. Lit Shader Pass for Solid Bodies
        lit_shader_->use();
        lit_shader_->set_mat4("uView", view.data());
        lit_shader_->set_mat4("uProj", proj.data());
        lit_shader_->set_vec3("uLightDir", Vec3(0.4f, 1.0f, 0.5f));
        lit_shader_->set_vec3("uViewPos", camera.position);

        if (flags.wireframe) {
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        } else {
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        }

        for (const auto& col : world.get_colliders()) {
            auto it = mesh_cache_.find(col.id);
            if (it == mesh_cache_.end()) {
                // Generate and cache mesh
                if (col.type == ColliderType::Box) {
                    float r = 0.75f, g = 0.78f, b = 0.82f;
                    if (col.is_static) {
                        r = 0.35f; g = 0.40f; b = 0.45f; // Ground
                    } else if (col.body_id >= 1000) {
                        r = 0.95f; g = 0.35f; b = 0.20f; // Projectile
                    }
                    mesh_cache_[col.id] = GLMesh::create_box(col.box_half_extents, r, g, b);
                } else if (col.type == ColliderType::ConvexPolyhedron) {
                    // Shard colors: subtle varied tones based on id
                    uint32_t cid = col.id;
                    float r = 0.55f + 0.3f * std::sin(static_cast<float>(cid) * 1.7f);
                    float g = 0.60f + 0.25f * std::cos(static_cast<float>(cid) * 2.3f);
                    float b = 0.65f + 0.25f * std::sin(static_cast<float>(cid) * 3.1f);
                    mesh_cache_[col.id] = GLMesh::create_polyhedron(col.mesh, r, g, b);
                }
                it = mesh_cache_.find(col.id);
            }

            if (it != mesh_cache_.end()) {
                std::array<float, 16> model = transform_to_mat4(col.world_transform);
                lit_shader_->set_mat4("uModel", model.data());
                it->second.draw_triangles();
            }
        }

        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        // 2. Debug Line Overlays
        std::vector<GLVertex> debug_lines;
        std::array<float, 16> ident = identity_mat4();

        // 2A. AABBs
        if (flags.show_aabbs) {
            for (const auto& col : world.get_colliders()) {
                const Aabb& b = col.world_aabb;
                if (b.is_empty()) continue;

                Vec3 p000(b.min_pt.x, b.min_pt.y, b.min_pt.z);
                Vec3 p100(b.max_pt.x, b.min_pt.y, b.min_pt.z);
                Vec3 p110(b.max_pt.x, b.max_pt.y, b.min_pt.z);
                Vec3 p010(b.min_pt.x, b.max_pt.y, b.min_pt.z);
                Vec3 p001(b.min_pt.x, b.min_pt.y, b.max_pt.z);
                Vec3 p101(b.max_pt.x, b.min_pt.y, b.max_pt.z);
                Vec3 p111(b.max_pt.x, b.max_pt.y, b.max_pt.z);
                Vec3 p011(b.min_pt.x, b.max_pt.y, b.max_pt.z);

                auto add_line = [&](const Vec3& p1, const Vec3& p2) {
                    debug_lines.emplace_back(p1, Vec3::zero(), 0.9f, 0.7f, 0.1f, 1.0f);
                    debug_lines.emplace_back(p2, Vec3::zero(), 0.9f, 0.7f, 0.1f, 1.0f);
                };

                add_line(p000, p100); add_line(p100, p110); add_line(p110, p010); add_line(p010, p000);
                add_line(p001, p101); add_line(p101, p111); add_line(p111, p011); add_line(p011, p001);
                add_line(p000, p001); add_line(p100, p101); add_line(p110, p111); add_line(p010, p011);
            }
        }

        // 2B. Contact Points & Normals
        if (flags.show_contacts) {
            for (const auto& manifold : world.get_active_manifolds()) {
                Vec3 norm = manifold.normal.normalize();
                for (const auto& cp : manifold.points) {
                    Vec3 p = cp.position_world;
                    // Contact point cross marker
                    float s = 0.04f;
                    debug_lines.emplace_back(p - Vec3(s, 0, 0), Vec3::zero(), 1.0f, 0.1f, 0.1f, 1.0f);
                    debug_lines.emplace_back(p + Vec3(s, 0, 0), Vec3::zero(), 1.0f, 0.1f, 0.1f, 1.0f);
                    debug_lines.emplace_back(p - Vec3(0, s, 0), Vec3::zero(), 1.0f, 0.1f, 0.1f, 1.0f);
                    debug_lines.emplace_back(p + Vec3(0, s, 0), Vec3::zero(), 1.0f, 0.1f, 0.1f, 1.0f);
                    debug_lines.emplace_back(p - Vec3(0, 0, s), Vec3::zero(), 1.0f, 0.1f, 0.1f, 1.0f);
                    debug_lines.emplace_back(p + Vec3(0, 0, s), Vec3::zero(), 1.0f, 0.1f, 0.1f, 1.0f);

                    // Normal vector line outward from contact point
                    debug_lines.emplace_back(p, Vec3::zero(), 0.2f, 0.8f, 1.0f, 1.0f);
                    debug_lines.emplace_back(p + norm * 0.25f, Vec3::zero(), 0.2f, 0.8f, 1.0f, 1.0f);
                }
            }
        }

        // 2C. Structural Support Edges
        if (flags.show_support_graph) {
            const auto& graph = world.get_graph();
            const auto& nodes = graph.get_nodes();
            std::unordered_map<uint32_t, Vec3> node_positions;
            for (const auto& n : nodes) {
                node_positions[n.body_id] = n.position;
            }

            for (const auto& e : graph.get_edges()) {
                auto it_a = node_positions.find(e.node_a_id);
                auto it_b = node_positions.find(e.node_b_id);
                if (it_a != node_positions.end() && it_b != node_positions.end()) {
                    if (e.is_broken) {
                        // Red line for broken support edge
                        debug_lines.emplace_back(it_a->second, Vec3::zero(), 1.0f, 0.0f, 0.0f, 1.0f);
                        debug_lines.emplace_back(it_b->second, Vec3::zero(), 1.0f, 0.0f, 0.0f, 1.0f);
                    } else {
                        // Green line for active support edge
                        debug_lines.emplace_back(it_a->second, Vec3::zero(), 0.1f, 0.95f, 0.2f, 1.0f);
                        debug_lines.emplace_back(it_b->second, Vec3::zero(), 0.1f, 0.95f, 0.2f, 1.0f);
                    }
                }
            }
        }

        // 2D. Centers of Mass
        if (flags.show_centers_of_mass) {
            for (const auto& body : world.get_bodies()) {
                Vec3 p = body.position;
                float s = 0.08f;
                debug_lines.emplace_back(p - Vec3(s, 0, 0), Vec3::zero(), 1.0f, 1.0f, 0.0f, 1.0f);
                debug_lines.emplace_back(p + Vec3(s, 0, 0), Vec3::zero(), 1.0f, 1.0f, 0.0f, 1.0f);
                debug_lines.emplace_back(p - Vec3(0, s, 0), Vec3::zero(), 1.0f, 1.0f, 0.0f, 1.0f);
                debug_lines.emplace_back(p + Vec3(0, s, 0), Vec3::zero(), 1.0f, 1.0f, 0.0f, 1.0f);
                debug_lines.emplace_back(p - Vec3(0, 0, s), Vec3::zero(), 1.0f, 1.0f, 0.0f, 1.0f);
                debug_lines.emplace_back(p + Vec3(0, 0, s), Vec3::zero(), 1.0f, 1.0f, 0.0f, 1.0f);
            }
        }

        // Render debug lines
        if (!debug_lines.empty()) {
            flat_shader_->use();
            flat_shader_->set_mat4("uView", view.data());
            flat_shader_->set_mat4("uProj", proj.data());
            flat_shader_->set_mat4("uModel", ident.data());
            flat_shader_->set_vec4("uColorOverride", 0.0f, 0.0f, 0.0f, 0.0f); // Use vertex colors

            line_mesh_.upload_lines(debug_lines);
            line_mesh_.draw_lines();
        }
    }
};

} // namespace destruction::render

#endif // DESTRUCTION_RENDERER_HPP
