#ifndef DESTRUCTION_GL_MESH_HPP
#define DESTRUCTION_GL_MESH_HPP

#include "destruction/render/gl_loader.hpp"
#include "destruction/math/vec3.hpp"
#include "destruction/fracture/polyhedron.hpp"
#include <vector>
#include <cstdint>
#include <cmath>

namespace destruction::render {

using namespace destruction::math;
using namespace destruction::fracture;

struct GLVertex {
    Vec3 position{Vec3::zero()};
    Vec3 normal{Vec3::unit_y()};
    float color[4]{1.0f, 1.0f, 1.0f, 1.0f};

    GLVertex() = default;
    GLVertex(const Vec3& pos, const Vec3& norm, float r, float g, float b, float a = 1.0f)
        : position(pos), normal(norm) {
        color[0] = r; color[1] = g; color[2] = b; color[3] = a;
    }
};

class GLMesh {
private:
    GLuint vao_{0};
    GLuint vbo_{0};
    GLuint ebo_{0};
    GLsizei index_count_{0};
    GLsizei vertex_count_{0};

public:
    GLMesh() = default;

    ~GLMesh() {
        destroy();
    }

    void destroy() {
        if (ebo_ != 0) { glDeleteBuffers(1, &ebo_); ebo_ = 0; }
        if (vbo_ != 0) { glDeleteBuffers(1, &vbo_); vbo_ = 0; }
        if (vao_ != 0) { glDeleteVertexArrays(1, &vao_); vao_ = 0; }
        index_count_ = 0;
        vertex_count_ = 0;
    }

    GLMesh(const GLMesh&) = delete;
    GLMesh& operator=(const GLMesh&) = delete;

    GLMesh(GLMesh&& other) noexcept
        : vao_(other.vao_), vbo_(other.vbo_), ebo_(other.ebo_),
          index_count_(other.index_count_), vertex_count_(other.vertex_count_) {
        other.vao_ = 0;
        other.vbo_ = 0;
        other.ebo_ = 0;
        other.index_count_ = 0;
        other.vertex_count_ = 0;
    }

    GLMesh& operator=(GLMesh&& other) noexcept {
        if (this != &other) {
            destroy();
            vao_ = other.vao_;
            vbo_ = other.vbo_;
            ebo_ = other.ebo_;
            index_count_ = other.index_count_;
            vertex_count_ = other.vertex_count_;

            other.vao_ = 0;
            other.vbo_ = 0;
            other.ebo_ = 0;
            other.index_count_ = 0;
            other.vertex_count_ = 0;
        }
        return *this;
    }

    void upload_triangles(const std::vector<GLVertex>& vertices, const std::vector<uint32_t>& indices) {
        destroy();

        vertex_count_ = static_cast<GLsizei>(vertices.size());
        index_count_ = static_cast<GLsizei>(indices.size());

        glGenVertexArrays(1, &vao_);
        glGenBuffers(1, &vbo_);
        glGenBuffers(1, &ebo_);

        glBindVertexArray(vao_);

        glBindBuffer(GL_ARRAY_BUFFER, vbo_);
        glBufferData(GL_ARRAY_BUFFER, sizeof(GLVertex) * vertices.size(), vertices.data(), GL_STATIC_DRAW);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(uint32_t) * indices.size(), indices.data(), GL_STATIC_DRAW);

        // Position: layout (location = 0)
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(GLVertex), reinterpret_cast<void*>(offsetof(GLVertex, position)));
        glEnableVertexAttribArray(0);

        // Normal: layout (location = 1)
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(GLVertex), reinterpret_cast<void*>(offsetof(GLVertex, normal)));
        glEnableVertexAttribArray(1);

        // Color: layout (location = 2)
        glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, sizeof(GLVertex), reinterpret_cast<void*>(offsetof(GLVertex, color)));
        glEnableVertexAttribArray(2);

        glBindVertexArray(0);
    }

    void upload_lines(const std::vector<GLVertex>& vertices) {
        destroy();

        vertex_count_ = static_cast<GLsizei>(vertices.size());
        index_count_ = 0;

        glGenVertexArrays(1, &vao_);
        glGenBuffers(1, &vbo_);

        glBindVertexArray(vao_);

        glBindBuffer(GL_ARRAY_BUFFER, vbo_);
        glBufferData(GL_ARRAY_BUFFER, sizeof(GLVertex) * vertices.size(), vertices.data(), GL_DYNAMIC_DRAW);

        // Position: layout (location = 0)
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(GLVertex), reinterpret_cast<void*>(offsetof(GLVertex, position)));
        glEnableVertexAttribArray(0);

        // Normal: layout (location = 1)
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(GLVertex), reinterpret_cast<void*>(offsetof(GLVertex, normal)));
        glEnableVertexAttribArray(1);

        // Color: layout (location = 2)
        glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, sizeof(GLVertex), reinterpret_cast<void*>(offsetof(GLVertex, color)));
        glEnableVertexAttribArray(2);

        glBindVertexArray(0);
    }

    void draw_triangles() const {
        if (vao_ != 0 && index_count_ > 0) {
            glBindVertexArray(vao_);
            glDrawElements(GL_TRIANGLES, index_count_, GL_UNSIGNED_INT, nullptr);
            glBindVertexArray(0);
        }
    }

    void draw_lines() const {
        if (vao_ != 0 && vertex_count_ > 0) {
            glBindVertexArray(vao_);
            glDrawArrays(GL_LINES, 0, vertex_count_);
            glBindVertexArray(0);
        }
    }

    static GLMesh create_box(const Vec3& half_extents, float r, float g, float b, float a = 1.0f) {
        Vec3 h = half_extents;
        // 6 faces, 4 vertices per face with distinct face normals
        struct FaceDef {
            Vec3 normal;
            Vec3 corners[4];
        };

        FaceDef faces[6] = {
            // Front (+Z)
            { Vec3(0, 0, 1), { Vec3(-h.x, -h.y,  h.z), Vec3( h.x, -h.y,  h.z), Vec3( h.x,  h.y,  h.z), Vec3(-h.x,  h.y,  h.z) } },
            // Back (-Z)
            { Vec3(0, 0, -1), { Vec3( h.x, -h.y, -h.z), Vec3(-h.x, -h.y, -h.z), Vec3(-h.x,  h.y, -h.z), Vec3( h.x,  h.y, -h.z) } },
            // Top (+Y)
            { Vec3(0, 1, 0), { Vec3(-h.x,  h.y,  h.z), Vec3( h.x,  h.y,  h.z), Vec3( h.x,  h.y, -h.z), Vec3(-h.x,  h.y, -h.z) } },
            // Bottom (-Y)
            { Vec3(0, -1, 0), { Vec3(-h.x, -h.y, -h.z), Vec3( h.x, -h.y, -h.z), Vec3( h.x, -h.y,  h.z), Vec3(-h.x, -h.y,  h.z) } },
            // Right (+X)
            { Vec3(1, 0, 0), { Vec3( h.x, -h.y,  h.z), Vec3( h.x, -h.y, -h.z), Vec3( h.x,  h.y, -h.z), Vec3( h.x,  h.y,  h.z) } },
            // Left (-X)
            { Vec3(-1, 0, 0), { Vec3(-h.x, -h.y, -h.z), Vec3(-h.x, -h.y,  h.z), Vec3(-h.x,  h.y,  h.z), Vec3(-h.x,  h.y, -h.z) } }
        };

        std::vector<GLVertex> verts;
        std::vector<uint32_t> idxs;
        verts.reserve(24);
        idxs.reserve(36);

        for (int f = 0; f < 6; ++f) {
            uint32_t base_idx = static_cast<uint32_t>(verts.size());
            for (int i = 0; i < 4; ++i) {
                verts.emplace_back(faces[f].corners[i], faces[f].normal, r, g, b, a);
            }
            idxs.push_back(base_idx);
            idxs.push_back(base_idx + 1);
            idxs.push_back(base_idx + 2);

            idxs.push_back(base_idx);
            idxs.push_back(base_idx + 2);
            idxs.push_back(base_idx + 3);
        }

        GLMesh mesh;
        mesh.upload_triangles(verts, idxs);
        return mesh;
    }

    static GLMesh create_polyhedron(const ConvexPolyhedron& poly, float r, float g, float b, float a = 1.0f) {
        std::vector<GLVertex> verts;
        std::vector<uint32_t> idxs;

        for (const auto& face : poly.faces) {
            if (face.vertex_indices.size() < 3) continue;

            Vec3 norm = face.normal;
            uint32_t base_idx = static_cast<uint32_t>(verts.size());

            for (int vi : face.vertex_indices) {
                verts.emplace_back(poly.vertices[static_cast<size_t>(vi)], norm, r, g, b, a);
            }

            // Triangulate face polygon via fan
            for (size_t i = 1; i + 1 < face.vertex_indices.size(); ++i) {
                idxs.push_back(base_idx);
                idxs.push_back(base_idx + static_cast<uint32_t>(i));
                idxs.push_back(base_idx + static_cast<uint32_t>(i + 1));
            }
        }

        GLMesh mesh;
        mesh.upload_triangles(verts, idxs);
        return mesh;
    }

    static GLMesh create_sphere(float radius, int rings, int sectors, float r, float g, float b, float a = 1.0f) {
        std::vector<GLVertex> verts;
        std::vector<uint32_t> idxs;

        float const R = 1.0f / static_cast<float>(rings - 1);
        float const S = 1.0f / static_cast<float>(sectors - 1);

        for (int r_idx = 0; r_idx < rings; ++r_idx) {
            for (int s_idx = 0; s_idx < sectors; ++s_idx) {
                float const y = std::sin(-3.14159265f / 2.0f + 3.14159265f * static_cast<float>(r_idx) * R);
                float const x = std::cos(2.0f * 3.14159265f * static_cast<float>(s_idx) * S) * std::sin(3.14159265f * static_cast<float>(r_idx) * R);
                float const z = std::sin(2.0f * 3.14159265f * static_cast<float>(s_idx) * S) * std::sin(3.14159265f * static_cast<float>(r_idx) * R);

                Vec3 norm(x, y, z);
                Vec3 pos = norm * radius;
                verts.emplace_back(pos, norm, r, g, b, a);
            }
        }

        for (int r_idx = 0; r_idx < rings - 1; ++r_idx) {
            for (int s_idx = 0; s_idx < sectors - 1; ++s_idx) {
                uint32_t cur = static_cast<uint32_t>(r_idx * sectors + s_idx);
                uint32_t next = static_cast<uint32_t>((r_idx + 1) * sectors + s_idx);

                idxs.push_back(cur);
                idxs.push_back(next);
                idxs.push_back(next + 1);

                idxs.push_back(cur);
                idxs.push_back(next + 1);
                idxs.push_back(cur + 1);
            }
        }

        GLMesh mesh;
        mesh.upload_triangles(verts, idxs);
        return mesh;
    }
};

} // namespace destruction::render

#endif // DESTRUCTION_GL_MESH_HPP
