#include "voxel_lab/gl_mesh.hpp"

namespace voxel_lab {

GLMesh::~GLMesh() {
    destroy();
}

GLMesh::GLMesh(GLMesh&& other) noexcept
    : vao(other.vao), vbo(other.vbo), ebo(other.ebo),
      index_count(other.index_count), vertex_count(other.vertex_count) {
    other.vao = 0;
    other.vbo = 0;
    other.ebo = 0;
    other.index_count = 0;
    other.vertex_count = 0;
}

GLMesh& GLMesh::operator=(GLMesh&& other) noexcept {
    if (this != &other) {
        destroy();
        vao = other.vao;
        vbo = other.vbo;
        ebo = other.ebo;
        index_count = other.index_count;
        vertex_count = other.vertex_count;
        other.vao = 0;
        other.vbo = 0;
        other.ebo = 0;
        other.index_count = 0;
        other.vertex_count = 0;
    }
    return *this;
}

void GLMesh::destroy() {
    if (ebo != 0) {
        glDeleteBuffers(1, &ebo);
        ebo = 0;
    }
    if (vbo != 0) {
        glDeleteBuffers(1, &vbo);
        vbo = 0;
    }
    if (vao != 0) {
        glDeleteVertexArrays(1, &vao);
        vao = 0;
    }
    index_count = 0;
    vertex_count = 0;
}

void GLMesh::upload(const MeshData& mesh_data) {
    destroy();

    if (mesh_data.vertices.empty() || mesh_data.indices.empty()) {
        return;
    }

    vertex_count = mesh_data.vertices.size();
    index_count = mesh_data.indices.size();

    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(mesh_data.vertices.size() * sizeof(MeshVertex)),
                 mesh_data.vertices.data(),
                 GL_STATIC_DRAW);

    glGenBuffers(1, &ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(mesh_data.indices.size() * sizeof(uint32_t)),
                 mesh_data.indices.data(),
                 GL_STATIC_DRAW);

    // Attribute 0: Position (3 floats)
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(MeshVertex), reinterpret_cast<const void*>(0));

    // Attribute 1: Normal (3 floats)
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(MeshVertex), reinterpret_cast<const void*>(3 * sizeof(float)));

    glBindVertexArray(0);
}

void GLMesh::draw() const {
    if (vao != 0 && index_count > 0) {
        glBindVertexArray(vao);
        glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(index_count), GL_UNSIGNED_INT, nullptr);
        glBindVertexArray(0);
    }
}

} // namespace voxel_lab
