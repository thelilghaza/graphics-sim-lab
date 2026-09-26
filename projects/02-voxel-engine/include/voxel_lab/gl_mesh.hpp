#pragma once

#include "voxel_lab/gl_loader.hpp"
#include "voxel_lab/mesh.hpp"

namespace voxel_lab {

class GLMesh {
public:
    GLMesh() = default;
    ~GLMesh();

    GLMesh(const GLMesh&) = delete;
    GLMesh& operator=(const GLMesh&) = delete;

    GLMesh(GLMesh&& other) noexcept;
    GLMesh& operator=(GLMesh&& other) noexcept;

    // Uploads CPU MeshData to GPU buffers
    void upload(const MeshData& mesh_data);

    // Draws the mesh using glDrawElements
    void draw() const;

    // Destroys allocated GPU buffers
    void destroy();

    size_t get_index_count() const noexcept { return index_count; }
    size_t get_vertex_count() const noexcept { return vertex_count; }

private:
    GLuint vao{0};
    GLuint vbo{0};
    GLuint ebo{0};
    size_t index_count{0};
    size_t vertex_count{0};
};

} // namespace voxel_lab
