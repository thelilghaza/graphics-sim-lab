#pragma once

#include "voxel_lab/gl_loader.hpp"
#include "voxel_lab/math.hpp"
#include <string>

namespace voxel_lab {

class GLShader {
public:
    GLShader() = default;
    ~GLShader();

    GLShader(const GLShader&) = delete;
    GLShader& operator=(const GLShader&) = delete;

    GLShader(GLShader&& other) noexcept;
    GLShader& operator=(GLShader&& other) noexcept;

    // Compiles and links from GLSL source strings
    bool compile(const char* vertex_src, const char* fragment_src);

    void use() const;
    void destroy();

    GLuint get_program_id() const noexcept { return program; }

    void set_mat4(const char* name, const Mat4& matrix) const;
    void set_vec3(const char* name, const Vec3& value) const;
    void set_vec3(const char* name, float x, float y, float z) const;

private:
    GLuint program{0};

    static GLuint compile_stage(GLenum type, const char* src);
};

} // namespace voxel_lab
