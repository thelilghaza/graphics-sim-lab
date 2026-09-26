#include "voxel_lab/gl_shader.hpp"
#include <iostream>
#include <vector>

namespace voxel_lab {

GLShader::~GLShader() {
    destroy();
}

GLShader::GLShader(GLShader&& other) noexcept : program(other.program) {
    other.program = 0;
}

GLShader& GLShader::operator=(GLShader&& other) noexcept {
    if (this != &other) {
        destroy();
        program = other.program;
        other.program = 0;
    }
    return *this;
}

void GLShader::destroy() {
    if (program != 0) {
        glDeleteProgram(program);
        program = 0;
    }
}

GLuint GLShader::compile_stage(GLenum type, const char* src) {
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &src, nullptr);
    glCompileShader(shader);

    GLint success = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        GLint log_len = 0;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &log_len);
        std::vector<char> info_log(log_len > 0 ? log_len : 512);
        glGetShaderInfoLog(shader, static_cast<GLsizei>(info_log.size()), nullptr, info_log.data());
        std::cerr << "[GLShader Compile Error] ("
                  << (type == GL_VERTEX_SHADER ? "Vertex" : "Fragment")
                  << "):\n" << info_log.data() << "\n";
        glDeleteShader(shader);
        return 0;
    }
    return shader;
}

bool GLShader::compile(const char* vertex_src, const char* fragment_src) {
    destroy();

    GLuint vs = compile_stage(GL_VERTEX_SHADER, vertex_src);
    if (vs == 0) return false;

    GLuint fs = compile_stage(GL_FRAGMENT_SHADER, fragment_src);
    if (fs == 0) {
        glDeleteShader(vs);
        return false;
    }

    program = glCreateProgram();
    glAttachShader(program, vs);
    glAttachShader(program, fs);
    glLinkProgram(program);

    glDeleteShader(vs);
    glDeleteShader(fs);

    GLint success = 0;
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success) {
        GLint log_len = 0;
        glGetProgramiv(program, GL_INFO_LOG_LENGTH, &log_len);
        std::vector<char> info_log(log_len > 0 ? log_len : 512);
        glGetProgramInfoLog(program, static_cast<GLsizei>(info_log.size()), nullptr, info_log.data());
        std::cerr << "[GLShader Link Error]:\n" << info_log.data() << "\n";
        destroy();
        return false;
    }

    return true;
}

void GLShader::use() const {
    if (program != 0) {
        glUseProgram(program);
    }
}

void GLShader::set_mat4(const char* name, const Mat4& matrix) const {
    GLint loc = glGetUniformLocation(program, name);
    if (loc != -1) {
        glUniformMatrix4fv(loc, 1, GL_FALSE, matrix.data());
    }
}

void GLShader::set_vec3(const char* name, const Vec3& value) const {
    GLint loc = glGetUniformLocation(program, name);
    if (loc != -1) {
        glUniform3f(loc, value.x, value.y, value.z);
    }
}

void GLShader::set_vec3(const char* name, float x, float y, float z) const {
    GLint loc = glGetUniformLocation(program, name);
    if (loc != -1) {
        glUniform3f(loc, x, y, z);
    }
}

} // namespace voxel_lab
