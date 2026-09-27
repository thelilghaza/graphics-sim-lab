#ifndef DESTRUCTION_SHADER_HPP
#define DESTRUCTION_SHADER_HPP

#include "destruction/render/gl_loader.hpp"
#include "destruction/math/vec3.hpp"
#include <string>
#include <iostream>
#include <vector>

namespace destruction::render {

using namespace destruction::math;

class Shader {
private:
    GLuint program_id_{0};

    static GLuint compile_shader_stage(GLenum type, const char* source) {
        GLuint shader = glCreateShader(type);
        glShaderSource(shader, 1, &source, nullptr);
        glCompileShader(shader);

        GLint success = 0;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
        if (!success) {
            GLint log_length = 0;
            glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &log_length);
            std::vector<GLchar> info_log(static_cast<size_t>(log_length > 0 ? log_length : 512));
            glGetShaderInfoLog(shader, static_cast<GLsizei>(info_log.size()), nullptr, info_log.data());
            std::cerr << "[Shader Error] Compilation failed:\n" << info_log.data() << "\n";
            glDeleteShader(shader);
            return 0;
        }
        return shader;
    }

public:
    Shader() = default;

    Shader(const char* vertex_src, const char* fragment_src) {
        GLuint vs = compile_shader_stage(GL_VERTEX_SHADER, vertex_src);
        GLuint fs = compile_shader_stage(GL_FRAGMENT_SHADER, fragment_src);

        if (vs == 0 || fs == 0) {
            if (vs != 0) glDeleteShader(vs);
            if (fs != 0) glDeleteShader(fs);
            return;
        }

        program_id_ = glCreateProgram();
        glAttachShader(program_id_, vs);
        glAttachShader(program_id_, fs);
        glLinkProgram(program_id_);

        GLint success = 0;
        glGetProgramiv(program_id_, GL_LINK_STATUS, &success);
        if (!success) {
            GLint log_length = 0;
            glGetProgramiv(program_id_, GL_INFO_LOG_LENGTH, &log_length);
            std::vector<GLchar> info_log(static_cast<size_t>(log_length > 0 ? log_length : 512));
            glGetProgramInfoLog(program_id_, static_cast<GLsizei>(info_log.size()), nullptr, info_log.data());
            std::cerr << "[Shader Error] Linking failed:\n" << info_log.data() << "\n";
            glDeleteProgram(program_id_);
            program_id_ = 0;
        }

        glDeleteShader(vs);
        glDeleteShader(fs);
    }

    ~Shader() {
        if (program_id_ != 0) {
            glDeleteProgram(program_id_);
            program_id_ = 0;
        }
    }

    Shader(const Shader&) = delete;
    Shader& operator=(const Shader&) = delete;

    Shader(Shader&& other) noexcept : program_id_(other.program_id_) {
        other.program_id_ = 0;
    }

    Shader& operator=(Shader&& other) noexcept {
        if (this != &other) {
            if (program_id_ != 0) {
                glDeleteProgram(program_id_);
            }
            program_id_ = other.program_id_;
            other.program_id_ = 0;
        }
        return *this;
    }

    bool is_valid() const {
        return program_id_ != 0;
    }

    GLuint get_id() const {
        return program_id_;
    }

    void use() const {
        if (program_id_ != 0) {
            glUseProgram(program_id_);
        }
    }

    void set_mat4(const char* name, const float* values) const {
        GLint loc = glGetUniformLocation(program_id_, name);
        if (loc >= 0) {
            glUniformMatrix4fv(loc, 1, GL_FALSE, values);
        }
    }

    void set_vec3(const char* name, const Vec3& v) const {
        GLint loc = glGetUniformLocation(program_id_, name);
        if (loc >= 0) {
            glUniform3f(loc, v.x, v.y, v.z);
        }
    }

    void set_vec4(const char* name, float x, float y, float z, float w) const {
        GLint loc = glGetUniformLocation(program_id_, name);
        if (loc >= 0) {
            glUniform4f(loc, x, y, z, w);
        }
    }

    void set_float(const char* name, float val) const {
        GLint loc = glGetUniformLocation(program_id_, name);
        if (loc >= 0) {
            glUniform1f(loc, val);
        }
    }

    void set_int(const char* name, int val) const {
        GLint loc = glGetUniformLocation(program_id_, name);
        if (loc >= 0) {
            glUniform1i(loc, val);
        }
    }
};

} // namespace destruction::render

#endif // DESTRUCTION_SHADER_HPP
