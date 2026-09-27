#include "destruction/render/gl_loader.hpp"
#include <iostream>

// Definitions of function pointers
PFNGLGENVERTEXARRAYSPROC glGenVertexArrays = nullptr;
PFNGLBINDVERTEXARRAYPROC glBindVertexArray = nullptr;
PFNGLDELETEVERTEXARRAYSPROC glDeleteVertexArrays = nullptr;
PFNGLGENBUFFERSPROC glGenBuffers = nullptr;
PFNGLBINDBUFFERPROC glBindBuffer = nullptr;
PFNGLBUFFERDATAPROC glBufferData = nullptr;
PFNGLDELETEBUFFERSPROC glDeleteBuffers = nullptr;
PFNGLVERTEXATTRIBPOINTERPROC glVertexAttribPointer = nullptr;
PFNGLENABLEVERTEXATTRIBARRAYPROC glEnableVertexAttribArray = nullptr;
PFNGLCREATESHADERPROC glCreateShader = nullptr;
PFNGLSHADERSOURCEPROC glShaderSource = nullptr;
PFNGLCOMPILESHADERPROC glCompileShader = nullptr;
PFNGLGETSHADERIVPROC glGetShaderiv = nullptr;
PFNGLGETSHADERINFOLOGPROC glGetShaderInfoLog = nullptr;
PFNGLDELETESHADERPROC glDeleteShader = nullptr;
PFNGLCREATEPROGRAMPROC glCreateProgram = nullptr;
PFNGLATTACHSHADERPROC glAttachShader = nullptr;
PFNGLLINKPROGRAMPROC glLinkProgram = nullptr;
PFNGLGETPROGRAMIVPROC glGetProgramiv = nullptr;
PFNGLGETPROGRAMINFOLOGPROC glGetProgramInfoLog = nullptr;
PFNGLUSEPROGRAMPROC glUseProgram = nullptr;
PFNGLDELETEPROGRAMPROC glDeleteProgram = nullptr;
PFNGLGETUNIFORMLOCATIONPROC glGetUniformLocation = nullptr;
PFNGLUNIFORMMATRIX4FVPROC glUniformMatrix4fv = nullptr;
PFNGLUNIFORM3FPROC glUniform3f = nullptr;
PFNGLUNIFORM4FPROC glUniform4f = nullptr;
PFNGLUNIFORM1FPROC glUniform1f = nullptr;
PFNGLUNIFORM1IPROC glUniform1i = nullptr;

namespace destruction::render {

bool init_gl_loader(GLProcLoader loader) {
    if (!loader) {
        std::cerr << "[OpenGL Loader Error] Null proc loader provided.\n";
        return false;
    }

    #define LOAD_PROC(type, name) \
        name = reinterpret_cast<type>(loader(#name)); \
        if (!name) { \
            std::cerr << "[OpenGL Loader Error] Failed to load function: " << #name << "\n"; \
            return false; \
        }

    LOAD_PROC(PFNGLGENVERTEXARRAYSPROC, glGenVertexArrays);
    LOAD_PROC(PFNGLBINDVERTEXARRAYPROC, glBindVertexArray);
    LOAD_PROC(PFNGLDELETEVERTEXARRAYSPROC, glDeleteVertexArrays);
    LOAD_PROC(PFNGLGENBUFFERSPROC, glGenBuffers);
    LOAD_PROC(PFNGLBINDBUFFERPROC, glBindBuffer);
    LOAD_PROC(PFNGLBUFFERDATAPROC, glBufferData);
    LOAD_PROC(PFNGLDELETEBUFFERSPROC, glDeleteBuffers);
    LOAD_PROC(PFNGLVERTEXATTRIBPOINTERPROC, glVertexAttribPointer);
    LOAD_PROC(PFNGLENABLEVERTEXATTRIBARRAYPROC, glEnableVertexAttribArray);
    LOAD_PROC(PFNGLCREATESHADERPROC, glCreateShader);
    LOAD_PROC(PFNGLSHADERSOURCEPROC, glShaderSource);
    LOAD_PROC(PFNGLCOMPILESHADERPROC, glCompileShader);
    LOAD_PROC(PFNGLGETSHADERIVPROC, glGetShaderiv);
    LOAD_PROC(PFNGLGETSHADERINFOLOGPROC, glGetShaderInfoLog);
    LOAD_PROC(PFNGLDELETESHADERPROC, glDeleteShader);
    LOAD_PROC(PFNGLCREATEPROGRAMPROC, glCreateProgram);
    LOAD_PROC(PFNGLATTACHSHADERPROC, glAttachShader);
    LOAD_PROC(PFNGLLINKPROGRAMPROC, glLinkProgram);
    LOAD_PROC(PFNGLGETPROGRAMIVPROC, glGetProgramiv);
    LOAD_PROC(PFNGLGETPROGRAMINFOLOGPROC, glGetProgramInfoLog);
    LOAD_PROC(PFNGLUSEPROGRAMPROC, glUseProgram);
    LOAD_PROC(PFNGLDELETEPROGRAMPROC, glDeleteProgram);
    LOAD_PROC(PFNGLGETUNIFORMLOCATIONPROC, glGetUniformLocation);
    LOAD_PROC(PFNGLUNIFORMMATRIX4FVPROC, glUniformMatrix4fv);
    LOAD_PROC(PFNGLUNIFORM3FPROC, glUniform3f);
    LOAD_PROC(PFNGLUNIFORM4FPROC, glUniform4f);
    LOAD_PROC(PFNGLUNIFORM1FPROC, glUniform1f);
    LOAD_PROC(PFNGLUNIFORM1IPROC, glUniform1i);

    #undef LOAD_PROC

    return true;
}

} // namespace destruction::render
