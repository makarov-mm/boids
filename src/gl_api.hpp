#pragma once
// OpenGL without GLAD / GLEW: <gl/GL.h> only declares OpenGL 1.1, so every
// function the viewer uses is fetched at runtime into the gl:: table below.

#ifndef NOMINMAX
#  define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#  define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <gl/GL.h>

#include <cstddef>

// ---- Types and enums newer than OpenGL 1.1 ----------------------------------
using GLchar     = char;
using GLsizeiptr = std::ptrdiff_t;

// (Plain defines with the official values, so a later <glext.h> agrees.)
#define GL_ARRAY_BUFFER      0x8892
#define GL_STREAM_DRAW       0x88E0
#define GL_STATIC_DRAW       0x88E4
#define GL_FRAGMENT_SHADER   0x8B30
#define GL_VERTEX_SHADER     0x8B31
#define GL_COMPILE_STATUS    0x8B81
#define GL_LINK_STATUS       0x8B82
#define GL_INFO_LOG_LENGTH   0x8B84
#define GL_TEXTURE0          0x84C0
#define GL_CLAMP_TO_EDGE     0x812F
#define GL_RED               0x1903
#define GL_R8                0x8229
#define GL_MAJOR_VERSION     0x821B
#define GL_MINOR_VERSION     0x821C

// ---- Entry points: X(return type, name without the "gl" prefix, parameters) --
#define BOIDS_GL_FUNCTIONS(X)                                                                                    \
    /* OpenGL 1.1 (opengl32.dll exports these) */                                                             \
    X(void,   Enable,           (GLenum cap))                                                                  \
    X(void,   Disable,          (GLenum cap))                                                                  \
    X(void,   Clear,            (GLbitfield mask))                                                             \
    X(void,   ClearColor,       (GLfloat r, GLfloat g, GLfloat b, GLfloat a))                                  \
    X(void,   Viewport,         (GLint x, GLint y, GLsizei width, GLsizei height))                             \
    X(void,   BlendFunc,        (GLenum sfactor, GLenum dfactor))                                              \
    X(void,   DrawArrays,       (GLenum mode, GLint first, GLsizei count))                                     \
    X(void,   GetIntegerv,      (GLenum pname, GLint* data))                                                   \
    X(void,   PixelStorei,      (GLenum pname, GLint param))                                                   \
    X(void,   GenTextures,      (GLsizei n, GLuint* textures))                                                 \
    X(void,   DeleteTextures,   (GLsizei n, const GLuint* textures))                                           \
    X(void,   BindTexture,      (GLenum target, GLuint texture))                                               \
    X(void,   TexParameteri,    (GLenum target, GLenum pname, GLint param))                                    \
    X(void,   TexImage2D,       (GLenum target, GLint level, GLint internalformat, GLsizei width,             \
                                 GLsizei height, GLint border, GLenum format, GLenum type, const void* pixels)) \
    /* Shaders */                                                                                             \
    X(GLuint, CreateShader,     (GLenum type))                                                                 \
    X(void,   ShaderSource,     (GLuint shader, GLsizei count, const GLchar* const* string, const GLint* length)) \
    X(void,   CompileShader,    (GLuint shader))                                                               \
    X(void,   GetShaderiv,      (GLuint shader, GLenum pname, GLint* params))                                  \
    X(void,   GetShaderInfoLog, (GLuint shader, GLsizei bufSize, GLsizei* length, GLchar* infoLog))            \
    X(void,   DeleteShader,     (GLuint shader))                                                               \
    X(GLuint, CreateProgram,    ())                                                                            \
    X(void,   AttachShader,     (GLuint program, GLuint shader))                                               \
    X(void,   LinkProgram,      (GLuint program))                                                              \
    X(void,   GetProgramiv,     (GLuint program, GLenum pname, GLint* params))                                 \
    X(void,   GetProgramInfoLog,(GLuint program, GLsizei bufSize, GLsizei* length, GLchar* infoLog))           \
    X(void,   DeleteProgram,    (GLuint program))                                                              \
    X(void,   UseProgram,       (GLuint program))                                                              \
    X(GLint,  GetUniformLocation, (GLuint program, const GLchar* name))                                        \
    X(void,   Uniform1i,        (GLint location, GLint v0))                                                    \
    X(void,   Uniform1f,        (GLint location, GLfloat v0))                                                  \
    X(void,   Uniform3f,        (GLint location, GLfloat v0, GLfloat v1, GLfloat v2))                          \
    X(void,   UniformMatrix4fv, (GLint location, GLsizei count, GLboolean transpose, const GLfloat* value))     \
    X(void,   ActiveTexture,    (GLenum texture))                                                              \
    /* Buffers, vertex arrays, instancing */                                                                  \
    X(void,   GenBuffers,       (GLsizei n, GLuint* buffers))                                                  \
    X(void,   DeleteBuffers,    (GLsizei n, const GLuint* buffers))                                            \
    X(void,   BindBuffer,       (GLenum target, GLuint buffer))                                                \
    X(void,   BufferData,       (GLenum target, GLsizeiptr size, const void* data, GLenum usage))              \
    X(void,   GenVertexArrays,  (GLsizei n, GLuint* arrays))                                                   \
    X(void,   DeleteVertexArrays, (GLsizei n, const GLuint* arrays))                                           \
    X(void,   BindVertexArray,  (GLuint array))                                                                \
    X(void,   EnableVertexAttribArray, (GLuint index))                                                         \
    X(void,   VertexAttribPointer, (GLuint index, GLint size, GLenum type, GLboolean normalized,              \
                                    GLsizei stride, const void* pointer))                                      \
    X(void,   VertexAttribDivisor, (GLuint index, GLuint divisor))                                             \
    X(void,   DrawArraysInstanced, (GLenum mode, GLint first, GLsizei count, GLsizei instancecount))

namespace gl {
#define BOIDS_GL_DECLARE(ret, name, params) inline ret(APIENTRY* name) params = nullptr;
BOIDS_GL_FUNCTIONS(BOIDS_GL_DECLARE)
#undef BOIDS_GL_DECLARE

// Fills the table; needs a current context. Returns the name of the first
// function the driver does not provide, or nullptr when all of them loaded.
[[nodiscard]] const char* load();
} // namespace gl
