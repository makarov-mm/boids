#include "gl_api.hpp"

namespace {
// wglGetProcAddress only knows the functions above OpenGL 1.1 (and may
// answer 1, 2, 3 or -1 instead of null); the rest come from opengl32.dll.
PROC getProc(const char* name) {
    PROC p = wglGetProcAddress(name);
    const auto bits = reinterpret_cast<INT_PTR>(p);
    if (bits == 0 || bits == 1 || bits == 2 || bits == 3 || bits == -1) {
        static const HMODULE opengl32 = GetModuleHandleW(L"opengl32.dll");
        p = opengl32 ? GetProcAddress(opengl32, name) : nullptr;
    }
    return p;
}
} // namespace

const char* gl::load() {
#define BOIDS_GL_LOAD(ret, name, params)                                  \
    name = reinterpret_cast<decltype(name)>(getProc("gl" #name));          \
    if (!name) return "gl" #name;
    BOIDS_GL_FUNCTIONS(BOIDS_GL_LOAD)
#undef BOIDS_GL_LOAD
    return nullptr;
}
