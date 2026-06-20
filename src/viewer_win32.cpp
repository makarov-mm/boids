// Boids 3D / Swarm Intelligence - native Win32/OpenGL viewer for Visual Studio.
//
// No GLFW/GLAD: the .sln builds with no external dependencies.
//
// Controls:
//   left-drag  orbit camera         mouse wheel  zoom
//   Space      pause                Up/Down      sim speed (substeps/frame)
//   O          obstacle on/off      P            predators on/off
//   R          reset                H            hide HUD       Esc  quit

#ifndef NOMINMAX
#define NOMINMAX
#endif
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <windowsx.h>
#include <gl/GL.h>

#include "mat4.hpp"
#include "boids.hpp"
#include "font_atlas.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstddef>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#ifndef GL_VERTEX_SHADER
#define GL_VERTEX_SHADER 0x8B31
#endif
#ifndef GL_FRAGMENT_SHADER
#define GL_FRAGMENT_SHADER 0x8B30
#endif
#ifndef GL_COMPILE_STATUS
#define GL_COMPILE_STATUS 0x8B81
#endif
#ifndef GL_LINK_STATUS
#define GL_LINK_STATUS 0x8B82
#endif
#ifndef GL_ARRAY_BUFFER
#define GL_ARRAY_BUFFER 0x8892
#endif
#ifndef GL_STATIC_DRAW
#define GL_STATIC_DRAW 0x88E4
#endif
#ifndef GL_STREAM_DRAW
#define GL_STREAM_DRAW 0x88E0
#endif
#ifndef GL_R8
#define GL_R8 0x8229
#endif
#ifndef GL_RED
#define GL_RED 0x1903
#endif
#ifndef GL_TEXTURE0
#define GL_TEXTURE0 0x84C0
#endif
#ifndef GL_ONE_MINUS_SRC_ALPHA
#define GL_ONE_MINUS_SRC_ALPHA 0x0303
#endif
#ifndef GL_UNPACK_ALIGNMENT
#define GL_UNPACK_ALIGNMENT 0x0CF5
#endif

using GLchar = char;
using GLsizeiptr = ptrdiff_t;

using PFNGLCREATESHADERPROC = GLuint(APIENTRY*)(GLenum type);
using PFNGLSHADERSOURCEPROC = void(APIENTRY*)(GLuint shader, GLsizei count, const GLchar* const* string, const GLint* length);
using PFNGLCOMPILESHADERPROC = void(APIENTRY*)(GLuint shader);
using PFNGLGETSHADERIVPROC = void(APIENTRY*)(GLuint shader, GLenum pname, GLint* params);
using PFNGLGETSHADERINFOLOGPROC = void(APIENTRY*)(GLuint shader, GLsizei bufSize, GLsizei* length, GLchar* infoLog);
using PFNGLCREATEPROGRAMPROC = GLuint(APIENTRY*)();
using PFNGLATTACHSHADERPROC = void(APIENTRY*)(GLuint program, GLuint shader);
using PFNGLLINKPROGRAMPROC = void(APIENTRY*)(GLuint program);
using PFNGLGETPROGRAMIVPROC = void(APIENTRY*)(GLuint program, GLenum pname, GLint* params);
using PFNGLGETPROGRAMINFOLOGPROC = void(APIENTRY*)(GLuint program, GLsizei bufSize, GLsizei* length, GLchar* infoLog);
using PFNGLDELETESHADERPROC = void(APIENTRY*)(GLuint shader);
using PFNGLGENVERTEXARRAYSPROC = void(APIENTRY*)(GLsizei n, GLuint* arrays);
using PFNGLBINDVERTEXARRAYPROC = void(APIENTRY*)(GLuint array);
using PFNGLGENBUFFERSPROC = void(APIENTRY*)(GLsizei n, GLuint* buffers);
using PFNGLBINDBUFFERPROC = void(APIENTRY*)(GLenum target, GLuint buffer);
using PFNGLBUFFERDATAPROC = void(APIENTRY*)(GLenum target, GLsizeiptr size, const void* data, GLenum usage);
using PFNGLENABLEVERTEXATTRIBARRAYPROC = void(APIENTRY*)(GLuint index);
using PFNGLVERTEXATTRIBPOINTERPROC = void(APIENTRY*)(GLuint index, GLint size, GLenum type, GLboolean normalized, GLsizei stride, const void* pointer);
using PFNGLUSEPROGRAMPROC = void(APIENTRY*)(GLuint program);
using PFNGLGETUNIFORMLOCATIONPROC = GLint(APIENTRY*)(GLuint program, const GLchar* name);
using PFNGLUNIFORMMATRIX4FVPROC = void(APIENTRY*)(GLint location, GLsizei count, GLboolean transpose, const GLfloat* value);
using PFNGLUNIFORM3FPROC = void(APIENTRY*)(GLint location, GLfloat v0, GLfloat v1, GLfloat v2);
using PFNGLUNIFORM1FPROC = void(APIENTRY*)(GLint location, GLfloat v0);
using PFNGLUNIFORM1IPROC = void(APIENTRY*)(GLint location, GLint v0);
using PFNGLACTIVETEXTUREPROC = void(APIENTRY*)(GLenum texture);
using PFNWGLSWAPINTERVALEXTPROC = BOOL(WINAPI*)(int interval);

static PFNGLCREATESHADERPROC glCreateShader_ = nullptr;
static PFNGLSHADERSOURCEPROC glShaderSource_ = nullptr;
static PFNGLCOMPILESHADERPROC glCompileShader_ = nullptr;
static PFNGLGETSHADERIVPROC glGetShaderiv_ = nullptr;
static PFNGLGETSHADERINFOLOGPROC glGetShaderInfoLog_ = nullptr;
static PFNGLCREATEPROGRAMPROC glCreateProgram_ = nullptr;
static PFNGLATTACHSHADERPROC glAttachShader_ = nullptr;
static PFNGLLINKPROGRAMPROC glLinkProgram_ = nullptr;
static PFNGLGETPROGRAMIVPROC glGetProgramiv_ = nullptr;
static PFNGLGETPROGRAMINFOLOGPROC glGetProgramInfoLog_ = nullptr;
static PFNGLDELETESHADERPROC glDeleteShader_ = nullptr;
static PFNGLGENVERTEXARRAYSPROC glGenVertexArrays_ = nullptr;
static PFNGLBINDVERTEXARRAYPROC glBindVertexArray_ = nullptr;
static PFNGLGENBUFFERSPROC glGenBuffers_ = nullptr;
static PFNGLBINDBUFFERPROC glBindBuffer_ = nullptr;
static PFNGLBUFFERDATAPROC glBufferData_ = nullptr;
static PFNGLENABLEVERTEXATTRIBARRAYPROC glEnableVertexAttribArray_ = nullptr;
static PFNGLVERTEXATTRIBPOINTERPROC glVertexAttribPointer_ = nullptr;
static PFNGLUSEPROGRAMPROC glUseProgram_ = nullptr;
static PFNGLGETUNIFORMLOCATIONPROC glGetUniformLocation_ = nullptr;
static PFNGLUNIFORMMATRIX4FVPROC glUniformMatrix4fv_ = nullptr;
static PFNGLUNIFORM3FPROC glUniform3f_ = nullptr;
static PFNGLUNIFORM1FPROC glUniform1f_ = nullptr;
static PFNGLUNIFORM1IPROC glUniform1i_ = nullptr;
static PFNGLACTIVETEXTUREPROC glActiveTexture_ = nullptr;
static PFNWGLSWAPINTERVALEXTPROC wglSwapIntervalEXT_ = nullptr;

static void* getGlProcAddress(const char* name) {
    void* p = reinterpret_cast<void*>(wglGetProcAddress(name));
    const auto ip = reinterpret_cast<std::uintptr_t>(p);
    if (p == nullptr || ip == 1 || ip == 2 || ip == 3 || p == reinterpret_cast<void*>(-1)) {
        HMODULE module = LoadLibraryA("opengl32.dll");
        p = module ? reinterpret_cast<void*>(GetProcAddress(module, name)) : nullptr;
    }
    return p;
}

#define LOAD_GL(name) \
    do { \
        name##_ = reinterpret_cast<decltype(name##_)>(getGlProcAddress(#name)); \
        if (!name##_) { \
            MessageBoxA(nullptr, "Required OpenGL function is missing: " #name, "Boids 3D", MB_ICONERROR | MB_OK); \
            return false; \
        } \
    } while (false)

static bool loadOpenGL() {
    LOAD_GL(glCreateShader);
    LOAD_GL(glShaderSource);
    LOAD_GL(glCompileShader);
    LOAD_GL(glGetShaderiv);
    LOAD_GL(glGetShaderInfoLog);
    LOAD_GL(glCreateProgram);
    LOAD_GL(glAttachShader);
    LOAD_GL(glLinkProgram);
    LOAD_GL(glGetProgramiv);
    LOAD_GL(glGetProgramInfoLog);
    LOAD_GL(glDeleteShader);
    LOAD_GL(glGenVertexArrays);
    LOAD_GL(glBindVertexArray);
    LOAD_GL(glGenBuffers);
    LOAD_GL(glBindBuffer);
    LOAD_GL(glBufferData);
    LOAD_GL(glEnableVertexAttribArray);
    LOAD_GL(glVertexAttribPointer);
    LOAD_GL(glUseProgram);
    LOAD_GL(glGetUniformLocation);
    LOAD_GL(glUniformMatrix4fv);
    LOAD_GL(glUniform3f);
    LOAD_GL(glUniform1f);
    LOAD_GL(glUniform1i);
    LOAD_GL(glActiveTexture);
    wglSwapIntervalEXT_ = reinterpret_cast<PFNWGLSWAPINTERVALEXTPROC>(getGlProcAddress("wglSwapIntervalEXT"));
    if (wglSwapIntervalEXT_) wglSwapIntervalEXT_(1);
    return true;
}

// ---- Boid arrowheads (flat colour, depth-tested) ----
static const char* BOID_VS = R"(#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aColor;
uniform mat4 uViewProj;
out vec3 vColor;
void main(){ vColor = aColor; gl_Position = uViewProj * vec4(aPos, 1.0); }
)";

static const char* BOID_FS = R"(#version 330 core
in vec3 vColor;
out vec4 frag;
void main(){ frag = vec4(vColor, 1.0); }
)";

static const char* LINE_VS = R"(#version 330 core
layout(location=0) in vec3 aPos;
uniform mat4 uViewProj;
void main(){ gl_Position = uViewProj * vec4(aPos, 1.0); }
)";

static const char* LINE_FS = R"(#version 330 core
out vec4 frag;
uniform vec3 uColor;
void main(){ frag = vec4(uColor, 1.0); }
)";

static const char* HUD_VS = R"(#version 330 core
layout(location=0) in vec2 aPos;
layout(location=1) in vec2 aUV;
out vec2 vUV;
void main(){ vUV = aUV; gl_Position = vec4(aPos, 0.0, 1.0); }
)";

static const char* HUD_FS = R"(#version 330 core
in vec2 vUV;
out vec4 frag;
uniform sampler2D uFont;
uniform vec3 uColor;
uniform float uAlpha;
uniform int uTextured;
void main(){
    if (uTextured == 1) { float c = texture(uFont, vUV).r; frag = vec4(uColor, uAlpha * c); }
    else frag = vec4(uColor, uAlpha);
}
)";

static void debugLog(const char* t) { OutputDebugStringA(t); OutputDebugStringA("\n"); }

static GLuint compileShader(GLenum type, const char* src) {
    GLuint sh = glCreateShader_(type);
    glShaderSource_(sh, 1, &src, nullptr);
    glCompileShader_(sh);
    GLint ok = 0; glGetShaderiv_(sh, GL_COMPILE_STATUS, &ok);
    if (!ok) { char log[2048]{}; glGetShaderInfoLog_(sh, sizeof(log), nullptr, log); debugLog(log);
               MessageBoxA(nullptr, log, "Shader compile error", MB_ICONERROR | MB_OK); }
    return sh;
}

static GLuint linkProgram(const char* vs, const char* fs) {
    GLuint pr = glCreateProgram_();
    GLuint v = compileShader(GL_VERTEX_SHADER, vs);
    GLuint f = compileShader(GL_FRAGMENT_SHADER, fs);
    glAttachShader_(pr, v); glAttachShader_(pr, f);
    glLinkProgram_(pr); glDeleteShader_(v); glDeleteShader_(f);
    GLint ok = 0; glGetProgramiv_(pr, GL_LINK_STATUS, &ok);
    if (!ok) { char log[2048]{}; glGetProgramInfoLog_(pr, sizeof(log), nullptr, log); debugLog(log);
               MessageBoxA(nullptr, log, "Shader link error", MB_ICONERROR | MB_OK); }
    return pr;
}

static std::unique_ptr<Boids> makeSim(bool obstacle, int predators) {
    BoidsParams p;
    p.boids = 12000;
    p.predators = predators;
    p.obstacle = obstacle;
    auto sim = std::make_unique<Boids>(p);
    sim->setThreads(static_cast<int>(std::thread::hardware_concurrency()));
    return sim;
}

static void buildBoxLines(const Vec3& a, const Vec3& b, std::vector<float>& out) {
    Vec3 c[8] = {{a.x,a.y,a.z},{b.x,a.y,a.z},{b.x,b.y,a.z},{a.x,b.y,a.z},
                 {a.x,a.y,b.z},{b.x,a.y,b.z},{b.x,b.y,b.z},{a.x,b.y,b.z}};
    int e[24] = {0,1,1,2,2,3,3,0, 4,5,5,6,6,7,7,4, 0,4,1,5,2,6,3,7};
    out.clear();
    for (int i : e) { out.push_back(c[i].x); out.push_back(c[i].y); out.push_back(c[i].z); }
}

static void appendSphereWire(const Vec3& c, float r, std::vector<float>& out) {
    const int seg = 32;
    auto ring = [&](int axis) {
        for (int i = 0; i < seg; ++i) {
            float a0 = (float)i / seg * 6.2831853f, a1 = (float)(i + 1) / seg * 6.2831853f;
            Vec3 p0, p1;
            if (axis == 0) { p0 = {c.x, c.y + r*std::cos(a0), c.z + r*std::sin(a0)}; p1 = {c.x, c.y + r*std::cos(a1), c.z + r*std::sin(a1)}; }
            else if (axis == 1) { p0 = {c.x + r*std::cos(a0), c.y, c.z + r*std::sin(a0)}; p1 = {c.x + r*std::cos(a1), c.y, c.z + r*std::sin(a1)}; }
            else { p0 = {c.x + r*std::cos(a0), c.y + r*std::sin(a0), c.z}; p1 = {c.x + r*std::cos(a1), c.y + r*std::sin(a1), c.z}; }
            out.push_back(p0.x); out.push_back(p0.y); out.push_back(p0.z);
            out.push_back(p1.x); out.push_back(p1.y); out.push_back(p1.z);
        }
    };
    ring(0); ring(1); ring(2);
}

struct ViewerState {
    HWND hwnd = nullptr;
    HDC hdc = nullptr;
    HGLRC glrc = nullptr;

    OrbitCamera camera;
    bool dragging = false, paused = false, showHud = true;
    bool obstacle = false; int predators = 3;
    double lastX = 0, lastY = 0;
    int substeps = 1, vpW = 1280, vpH = 800;
    double fps = 0;
    float boidSize = 1.4f;

    GLuint boidProgram = 0, lineProgram = 0, hudProgram = 0;
    GLuint boidVao = 0, boidVbo = 0, predVao = 0, predVbo = 0, lineVao = 0, lineVbo = 0, hudVao = 0, hudVbo = 0;
    GLuint fontTex = 0;

    std::unique_ptr<Boids> sim;
    std::vector<float> boxLines, boidVerts, predVerts;

    int frames = 0;
    std::chrono::steady_clock::time_point lastTitleUpdate = std::chrono::steady_clock::now();
};

static ViewerState* g_state = nullptr;

static void rebuildLines(ViewerState& s) {
    buildBoxLines(s.sim->params().boundsMin, s.sim->params().boundsMax, s.boxLines);
    if (s.sim->params().obstacle)
        appendSphereWire(s.sim->params().obstacleCenter, s.sim->params().obstacleRadius, s.boxLines);
    glBindVertexArray_(s.lineVao);
    glBindBuffer_(GL_ARRAY_BUFFER, s.lineVbo);
    glBufferData_(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(s.boxLines.size() * sizeof(float)), s.boxLines.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray_(0);
    glVertexAttribPointer_(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), reinterpret_cast<void*>(0));
}

static void resetSimulation(ViewerState& s) {
    s.sim = makeSim(s.obstacle, s.predators);
    rebuildLines(s);
}

static bool createOpenGLContext(ViewerState& s) {
    s.hdc = GetDC(s.hwnd);
    if (!s.hdc) return false;
    PIXELFORMATDESCRIPTOR pfd{};
    pfd.nSize = sizeof(pfd); pfd.nVersion = 1;
    pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
    pfd.iPixelType = PFD_TYPE_RGBA; pfd.cColorBits = 32; pfd.cDepthBits = 24; pfd.cStencilBits = 8;
    pfd.iLayerType = PFD_MAIN_PLANE;
    int format = ChoosePixelFormat(s.hdc, &pfd);
    if (format == 0 || !SetPixelFormat(s.hdc, format, &pfd)) return false;
    s.glrc = wglCreateContext(s.hdc);
    if (!s.glrc || !wglMakeCurrent(s.hdc, s.glrc)) return false;
    return loadOpenGL();
}

static void setupDynamicVbo(GLuint vao, GLuint vbo) {
    glBindVertexArray_(vao);
    glBindBuffer_(GL_ARRAY_BUFFER, vbo);
    glEnableVertexAttribArray_(0);
    glVertexAttribPointer_(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(0));
    glEnableVertexAttribArray_(1);
    glVertexAttribPointer_(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(3 * sizeof(float)));
}

static bool initRenderer(ViewerState& s) {
    s.sim = makeSim(s.obstacle, s.predators);
    Vec3 c = (s.sim->params().boundsMin + s.sim->params().boundsMax) * 0.5f;
    Vec3 ext = s.sim->params().boundsMax - s.sim->params().boundsMin;
    s.camera.target = c;
    s.camera.distance = std::max(ext.x, std::max(ext.y, ext.z)) * 1.7f;

    glEnable(GL_DEPTH_TEST);

    s.boidProgram = linkProgram(BOID_VS, BOID_FS);
    s.lineProgram = linkProgram(LINE_VS, LINE_FS);
    s.hudProgram = linkProgram(HUD_VS, HUD_FS);

    glGenTextures(1, &s.fontTex);
    glActiveTexture_(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, s.fontTex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, FONT_W, FONT_H, 0, GL_RED, GL_UNSIGNED_BYTE, FONT_PIX);

    glGenVertexArrays_(1, &s.boidVao); glGenBuffers_(1, &s.boidVbo); setupDynamicVbo(s.boidVao, s.boidVbo);
    glGenVertexArrays_(1, &s.predVao); glGenBuffers_(1, &s.predVbo); setupDynamicVbo(s.predVao, s.predVbo);

    glGenVertexArrays_(1, &s.lineVao); glGenBuffers_(1, &s.lineVbo);
    rebuildLines(s);

    glGenVertexArrays_(1, &s.hudVao); glGenBuffers_(1, &s.hudVbo);
    glBindVertexArray_(s.hudVao); glBindBuffer_(GL_ARRAY_BUFFER, s.hudVbo);
    glEnableVertexAttribArray_(0);
    glVertexAttribPointer_(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), reinterpret_cast<void*>(0));
    glEnableVertexAttribArray_(1);
    glVertexAttribPointer_(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), reinterpret_cast<void*>(2 * sizeof(float)));

    RECT rc{}; GetClientRect(s.hwnd, &rc);
    s.vpW = std::max(1L, rc.right - rc.left);
    s.vpH = std::max(1L, rc.bottom - rc.top);
    s.camera.aspect = (float)s.vpW / (float)s.vpH;
    glViewport(0, 0, s.vpW, s.vpH);
    return true;
}

// Build a camera-facing arrowhead per agent, oriented along velocity.
static void buildAgents(const std::vector<Vec3>& pos, const std::vector<Vec3>& vel,
                        const Vec3& eye, float size, float maxSpeed,
                        bool predator, std::vector<float>& out) {
    out.clear();
    out.reserve(pos.size() * 3 * 6);
    for (size_t i = 0; i < pos.size(); ++i) {
        const Vec3 p = pos[i];
        float sp = vel[i].length();
        Vec3 f = sp > 1e-5f ? vel[i] * (1.0f / sp) : Vec3{1, 0, 0};
        Vec3 toCam = (eye - p).normalized();
        Vec3 side = f.cross(toCam);
        if (side.lengthSquared() < 1e-6f) side = f.cross(Vec3{0, 1, 0});
        side = side.normalized();

        Vec3 tip = p + f * size;
        Vec3 bl = p - f * (size * 0.5f) + side * (size * 0.45f);
        Vec3 br = p - f * (size * 0.5f) - side * (size * 0.45f);

        Vec3 col;
        if (predator) {
            col = {1.0f, 0.25f, 0.18f};
        } else {
            float b = 0.55f + 0.45f * std::min(sp / maxSpeed, 1.0f);
            col = {(0.5f + 0.5f * f.x) * b, (0.5f + 0.5f * f.z) * b, (0.62f + 0.38f * f.y) * b};
        }
        Vec3 tri[3] = {tip, bl, br};
        for (const Vec3& v : tri) {
            out.push_back(v.x); out.push_back(v.y); out.push_back(v.z);
            out.push_back(col.x); out.push_back(col.y); out.push_back(col.z);
        }
    }
}

static void drawAgents(ViewerState& s, GLuint vao, GLuint vbo, std::vector<float>& verts, const Mat4& vp) {
    if (verts.empty()) return;
    glBindBuffer_(GL_ARRAY_BUFFER, vbo);
    glBufferData_(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(verts.size() * sizeof(float)), verts.data(), GL_STREAM_DRAW);
    glUseProgram_(s.boidProgram);
    glUniformMatrix4fv_(glGetUniformLocation_(s.boidProgram, "uViewProj"), 1, GL_FALSE, vp.data());
    glBindVertexArray_(vao);
    glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(verts.size() / 6));
}

static void renderHUD(ViewerState& s) {
    char buf[128];
    std::vector<std::string> lines;
    lines.emplace_back("Boids 3D / Swarm Intelligence");
    std::snprintf(buf, sizeof(buf), "fps %.0f", s.fps); lines.emplace_back(buf);
    std::snprintf(buf, sizeof(buf), "%zu boids   %d predators%s", s.sim->count(), s.predators,
                  s.obstacle ? "   obstacle" : ""); lines.emplace_back(buf);
    std::snprintf(buf, sizeof(buf), "speed x%d%s", s.substeps, s.paused ? "   [PAUSED]" : ""); lines.emplace_back(buf);
    lines.emplace_back("");
    lines.emplace_back("drag / wheel  orbit / zoom");
    lines.emplace_back("space    pause");
    lines.emplace_back("up / dn  sim speed");
    lines.emplace_back("O   obstacle on/off");
    lines.emplace_back("P   predators on/off");
    lines.emplace_back("R   reset    H  hide HUD");
    lines.emplace_back("esc      quit");

    const float sc = 1.5f, gw = FONT_CW * sc, gh = FONT_CH * sc, pad = 10, mx = 12, my = 10;
    size_t maxc = 0; for (auto& l : lines) maxc = std::max(maxc, l.size());
    float panelW = mx + maxc * gw + pad, panelH = my + lines.size() * gh + pad;
    auto ndcX = [&](float px) { return px / s.vpW * 2.0f - 1.0f; };
    auto ndcY = [&](float py) { return 1.0f - py / s.vpH * 2.0f; };
    std::vector<float> v; v.reserve(2048);
    auto quad = [&](float x0, float y0, float x1, float y1, float u0, float w0, float u1, float w1) {
        float ax = ndcX(x0), ay = ndcY(y0), bx = ndcX(x1), by = ndcY(y1);
        float r[6][4] = {{ax,ay,u0,w0},{bx,ay,u1,w0},{bx,by,u1,w1},{ax,ay,u0,w0},{bx,by,u1,w1},{ax,by,u0,w1}};
        for (auto& q : r) { v.push_back(q[0]); v.push_back(q[1]); v.push_back(q[2]); v.push_back(q[3]); }
    };
    quad(4, 4, 4 + panelW, 4 + panelH, 0, 0, 0, 0);
    for (size_t li = 0; li < lines.size(); ++li) {
        float y0 = 4 + my + li * gh;
        const std::string& ln = lines[li];
        for (size_t ci = 0; ci < ln.size(); ++ci) {
            int code = (unsigned char)ln[ci];
            if (code <= 32 || code >= FONT_FIRST + FONT_COUNT) continue;
            int idx = code - FONT_FIRST;
            float x0 = 4 + mx + ci * gw;
            float u0 = (idx * FONT_CW) / (float)FONT_W, u1 = (idx * FONT_CW + FONT_CW) / (float)FONT_W;
            quad(x0, y0, x0 + gw, y0 + gh, u0, 0.0f, u1, 1.0f);
        }
    }
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glActiveTexture_(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, s.fontTex);
    glBindBuffer_(GL_ARRAY_BUFFER, s.hudVbo);
    glBufferData_(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(v.size() * sizeof(float)), v.data(), GL_STREAM_DRAW);
    glUseProgram_(s.hudProgram);
    glUniform1i_(glGetUniformLocation_(s.hudProgram, "uFont"), 0);
    glBindVertexArray_(s.hudVao);
    glUniform1i_(glGetUniformLocation_(s.hudProgram, "uTextured"), 0);
    glUniform3f_(glGetUniformLocation_(s.hudProgram, "uColor"), 0.02f, 0.03f, 0.06f);
    glUniform1f_(glGetUniformLocation_(s.hudProgram, "uAlpha"), 0.55f);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glUniform1i_(glGetUniformLocation_(s.hudProgram, "uTextured"), 1);
    glUniform3f_(glGetUniformLocation_(s.hudProgram, "uColor"), 0.88f, 0.94f, 1.0f);
    glUniform1f_(glGetUniformLocation_(s.hudProgram, "uAlpha"), 1.0f);
    glDrawArrays(GL_TRIANGLES, 6, static_cast<GLsizei>(v.size() / 4 - 6));
    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
}

static void renderFrame(ViewerState& s) {
    if (!s.paused) for (int i = 0; i < s.substeps; ++i) s.sim->step();

    glClearColor(0.015f, 0.02f, 0.035f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    Mat4 vp = s.camera.viewProj();
    const Vec3 eye = s.camera.eye();

    glUseProgram_(s.lineProgram);
    glUniformMatrix4fv_(glGetUniformLocation_(s.lineProgram, "uViewProj"), 1, GL_FALSE, vp.data());
    glUniform3f_(glGetUniformLocation_(s.lineProgram, "uColor"), 0.20f, 0.26f, 0.36f);
    glBindVertexArray_(s.lineVao);
    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(s.boxLines.size() / 3));

    buildAgents(s.sim->positions(), s.sim->velocities(), eye, s.boidSize, s.sim->params().maxSpeed, false, s.boidVerts);
    drawAgents(s, s.boidVao, s.boidVbo, s.boidVerts, vp);

    if (!s.sim->predatorPos().empty()) {
        buildAgents(s.sim->predatorPos(), s.sim->predatorVel(), eye, s.boidSize * 2.4f,
                    s.sim->params().predatorMaxSpeed, true, s.predVerts);
        drawAgents(s, s.predVao, s.predVbo, s.predVerts, vp);
    }

    if (s.showHud) renderHUD(s);

    SwapBuffers(s.hdc);

    ++s.frames;
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now - s.lastTitleUpdate).count();
    if (dt >= 0.5) {
        s.fps = s.frames / dt; s.frames = 0; s.lastTitleUpdate = now;
        char title[192];
        std::snprintf(title, sizeof(title), "Boids 3D  |  %zu boids  |  %.0f fps  |  x%d%s",
                      s.sim->count(), s.fps, s.substeps, s.paused ? "  [PAUSED]" : "");
        SetWindowTextA(s.hwnd, title);
    }
}

static LRESULT CALLBACK windowProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    ViewerState* s = g_state;
    switch (msg) {
    case WM_SIZE:
        if (s) {
            s->vpW = std::max(1, (int)LOWORD(lParam));
            s->vpH = std::max(1, (int)HIWORD(lParam));
            s->camera.aspect = (float)s->vpW / (float)s->vpH;
            if (s->glrc) glViewport(0, 0, s->vpW, s->vpH);
        }
        return 0;
    case WM_LBUTTONDOWN:
        if (s) { s->dragging = true; s->lastX = GET_X_LPARAM(lParam); s->lastY = GET_Y_LPARAM(lParam); SetCapture(hwnd); }
        return 0;
    case WM_LBUTTONUP:
        if (s) { s->dragging = false; ReleaseCapture(); }
        return 0;
    case WM_MOUSEMOVE:
        if (s) {
            double x = GET_X_LPARAM(lParam), y = GET_Y_LPARAM(lParam);
            if (s->dragging) s->camera.rotate((float)(x - s->lastX), (float)(y - s->lastY));
            s->lastX = x; s->lastY = y;
        }
        return 0;
    case WM_MOUSEWHEEL:
        if (s) s->camera.zoom((float)GET_WHEEL_DELTA_WPARAM(wParam) / (float)WHEEL_DELTA);
        return 0;
    case WM_KEYDOWN:
        if (s) {
            switch (wParam) {
            case VK_SPACE: s->paused = !s->paused; break;
            case VK_UP:    s->substeps = std::min(s->substeps + 1, 8); break;
            case VK_DOWN:  s->substeps = std::max(s->substeps - 1, 1); break;
            case 'O':      s->obstacle = !s->obstacle; resetSimulation(*s); break;
            case 'P':      s->predators = s->predators ? 0 : 3; resetSimulation(*s); break;
            case 'R':      resetSimulation(*s); break;
            case 'H':      s->showHud = !s->showHud; break;
            case VK_ESCAPE: DestroyWindow(hwnd); break;
            }
        }
        return 0;
    case WM_CLOSE: DestroyWindow(hwnd); return 0;
    case WM_DESTROY: PostQuitMessage(0); return 0;
    }
    return DefWindowProcA(hwnd, msg, wParam, lParam);
}

int APIENTRY WinMain(HINSTANCE hInstance, HINSTANCE, LPSTR, int nCmdShow) {
    SetProcessDPIAware();
    ViewerState state;
    g_state = &state;

    WNDCLASSEXA wc{};
    wc.cbSize = sizeof(wc); wc.style = CS_OWNDC; wc.lpfnWndProc = windowProc;
    wc.hInstance = hInstance; wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.lpszClassName = "BoidsWindow";
    if (!RegisterClassExA(&wc)) { MessageBoxA(nullptr, "RegisterClassEx failed", "Boids 3D", MB_ICONERROR | MB_OK); return 1; }

    RECT rect{0, 0, 1280, 800};
    AdjustWindowRect(&rect, WS_OVERLAPPEDWINDOW, FALSE);
    state.hwnd = CreateWindowExA(0, wc.lpszClassName, "Boids 3D",
        WS_OVERLAPPEDWINDOW | WS_VISIBLE, CW_USEDEFAULT, CW_USEDEFAULT,
        rect.right - rect.left, rect.bottom - rect.top, nullptr, nullptr, hInstance, nullptr);
    if (!state.hwnd) { MessageBoxA(nullptr, "CreateWindowEx failed", "Boids 3D", MB_ICONERROR | MB_OK); return 1; }

    if (!createOpenGLContext(state)) { MessageBoxA(nullptr, "Could not create an OpenGL context", "Boids 3D", MB_ICONERROR | MB_OK); return 1; }
    if (!initRenderer(state)) { MessageBoxA(nullptr, "Renderer initialization failed", "Boids 3D", MB_ICONERROR | MB_OK); return 1; }

    ShowWindow(state.hwnd, nCmdShow);
    UpdateWindow(state.hwnd);

    MSG msg{};
    bool running = true;
    while (running) {
        while (PeekMessageA(&msg, nullptr, 0, 0, PM_REMOVE)) {
            if (msg.message == WM_QUIT) { running = false; break; }
            TranslateMessage(&msg);
            DispatchMessageA(&msg);
        }
        if (running) renderFrame(state);
    }

    if (state.glrc) { wglMakeCurrent(nullptr, nullptr); wglDeleteContext(state.glrc); }
    if (state.hwnd && state.hdc) ReleaseDC(state.hwnd, state.hdc);
    return 0;
}
