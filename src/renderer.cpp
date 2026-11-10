#include "renderer.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <numbers>
#include <stdexcept>
#include <utility>

#include "font_atlas.hpp"

namespace {
constexpr float BOID_SIZE      = 1.4f;
constexpr float PREDATOR_SCALE = 2.4f;   // predators are drawn larger than boids

// ---- Agents: one camera-facing arrowhead per instance ------------------------
// Vertex 0 is the tip, 1 and 2 the base corners, matching the classic CPU
// construction: tip = p + f*s, base = p - f*s/2 +/- side*0.45*s.
constexpr const char* AGENT_VS = R"(#version 330 core
layout(location = 0) in vec3 aPos;   // per instance
layout(location = 1) in vec3 aVel;   // per instance
uniform mat4  uViewProj;
uniform vec3  uEye;
uniform float uSize;
uniform float uMaxSpeed;
uniform bool  uPredator;
out vec3 vColor;

vec3 safeNormalize(vec3 v) { float l = length(v); return l > 0.0 ? v / l : vec3(0.0); }

void main() {
    float sp   = length(aVel);
    vec3  f    = sp > 1e-5 ? aVel * (1.0 / sp) : vec3(1.0, 0.0, 0.0);
    vec3  side = cross(f, safeNormalize(uEye - aPos));
    if (dot(side, side) < 1e-6) side = cross(f, vec3(0.0, 1.0, 0.0));
    side = safeNormalize(side);

    vec3 p = aPos + f * uSize;
    if (gl_VertexID != 0) {
        float s = (gl_VertexID == 1) ? 1.0 : -1.0;
        p = aPos - f * (uSize * 0.5) + side * (s * uSize * 0.45);
    }
    gl_Position = uViewProj * vec4(p, 1.0);

    // Colour by heading, so locally aligned groups share a colour; brighter when faster.
    float b = 0.55 + 0.45 * min(sp / uMaxSpeed, 1.0);
    vColor  = uPredator ? vec3(1.0, 0.25, 0.18)
                        : vec3(0.5 + 0.5 * f.x, 0.5 + 0.5 * f.z, 0.62 + 0.38 * f.y) * b;
}
)";

constexpr const char* AGENT_FS = R"(#version 330 core
in vec3 vColor;
out vec4 frag;
void main() { frag = vec4(vColor, 1.0); }
)";

constexpr const char* LINE_VS = R"(#version 330 core
layout(location = 0) in vec3 aPos;
uniform mat4 uViewProj;
void main() { gl_Position = uViewProj * vec4(aPos, 1.0); }
)";

constexpr const char* LINE_FS = R"(#version 330 core
out vec4 frag;
uniform vec3 uColor;
void main() { frag = vec4(uColor, 1.0); }
)";

constexpr const char* HUD_VS = R"(#version 330 core
layout(location = 0) in vec2 aPos;
layout(location = 1) in vec2 aUV;
out vec2 vUV;
void main() { vUV = aUV; gl_Position = vec4(aPos, 0.0, 1.0); }
)";

constexpr const char* HUD_FS = R"(#version 330 core
in vec2 vUV;
out vec4 frag;
uniform sampler2D uFont;
uniform vec3  uColor;
uniform float uAlpha;
uniform bool  uTextured;
void main() {
    float coverage = uTextured ? texture(uFont, vUV).r : 1.0;
    frag = vec4(uColor, uAlpha * coverage);
}
)";

GLuint compileShader(GLenum type, const char* source) {
    const GLuint sh = gl::CreateShader(type);
    gl::ShaderSource(sh, 1, &source, nullptr);
    gl::CompileShader(sh);
    GLint ok = 0;
    gl::GetShaderiv(sh, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        GLint length = 0;
        gl::GetShaderiv(sh, GL_INFO_LOG_LENGTH, &length);
        std::string log(static_cast<std::size_t>(std::max(length, 1)), '\0');
        gl::GetShaderInfoLog(sh, length, nullptr, log.data());
        gl::DeleteShader(sh);
        throw std::runtime_error(std::format("{} shader does not compile:\n{}",
                                             type == GL_VERTEX_SHADER ? "Vertex" : "Fragment", log.c_str()));
    }
    return sh;
}

GLuint linkProgram(const char* vs, const char* fs) {
    const GLuint v = compileShader(GL_VERTEX_SHADER, vs);
    GLuint f = 0;
    try {
        f = compileShader(GL_FRAGMENT_SHADER, fs);
    } catch (...) {
        gl::DeleteShader(v);
        throw;
    }
    const GLuint program = gl::CreateProgram();
    gl::AttachShader(program, v);
    gl::AttachShader(program, f);
    gl::LinkProgram(program);
    gl::DeleteShader(v);   // flagged; freed with the program
    gl::DeleteShader(f);

    GLint ok = 0;
    gl::GetProgramiv(program, GL_LINK_STATUS, &ok);
    if (!ok) {
        GLint length = 0;
        gl::GetProgramiv(program, GL_INFO_LOG_LENGTH, &length);
        std::string log(static_cast<std::size_t>(std::max(length, 1)), '\0');
        gl::GetProgramInfoLog(program, length, nullptr, log.data());
        gl::DeleteProgram(program);
        throw std::runtime_error(std::format("Shader program does not link:\n{}", log.c_str()));
    }
    return program;
}

void appendBox(const Vec3& a, const Vec3& b, std::vector<float>& out) {
    const Vec3 c[8] = { { a.x, a.y, a.z }, { b.x, a.y, a.z }, { b.x, b.y, a.z }, { a.x, b.y, a.z },
                        { a.x, a.y, b.z }, { b.x, a.y, b.z }, { b.x, b.y, b.z }, { a.x, b.y, b.z } };
    constexpr int edges[24] = { 0, 1, 1, 2, 2, 3, 3, 0,  4, 5, 5, 6, 6, 7, 7, 4,  0, 4, 1, 5, 2, 6, 3, 7 };
    for (int i : edges) out.insert(out.end(), { c[i].x, c[i].y, c[i].z });
}

// Three great circles of the sphere, one per axis plane.
void appendSphereWire(const Vec3& c, float r, std::vector<float>& out) {
    constexpr int segments = 32;
    for (int axis = 0; axis < 3; ++axis) {
        auto point = [&](int i) {
            const float a  = static_cast<float>(i) / segments * 2.0f * std::numbers::pi_v<float>;
            const float ca = r * std::cos(a), sa = r * std::sin(a);
            return axis == 0 ? Vec3{ c.x, c.y + ca, c.z + sa }
                 : axis == 1 ? Vec3{ c.x + ca, c.y, c.z + sa }
                             : Vec3{ c.x + ca, c.y + sa, c.z };
        };
        for (int i = 0; i < segments; ++i) {
            const Vec3 p0 = point(i), p1 = point(i + 1);
            out.insert(out.end(), { p0.x, p0.y, p0.z, p1.x, p1.y, p1.z });
        }
    }
}

void* offset(std::size_t bytes) noexcept { return reinterpret_cast<void*>(bytes); }
} // namespace

// =============================================================================
//  Setup / teardown
// =============================================================================
Renderer::Renderer() {
    try {
        agentProgram_ = linkProgram(AGENT_VS, AGENT_FS);
        lineProgram_  = linkProgram(LINE_VS, LINE_FS);
        hudProgram_   = linkProgram(HUD_VS, HUD_FS);
    } catch (...) {
        for (GLuint p : { agentProgram_, lineProgram_, hudProgram_ }) if (p) gl::DeleteProgram(p);
        throw;
    }
    agentUniform_ = { gl::GetUniformLocation(agentProgram_, "uViewProj"), gl::GetUniformLocation(agentProgram_, "uEye"),
                      gl::GetUniformLocation(agentProgram_, "uSize"), gl::GetUniformLocation(agentProgram_, "uMaxSpeed"),
                      gl::GetUniformLocation(agentProgram_, "uPredator") };
    lineUniform_  = { gl::GetUniformLocation(lineProgram_, "uViewProj"), gl::GetUniformLocation(lineProgram_, "uColor") };
    hudUniform_   = { gl::GetUniformLocation(hudProgram_, "uFont"), gl::GetUniformLocation(hudProgram_, "uColor"),
                      gl::GetUniformLocation(hudProgram_, "uAlpha"), gl::GetUniformLocation(hudProgram_, "uTextured") };

    // Font atlas: one 8-bit coverage channel.
    gl::GenTextures(1, &fontTex_);
    gl::ActiveTexture(GL_TEXTURE0);
    gl::BindTexture(GL_TEXTURE_2D, fontTex_);
    gl::TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    gl::TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    gl::TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    gl::TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    gl::PixelStorei(GL_UNPACK_ALIGNMENT, 1);
    gl::TexImage2D(GL_TEXTURE_2D, 0, GL_R8, FONT_W, FONT_H, 0, GL_RED, GL_UNSIGNED_BYTE, FONT_PIX);

    createAgents(boids_);
    createAgents(predators_);

    gl::GenVertexArrays(1, &lineVao_);
    gl::GenBuffers(1, &lineVbo_);
    gl::BindVertexArray(lineVao_);
    gl::BindBuffer(GL_ARRAY_BUFFER, lineVbo_);
    gl::EnableVertexAttribArray(0);
    gl::VertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, static_cast<GLsizei>(3 * sizeof(float)), offset(0));

    gl::GenVertexArrays(1, &hudVao_);
    gl::GenBuffers(1, &hudVbo_);
    gl::BindVertexArray(hudVao_);
    gl::BindBuffer(GL_ARRAY_BUFFER, hudVbo_);
    gl::EnableVertexAttribArray(0);
    gl::VertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, static_cast<GLsizei>(4 * sizeof(float)), offset(0));
    gl::EnableVertexAttribArray(1);
    gl::VertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, static_cast<GLsizei>(4 * sizeof(float)), offset(2 * sizeof(float)));
    gl::BindVertexArray(0);

    gl::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void Renderer::createAgents(Agents& a) {
    gl::GenVertexArrays(1, &a.vao);
    gl::GenBuffers(1, &a.pos);
    gl::GenBuffers(1, &a.vel);
    gl::BindVertexArray(a.vao);
    for (const auto& [location, buffer] : { std::pair{ 0u, a.pos }, std::pair{ 1u, a.vel } }) {
        gl::BindBuffer(GL_ARRAY_BUFFER, buffer);
        gl::EnableVertexAttribArray(location);
        gl::VertexAttribPointer(location, 3, GL_FLOAT, GL_FALSE, static_cast<GLsizei>(sizeof(Vec3)), offset(0));
        gl::VertexAttribDivisor(location, 1);   // one value per arrowhead
    }
}

Renderer::~Renderer() {
    for (GLuint p : { agentProgram_, lineProgram_, hudProgram_ }) gl::DeleteProgram(p);
    const GLuint buffers[] = { boids_.pos, boids_.vel, predators_.pos, predators_.vel, lineVbo_, hudVbo_ };
    const GLuint arrays[]  = { boids_.vao, predators_.vao, lineVao_, hudVao_ };
    gl::DeleteBuffers(static_cast<GLsizei>(std::size(buffers)), buffers);
    gl::DeleteVertexArrays(static_cast<GLsizei>(std::size(arrays)), arrays);
    gl::DeleteTextures(1, &fontTex_);
}

void Renderer::resize(int width, int height) {
    width_  = std::max(width, 1);
    height_ = std::max(height, 1);
    gl::Viewport(0, 0, width_, height_);
}

void Renderer::setWorld(const BoidsParams& p) {
    std::vector<float> lines;
    appendBox(p.boundsMin, p.boundsMax, lines);
    if (p.obstacle) appendSphereWire(p.obstacleCenter, p.obstacleRadius, lines);
    gl::BindBuffer(GL_ARRAY_BUFFER, lineVbo_);
    gl::BufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(lines.size() * sizeof(float)), lines.data(), GL_STATIC_DRAW);
    lineVertices_ = static_cast<GLsizei>(lines.size() / 3);
}

// =============================================================================
//  Frame
// =============================================================================
void Renderer::drawScene(const OrbitCamera& camera, const Boids& sim) {
    gl::ClearColor(0.015f, 0.02f, 0.035f, 1.0f);
    gl::Clear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    gl::Enable(GL_DEPTH_TEST);

    const Mat4 viewProj = camera.viewProj();

    gl::UseProgram(lineProgram_);
    gl::UniformMatrix4fv(lineUniform_.viewProj, 1, GL_FALSE, viewProj.data());
    gl::Uniform3f(lineUniform_.color, 0.20f, 0.26f, 0.36f);
    gl::BindVertexArray(lineVao_);
    gl::DrawArrays(GL_LINES, 0, lineVertices_);

    const Vec3 eye = camera.eye();
    gl::UseProgram(agentProgram_);
    gl::UniformMatrix4fv(agentUniform_.viewProj, 1, GL_FALSE, viewProj.data());
    gl::Uniform3f(agentUniform_.eye, eye.x, eye.y, eye.z);
    drawAgents(boids_, sim.positions(), sim.velocities(), BOID_SIZE, sim.params().maxSpeed, false);
    drawAgents(predators_, sim.predatorPos(), sim.predatorVel(), BOID_SIZE * PREDATOR_SCALE,
               sim.params().predatorMaxSpeed, true);
}

void Renderer::drawAgents(const Agents& a, std::span<const Vec3> pos, std::span<const Vec3> vel,
                          float size, float maxSpeed, bool predator) {
    if (pos.empty()) return;
    // Streamed every frame (fresh storage, so no stall on last frame's draw).
    gl::BindBuffer(GL_ARRAY_BUFFER, a.pos);
    gl::BufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(pos.size_bytes()), pos.data(), GL_STREAM_DRAW);
    gl::BindBuffer(GL_ARRAY_BUFFER, a.vel);
    gl::BufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(vel.size_bytes()), vel.data(), GL_STREAM_DRAW);

    gl::Uniform1f(agentUniform_.size, size);
    gl::Uniform1f(agentUniform_.maxSpeed, maxSpeed);
    gl::Uniform1i(agentUniform_.predator, predator ? 1 : 0);
    gl::BindVertexArray(a.vao);
    gl::DrawArraysInstanced(GL_TRIANGLES, 0, 3, static_cast<GLsizei>(pos.size()));
}

// A translucent panel with one text line per entry, in the top-left corner.
// Layout in pixels at 96 DPI, scaled by `dpiScale`.
void Renderer::drawHud(std::span<const std::string> lines, float dpiScale) {
    const float glyphScale = 1.5f * dpiScale;
    const float gw = FONT_CW * glyphScale, gh = FONT_CH * glyphScale;
    const float inset = 4 * dpiScale, padX = 12 * dpiScale, padY = 10 * dpiScale, padEnd = 10 * dpiScale;

    std::size_t longest = 0;
    for (const std::string& l : lines) longest = std::max(longest, l.size());
    const float panelW = padX + static_cast<float>(longest) * gw + padEnd;
    const float panelH = padY + static_cast<float>(lines.size()) * gh + padEnd;

    // Pixel rectangle -> two triangles of (ndc.xy, uv).
    hudVerts_.clear();
    auto quad = [&](float x0, float y0, float x1, float y1, float u0, float v0, float u1, float v1) {
        const float ax = x0 / static_cast<float>(width_) * 2.0f - 1.0f, ay = 1.0f - y0 / static_cast<float>(height_) * 2.0f;
        const float bx = x1 / static_cast<float>(width_) * 2.0f - 1.0f, by = 1.0f - y1 / static_cast<float>(height_) * 2.0f;
        hudVerts_.insert(hudVerts_.end(), { ax, ay, u0, v0,  bx, ay, u1, v0,  bx, by, u1, v1,
                                            ax, ay, u0, v0,  bx, by, u1, v1,  ax, by, u0, v1 });
    };
    quad(inset, inset, inset + panelW, inset + panelH, 0, 0, 0, 0);   // background
    for (std::size_t li = 0; li < lines.size(); ++li) {
        const float y0 = inset + padY + static_cast<float>(li) * gh;
        for (std::size_t ci = 0; ci < lines[li].size(); ++ci) {
            const int code = static_cast<unsigned char>(lines[li][ci]);
            if (code <= FONT_FIRST || code >= FONT_FIRST + FONT_COUNT) continue;   // space or not in the atlas
            // Half a texel in from the cell edges: linear filtering would
            // otherwise pull in the neighbouring glyph's edge column.
            const int   glyph = code - FONT_FIRST;
            const float x0    = inset + padX + static_cast<float>(ci) * gw;
            const float u0    = (static_cast<float>(glyph * FONT_CW) + 0.5f) / FONT_W;
            const float u1    = (static_cast<float>(glyph * FONT_CW + FONT_CW) - 0.5f) / FONT_W;
            quad(x0, y0, x0 + gw, y0 + gh, u0, 0.0f, u1, 1.0f);
        }
    }

    gl::Disable(GL_DEPTH_TEST);
    gl::Enable(GL_BLEND);
    gl::ActiveTexture(GL_TEXTURE0);
    gl::BindTexture(GL_TEXTURE_2D, fontTex_);
    gl::BindBuffer(GL_ARRAY_BUFFER, hudVbo_);
    gl::BufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(hudVerts_.size() * sizeof(float)), hudVerts_.data(), GL_STREAM_DRAW);
    gl::UseProgram(hudProgram_);
    gl::Uniform1i(hudUniform_.font, 0);
    gl::BindVertexArray(hudVao_);

    gl::Uniform1i(hudUniform_.textured, 0);
    gl::Uniform3f(hudUniform_.color, 0.02f, 0.03f, 0.06f);
    gl::Uniform1f(hudUniform_.alpha, 0.55f);
    gl::DrawArrays(GL_TRIANGLES, 0, 6);

    gl::Uniform1i(hudUniform_.textured, 1);
    gl::Uniform3f(hudUniform_.color, 0.88f, 0.94f, 1.0f);
    gl::Uniform1f(hudUniform_.alpha, 1.0f);
    gl::DrawArrays(GL_TRIANGLES, 6, static_cast<GLsizei>(hudVerts_.size() / 4 - 6));

    gl::Disable(GL_BLEND);
    gl::Enable(GL_DEPTH_TEST);
}
