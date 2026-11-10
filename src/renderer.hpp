#pragma once
#include <span>
#include <string>
#include <vector>

#include "boids.hpp"
#include "gl_api.hpp"
#include "mat4.hpp"
#include "vec3.hpp"

// =============================================================================
//  Draws the flock: the bounding box (and obstacle) as lines, every agent as a
//  camera-facing arrowhead -- one instanced draw for the boids, one for the
//  predators, built in the vertex shader straight from the simulation's
//  position and velocity arrays -- and the HUD as a screen-space text panel.
//
//  Needs a current OpenGL 3.3 context with gl::load() done. Throws
//  std::runtime_error when a shader does not compile or link.
// =============================================================================
class Renderer {
public:
    Renderer();
    ~Renderer();

    Renderer(const Renderer&)            = delete;
    Renderer& operator=(const Renderer&) = delete;

    void resize(int width, int height);
    void setWorld(const BoidsParams& p);   // box + obstacle wireframe (call when they change)
    void drawScene(const OrbitCamera& camera, const Boids& sim);
    void drawHud(std::span<const std::string> lines, float dpiScale);

private:
    // Per-instance position and velocity buffers of one agent kind.
    struct Agents {
        GLuint vao = 0, pos = 0, vel = 0;
    };
    void createAgents(Agents& a);
    void drawAgents(const Agents& a, std::span<const Vec3> pos, std::span<const Vec3> vel,
                    float size, float maxSpeed, bool predator);

    int width_ = 1, height_ = 1;

    GLuint agentProgram_ = 0, lineProgram_ = 0, hudProgram_ = 0;
    struct { GLint viewProj, eye, size, maxSpeed, predator; } agentUniform_{};
    struct { GLint viewProj, color; }                         lineUniform_{};
    struct { GLint font, color, alpha, textured; }            hudUniform_{};

    Agents  boids_, predators_;
    GLuint  lineVao_ = 0, lineVbo_ = 0;
    GLsizei lineVertices_ = 0;
    GLuint  hudVao_ = 0, hudVbo_ = 0, fontTex_ = 0;
    std::vector<float> hudVerts_;   // rebuilt every frame, storage reused
};
