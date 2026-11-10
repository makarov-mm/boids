#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>

#include "vec3.hpp"

// Column-major 4x4 matrix, OpenGL convention.
struct Mat4 {
    std::array<float, 16> m{};

    [[nodiscard]] static constexpr Mat4 identity() noexcept {
        Mat4 r;
        r.m[0] = r.m[5] = r.m[10] = r.m[15] = 1.0f;
        return r;
    }

    // Right-handed perspective, clip-space depth in [-1, 1].
    [[nodiscard]] static Mat4 perspective(float fovY, float aspect, float zNear, float zFar) noexcept {
        const float f = 1.0f / std::tan(fovY * 0.5f);
        Mat4 r;
        r.m[0]  = f / aspect;
        r.m[5]  = f;
        r.m[10] = (zFar + zNear) / (zNear - zFar);
        r.m[11] = -1.0f;
        r.m[14] = (2.0f * zFar * zNear) / (zNear - zFar);
        return r;
    }

    [[nodiscard]] static Mat4 lookAt(const Vec3& eye, const Vec3& center, const Vec3& up) noexcept {
        const Vec3 f = (center - eye).normalized();
        const Vec3 s = f.cross(up).normalized();
        const Vec3 u = s.cross(f);
        Mat4 r;
        r.m[0] = s.x;  r.m[4] = s.y;  r.m[8]  = s.z;
        r.m[1] = u.x;  r.m[5] = u.y;  r.m[9]  = u.z;
        r.m[2] = -f.x; r.m[6] = -f.y; r.m[10] = -f.z;
        r.m[12] = -s.dot(eye);
        r.m[13] = -u.dot(eye);
        r.m[14] = f.dot(eye);
        r.m[15] = 1.0f;
        return r;
    }

    [[nodiscard]] constexpr Mat4 operator*(const Mat4& b) const noexcept {
        Mat4 r;
        for (int col = 0; col < 4; ++col)
            for (int row = 0; row < 4; ++row) {
                float sum = 0;
                for (int k = 0; k < 4; ++k) sum += m[k * 4 + row] * b.m[col * 4 + k];
                r.m[col * 4 + row] = sum;
            }
        return r;
    }

    [[nodiscard]] const float* data() const noexcept { return m.data(); }
};

// Orbit camera around `target`: drag to rotate, wheel to zoom.
struct OrbitCamera {
    Vec3  target{};
    float distance = 60, yaw = 0.7f, pitch = 0.5f;
    float fovY = 55.0f * std::numbers::pi_v<float> / 180.0f, aspect = 1.6f, zNear = 0.1f, zFar = 5000.0f;

    [[nodiscard]] Vec3 eye() const noexcept {
        const float cp = std::cos(pitch);
        return target + Vec3{ cp * std::cos(yaw), std::sin(pitch), cp * std::sin(yaw) } * distance;
    }

    [[nodiscard]] Mat4 viewProj() const noexcept {
        return Mat4::perspective(fovY, aspect, zNear, zFar) * Mat4::lookAt(eye(), target, { 0, 1, 0 });
    }

    void rotate(float dx, float dy) noexcept {
        yaw  += dx * 0.005f;
        pitch = std::clamp(pitch + dy * 0.005f, -1.55f, 1.55f);
    }

    void zoom(float wheelSteps) noexcept {
        distance = std::clamp(distance * std::exp(-wheelSteps * 0.12f), 3.0f, 4000.0f);
    }
};
