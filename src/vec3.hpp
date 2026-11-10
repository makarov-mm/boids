#pragma once
#include <cmath>

// Minimal 3D vector. No external math library by design.
struct Vec3 {
    float x = 0, y = 0, z = 0;

    constexpr Vec3() = default;
    constexpr Vec3(float x_, float y_, float z_) noexcept : x(x_), y(y_), z(z_) {}

    [[nodiscard]] constexpr Vec3 operator+(const Vec3& o) const noexcept { return { x + o.x, y + o.y, z + o.z }; }
    [[nodiscard]] constexpr Vec3 operator-(const Vec3& o) const noexcept { return { x - o.x, y - o.y, z - o.z }; }
    [[nodiscard]] constexpr Vec3 operator-()              const noexcept { return { -x, -y, -z }; }
    [[nodiscard]] constexpr Vec3 operator*(float s)       const noexcept { return { x * s, y * s, z * s }; }
    [[nodiscard]] constexpr Vec3 operator/(float s)       const noexcept { return { x / s, y / s, z / s }; }

    constexpr Vec3& operator+=(const Vec3& o) noexcept { x += o.x; y += o.y; z += o.z; return *this; }
    constexpr Vec3& operator-=(const Vec3& o) noexcept { x -= o.x; y -= o.y; z -= o.z; return *this; }
    constexpr Vec3& operator*=(float s)       noexcept { x *= s;   y *= s;   z *= s;   return *this; }

    [[nodiscard]] constexpr float dot(const Vec3& o) const noexcept { return x * o.x + y * o.y + z * o.z; }
    [[nodiscard]] constexpr float lengthSquared()    const noexcept { return dot(*this); }
    [[nodiscard]] float           length()           const noexcept { return std::sqrt(lengthSquared()); }

    [[nodiscard]] Vec3 normalized() const noexcept {
        const float l = length();
        return l > 0 ? *this / l : Vec3{};
    }

    [[nodiscard]] constexpr Vec3 cross(const Vec3& o) const noexcept {
        return { y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x };
    }

    friend constexpr bool operator==(const Vec3&, const Vec3&) = default;
};

[[nodiscard]] constexpr Vec3 operator*(float s, const Vec3& v) noexcept { return v * s; }

// Positions and velocities are uploaded to the GPU straight from std::vector<Vec3>.
static_assert(sizeof(Vec3) == 3 * sizeof(float), "Vec3 must stay a tightly packed vec3");
