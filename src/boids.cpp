#include "boids.hpp"
#include <algorithm>
#include <cmath>
#include <thread>

template <class Fn>
static void parallelFor(int n, int threads, Fn&& fn) {
    if (threads <= 1 || n < 2048) { for (int i = 0; i < n; ++i) fn(i); return; }
    std::vector<std::thread> pool;
    int chunk = (n + threads - 1) / threads;
    for (int t = 0; t < threads; ++t) {
        int b = t * chunk, e = std::min(b + chunk, n);
        if (b >= e) break;
        pool.emplace_back([&fn, b, e]() { for (int i = b; i < e; ++i) fn(i); });
    }
    for (auto& th : pool) th.join();
}

static inline uint32_t xorshift(uint64_t& s) {
    s ^= s << 13; s ^= s >> 7; s ^= s << 17;
    return (uint32_t)(s >> 32);
}
static inline float frand(uint64_t& s) { return (xorshift(s) >> 8) * (1.0f / 16777216.0f); }
static inline float frand(uint64_t& s, float a, float b) { return a + (b - a) * frand(s); }

static inline Vec3 limit(const Vec3& v, float m) {
    float l2 = v.lengthSquared();
    if (l2 > m * m && l2 > 1e-12f) return v * (m / std::sqrt(l2));
    return v;
}
static inline Vec3 setMag(const Vec3& v, float m) {
    float l = v.length();
    return l > 1e-6f ? v * (m / l) : Vec3{};
}

Boids::Boids(const BoidsParams& p) : p_(p) {
    const int n = p_.boids;
    pos_.resize(n); vel_.resize(n); acc_.assign(n, Vec3{});
    uint64_t s = p_.seed ? p_.seed : 1;
    Vec3 c = (p_.boundsMin + p_.boundsMax) * 0.5f;
    Vec3 ext = (p_.boundsMax - p_.boundsMin);
    for (int i = 0; i < n; ++i) {
        pos_[i] = {c.x + frand(s, -0.3f, 0.3f) * ext.x,
                   c.y + frand(s, -0.3f, 0.3f) * ext.y,
                   c.z + frand(s, -0.3f, 0.3f) * ext.z};
        Vec3 d{frand(s, -1, 1), frand(s, -1, 1), frand(s, -1, 1)};
        vel_[i] = setMag(d, frand(s, p_.minSpeed, p_.maxSpeed));
    }
    ppos_.resize(p_.predators); pvel_.resize(p_.predators);
    for (int i = 0; i < p_.predators; ++i) {
        ppos_[i] = {frand(s, p_.boundsMin.x, p_.boundsMax.x),
                    frand(s, p_.boundsMin.y, p_.boundsMax.y),
                    frand(s, p_.boundsMin.z, p_.boundsMax.z)};
        Vec3 d{frand(s, -1, 1), frand(s, -1, 1), frand(s, -1, 1)};
        pvel_[i] = setMag(d, p_.predatorMaxSpeed * 0.6f);
    }
}

void Boids::step() {
    const int n = (int)pos_.size();
    grid_.build(pos_, p_.perception, p_.boundsMin, p_.boundsMax);

    const float per2 = p_.perception * p_.perception;
    const float sep2 = p_.separationDist * p_.separationDist;
    const float flee2 = p_.fleeRadius * p_.fleeRadius;

    // Phase 1: compute accelerations (read-only positions/velocities + grid).
    parallelFor(n, threads_, [&](int i) {
        const Vec3 pi = pos_[i];
        const Vec3 vi = vel_[i];
        Vec3 sepSum{}, aliSum{}, cohSum{};
        int cohCount = 0, aliCount = 0;

        grid_.forNeighbors(pi, [&](int j) {
            if (j == i) return;
            Vec3 d = pi - pos_[j];
            float dist2 = d.lengthSquared();
            if (dist2 > per2 || dist2 < 1e-8f) return;
            cohSum += pos_[j]; aliSum += vel_[j]; ++cohCount; ++aliCount;
            if (dist2 < sep2) sepSum += d * (1.0f / dist2);   // weighted push apart
        });

        Vec3 acc{};
        if (aliCount > 0) {
            Vec3 desired = setMag(aliSum * (1.0f / aliCount), p_.maxSpeed);
            acc += limit(desired - vi, p_.maxForce) * p_.wAlignment;
            Vec3 center = cohSum * (1.0f / cohCount);
            Vec3 cohDesired = setMag(center - pi, p_.maxSpeed);
            acc += limit(cohDesired - vi, p_.maxForce) * p_.wCohesion;
        }
        if (sepSum.lengthSquared() > 0) {
            Vec3 desired = setMag(sepSum, p_.maxSpeed);
            acc += limit(desired - vi, p_.maxForce) * p_.wSeparation;
        }

        // flee predators
        Vec3 flee{};
        for (int k = 0; k < (int)ppos_.size(); ++k) {
            Vec3 d = pi - ppos_[k];
            float d2 = d.lengthSquared();
            if (d2 < flee2 && d2 > 1e-6f) flee += d * (1.0f / d2);
        }
        if (flee.lengthSquared() > 0) {
            Vec3 desired = setMag(flee, p_.maxSpeed);
            acc += limit(desired - vi, p_.maxForce) * p_.wFlee;
        }

        // soft bounds: steer back when inside the margin band
        Vec3 b{};
        if (pi.x < p_.boundsMin.x + p_.margin) b.x += 1; else if (pi.x > p_.boundsMax.x - p_.margin) b.x -= 1;
        if (pi.y < p_.boundsMin.y + p_.margin) b.y += 1; else if (pi.y > p_.boundsMax.y - p_.margin) b.y -= 1;
        if (pi.z < p_.boundsMin.z + p_.margin) b.z += 1; else if (pi.z > p_.boundsMax.z - p_.margin) b.z -= 1;
        if (b.lengthSquared() > 0) acc += setMag(b, p_.maxForce) * p_.wBounds;

        // optional obstacle avoidance
        if (p_.obstacle) {
            Vec3 d = pi - p_.obstacleCenter;
            float dist = d.length();
            float safe = p_.obstacleRadius + p_.margin;
            if (dist < safe && dist > 1e-4f) {
                float strength = (safe - dist) / safe;
                acc += setMag(d, p_.maxForce) * (p_.wObstacle * strength);
            }
        }

        acc_[i] = acc;
    });

    // Phase 2: integrate boids.
    parallelFor(n, threads_, [&](int i) {
        Vec3 v = vel_[i] + acc_[i];
        float sp = v.length();
        if (sp > p_.maxSpeed) v = v * (p_.maxSpeed / sp);
        else if (sp < p_.minSpeed && sp > 1e-5f) v = v * (p_.minSpeed / sp);
        vel_[i] = v;
        pos_[i] += v * p_.dt;
    });

    // Predators: seek the nearest boid, bounce softly off walls.
    for (int k = 0; k < (int)ppos_.size(); ++k) {
        Vec3 pk = ppos_[k];
        int best = -1; float bestD2 = 1e30f;
        // sample via grid neighbourhood first; fall back to coarse scan
        grid_.forNeighbors(pk, [&](int j) {
            float d2 = (pos_[j] - pk).lengthSquared();
            if (d2 < bestD2) { bestD2 = d2; best = j; }
        });
        if (best < 0) {
            for (int j = 0; j < n; j += 7) {
                float d2 = (pos_[j] - pk).lengthSquared();
                if (d2 < bestD2) { bestD2 = d2; best = j; }
            }
        }
        Vec3 acc{};
        if (best >= 0) {
            Vec3 desired = setMag(pos_[best] - pk, p_.predatorMaxSpeed);
            acc += limit(desired - pvel_[k], p_.predatorMaxForce);
        }
        Vec3 b{};
        if (pk.x < p_.boundsMin.x + p_.margin) b.x += 1; else if (pk.x > p_.boundsMax.x - p_.margin) b.x -= 1;
        if (pk.y < p_.boundsMin.y + p_.margin) b.y += 1; else if (pk.y > p_.boundsMax.y - p_.margin) b.y -= 1;
        if (pk.z < p_.boundsMin.z + p_.margin) b.z += 1; else if (pk.z > p_.boundsMax.z - p_.margin) b.z -= 1;
        if (b.lengthSquared() > 0) acc += setMag(b, p_.predatorMaxForce) * 2.5f;

        Vec3 v = pvel_[k] + acc;
        float sp = v.length();
        if (sp > p_.predatorMaxSpeed) v = v * (p_.predatorMaxSpeed / sp);
        pvel_[k] = v;
        ppos_[k] += v * p_.dt;
    }
}
