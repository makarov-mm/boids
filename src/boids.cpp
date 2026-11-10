#include "boids.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <span>

namespace {
std::uint32_t xorshift(std::uint64_t& s) noexcept {
    s ^= s << 13; s ^= s >> 7; s ^= s << 17;
    return static_cast<std::uint32_t>(s >> 32);
}
float frand(std::uint64_t& s) noexcept { return static_cast<float>(xorshift(s) >> 8) * (1.0f / 16777216.0f); }
float frand(std::uint64_t& s, float a, float b) noexcept { return a + (b - a) * frand(s); }

Vec3 limit(const Vec3& v, float m) noexcept {
    const float l2 = v.lengthSquared();
    if (l2 > m * m && l2 > 1e-12f) return v * (m / std::sqrt(l2));
    return v;
}

Vec3 setMag(const Vec3& v, float m) noexcept {
    const float l = v.length();
    return l > 1e-6f ? v * (m / l) : Vec3{};
}

// Unit push back into the box for each axis where `p` is inside the wall band.
Vec3 wallPush(const Vec3& p, const BoidsParams& prm) noexcept {
    Vec3 b{};
    if (p.x < prm.boundsMin.x + prm.margin) b.x += 1; else if (p.x > prm.boundsMax.x - prm.margin) b.x -= 1;
    if (p.y < prm.boundsMin.y + prm.margin) b.y += 1; else if (p.y > prm.boundsMax.y - prm.margin) b.y -= 1;
    if (p.z < prm.boundsMin.z + prm.margin) b.z += 1; else if (p.z > prm.boundsMax.z - prm.margin) b.z -= 1;
    return b;
}

// Work split: the neighbour phase varies a lot per boid (dense flocks vs
// stragglers), so it is handed out in small blocks; the rest is uniform.
constexpr int STEER_BLOCK = 64;
constexpr int BULK_BLOCK  = 4096;
} // namespace

Boids::Boids(const BoidsParams& p) : p_(p), pool_(p.threads) {
    reset();
}

void Boids::reset() {
    rng_ = p_.seed ? p_.seed : 1;

    const int n = std::max(p_.boids, 0);
    pos_.resize(n);
    vel_.resize(n);
    acc_.assign(n, Vec3{});
    const Vec3 c   = (p_.boundsMin + p_.boundsMax) * 0.5f;
    const Vec3 ext = p_.boundsMax - p_.boundsMin;
    for (int i = 0; i < n; ++i) {
        pos_[i] = { c.x + frand(rng_, -0.3f, 0.3f) * ext.x,
                    c.y + frand(rng_, -0.3f, 0.3f) * ext.y,
                    c.z + frand(rng_, -0.3f, 0.3f) * ext.z };
        const Vec3 d{ frand(rng_, -1, 1), frand(rng_, -1, 1), frand(rng_, -1, 1) };
        vel_[i] = setMag(d, frand(rng_, p_.minSpeed, p_.maxSpeed));
    }

    ppos_.clear();
    pvel_.clear();
    for (int i = 0; i < p_.predators; ++i) spawnPredator();
}

void Boids::setPredators(int count) {
    p_.predators = std::max(count, 0);
    if (ppos_.size() > static_cast<std::size_t>(p_.predators)) {
        ppos_.resize(p_.predators);
        pvel_.resize(p_.predators);
    }
    while (ppos_.size() < static_cast<std::size_t>(p_.predators)) spawnPredator();
}

void Boids::spawnPredator() {
    const Vec3 at{ frand(rng_, p_.boundsMin.x, p_.boundsMax.x),
                   frand(rng_, p_.boundsMin.y, p_.boundsMax.y),
                   frand(rng_, p_.boundsMin.z, p_.boundsMax.z) };
    const Vec3 d{ frand(rng_, -1, 1), frand(rng_, -1, 1), frand(rng_, -1, 1) };
    ppos_.push_back(at);
    pvel_.push_back(setMag(d, p_.predatorMaxSpeed * 0.6f));
}

void Boids::step() {
    const int n = static_cast<int>(pos_.size());
    grid_.build(pos_, p_.perception, p_.boundsMin, p_.boundsMax);

    // Gather positions and velocities in cell order. Neighbour queries then
    // read contiguous memory, and consecutive boids (same or adjacent cells)
    // reuse what the previous query pulled into the cache.
    const std::span<const int> order = grid_.order();
    sortedPos_.resize(n);
    sortedVel_.resize(n);
    pool_.parallelFor(n, BULK_BLOCK, [&](int begin, int end) {
        for (int k = begin; k < end; ++k) {
            sortedPos_[k] = pos_[order[k]];
            sortedVel_[k] = vel_[order[k]];
        }
    });

    // Phase 1: accelerations (read-only positions / velocities + grid), in
    // grid order for cache locality.
    const Radii radii{ p_.perception * p_.perception, p_.separationDist * p_.separationDist, p_.fleeRadius * p_.fleeRadius };
    pool_.parallelFor(n, STEER_BLOCK, [&](int begin, int end) {
        for (int k = begin; k < end; ++k) acc_[order[k]] = steering(k, radii);
    });

    // Phase 2: integrate boids.
    pool_.parallelFor(n, BULK_BLOCK, [&](int begin, int end) {
        for (int i = begin; i < end; ++i) {
            Vec3 v = vel_[i] + acc_[i];
            const float sp = v.length();
            if (sp > p_.maxSpeed) v = v * (p_.maxSpeed / sp);
            else if (sp < p_.minSpeed && sp > 1e-5f) v = v * (p_.minSpeed / sp);
            vel_[i] = v;
            pos_[i] += v * p_.dt;
        }
    });

    stepPredators();
}

Vec3 Boids::steering(int k, const Radii& r) const {
    const Vec3 pi = sortedPos_[k];
    const Vec3 vi = sortedVel_[k];
    Vec3 sepSum{}, aliSum{}, cohSum{};
    int  neighbours = 0;

    // Two passes over the candidates. The first only filters: about two out
    // of three lie outside the perception sphere, at random, so a branch per
    // candidate would mispredict all the time -- the test is computed instead
    // and the survivors are appended to a small buffer. The second
    // accumulates the neighbours in their original order.
    std::array<int, 128> hits;
    int count = 0;
    auto accumulate = [&] {
        for (int h = 0; h < count; ++h) {
            const int   j     = hits[h];
            const Vec3  d     = pi - sortedPos_[j];
            const float dist2 = d.lengthSquared();
            cohSum += sortedPos_[j];
            aliSum += sortedVel_[j];
            if (dist2 < r.separation2) sepSum += d * (1.0f / dist2);   // weighted push apart
        }
        neighbours += count;
        count = 0;
    };
    grid_.forNeighborRanges(pi, [&](int begin, int end) {
        for (int j = begin; j < end; ++j) {
            const float dist2 = (pi - sortedPos_[j]).lengthSquared();
            hits[count] = j;
            count += (dist2 <= r.perception2) & (dist2 >= 1e-8f);   // the boid itself has dist2 == 0
            if (count == static_cast<int>(hits.size())) accumulate();
        }
    });
    accumulate();

    Vec3 acc{};
    if (neighbours > 0) {
        const float inv = 1.0f / static_cast<float>(neighbours);
        acc += limit(setMag(aliSum * inv, p_.maxSpeed) - vi, p_.maxForce) * p_.wAlignment;
        acc += limit(setMag(cohSum * inv - pi, p_.maxSpeed) - vi, p_.maxForce) * p_.wCohesion;
    }
    if (sepSum.lengthSquared() > 0)
        acc += limit(setMag(sepSum, p_.maxSpeed) - vi, p_.maxForce) * p_.wSeparation;

    // flee predators
    Vec3 flee{};
    for (const Vec3& predator : ppos_) {
        const Vec3  d  = pi - predator;
        const float d2 = d.lengthSquared();
        if (d2 < r.flee2 && d2 > 1e-6f) flee += d * (1.0f / d2);
    }
    if (flee.lengthSquared() > 0)
        acc += limit(setMag(flee, p_.maxSpeed) - vi, p_.maxForce) * p_.wFlee;

    // soft bounds: steer back when inside the margin band
    if (const Vec3 b = wallPush(pi, p_); b.lengthSquared() > 0)
        acc += setMag(b, p_.maxForce) * p_.wBounds;

    // optional obstacle avoidance
    if (p_.obstacle) {
        const Vec3  d    = pi - p_.obstacleCenter;
        const float dist = d.length();
        const float safe = p_.obstacleRadius + p_.margin;
        if (dist < safe && dist > 1e-4f) {
            const float strength = (safe - dist) / safe;
            acc += setMag(d, p_.maxForce) * (p_.wObstacle * strength);
        }
    }
    return acc;
}

// Predators: seek the nearest boid, bounce softly off walls. The grid still
// holds this step's cells; the boids have moved by at most one step since.
void Boids::stepPredators() {
    const int n = static_cast<int>(pos_.size());
    for (std::size_t k = 0; k < ppos_.size(); ++k) {
        const Vec3 pk = ppos_[k];
        int   best   = -1;
        float bestD2 = 1e30f;
        auto consider = [&](int j) {
            const float d2 = (pos_[j] - pk).lengthSquared();
            if (d2 < bestD2) { bestD2 = d2; best = j; }
        };
        grid_.forNeighbors(pk, consider);           // the neighbourhood first,
        if (best < 0)
            for (int j = 0; j < n; j += 7) consider(j);   // a coarse scan when it is empty

        Vec3 acc{};
        if (best >= 0)
            acc += limit(setMag(pos_[best] - pk, p_.predatorMaxSpeed) - pvel_[k], p_.predatorMaxForce);
        if (const Vec3 b = wallPush(pk, p_); b.lengthSquared() > 0)
            acc += setMag(b, p_.predatorMaxForce) * 2.5f;

        Vec3 v = pvel_[k] + acc;
        const float sp = v.length();
        if (sp > p_.predatorMaxSpeed) v = v * (p_.predatorMaxSpeed / sp);
        pvel_[k] = v;
        ppos_[k] += v * p_.dt;
    }
}
