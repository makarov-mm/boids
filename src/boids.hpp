#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>

#include "hashgrid.hpp"
#include "thread_pool.hpp"
#include "vec3.hpp"

struct BoidsParams {
    int  boids     = 4000;
    int  predators = 2;
    Vec3 boundsMin{ 0, 0, 0 };
    Vec3 boundsMax{ 140, 90, 140 };

    float perception     = 9.5f;    // neighbour radius for align/cohesion
    float separationDist = 4.0f;    // closer than this -> push apart
    float maxSpeed       = 16.0f;
    float minSpeed       = 5.0f;
    float maxForce       = 0.8f;    // steering clamp per rule

    float wSeparation = 1.7f;
    float wAlignment  = 1.4f;
    float wCohesion   = 1.15f;
    float wBounds     = 2.5f;
    float wFlee       = 4.0f;

    // predators
    float predatorMaxSpeed = 17.0f;
    float predatorMaxForce = 0.7f;
    float fleeRadius       = 14.0f;   // boids flee predators within this range

    // optional spherical obstacle
    bool  obstacle = false;
    Vec3  obstacleCenter{ 70, 45, 70 };
    float obstacleRadius = 14.0f;
    float wObstacle      = 3.5f;

    float         dt      = 0.1f;
    float         margin  = 12.0f;    // wall turn-around band
    std::uint64_t seed    = 1234567;
    unsigned      threads = 0;        // simulation threads, 0 = one per hardware thread
};

class Boids {
public:
    explicit Boids(const BoidsParams& p);

    void reset();   // a fresh flock from the seed: the same start every time
    void step();

    // Live changes, no reset: the flock reacts from its current state.
    void setObstacle(bool on) noexcept { p_.obstacle = on; }
    void setPredators(int count);       // new predators appear at random positions

    [[nodiscard]] const std::vector<Vec3>& positions()   const noexcept { return pos_; }
    [[nodiscard]] const std::vector<Vec3>& velocities()  const noexcept { return vel_; }
    [[nodiscard]] const std::vector<Vec3>& predatorPos() const noexcept { return ppos_; }
    [[nodiscard]] const std::vector<Vec3>& predatorVel() const noexcept { return pvel_; }
    [[nodiscard]] const BoidsParams&       params()      const noexcept { return p_; }
    [[nodiscard]] std::size_t              count()       const noexcept { return pos_.size(); }

private:
    struct Radii { float perception2, separation2, flee2; };

    [[nodiscard]] Vec3 steering(int k, const Radii& r) const;   // acceleration of the boid at grid slot k
    void stepPredators();
    void spawnPredator();

    BoidsParams   p_;
    std::uint64_t rng_ = 1;
    ThreadPool    pool_;
    HashGrid      grid_;

    std::vector<Vec3> pos_, vel_, acc_;
    std::vector<Vec3> sortedPos_, sortedVel_;   // pos_ / vel_ gathered in grid order
    std::vector<Vec3> ppos_, pvel_;
};
