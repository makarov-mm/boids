#pragma once
#include "vec3.hpp"
#include "hashgrid.hpp"
#include <cstdint>
#include <vector>

struct BoidsParams {
    int boids = 4000;
    int predators = 2;
    Vec3 boundsMin{0, 0, 0};
    Vec3 boundsMax{140, 90, 140};

    float perception = 9.5f;       // neighbour radius for align/cohesion
    float separationDist = 4.0f;   // closer than this -> push apart
    float maxSpeed = 16.0f;
    float minSpeed = 5.0f;
    float maxForce = 0.8f;         // steering clamp per rule

    float wSeparation = 1.7f;
    float wAlignment = 1.4f;
    float wCohesion = 1.15f;
    float wBounds = 2.5f;
    float wFlee = 4.0f;

    // predators
    float predatorMaxSpeed = 17.0f;
    float predatorMaxForce = 0.7f;
    float fleeRadius = 14.0f;       // boids flee predators within this range

    // optional spherical obstacle
    bool obstacle = false;
    Vec3 obstacleCenter{70, 45, 70};
    float obstacleRadius = 14.0f;
    float wObstacle = 3.5f;

    float dt = 0.1f;
    float margin = 12.0f;           // wall turn-around band
    uint64_t seed = 1234567;
};

class Boids {
public:
    explicit Boids(const BoidsParams& p);
    void step();

    const std::vector<Vec3>& positions() const { return pos_; }
    const std::vector<Vec3>& velocities() const { return vel_; }
    const std::vector<Vec3>& predatorPos() const { return ppos_; }
    const std::vector<Vec3>& predatorVel() const { return pvel_; }
    const BoidsParams& params() const { return p_; }
    std::size_t count() const { return pos_.size(); }
    void setThreads(int t) { threads_ = t < 1 ? 1 : t; }

private:
    BoidsParams p_;
    HashGrid grid_;
    int threads_ = 1;
    std::vector<Vec3> pos_, vel_, acc_;
    std::vector<Vec3> ppos_, pvel_;
};
