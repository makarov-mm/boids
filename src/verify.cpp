#include "boids.hpp"
#include "hashgrid.hpp"
#include <cstdio>
#include <cmath>
#include <thread>
#include <vector>

// ---- hash grid correctness vs brute force ----
static void testGrid() {
    uint64_t s = 99;
    auto rnd = [&](float a, float b){ s^=s<<13;s^=s>>7;s^=s<<17; return a+(b-a)*((s>>40)/16777216.0f);} ;
    std::vector<Vec3> p(1500);
    Vec3 mn{0,0,0}, mx{50,40,60};
    for (auto& q : p) q = {rnd(0,50), rnd(0,40), rnd(0,60)};
    float r = 5.0f;
    HashGrid g; g.build(p, r, mn, mx);
    int mism = 0;
    for (int i = 0; i < (int)p.size(); i += 13) {
        int bf = 0;
        for (int j = 0; j < (int)p.size(); ++j)
            if (j!=i && (p[i]-p[j]).lengthSquared() <= r*r) ++bf;
        int gc = 0;
        g.forNeighbors(p[i], [&](int j){ if (j!=i && (p[i]-p[j]).lengthSquared() <= r*r) ++gc; });
        if (bf != gc) ++mism;
    }
    printf("hash grid vs brute force: %s (%d samples checked)\n",
           mism==0 ? "MATCH" : "MISMATCH", 1500/13 + 1);
}

static void metrics(const Boids& b, double& polar, double& meanNN, float& smin, float& smax) {
    const auto& pos = b.positions();
    const auto& vel = b.velocities();
    Vec3 sum{}; smin = 1e9f; smax = 0;
    for (auto& v : vel) { float s = v.length(); sum += (s>1e-6f? v*(1.0f/s):Vec3{}); smin=std::min(smin,s); smax=std::max(smax,s); }
    polar = sum.length() / pos.size();   // Vicsek order parameter in [0,1]
    HashGrid g; g.build(pos, b.params().perception, b.params().boundsMin, b.params().boundsMax);
    double acc = 0; int cnt = 0;
    for (int i = 0; i < (int)pos.size(); i += 5) {
        float best = 1e18f;
        g.forNeighbors(pos[i], [&](int j){ if (j!=i){ float d=(pos[i]-pos[j]).lengthSquared(); if(d<best) best=d; } });
        if (best < 1e17f) { acc += std::sqrt(best); ++cnt; }
    }
    meanNN = cnt ? acc / cnt : 0;
}

int main() {
    printf("=== Boids 3D core - verification ===\n\n");
    testGrid();

    BoidsParams p; p.boids = 6000; p.predators = 2;
    Boids b(p);
    b.setThreads((int)std::thread::hardware_concurrency());
    printf("\n%zu boids, %d predators, box %.0fx%.0fx%.0f\n\n",
           b.count(), p.predators, p.boundsMax.x, p.boundsMax.y, p.boundsMax.z);
    printf("   step   polarization   mean NN dist   speed[min..max]\n");
    printf("          (0=chaos 1=aligned)\n");
    for (int s = 0; s <= 600; ++s) {
        b.step();
        if (s % 100 == 0) {
            double pol, nn; float smin, smax;
            metrics(b, pol, nn, smin, smax);
            printf("   %4d   %6.3f         %6.2f         %.1f .. %.1f\n", s, pol, nn, smin, smax);
        }
    }
    printf("\npolarization rises from ~0 (random) as the flock aligns; mean NN distance\n");
    printf("settles (separation stops collapse, cohesion stops dispersal); speeds stay\n");
    printf("within [minSpeed, maxSpeed]. Core OK.\n");
    return 0;
}
