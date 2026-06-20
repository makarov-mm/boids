#pragma once
#include "vec3.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

// Uniform spatial hash over an axis-aligned box. Cell size = neighbour radius,
// so each query visits its own cell plus the 26 neighbours. Built with a
// counting sort: O(N), cache-friendly, no per-cell allocations.
class HashGrid {
public:
    void build(const std::vector<Vec3>& p, float cellSize, const Vec3& mn, const Vec3& mx) {
        cell_ = cellSize;
        inv_ = 1.0f / cellSize;
        mn_ = mn;
        nx_ = std::max(1, (int)std::ceil((mx.x - mn.x) * inv_));
        ny_ = std::max(1, (int)std::ceil((mx.y - mn.y) * inv_));
        nz_ = std::max(1, (int)std::ceil((mx.z - mn.z) * inv_));
        const int nc = nx_ * ny_ * nz_;
        const int n = (int)p.size();

        cellOf_.resize(n);
        start_.assign(nc + 1, 0);
        for (int i = 0; i < n; ++i) {
            int c = cellIndex(p[i]);
            cellOf_[i] = c;
            ++start_[c + 1];
        }
        for (int c = 0; c < nc; ++c) start_[c + 1] += start_[c];
        order_.resize(n);
        std::vector<int> cur(start_.begin(), start_.end() - 1);
        for (int i = 0; i < n; ++i) order_[cur[cellOf_[i]]++] = i;
    }

    // Visit every candidate index in the 3x3x3 block of cells around `pos`.
    template <class Fn>
    void forNeighbors(const Vec3& pos, Fn&& fn) const {
        int cx = clampi((int)std::floor((pos.x - mn_.x) * inv_), nx_);
        int cy = clampi((int)std::floor((pos.y - mn_.y) * inv_), ny_);
        int cz = clampi((int)std::floor((pos.z - mn_.z) * inv_), nz_);
        for (int dz = -1; dz <= 1; ++dz) {
            int z = cz + dz; if (z < 0 || z >= nz_) continue;
            for (int dy = -1; dy <= 1; ++dy) {
                int y = cy + dy; if (y < 0 || y >= ny_) continue;
                for (int dx = -1; dx <= 1; ++dx) {
                    int x = cx + dx; if (x < 0 || x >= nx_) continue;
                    int c = (z * ny_ + y) * nx_ + x;
                    for (int k = start_[c]; k < start_[c + 1]; ++k) fn(order_[k]);
                }
            }
        }
    }

private:
    static int clampi(int v, int n) { return v < 0 ? 0 : (v >= n ? n - 1 : v); }
    int cellIndex(const Vec3& p) const {
        int x = clampi((int)std::floor((p.x - mn_.x) * inv_), nx_);
        int y = clampi((int)std::floor((p.y - mn_.y) * inv_), ny_);
        int z = clampi((int)std::floor((p.z - mn_.z) * inv_), nz_);
        return (z * ny_ + y) * nx_ + x;
    }

    float cell_ = 1, inv_ = 1;
    Vec3 mn_{};
    int nx_ = 1, ny_ = 1, nz_ = 1;
    std::vector<int> cellOf_, start_, order_;
};
