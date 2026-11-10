#pragma once
#include <algorithm>
#include <cmath>
#include <span>
#include <vector>

#include "vec3.hpp"

// Uniform spatial hash over an axis-aligned box. Cell size = neighbour radius,
// so a query visits its own cell plus the 26 neighbours. Built with a stable
// counting sort: O(N), no per-cell allocations, and every cell lists its
// points in increasing index order.
class HashGrid {
public:
    // Points outside [mn, mx] are clamped into the border cells.
    void build(std::span<const Vec3> points, float cellSize, const Vec3& mn, const Vec3& mx) {
        inv_ = 1.0f / cellSize;
        mn_  = mn;
        nx_  = std::max(1, static_cast<int>(std::ceil((mx.x - mn.x) * inv_)));
        ny_  = std::max(1, static_cast<int>(std::ceil((mx.y - mn.y) * inv_)));
        nz_  = std::max(1, static_cast<int>(std::ceil((mx.z - mn.z) * inv_)));
        const int cells = nx_ * ny_ * nz_;
        const int n     = static_cast<int>(points.size());

        cellOf_.resize(n);
        start_.assign(cells + 1, 0);
        for (int i = 0; i < n; ++i) {
            const int c = cellIndex(points[i]);
            cellOf_[i] = c;
            ++start_[c + 1];
        }
        for (int c = 0; c < cells; ++c) start_[c + 1] += start_[c];

        cursor_.assign(start_.begin(), start_.end() - 1);
        order_.resize(n);
        for (int i = 0; i < n; ++i) order_[cursor_[cellOf_[i]]++] = i;
    }

    // Point indices sorted by cell. Each cell's slice is contiguous, so per-
    // point data gathered in this order is read sequentially by the queries.
    [[nodiscard]] std::span<const int> order() const noexcept { return order_; }

    // Calls fn(begin, end) for the slices of order() that cover the 3x3x3
    // block of cells around `pos`: one slice per row of up to three cells
    // (adjacent cells in x are adjacent in order()), in (z, y, x) order.
    template <class Fn>
    void forNeighborRanges(const Vec3& pos, Fn&& fn) const {
        const int cx = cellCoord(pos.x, mn_.x, nx_);
        const int cy = cellCoord(pos.y, mn_.y, ny_);
        const int cz = cellCoord(pos.z, mn_.z, nz_);
        const int x0 = std::max(cx - 1, 0), x1 = std::min(cx + 1, nx_ - 1);
        for (int z = std::max(cz - 1, 0); z <= std::min(cz + 1, nz_ - 1); ++z) {
            for (int y = std::max(cy - 1, 0); y <= std::min(cy + 1, ny_ - 1); ++y) {
                const int row = (z * ny_ + y) * nx_;
                fn(start_[row + x0], start_[row + x1 + 1]);
            }
        }
    }

    // Calls fn(index) for every candidate point around `pos`.
    template <class Fn>
    void forNeighbors(const Vec3& pos, Fn&& fn) const {
        forNeighborRanges(pos, [&](int begin, int end) {
            for (int k = begin; k < end; ++k) fn(order_[k]);
        });
    }

private:
    [[nodiscard]] int cellCoord(float v, float lo, int n) const noexcept {
        return std::clamp(static_cast<int>(std::floor((v - lo) * inv_)), 0, n - 1);
    }
    [[nodiscard]] int cellIndex(const Vec3& p) const noexcept {
        return (cellCoord(p.z, mn_.z, nz_) * ny_ + cellCoord(p.y, mn_.y, ny_)) * nx_ + cellCoord(p.x, mn_.x, nx_);
    }

    float inv_ = 1;
    Vec3  mn_{};
    int   nx_ = 1, ny_ = 1, nz_ = 1;
    std::vector<int> cellOf_, start_, cursor_, order_;
};
