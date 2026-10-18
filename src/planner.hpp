// Pure C++ path planner (no Geode types) so it can be unit-tested off-device.
#pragma once
#include <algorithm>
#include <cmath>
#include <vector>

namespace wa {

constexpr int   N    = 16;    // bands planned ahead
constexpr int   ROWS = 90;    // vertical cells
constexpr float CELL = 10.f;  // units per cell
constexpr float INF  = 1e9f;

struct Obs {            // one obstacle (solid, slope or hazard) as an AABB
    float x0, y0, x1, y1;
    bool  tri = false;  // right-triangle slope?
    int   sx = 1, sy = -1; // corner (of the AABB) that holds the right angle
};

struct Plan {
    bool  ok = false;     // a safe path exists
    bool  tight = false;  // closed / tight corridor ahead
    float targetY = 0.f;  // where to steer next
    float minGap = INF;   // smallest real gap seen in the near bands
};

inline Plan makePlan(const std::vector<Obs>& obs, float px, float py, float step,
                     float slope, float margin, float floorY, float ceilY,
                     float gapLimit, int lookBands = 8) {
    static bool           blocked[N][ROWS];
    static unsigned char  clr[N][ROWS];
    static float          cost[N + 1][ROWS];
    static short          par[N + 1][ROWS];

    Plan plan;
    const float X0 = px - 8.f;
    const float ry0 = py - ROWS * CELL * 0.5f;

    for (auto& b : blocked) std::fill(std::begin(b), std::end(b), false);

    // level floor / ceiling
    if (ceilY - floorY > 150.f && ceilY - floorY < 2500.f) {
        for (int c = 0; c < ROWS; ++c) {
            float y = ry0 + (c + 0.5f) * CELL;
            if (y < floorY + margin || y > ceilY - margin)
                for (int b = 0; b < N; ++b) blocked[b][c] = true;
        }
    }

    for (const Obs& o : obs) {
        float x0 = o.x0 - margin, x1 = o.x1 + margin;
        float y0 = o.y0 - margin, y1 = o.y1 + margin;
        int b0 = std::max(0, (int)std::floor((x0 - X0) / step));
        int b1 = std::min(N - 1, (int)std::floor((x1 - X0) / step));
        int c0 = std::max(0, (int)std::floor((y0 - ry0) / CELL));
        int c1 = std::min(ROWS - 1, (int)std::floor((y1 - ry0) / CELL));
        for (int b = b0; b <= b1; ++b) {
            for (int c = c0; c <= c1; ++c) {
                if (o.tri) {
                    float bs = X0 + b * step, be = bs + step;
                    float cx = (std::max(x0, bs) + std::min(x1, be)) * 0.5f;
                    float cy = ry0 + (c + 0.5f) * CELL;
                    float u = 2.f * (cx - (x0 + x1) * 0.5f) / std::max(1.f, x1 - x0);
                    float v = 2.f * (cy - (y0 + y1) * 0.5f) / std::max(1.f, y1 - y0);
                    if (u * o.sx + v * o.sy < 0.f) continue; // empty half
                }
                blocked[b][c] = true;
            }
        }
    }

    int cs = std::clamp((int)std::lround((py - ry0) / CELL - 0.5f), 0, ROWS - 1);
    for (int c = std::max(0, cs - 2); c <= std::min(ROWS - 1, cs + 2); ++c) blocked[0][c] = false;

    // gap measurement around the player in the near bands
    int look = std::min(N, lookBands);
    for (int b = 0; b < look; ++b) {
        int c = cs;
        if (blocked[b][c]) { // nearest free cell
            int best = -1;
            for (int d = 1; d < ROWS && best < 0; ++d) {
                if (c + d < ROWS && !blocked[b][c + d]) best = c + d;
                else if (c - d >= 0 && !blocked[b][c - d]) best = c - d;
            }
            if (best < 0) { plan.minGap = 0.f; continue; }
            c = best;
        }
        int lo = c, hi = c;
        while (lo > 0 && !blocked[b][lo - 1]) --lo;
        while (hi < ROWS - 1 && !blocked[b][hi + 1]) ++hi;
        if (lo == 0 || hi == ROWS - 1) continue; // open on one side -> not enclosed
        float gap = (hi - lo + 1) * CELL + 2.f * margin;
        plan.minGap = std::min(plan.minGap, gap);
    }
    plan.tight = plan.minGap < gapLimit;

    // clearance (distance to nearest blocked cell, capped)
    for (int b = 0; b < N; ++b) {
        int d = 99;
        for (int c = 0; c < ROWS; ++c) { d = blocked[b][c] ? 0 : d + 1; clr[b][c] = (unsigned char)std::min(d, 6); }
        d = 99;
        for (int c = ROWS - 1; c >= 0; --c) { d = blocked[b][c] ? 0 : d + 1; clr[b][c] = (unsigned char)std::min<int>(clr[b][c], std::min(d, 6)); }
    }

    // DP over bands
    for (auto& r : cost) std::fill(std::begin(r), std::end(r), INF);
    cost[0][cs] = 0.f;
    int maxD = std::max(1, (int)std::lround(step * slope / CELL));
    for (int b = 0; b < N; ++b) {
        for (int c = 0; c < ROWS; ++c) {
            float base = cost[b][c];
            if (base >= INF) continue;
            auto relax = [&](int c2, int d) {
                float nc = base + 0.1f * std::abs(d) + (6 - clr[b][c2]) * 0.5f;
                if (nc < cost[b + 1][c2]) { cost[b + 1][c2] = nc; par[b + 1][c2] = (short)c; }
            };
            if (!blocked[b][c]) relax(c, 0);
            for (int d = 1; d <= maxD; ++d) { int c2 = c + d; if (c2 >= ROWS || blocked[b][c2]) break; relax(c2, d); }
            for (int d = 1; d <= maxD; ++d) { int c2 = c - d; if (c2 < 0 || blocked[b][c2]) break; relax(c2, d); }
        }
    }
    int best = -1; float bc = INF;
    for (int c = 0; c < ROWS; ++c) if (cost[N][c] < bc) { bc = cost[N][c]; best = c; }
    if (best < 0) return plan;

    int c = best;
    for (int b = N; b > 1; --b) c = par[b][c];   // walk back to band 1
    plan.targetY = ry0 + (c + 0.5f) * CELL;
    plan.ok = true;
    return plan;
}

} // namespace wa
