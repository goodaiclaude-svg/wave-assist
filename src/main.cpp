#include <Geode/Geode.hpp>
#include <Geode/modify/GJBaseGameLayer.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <type_traits>
#include <vector>

using namespace geode::prelude;

namespace wa {

struct Box { float x1, y1, x2, y2; };

inline bool sb(char const* k) { return Mod::get()->getSettingValue<bool>(k); }
inline double sf(char const* k) { return Mod::get()->getSettingValue<double>(k); }
inline int si(char const* k) { return static_cast<int>(Mod::get()->getSettingValue<int64_t>(k)); }

// ---------- obstacle gathering ----------
inline void pushObj(GameObject* o, std::vector<Box>& out, bool flip, bool skipSlope) {
    if (!o) return;
    if (skipSlope && o->m_objectType == GameObjectType::Slope) return;
    auto r = o->getObjectRect();
    if (r.size.width <= 0.f || r.size.height <= 0.f) return;
    float y1 = r.getMinY(), y2 = r.getMaxY();
    if (flip) { float t = y1; y1 = -y2; y2 = -t; }
    out.push_back({r.getMinX(), y1, r.getMaxX(), y2});
}

template <class C>
void takeAll(C&& c, std::vector<Box>& out, bool flip = false, bool skipSlope = false) {
    using T = std::remove_cv_t<std::remove_reference_t<C>>;
    if constexpr (std::is_pointer_v<T>) {
        using U = std::remove_cv_t<std::remove_pointer_t<T>>;
        if (!c) return;
        if constexpr (std::is_same_v<U, CCArray>) {
            for (auto* o : CCArrayExt<GameObject*>(c)) pushObj(o, out, flip, skipSlope);
        } else {
            for (auto* o : *c) pushObj(o, out, flip, skipSlope);
        }
    } else {
        for (auto* o : c) pushObj(o, out, flip, skipSlope);
    }
}

inline void gather(GJBaseGameLayer* gl, float x1, float y1, float x2, float y2, std::vector<Box>& out) {
    CCRect q(x1, y1, x2 - x1, y2 - y1);
    takeAll(gl->staticObjectsInRect(q, false), out);
    takeAll(gl->damagingObjectsInRect(q, false), out);
}

inline void updKy(double& ky, bool& init, int& rej, double smp) {
    if (!(smp > 0)) return;
    if (!init) { ky = smp; init = true; rej = 0; }
    else if (smp > ky * 0.25 && smp < ky * 4) { ky = ky * 0.9 + smp * 0.1; rej = 0; }
    else if (++rej > 20) { ky = smp; rej = 0; }
}

// ---------- gap scan ----------
struct GapInfo { float minGap; float center; };

inline GapInfo scanGap(std::vector<Box> const& obs, float px, float py, float xEnd, float lo, float hi) {
    GapInfo g{1e9f, py};
    auto at = [&](float xa, float xb, float& up, float& dn) {
        up = hi; dn = lo;
        for (auto const& b : obs) {
            if (b.x2 < xa || b.x1 > xb) continue;
            if (b.y1 >= py) up = std::min(up, b.y1);
            else if (b.y2 <= py) dn = std::max(dn, b.y2);
        }
    };
    float up, dn;
    at(px, px + 70.f, up, dn);
    g.center = (up + dn) * 0.5f;
    for (float x = px; x <= xEnd; x += 10.f) {
        at(x, x, up, dn);
        g.minGap = std::min(g.minGap, up - dn);
    }
    return g;
}

// ---------- WAVE planner: survival depth + smooth min-cost path ----------
// index 0 = world-down first step, 1 = world-up first step
struct WavePlan { int dT[2]; float cost[2]; int N; };

inline WavePlan planWave(float x0, float y0, float lo, float hi, float slope, float dx, int N,
                         std::vector<Box> const& obs, float half, float mT, float mL,
                         int d0, float Cs, float wc) {
    WavePlan P{};
    P.N = N;
    P.cost[0] = P.cost[1] = 1e9f;
    float dy = slope * dx;
    int rows = 2 * N + 1;
    int jmin = std::max(-N, static_cast<int>(std::ceil((lo + half - y0) / dy)));
    int jmax = std::min(N, static_cast<int>(std::floor((hi - half - y0) / dy)));
    if (jmin > 0) jmin = 0;
    if (jmax < 0) jmax = 0;

    size_t sz = static_cast<size_t>(N + 1) * rows;
    std::vector<uint8_t> blkT(sz, 0), blkL(sz, 0);
    for (int pass = 0; pass < 2; ++pass) {
        auto& blk = pass == 0 ? blkT : blkL;
        float e = half + (pass == 0 ? mT : mL);
        for (auto const& bx : obs) {
            int n1 = std::max(1, static_cast<int>(std::ceil((bx.x1 - e - x0) / dx)));
            int n2 = std::min(N, static_cast<int>(std::floor((bx.x2 + e - x0) / dx)));
            if (n1 > n2) continue;
            int j1 = std::max(jmin, static_cast<int>(std::ceil((bx.y1 - e - y0) / dy)));
            int j2 = std::min(jmax, static_cast<int>(std::floor((bx.y2 + e - y0) / dy)));
            if (j1 > j2) continue;
            for (int n = n1; n <= n2; ++n)
                for (int j = j1; j <= j2; ++j)
                    blk[static_cast<size_t>(n) * rows + (j + N)] = 1;
        }
    }

    // survival depth (tight margin)
    {
        std::vector<int16_t> a(rows, 0), b(rows, 0);
        for (int n = N; n >= 1; --n) {
            for (int j = jmin; j <= jmax; ++j) {
                if (blkT[static_cast<size_t>(n) * rows + (j + N)]) { b[j + N] = 0; continue; }
                int ju = std::min(j + 1, jmax), jd = std::max(j - 1, jmin);
                b[j + N] = static_cast<int16_t>(1 + std::max(a[ju + N], a[jd + N]));
            }
            std::swap(a, b);
        }
        P.dT[1] = a[std::min(1, jmax) + N];
        P.dT[0] = a[std::max(-1, jmin) + N];
    }

    // centre line of the corridor we are following
    std::vector<float> cj(N + 1, 0.f);
    {
        float prev = 0.f;
        for (int n = 1; n <= N; ++n) {
            float best = prev, bestd = 1e9f;
            bool any = false;
            int j = jmin;
            while (j <= jmax) {
                if (blkT[static_cast<size_t>(n) * rows + (j + N)]) { ++j; continue; }
                int a = j;
                while (j <= jmax && !blkT[static_cast<size_t>(n) * rows + (j + N)]) ++j;
                int b = j - 1;
                float mid = (a + b) * 0.5f;
                float d = prev < a ? a - prev : (prev > b ? prev - b : 0.f);
                if (d < bestd || (d == bestd && std::fabs(mid - prev) < std::fabs(best - prev))) {
                    bestd = d; best = mid; any = true;
                }
            }
            cj[n] = any ? best : prev;
            prev = cj[n];
        }
    }

    // min-cost DP: state (n, row, dir). cost = switches + closeness to walls + off-centre
    const float INF = 1e9f;
    std::vector<float> A(static_cast<size_t>(2) * rows, 0.f), B(static_cast<size_t>(2) * rows, 0.f);
    for (int n = N; n >= 1; --n) {
        for (int j = jmin; j <= jmax; ++j) {
            size_t cell = static_cast<size_t>(n) * rows + (j + N);
            bool bt = blkT[cell] != 0, bl = blkL[cell] != 0;
            float pen = (bl ? 4.f : 0.f) + wc * std::fabs(static_cast<float>(j) - cj[n]);
            for (int d = 0; d < 2; ++d) {
                size_t idx = static_cast<size_t>(d) * rows + (j + N);
                if (bt) { B[idx] = INF; continue; }
                float best = INF;
                for (int d2 = 0; d2 < 2; ++d2) {
                    int j2 = std::clamp(j + (d2 ? 1 : -1), jmin, jmax);
                    float c = A[static_cast<size_t>(d2) * rows + (j2 + N)] + (d2 != d ? Cs : 0.f);
                    best = std::min(best, c);
                }
                B[idx] = best >= INF ? INF : pen + best;
            }
        }
        std::swap(A, B);
    }
    for (int a = 0; a < 2; ++a) {
        int j1 = std::clamp(a ? 1 : -1, jmin, jmax);
        float c = A[static_cast<size_t>(a) * rows + (j1 + N)];
        P.cost[a] = c >= INF ? INF : c + (a != d0 ? Cs : 0.f);
    }
    return P;
}

// ---------- SHIP model + beam planner ----------
struct Acc { double v = 0; int n = 0; };
struct ShipModel {
    Acc acc[2][2][2];          // [upsideDown][hold][velocity>=0]
    double ky = 0; bool kyInit = false; int kyRej = 0;
    double vmaxUp = 0, vmaxDn = 0;
};

struct Grid {
    float x0 = 0, y0 = 0, cs = 5.f; int w = 0, h = 0;
    std::vector<uint8_t> b;
    bool blocked(float x, float y) const {
        int ix = static_cast<int>(std::floor((x - x0) / cs));
        int iy = static_cast<int>(std::floor((y - y0) / cs));
        if (ix < 0 || ix >= w || iy < 0 || iy >= h) return false;
        return b[static_cast<size_t>(iy) * w + ix] != 0;
    }
    float clearance(float x, float y) const {
        int c = 0, d = 0;
        for (int k = 1; k <= 8; ++k) { if (blocked(x, y + k * cs)) break; ++c; }
        for (int k = 1; k <= 8; ++k) { if (blocked(x, y - k * cs)) break; ++d; }
        return static_cast<float>(std::min(c, d));
    }
};

inline Grid buildGrid(std::vector<Box> const& obs, float gx0, float gx1, float gy0, float gy1, float e) {
    Grid g;
    g.x0 = gx0; g.y0 = gy0;
    g.w = std::max(1, static_cast<int>(std::ceil((gx1 - gx0) / g.cs)));
    g.h = std::max(1, static_cast<int>(std::ceil((gy1 - gy0) / g.cs)));
    g.b.assign(static_cast<size_t>(g.w) * g.h, 0);
    for (auto const& bx : obs) {
        int ix1 = std::max(0, static_cast<int>(std::floor((bx.x1 - e - gx0) / g.cs)));
        int ix2 = std::min(g.w - 1, static_cast<int>(std::floor((bx.x2 + e - gx0) / g.cs)));
        int iy1 = std::max(0, static_cast<int>(std::floor((bx.y1 - e - gy0) / g.cs)));
        int iy2 = std::min(g.h - 1, static_cast<int>(std::floor((bx.y2 + e - gy0) / g.cs)));
        for (int iy = iy1; iy <= iy2; ++iy)
            for (int ix = ix1; ix <= ix2; ++ix)
                g.b[static_cast<size_t>(iy) * g.w + ix] = 1;
    }
    return g;
}

inline double accOf(ShipModel const& m, int ud, int hold, double v) {
    Acc const* p = &m.acc[ud][hold][v >= 0];
    if (p->n < 2) p = &m.acc[ud][hold][v < 0];
    return p->n >= 2 ? p->v : 0.0;
}

struct ShipPlan { int depth[2]; float clear[2]; int full; };
struct BS { double y, v; int first; float score; };

inline ShipPlan planShip(ShipModel const& m, int ud, double y0, double v0, float x0, double dxTick,
                         int ticksPer, int D, Grid const& g, float yLoC, float yHiC) {
    ShipPlan P{};
    P.full = D * ticksPer;
    std::vector<BS> cur, nxt;
    cur.push_back({y0, v0, -1, 0.f});
    float x = x0;
    for (int k = 0; k < D && !cur.empty(); ++k) {
        nxt.clear();
        for (auto const& s : cur) {
            for (int a = 0; a < 2; ++a) {
                BS t = s;
                if (k == 0) t.first = a;
                bool dead = false;
                float xx = x;
                for (int tk = 0; tk < ticksPer; ++tk) {
                    t.v += accOf(m, ud, a, t.v);
                    if (m.vmaxUp > 0 && t.v > m.vmaxUp) t.v = m.vmaxUp;
                    if (m.vmaxDn > 0 && t.v < -m.vmaxDn) t.v = -m.vmaxDn;
                    t.y += m.ky * t.v;
                    if (t.y > yHiC) { t.y = yHiC; if (t.v > 0) t.v = 0; }
                    if (t.y < yLoC) { t.y = yLoC; if (t.v < 0) t.v = 0; }
                    xx += static_cast<float>(dxTick);
                    if (g.blocked(xx, static_cast<float>(t.y))) {
                        P.depth[t.first] = std::max(P.depth[t.first], k * ticksPer + tk);
                        dead = true;
                        break;
                    }
                }
                if (!dead) {
                    t.score = g.clearance(xx, static_cast<float>(t.y));
                    nxt.push_back(t);
                }
            }
        }
        x += static_cast<float>(dxTick * ticksPer);
        std::vector<BS> keep;
        for (int fa = 0; fa < 2; ++fa) {
            std::vector<BS> part;
            for (auto const& s : nxt) if (s.first == fa) part.push_back(s);
            std::sort(part.begin(), part.end(), [](BS const& a, BS const& b) { return a.score > b.score; });
            if (part.size() > 8) part.resize(8);
            keep.insert(keep.end(), part.begin(), part.end());
        }
        cur.swap(keep);
    }
    for (auto const& s : cur) {
        P.depth[s.first] = P.full;
        P.clear[s.first] = std::max(P.clear[s.first], s.score);
    }
    return P;
}

// ---------- CUBE orb reader ----------
// All coordinates here are "relative": y is flipped when gravity is upside-down, so up = away from the floor.
// returns true if the cube survives (lands, or lives through the horizon)
inline bool simCube(double x, double y, double v, double dxTick, double ky, double g, double vmin,
                    int ticks, float hS, float hH, std::vector<Box> const& solids,
                    std::vector<Box> const& haz, double floorR) {
    for (int t = 0; t < ticks; ++t) {
        v += g;
        if (vmin < 0 && v < vmin) v = vmin;
        double yPrev = y;
        y += ky * v;
        x += dxTick;
        if (y <= floorR + hS) return true;                       // landed on the ground
        for (auto const& b : haz) {
            if (x + hH > b.x1 && x - hH < b.x2 && y + hH > b.y1 && y - hH < b.y2) return false;
        }
        double tol = std::max(5.0, std::fabs(ky * v) + 2.0);
        for (auto const& b : solids) {
            if (x + hS > b.x1 && x - hS < b.x2 && y + hS > b.y1 && y - hS < b.y2) {
                if (v <= 0 && yPrev - hS >= b.y2 - tol) return true;  // landed on top of a block
                return false;                                          // hit a side / underside
            }
        }
    }
    return true;
}

struct CubeEnv {
    std::vector<Box> solids, haz;
    RingObject* ring = nullptr;
};

inline RingObject* findRing(PlayerObject* p, std::vector<RingObject*> const& used) {
    if (!p->m_touchingRings || p->m_touchingRings->count() == 0) return nullptr;
    for (auto* o : CCArrayExt<GameObject*>(p->m_touchingRings)) {
        auto* r = typeinfo_cast<RingObject*>(o);
        if (!r) continue;
        if (r->m_objectID == 1704 || r->m_objectID == 1751) continue; // dash orbs: left to you
        if (std::find(used.begin(), used.end(), r) != used.end()) continue;
        return r;
    }
    return nullptr;
}

// ---------- per-player state ----------
struct PS {
    bool valid = false;
    float px = 0, py = 0; double pv = 0;
    int mode = -1; bool mini = false;
    bool prevH = false; int effStable = 0;
    float dxEma = 0, slope = 1.f; bool holdUp = true;
    float engagedUntil = -1.f;
    std::vector<Box> obs; float cPx = -1e9f; float cL = 0; int cAge = 999;
    ShipModel ship; float lastDt = 0; bool tog = false; int lastRet = -1; int lastAct = -1;
    // cube / orbs
    int touchTicks = 0, holdTicks = 0;
    bool prevUser = false, prevGround = false;
    std::vector<RingObject*> used;
    double pvr = 0;
    double g = 0; int gN = 0;
    double J = 0; int jN = 0;
    double cky = 0; bool ckyInit = false; int ckyRej = 0;
    double vminR = 0;
    std::map<int, double> kick;      // learned orb kick by object id (relative velocity)
    int pendKey = -1, pendUd = 0;
    CubeEnv env;
};

} // namespace wa

class $modify(WAGameLayer, GJBaseGameLayer) {
    struct Fields {
        wa::PS ps[2];
        bool userHeld[2] = {false, false};
        bool eff[2] = {false, false};
        bool owned[2] = {false, false};
        bool injecting = false;
        float clock = 0.f;
        float marginBoost = 0.f;     // grows after crashes while assisting
    };

    void waInject(int i, bool down) {
        auto* f = m_fields.self();
        f->injecting = true;
        GJBaseGameLayer::handleButton(down, 1, i == 0);
        f->injecting = false;
        f->eff[i] = down;
    }

    bool waActive() {
        if (!wa::sb("enabled")) return false;
        auto* pl = typeinfo_cast<PlayLayer*>(static_cast<GJBaseGameLayer*>(this));
        if (!pl) return false;
        if (wa::sb("practice-only") && !(pl->m_isPracticeMode || pl->m_isTestMode)) return false;
        if (!m_player1 || m_player1->m_isPlatformer) return false;
        return true;
    }

    // true = clicking this orb looks fatal while leaving it alone is survivable
    bool waOrbVeto(wa::PS& s, PlayerObject* p, RingObject* ring, float dt, float px, float py, int ud, double vr) {
        int id = ring->m_objectID;
        int ri = id == 36 ? 0 : (id == 141 ? 1 : (id == 1333 ? 2 : -1));
        auto it = s.kick.find(id);
        if (ri < 0 && it == s.kick.end()) return false;
        if (!s.ckyInit || s.gN < 3 || s.dxEma <= 0.05f) return false;   // physics not learned yet
        double kick;
        if (it != s.kick.end()) kick = it->second;
        else {
            if (s.jN < 2) return false;
            static const double ratio[3] = {1.0, 0.7, 1.4};
            kick = s.J * ratio[ri];
        }
        if (kick <= 0) return false;

        float size = std::max(0.3f, p->m_vehicleSize);
        float hS = 14.f * size, hH = 10.f * size;
        float lo = this->getMinPortalY(), hi = this->getMaxPortalY();
        if (!(hi - lo > 60.f) || hi - lo > 2000.f) { lo = py - 450.f; hi = py + 450.f; }
        int T = std::clamp(static_cast<int>(std::lround(0.75f / std::max(dt, 0.0005f))), 30, 360);

        if (s.env.ring != ring) {
            s.env = wa::CubeEnv();
            s.env.ring = ring;
            float w = s.dxEma * T + 60.f;
            CCRect q(px - 20.f, py - 420.f, w, 840.f);
            wa::takeAll(this->staticObjectsInRect(q, false), s.env.solids, ud == 1, true);
            wa::takeAll(this->damagingObjectsInRect(q, false), s.env.haz, ud == 1, false);
        }
        double yr = ud ? -py : py;
        double floorR = ud ? -hi : lo;
        auto sim = [&](double v0, float a, float b) {
            return wa::simCube(px, yr, v0, s.dxEma, s.cky, s.g, s.vminR, T, a, b, s.env.solids, s.env.haz, floorR);
        };
        bool deadLo = !sim(kick * 0.8, hS * 0.9f, hH * 0.85f);
        bool deadHi = !sim(kick * 1.2, hS * 1.1f, hH * 1.15f);
        if (!(deadLo && deadHi)) return false;
        return sim(vr, hS * 1.1f, hH * 1.15f);     // leaving it alone must be survivable
    }

    // returns -1 = leave input to the user, 0 = force release, 1 = force hold
    int waDecide(int i, PlayerObject* p, float dt) {
        auto* f = m_fields.self();
        auto& s = f->ps[i];

        float px = p->getPositionX(), py = p->getPositionY();
        bool wave = p->m_isDart, ship = p->m_isShip;
        int mode = wave ? 1 : (ship ? 2 : 0);
        bool mini = p->m_vehicleSize < 0.9f;
        int ud = p->m_isUpsideDown ? 1 : 0;
        double v = p->m_yVelocity;
        bool h = f->eff[i];

        auto fresh = [&]() {
            s = wa::PS();
            s.valid = true; s.px = px; s.py = py; s.pv = v;
            s.mode = mode; s.mini = mini; s.lastDt = dt; s.prevH = h;
            s.slope = mini ? 2.f : 1.f; s.holdUp = !p->m_isUpsideDown;
        };

        if (!s.valid || std::fabs(px - s.px) > 60.f) {      // first frame or respawn
            if (s.valid && (s.mode == 1 || s.mode == 2) && f->clock < s.engagedUntil + 0.25f)
                f->marginBoost = std::min(6.f, f->marginBoost + 0.75f);   // crashed while assisting
            fresh();
            f->userHeld[i] = false; f->eff[i] = false;
            return -1;
        }
        if (mode != s.mode || mini != s.mini) { fresh(); return -1; }

        float dxt = px - s.px, dyt = py - s.py;
        if (dxt > 0.01f) s.dxEma = s.dxEma <= 0.f ? dxt : s.dxEma * 0.9f + dxt * 0.1f;
        s.effStable = (h == s.prevH) ? s.effStable + 1 : 0;
        s.prevH = h;
        bool dtChanged = std::fabs(dt - s.lastDt) > 0.15f * s.lastDt;
        s.lastDt = dt;

        // ---- learn physics ----
        if (wave && dxt > 0.05f && s.effStable >= 2) {
            float smp = std::fabs(dyt) / dxt;
            if (smp > 0.5f && smp < 3.5f) {
                s.slope = s.slope * 0.85f + smp * 0.15f;
                s.holdUp = (h == (dyt > 0.f));
            }
        }
        if (ship) {
            auto& m = s.ship;
            if (dtChanged) m = wa::ShipModel();
            m.vmaxUp = std::max(m.vmaxUp * 0.9995, std::max(0.0, v));
            m.vmaxDn = std::max(m.vmaxDn * 0.9995, std::max(0.0, -v));
            double vm = std::max(m.vmaxUp, m.vmaxDn);
            if (s.effStable >= 2 && std::fabs(v) < 0.85 * vm) {
                auto& a = m.acc[ud][h ? 1 : 0][s.pv >= 0 ? 1 : 0];
                double dv = v - s.pv;
                a.v = a.n ? a.v * 0.8 + dv * 0.2 : dv;
                if (a.n < 1000) a.n++;
            }
            if (std::fabs(v) > 1e-3 && std::fabs(dyt) > 1e-4)
                wa::updKy(m.ky, m.kyInit, m.kyRej, dyt / v);
        }
        s.px = px; s.py = py; s.pv = v;
        bool user = f->userHeld[i];

        // ================= CUBE / BALL / UFO / ROBOT / SPIDER / SWING =================
        if (mode == 0) {
            bool cubeMode = !(p->m_isBall || p->m_isBird || p->m_isRobot || p->m_isSpider || p->m_isSwing);
            double sg = ud ? -1.0 : 1.0;
            double vr = sg * v, dyr = sg * dyt;
            bool gnd = p->m_isOnGround;
            auto* ring = wa::findRing(p, s.used);

            if (cubeMode) {                              // learn gravity, jump, orb kicks from your play
                if (std::fabs(vr) > 1e-3 && std::fabs(dyr) > 1e-4)
                    wa::updKy(s.cky, s.ckyInit, s.ckyRej, dyr / vr);
                double dv = vr - s.pvr;
                if (s.pendKey >= 0) {
                    if (ud == s.pendUd) s.kick[s.pendKey] = vr - (s.gN >= 3 ? s.g : 0.0);
                    s.pendKey = -1;
                } else if (!ring) {
                    if (!gnd && !s.prevGround && dv < -1e-6 && dv > -1e3) {
                        s.g = s.gN ? s.g * 0.9 + dv * 0.1 : dv;
                        if (s.gN < 1000) s.gN++;
                    }
                    if (s.prevGround && !gnd && vr > 1e-3) {
                        double jj = vr - (s.gN >= 3 ? s.g : 0.0);
                        if (s.jN < 3 || (jj > 0.6 * s.J && jj < 1.5 * s.J)) {
                            s.J = s.jN ? s.J * 0.8 + jj * 0.2 : jj;
                            if (s.jN < 1000) s.jN++;
                        }
                    }
                }
                s.vminR = std::min(s.vminR * 0.9995, vr);
            }
            s.pvr = vr; s.prevGround = gnd;

            if (!wa::sb("cube-orbs")) { s.prevUser = user; return -1; }

            if (ring && user && !s.prevUser) {           // you clicked this orb yourself
                s.pendKey = ring->m_objectID; s.pendUd = ud;
                s.used.push_back(ring);
                ring = wa::findRing(p, s.used);
            }
            s.prevUser = user;
            s.touchTicks = ring ? s.touchTicks + 1 : 0;
            if (s.holdTicks > 0) { --s.holdTicks; return 1; }
            if (ring && s.touchTicks > wa::si("orb-delay")) {
                if (cubeMode && wa::sb("orb-reader") && waOrbVeto(s, p, ring, dt, px, py, ud, vr))
                    return -1;                           // trap orb: leave it alone
                if (f->eff[i]) return 0;                 // let go this tick, press on the next
                s.used.push_back(ring);
                s.pendKey = ring->m_objectID; s.pendUd = ud;
                s.holdTicks = 2;
                return 1;
            }
            return -1;
        }

        if (mode == 1 && !wa::sb("wave")) return -1;
        if (mode == 2 && !wa::sb("ship")) return -1;
        if (s.dxEma <= 0.05f) return -1;       // not moving right (or reversed level): don't guess

        // ---- environment ----
        float size = std::max(0.3f, p->m_vehicleSize);
        float half = (wave ? 4.5f : 12.f) * size;
        float lookSec = static_cast<float>(wa::sf("lookahead"));
        float spd = s.dxEma / std::max(dt, 0.0005f);
        float L = std::clamp(spd * lookSec, 220.f, wave ? 720.f : 560.f);

        float lo = this->getMinPortalY(), hi = this->getMaxPortalY();
        if (!(hi - lo > 60.f) || hi - lo > 2000.f) { lo = py - 450.f; hi = py + 450.f; }
        lo = std::max(lo, py - 500.f); hi = std::min(hi, py + 500.f);

        if (s.cAge >= 3 || std::fabs(px - s.cPx) > 20.f || std::fabs(L - s.cL) > 40.f) {
            s.obs.clear();
            wa::gather(this, px - 20.f, lo - 10.f, px + L + 40.f, hi + 10.f, s.obs);
            s.cPx = px; s.cL = L; s.cAge = 0;
        } else {
            s.cAge++;
        }

        float gapThr = static_cast<float>(wa::sf(wave ? "wave-gap" : "ship-gap"));
        auto gap = wa::scanGap(s.obs, px, py, px + std::min(L * 0.6f, 360.f), lo, hi);
        bool trigTight = gap.minGap < gapThr;
        bool rescueOn = wa::sb("rescue");
        float holdT = static_cast<float>(wa::sf("hold-time"));

        // ================= WAVE =================
        if (wave) {
            float dx = std::max(5.f, L / 80.f);
            int N = static_cast<int>(std::ceil(L / dx));
            float mT = static_cast<float>(wa::sf("wave-margin")) + f->marginBoost;
            int d0 = (h == s.holdUp) ? 1 : 0;
            float Cs = static_cast<float>(wa::sf("smoothness"));
            auto P = wa::planWave(px, py, lo, hi, s.slope, dx, N, s.obs, half, mT, mT + 8.f, d0, Cs, 0.2f);

            bool userUp = (user == s.holdUp);
            int uD = P.dT[userUp ? 1 : 0], oD = P.dT[userUp ? 0 : 1];
            bool trigRescue = rescueOn && uD < P.N && uD < oD;
            if (trigTight || trigRescue) s.engagedUntil = f->clock + holdT;
            if (f->clock >= s.engagedUntil) return -1;

            bool up;
            bool uInf = P.cost[1] >= 1e8f, dInf = P.cost[0] >= 1e8f;
            if (uInf && dInf) up = (P.dT[1] != P.dT[0]) ? P.dT[1] > P.dT[0] : userUp;
            else if (uInf) up = false;
            else if (dInf) up = true;
            else if (std::fabs(P.cost[1] - P.cost[0]) > 0.01f) up = P.cost[1] < P.cost[0];
            else up = (d0 == 1);                       // equal: keep flying the same way
            return (up == s.holdUp) ? 1 : 0;
        }

        // ================= SHIP =================
        auto& m = s.ship;
        auto okBucket = [&](int hh) { return m.acc[ud][hh][0].n >= 2 || m.acc[ud][hh][1].n >= 2; };
        if (!(m.kyInit && okBucket(0) && okBucket(1))) return -1;   // still learning your ship
        if (s.obs.empty()) return -1;

        if (dt < 0.006f) { s.tog = !s.tog; if (s.tog) return s.lastRet; }

        float decDx = 14.f;
        int ticksPer = std::max(1, static_cast<int>(std::lround(decDx / s.dxEma)));
        int D = std::clamp(static_cast<int>(L / decDx), 8, 32);
        float mS = static_cast<float>(wa::sf("ship-margin")) + f->marginBoost;
        auto grid = wa::buildGrid(s.obs, px - 5.f, px + D * decDx + 60.f, lo, hi, half + mS);
        auto P = wa::planShip(m, ud, py, v, px, s.dxEma, ticksPer, D, grid, lo + half, hi - half);

        int u = user ? 1 : 0;
        bool trigRescue = rescueOn && P.depth[u] < P.full && P.depth[u] < P.depth[1 - u];
        if (trigTight || trigRescue) s.engagedUntil = f->clock + holdT;
        int ret = -1;
        if (f->clock < s.engagedUntil) {
            int prefer = s.lastAct >= 0 ? s.lastAct : u;
            if (P.depth[1] != P.depth[0]) ret = P.depth[1] > P.depth[0] ? 1 : 0;
            else if (std::fabs(P.clear[1] - P.clear[0]) > 1.0f) ret = P.clear[1] > P.clear[0] ? 1 : 0;
            else ret = prefer;
            s.lastAct = ret;
        } else {
            s.lastAct = -1;
        }
        s.lastRet = ret;
        return ret;
    }

    void handleButton(bool down, int button, bool isPlayer1) {
        auto* f = m_fields.self();
        if (button == 1 && !f->injecting) {
            int i = isPlayer1 ? 0 : 1;
            f->userHeld[i] = down;
            if (f->owned[i]) return;      // assist owns this player's input: swallow
            f->eff[i] = down;
        }
        GJBaseGameLayer::handleButton(down, button, isPlayer1);
    }

    void processCommands(float dt, bool isHalfTick, bool isLastTick) {
        auto* f = m_fields.self();
        if (!isHalfTick) {
            f->clock += dt;
            if (waActive()) {
                bool dual = m_gameState.m_isDualMode;
                bool twoP = m_levelSettings && m_levelSettings->m_twoPlayerMode;
                bool mirror = dual && !twoP;
                PlayerObject* pls[2] = {m_player1, (dual && wa::sb("player2")) ? m_player2 : nullptr};
                int want[2] = {-1, -1};
                for (int i = 0; i < 2; ++i)
                    if (pls[i] && !pls[i]->m_isDead) want[i] = waDecide(i, pls[i], dt);
                if (mirror) {
                    int w = want[0] >= 0 ? want[0] : want[1];
                    want[0] = want[1] = w;
                }
                for (int i = 0; i < 2; ++i) {
                    if (i == 1 && !dual) { f->owned[1] = false; continue; }
                    bool own = want[i] >= 0;
                    f->owned[i] = own;
                    bool target = own ? (want[i] == 1) : f->userHeld[i];
                    if (target != f->eff[i]) waInject(i, target);
                }
            } else {
                for (int i = 0; i < 2; ++i) {
                    if (f->owned[i]) {
                        f->owned[i] = false;
                        if (f->userHeld[i] != f->eff[i]) waInject(i, f->userHeld[i]);
                    }
                }
            }
        }
        GJBaseGameLayer::processCommands(dt, isHalfTick, isLastTick);
    }
};
