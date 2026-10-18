#include <Geode/Geode.hpp>
#include <Geode/modify/GJBaseGameLayer.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <type_traits>
#include <vector>

using namespace geode::prelude;

namespace wa {

struct Box { float x1, y1, x2, y2; };

inline bool sb(char const* k) { return Mod::get()->getSettingValue<bool>(k); }
inline double sf(char const* k) { return Mod::get()->getSettingValue<double>(k); }
inline int si(char const* k) { return static_cast<int>(Mod::get()->getSettingValue<int64_t>(k)); }

// ---------- obstacle gathering ----------
inline void pushObj(GameObject* o, std::vector<Box>& out) {
    if (!o) return;
    auto r = o->getObjectRect();
    if (r.size.width <= 0.f || r.size.height <= 0.f) return;
    out.push_back({r.getMinX(), r.getMinY(), r.getMaxX(), r.getMaxY()});
}

template <class C>
void takeAll(C&& c, std::vector<Box>& out) {
    using T = std::remove_cv_t<std::remove_reference_t<C>>;
    if constexpr (std::is_pointer_v<T>) {
        using U = std::remove_cv_t<std::remove_pointer_t<T>>;
        if (!c) return;
        if constexpr (std::is_same_v<U, CCArray>) {
            for (auto* o : CCArrayExt<GameObject*>(c)) pushObj(o, out);
        } else {
            for (auto* o : *c) pushObj(o, out);
        }
    } else {
        for (auto* o : c) pushObj(o, out);
    }
}

inline void gather(GJBaseGameLayer* gl, float x1, float y1, float x2, float y2, std::vector<Box>& out) {
    CCRect q(x1, y1, x2 - x1, y2 - y1);
    takeAll(gl->staticObjectsInRect(q, false), out);
    takeAll(gl->damagingObjectsInRect(q, false), out);
}

// ---------- gap / centre scan ----------
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

// ---------- WAVE planner (lattice DP) ----------
// index 0 = world-down first step, 1 = world-up first step. dT = tight margin, dL = loose margin.
struct WavePlan { int dT[2]; int dL[2]; int N; };

inline WavePlan planWave(float x0, float y0, float lo, float hi, float slope, float dx, int N,
                         std::vector<Box> const& obs, float half, float mT, float mL) {
    WavePlan P{};
    P.N = N;
    float dy = slope * dx;
    int rows = 2 * N + 1;
    int jmin = std::max(-N, static_cast<int>(std::ceil((lo + half - y0) / dy)));
    int jmax = std::min(N, static_cast<int>(std::floor((hi - half - y0) / dy)));
    if (jmin > 0) jmin = 0;
    if (jmax < 0) jmax = 0;

    std::vector<uint8_t> blk(static_cast<size_t>(N + 1) * rows);
    std::vector<int16_t> a(rows), b(rows);

    for (int pass = 0; pass < 2; ++pass) {
        float e = half + (pass == 0 ? mT : mL);
        std::fill(blk.begin(), blk.end(), 0);
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
        std::fill(a.begin(), a.end(), 0);
        std::fill(b.begin(), b.end(), 0);
        for (int n = N; n >= 1; --n) {
            for (int j = jmin; j <= jmax; ++j) {
                if (blk[static_cast<size_t>(n) * rows + (j + N)]) { b[j + N] = 0; continue; }
                int ju = std::min(j + 1, jmax), jd = std::max(j - 1, jmin);
                b[j + N] = static_cast<int16_t>(1 + std::max(a[ju + N], a[jd + N]));
            }
            std::swap(a, b);
        }
        int up = std::min(1, jmax), dn = std::max(-1, jmin);
        int* dst = pass == 0 ? P.dT : P.dL;
        dst[1] = a[up + N];
        dst[0] = a[dn + N];
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
        // keep best few of each first action so neither branch is pruned away
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

// ---------- orbs ----------
inline RingObject* findRing(PlayerObject* p, bool p2) {
    if (!p->m_touchingRings || p->m_touchingRings->count() == 0) return nullptr;
    for (auto* o : CCArrayExt<GameObject*>(p->m_touchingRings)) {
        auto* r = typeinfo_cast<RingObject*>(o);
        if (!r) continue;
        if (r->m_objectID == 1704 || r->m_objectID == 1751) continue; // dash orbs: left to you
        if (p2 ? r->m_hasBeenActivatedP2 : r->m_hasBeenActivated) continue;
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
    ShipModel ship; float lastDt = 0; bool tog = false; int lastRet = -1;
    int touchTicks = 0, holdTicks = 0;
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
            if (std::fabs(v) > 1e-3 && std::fabs(dyt) > 1e-4) {
                double smp = dyt / v;
                if (smp > 0) {
                    if (!m.kyInit) { m.ky = smp; m.kyInit = true; }
                    else if (smp > m.ky * 0.25 && smp < m.ky * 4) { m.ky = m.ky * 0.9 + smp * 0.1; m.kyRej = 0; }
                    else if (++m.kyRej > 20) { m.ky = smp; m.kyRej = 0; }
                }
            }
        }
        s.px = px; s.py = py; s.pv = v;
        bool user = f->userHeld[i];

        // ---- cube / ball / ufo / robot / spider / swing : real-orb auto click ----
        if (mode == 0) {
            if (!wa::sb("cube-orbs")) return -1;
            auto* ring = wa::findRing(p, i == 1);
            s.touchTicks = ring ? s.touchTicks + 1 : 0;
            if (s.holdTicks > 0) { --s.holdTicks; return 1; }
            if (ring && s.touchTicks > wa::si("orb-delay")) {
                if (f->eff[i]) return 0;       // let go this tick, press on the next
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
            float mT = static_cast<float>(wa::sf("wave-margin"));
            auto P = wa::planWave(px, py, lo, hi, s.slope, dx, N, s.obs, half, mT, mT + 8.f);

            bool userUp = (user == s.holdUp);
            int uD = P.dT[userUp ? 1 : 0], oD = P.dT[userUp ? 0 : 1];
            bool trigRescue = rescueOn && uD < P.N && uD < oD;
            if (trigTight || trigRescue) s.engagedUntil = f->clock + holdT;
            if (f->clock >= s.engagedUntil) return -1;

            bool up;
            if (P.dL[1] != P.dL[0]) up = P.dL[1] > P.dL[0];
            else if (P.dT[1] != P.dT[0]) up = P.dT[1] > P.dT[0];
            else if (trigTight && std::fabs(py - gap.center) > s.slope * dx * 0.25f) up = py < gap.center;
            else up = userUp;
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
        float mS = static_cast<float>(wa::sf("ship-margin"));
        auto grid = wa::buildGrid(s.obs, px - 5.f, px + D * decDx + 60.f, lo, hi, half + mS);
        auto P = wa::planShip(m, ud, py, v, px, s.dxEma, ticksPer, D, grid, lo + half, hi - half);

        int u = user ? 1 : 0;
        bool trigRescue = rescueOn && P.depth[u] < P.full && P.depth[u] < P.depth[1 - u];
        if (trigTight || trigRescue) s.engagedUntil = f->clock + holdT;
        int ret = -1;
        if (f->clock < s.engagedUntil) {
            if (P.depth[1] != P.depth[0]) ret = P.depth[1] > P.depth[0] ? 1 : 0;
            else if (std::fabs(P.clear[1] - P.clear[0]) > 0.5f) ret = P.clear[1] > P.clear[0] ? 1 : 0;
            else ret = u;
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
