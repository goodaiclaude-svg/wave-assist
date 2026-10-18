#include <Geode/Geode.hpp>
#include <Geode/modify/GJBaseGameLayer.hpp>
#include "planner.hpp"

using namespace geode::prelude;

static void addObjects(CCArray* arr, std::vector<wa::Obs>& out) {
    if (!arr) return;
    for (unsigned i = 0; i < arr->count(); ++i) {
        auto* o = static_cast<GameObject*>(arr->objectAtIndex(i));
        if (!o) continue;
        auto t = o->m_objectType;
        bool slope = t == GameObjectType::Slope;
        if (t != GameObjectType::Solid && t != GameObjectType::Hazard && !slope) continue;
        CCRect r = o->getObjectRect();
        wa::Obs ob{r.getMinX(), r.getMinY(), r.getMaxX(), r.getMaxY()};
        if (slope) {
            // base slope: right angle at bottom-right. apply flips, then clockwise rotation.
            int sx = 1, sy = -1;
            if (o->m_isFlipX) sx = -sx;
            if (o->m_isFlipY) sy = -sy;
            float rot = std::fmod(o->getRotation(), 360.f);
            if (rot < 0.f) rot += 360.f;
            int q = ((int)std::lround(rot / 90.f)) & 3;
            for (int k = 0; k < q; ++k) { int nx = sy, ny = -sx; sx = nx; sy = ny; }
            ob.tri = true; ob.sx = sx; ob.sy = sy;
        }
        out.push_back(ob);
    }
}

// Ask the game for solids / slopes / hazards in a box around the player.
static void collectObstacles(GJBaseGameLayer* gl, float py, float x0, float x1, std::vector<wa::Obs>& out) {
    out.clear();
    CCRect box(x0, py - 450.f, x1 - x0, 900.f);
    addObjects(gl->staticObjectsInRect(box, true), out);
    addObjects(gl->damagingObjectsInRect(box, true), out);
}

class $modify(WAGameLayer, GJBaseGameLayer) {
    struct Fields {
        bool  userHeld[2] = {false, false}; // what your finger is doing
        bool  curHeld[2]  = {false, false}; // what the player is actually being told
        bool  ai[2]       = {false, false}; // assist owns this player's input
        float hold[2]     = {0.f, 0.f};     // keeps assist on briefly after a corridor ends
        float lastY[2]    = {0.f, 0.f};
        float vy[2]       = {0.f, 0.f};
        bool  inject      = false;
        std::vector<wa::Obs> obs;
    };

    // Your touches: remember them, and block them while the assist is steering.
    void handleButton(bool down, int button, bool isPlayer1) {
        if (!m_fields->inject && button == 1) {
            int i = isPlayer1 ? 0 : 1;
            m_fields->userHeld[i] = down;
            if (m_fields->ai[i]) return;
            m_fields->curHeld[i] = down;
        }
        GJBaseGameLayer::handleButton(down, button, isPlayer1);
    }

    void waSend(int i, bool down) {
        m_fields->inject = true;
        GJBaseGameLayer::handleButton(down, 1, i == 0);
        m_fields->inject = false;
        m_fields->curHeld[i] = down;
    }

    void waRelease(int i) {
        if (!m_fields->ai[i]) return;
        m_fields->ai[i] = false;                      // hand control back to your finger
        if (m_fields->curHeld[i] != m_fields->userHeld[i]) waSend(i, m_fields->userHeld[i]);
    }

    void waStep(float dt) {
        auto mod = Mod::get();
        bool on = mod->getSettingValue<bool>("enabled");
        if (on && mod->getSettingValue<bool>("practice-only")) {
            auto pl = PlayLayer::get();
            on = pl && (pl->m_isPracticeMode || pl->m_isTestMode);
        }
        for (int i = 0; i < 2; ++i) {
            PlayerObject* p = i == 0 ? m_player1 : m_player2;
            bool active = on && p && !p->m_isDead &&
                (i == 0 || (m_gameState.m_isDualMode && mod->getSettingValue<bool>("player2")));
            bool wave = active && p->m_isDart && mod->getSettingValue<bool>("wave");
            bool ship = active && !p->m_isDart && p->m_isShip && mod->getSettingValue<bool>("ship");
            if (!wave && !ship) { waRelease(i); continue; }

            float px = p->getPositionX(), py = p->getPositionY();
            float speed = std::max(1.f, p->m_playerSpeed);
            float step = 30.f * speed;
            float gap = (float)mod->getSettingValue<double>(ship ? "ship-gap" : "wave-gap");

            collectObstacles(this, py, px - 40.f, px + wa::N * step, m_fields->obs);
            auto plan = wa::makePlan(m_fields->obs, px, py, step, ship ? 0.55f : 1.0f,
                                     ship ? 14.f : 8.f, getMinPortalY(), getMaxPortalY(), gap);

            float vyRaw = dt > 0.f ? (py - m_fields->lastY[i]) / dt : 0.f;
            if (std::abs(vyRaw) > 4000.f) vyRaw = 0.f;
            m_fields->vy[i] = 0.6f * m_fields->vy[i] + 0.4f * vyRaw;
            m_fields->lastY[i] = py;

            if (plan.tight && plan.ok) m_fields->hold[i] = (float)mod->getSettingValue<double>("hold-time");
            else m_fields->hold[i] -= dt;

            if (m_fields->hold[i] <= 0.f || !plan.ok) { waRelease(i); continue; }

            m_fields->ai[i] = true;
            float dir = p->m_isUpsideDown ? -1.f : 1.f;
            float y = ship ? py + m_fields->vy[i] * 0.12f : py;   // ship: steer by predicted height
            bool want = (plan.targetY - y) * dir > 0.f;
            if (want != m_fields->curHeld[i]) waSend(i, want);
        }
    }

    void processCommands(float dt, bool isHalfTick, bool isLastTick) {
        waStep(dt);
        GJBaseGameLayer::processCommands(dt, isHalfTick, isLastTick);
    }
};
