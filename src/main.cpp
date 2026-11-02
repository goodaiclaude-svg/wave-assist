#include <Geode/Geode.hpp>
#include <Geode/modify/PlayerObject.hpp>
#include <algorithm>
#include <deque>

using namespace geode::prelude;

namespace {
    struct Ev { float due; bool push; };
    bool flag(char const* id) { return Mod::get()->getSettingValue<bool>(id); }
    float num(char const* id) { return static_cast<float>(Mod::get()->getSettingValue<double>(id)); }
}

class $modify(MonsterPlayer, PlayerObject) {
    struct Fields {
        float clock = 0.f;
        float pendingAt = 0.f;
        float leftGroundAt = -100.f;
        float ema = 0.f;
        float lastPushDue = -100.f;
        float lastReleaseDue = -100.f;
        bool pending = false;
        bool wasGround = false;
        bool coyoteUsed = false;
        bool userHolding = false;
        bool releaseNext = false;
        std::deque<Ev> queue;
    };

    bool isGround() { return !(m_isShip || m_isBird || m_isDart || m_isSwing); }
    bool ringModes() { return isGround() || m_isShip || m_isBird; }

    bool active() {
        if (!flag("enabled")) return false;
        if (m_isShip) return flag("ship-assist");
        if (m_isBird) return flag("ufo-assist");
        if (m_isDart) return flag("wave-assist");
        return !m_isSwing;
    }

    bool pulseOn() {
        return active() && (m_isShip || m_isBird || m_isDart) && flag("pulse-guard");
    }

    bool canAct() {
        bool ring = m_touchingRings && m_touchingRings->count() > 0;
        return isGround() ? (m_isOnGround || ring) : ring;
    }

    float pulseSec() {
        float frameMs = CCDirector::sharedDirector()->getAnimationInterval() * 1000.f;
        return std::max(num("min-pulse-ms"), frameMs) / 1000.f;
    }

    float windowMs() {
        auto f = m_fields.self();
        float frameFloor = CCDirector::sharedDirector()->getAnimationInterval() * 2000.f;
        float w = std::max(num("window-ms"), frameFloor);
        if (flag("adaptive")) w += f->ema * 0.5f;
        return std::min(w, num("max-window-ms"));
    }

    bool realPush(PlayerButton btn) {
        auto f = m_fields.self();
        if (!active() || !ringModes()) return PlayerObject::pushButton(btn);

        if (canAct()) {
            f->coyoteUsed = true;
            return PlayerObject::pushButton(btn);
        }

        if (isGround()) {
            float coyote = num("coyote-ms") / 1000.f;
            if (!f->coyoteUsed && f->clock - f->leftGroundAt <= coyote) {
                f->coyoteUsed = true;
                m_isOnGround = true;
                bool r = PlayerObject::pushButton(btn);
                m_isOnGround = false;
                return r;
            }
            f->pending = true;
            f->pendingAt = f->clock;
        } else if (flag("fly-ring-buffer")) {
            f->pending = true;
            f->pendingAt = f->clock;
        }
        return PlayerObject::pushButton(btn);
    }

    bool pushButton(PlayerButton btn) {
        auto f = m_fields.self();
        if (btn != PlayerButton::Jump) return PlayerObject::pushButton(btn);
        f->userHolding = true;

        if (pulseOn()) {
            float due = std::max(f->clock, f->lastReleaseDue + pulseSec());
            f->lastPushDue = due;
            if (due > f->clock || !f->queue.empty()) {
                f->queue.push_back({due, true});
                return true;
            }
        }
        return realPush(btn);
    }

    bool releaseButton(PlayerButton btn) {
        auto f = m_fields.self();
        if (btn != PlayerButton::Jump) return PlayerObject::releaseButton(btn);
        f->userHolding = false;

        if (pulseOn()) {
            float due = std::max(f->clock, f->lastPushDue + pulseSec());
            f->lastReleaseDue = due;
            if (due > f->clock || !f->queue.empty()) {
                f->queue.push_back({due, false});
                return true;
            }
        }
        return PlayerObject::releaseButton(btn);
    }

    void update(float dt) {
        auto f = m_fields.self();
        f->clock += dt;

        bool flush = !pulseOn();
        while (!f->queue.empty() && (flush || f->queue.front().due <= f->clock)) {
            auto e = f->queue.front();
            f->queue.pop_front();
            if (e.push) realPush(PlayerButton::Jump);
            else PlayerObject::releaseButton(PlayerButton::Jump);
        }

        if (f->releaseNext) {
            f->releaseNext = false;
            PlayerObject::releaseButton(PlayerButton::Jump);
        }

        bool ground = m_isOnGround;
        if (f->wasGround && !ground) f->leftGroundAt = f->clock;
        if (ground) f->coyoteUsed = false;
        f->wasGround = ground;

        if (f->pending) {
            float ageMs = (f->clock - f->pendingAt) * 1000.f;
            if (!active() || !ringModes()) {
                f->pending = false;
            } else if (canAct()) {
                f->pending = false;
                f->coyoteUsed = true;
                f->ema = f->ema * 0.8f + ageMs * 0.2f;
                PlayerObject::pushButton(PlayerButton::Jump);
                if (!f->userHolding) f->releaseNext = true;
            } else if (ageMs > windowMs()) {
                f->pending = false;
                f->ema *= 0.9f;
            }
        }

        PlayerObject::update(dt);
    }
};
