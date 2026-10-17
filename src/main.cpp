#include <Geode/Geode.hpp>
#include <Geode/modify/GJBaseGameLayer.hpp>

using namespace geode::prelude;

namespace {
    bool assistActive(GJBaseGameLayer* gl) {
        if (!Mod::get()->getSettingValue<bool>("enabled")) return false;
        if (Mod::get()->getSettingValue<bool>("practice-only")) {
            auto pl = PlayLayer::get();
            if (!pl) return false;
            if (!(pl->m_isPracticeMode || pl->m_isTestMode)) return false;
        }
        return typeinfo_cast<PlayLayer*>(gl) != nullptr;
    }

    bool inWave(GJBaseGameLayer* gl, int idx) {
        auto* p = idx == 0 ? gl->m_player1 : gl->m_player2;
        return p && p->m_isDart;
    }
}

class $modify(WAGameLayer, GJBaseGameLayer) {
    struct Fields {
        bool fingerDown[2] = {false, false};
        bool spamming[2]   = {false, false};
        bool synthDown[2]  = {false, false};
        float phase[2]     = {0.f, 0.f};
        bool inSynth       = false;
    };

    void sendSynth(bool down, int idx) {
        auto f = m_fields.self();
        if (f->synthDown[idx] == down) return;
        f->synthDown[idx] = down;
        f->inSynth = true;
        GJBaseGameLayer::handleButton(down, 1, idx == 0);
        f->inSynth = false;
    }

    void handleButton(bool down, int button, bool isPlayer1) {
        auto f = m_fields.self();
        int idx = isPlayer1 ? 0 : 1;

        if (f->inSynth || button != 1 || !assistActive(this)
            || (idx == 1 && !Mod::get()->getSettingValue<bool>("player2"))) {
            GJBaseGameLayer::handleButton(down, button, isPlayer1);
            return;
        }

        bool waveOnly = Mod::get()->getSettingValue<bool>("wave-only");

        if (down) {
            f->fingerDown[idx] = true;
            if (waveOnly && !inWave(this, idx)) {
                f->spamming[idx] = false;
                f->synthDown[idx] = true;
                GJBaseGameLayer::handleButton(true, button, isPlayer1);
                return;
            }
            f->spamming[idx] = true;
            f->phase[idx] = 0.f;
            f->synthDown[idx] = false;
            sendSynth(true, idx);
            return;
        }

        f->fingerDown[idx] = false;
        if (f->spamming[idx]) {
            f->spamming[idx] = false;
            if (f->synthDown[idx]) {
                f->synthDown[idx] = false;
                f->inSynth = true;
                GJBaseGameLayer::handleButton(false, 1, isPlayer1);
                f->inSynth = false;
            }
            return;
        }
        f->synthDown[idx] = false;
        GJBaseGameLayer::handleButton(false, button, isPlayer1);
    }

    void processCommands(float dt) {
        auto f = m_fields.self();

        for (int idx = 0; idx < 2; ++idx) {
            if (!f->spamming[idx]) continue;

            bool stop = !assistActive(this)
                || (Mod::get()->getSettingValue<bool>("wave-only") && !inWave(this, idx));
            if (stop) {
                f->spamming[idx] = false;
                if (f->fingerDown[idx] && !f->synthDown[idx]) sendSynth(true, idx);
                continue;
            }

            float cps  = static_cast<float>(Mod::get()->getSettingValue<double>("cps"));
            float duty = static_cast<float>(Mod::get()->getSettingValue<double>("duty"));
            float period = 1.f / std::max(cps, 1.f);

            f->phase[idx] += dt;
            while (f->phase[idx] >= period) f->phase[idx] -= period;

            bool wantDown = f->phase[idx] < period * duty;
            sendSynth(wantDown, idx);
        }

        GJBaseGameLayer::processCommands(dt);
    }
};
