/**
 * @file RF.cpp
 */

#include "RF.h"
#include "Motor.h"
#include "GateNetwork.h"
#include "Storage.h"
#include "Log.hpp"

RF rf;

void RF::begin()
{
    _sw.setReceiveTolerance(90); // Increase timing tolerance to survive ESP32 WiFi jitter
    _sw.enableReceive(digitalPinToInterrupt(PIN_RF_RX));
}

void RF::setMaintButton(Button2 *btn)
{
    _maintBtn = btn;
}

// ─────────────────────────────────────────────────────────────────────────────
void RF::addKey(unsigned long code, uint8_t func)
{
    keys.push_back({ code, func });
    storage.saveRF(keys);
    storage.log("[RF] Added code " + String(code));
}

void RF::deleteKey(int index)
{
    if (index < 0 || static_cast<size_t>(index) >= keys.size()) return;
    keys.erase(keys.begin() + index);
    storage.saveRF(keys);
    storage.log("[RF] Deleted index " + String(index));
}

void RF::startLearning()
{
    learnState   = RFLearnState::WAIT_OPEN;
    _learnStart  = millis();
    motor.blinkMs = BLINK_RF_LEARN;
    if (_maintBtn) _maintBtn->setLongClickTime(T_RF_SAVE_PRESS);
    storage.log("[RF] Learn mode started.");
}

// ─────────────────────────────────────────────────────────────────────────────
// tick() — called from vRFTask
// ─────────────────────────────────────────────────────────────────────────────
void RF::tick()
{
    // Learn/scan mode timeout
    if (learnState != RFLearnState::INACTIVE) {
        if (millis() - _learnStart > T_RF_LEARN_TIMEOUT) {
            learnState     = RFLearnState::INACTIVE;
            motor.blinkMs  = 0;
            if (_maintBtn) _maintBtn->setLongClickTime(T_CAL_LONG_PRESS);
            storage.log("[RF] Learn mode timed out.");
            return;
        }
    }

    if (!_sw.available()) return;
    const unsigned long code = _sw.getReceivedValue();
    _sw.resetAvailable();
    if (code == 0) return;

    if (learnState != RFLearnState::INACTIVE)
        _onLearning(code);
    else
        _onSignal(code);
}

// ─────────────────────────────────────────────────────────────────────────────
// Private
// ─────────────────────────────────────────────────────────────────────────────
void RF::_dispatch(GateCommand cmd, float pos)
{
    Command msg = { cmd, pos, 0, 0 };
    xQueueSend(cmdQueue, &msg, 0);
}

void RF::_onSignal(unsigned long code)
{
    // Debounce repeated signals
    if (code == lastCode && millis() - _lastTime < storage.cfg.rfDebounce) return;
    lastCode  = code;
    _lastTime = millis();
    gateNet.requestUpdate();

    for (const auto &k : keys) {
        if (k.code != code) continue;
        storage.log("[RF] Match: " + String(code));
        switch (k.function) {
        case 0: _dispatch(CMD_OPEN);      break;
        case 1: _dispatch(CMD_CLOSE);     break;
        case 2: _dispatch(CMD_STOP_ONLY); break;
        case 3: _dispatch(CMD_MOVE_TO_POSITION,
                          float(storage.cfg.pedPercent) / 100.0f); break;
        case 4: _dispatch(CMD_TOGGLE);    break;
        }
        return;
    }
}

void RF::_onLearning(unsigned long code)
{
    if (learnState == RFLearnState::SCANNING_WEB) {
        scannedCode = code;
        _learnStart = millis();
        gateNet.requestUpdate();
        return;
    }

    storage.log("[RF] Learn received: " + String(code));
    gateNet.requestUpdate();

    // Map current learn state to a function index and advance
    const uint8_t func = static_cast<uint8_t>(learnState) - 1; // WAIT_OPEN=1 → func 0
    keys.push_back({ code, func });

    const auto next = static_cast<uint8_t>(learnState) + 1;
    if (next > static_cast<uint8_t>(RFLearnState::WAIT_PED)) {
        storage.saveRF(keys);
        learnState    = RFLearnState::INACTIVE;
        motor.blinkMs = 0;
        if (_maintBtn) _maintBtn->setLongClickTime(T_CAL_LONG_PRESS);
        storage.log("[RF] Learn complete.");
    } else {
        learnState  = static_cast<RFLearnState>(next);
        _learnStart = millis();
    }
}
