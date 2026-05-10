/**
 * @file Motor.cpp
 */

#include "Motor.h"
#include "GateNetwork.h"   // for gateNet.requestUpdate()
#include "Storage.h"
#include "Log.hpp"

Motor motor;

// ─────────────────────────────────────────────────────────────────────────────
// begin()
// ─────────────────────────────────────────────────────────────────────────────
void Motor::begin()
{
    pinMode(PIN_RELAY_1, OUTPUT); digitalWrite(PIN_RELAY_1, LOW);
    pinMode(PIN_RELAY_2, OUTPUT); digitalWrite(PIN_RELAY_2, LOW);

    pinMode(PIN_INDICATOR, OUTPUT);   digitalWrite(PIN_INDICATOR, LOW);
    pinMode(PIN_LIM_OPEN,  INPUT_PULLUP);
    pinMode(PIN_LIM_CLOSE, INPUT_PULLUP);
    pinMode(PIN_BARRIER,   INPUT_PULLUP);
}

// ─────────────────────────────────────────────────────────────────────────────
// Sensor reads
// ─────────────────────────────────────────────────────────────────────────────
bool Motor::barrierTriggered() const
{
    const bool raw = (digitalRead(PIN_BARRIER) == HIGH);
    return storage.cfg.barrierActiveHigh ? raw : !raw;
}

bool Motor::openLimit() const
{
    const bool raw = (digitalRead(PIN_LIM_OPEN) == HIGH);
    return storage.cfg.limitsActiveHigh ? raw : !raw;
}

bool Motor::closeLimit() const
{
    const bool raw = (digitalRead(PIN_LIM_CLOSE) == HIGH);
    return storage.cfg.limitsActiveHigh ? raw : !raw;
}

// ─────────────────────────────────────────────────────────────────────────────
// tick() — the single call from vMotorTask
// ─────────────────────────────────────────────────────────────────────────────
void Motor::tick()
{
    _checkSafety();
    _handleRelays();
    _updatePosition();
    if (calState != CalState::INACTIVE) _runCalibration();
    _handleAutoClose();
}

// ─────────────────────────────────────────────────────────────────────────────
// updateLed() — called from vLedTask
// ─────────────────────────────────────────────────────────────────────────────
void Motor::updateLed()
{
    if (blinkMs == 0) {
        if (_ledState) { digitalWrite(PIN_INDICATOR, LOW); _ledState = false; }
        return;
    }
    const unsigned long now = millis();
    if (now - _ledLast >= blinkMs) {
        _ledLast  = now;
        _ledState = !_ledState;
        digitalWrite(PIN_INDICATOR, _ledState ? HIGH : LOW);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Public commands
// ─────────────────────────────────────────────────────────────────────────────
void Motor::open()
{
    lastDir = MotorState::IDLE;
    if (state != MotorState::IDLE || calState != CalState::INACTIVE) return;
    if (position >= 0.99f || openLimit()) { LOG_PRINTLN("[Motor] Open blocked: at limit"); return; }
    LOG_PRINTLN("[Motor] OPEN");
    _startOpen();
}

void Motor::close()
{
    lastDir = MotorState::IDLE;
    if (state != MotorState::IDLE || calState != CalState::INACTIVE) return;
    if (position <= 0.01f || closeLimit()) { LOG_PRINTLN("[Motor] Close blocked: at limit"); return; }
    if (barrierTriggered())               { LOG_PRINTLN("[Motor] Close blocked: barrier"); return; }
    LOG_PRINTLN("[Motor] CLOSE");
    _startClose();
}

void Motor::stop(bool userTriggered)
{
    if (state == MotorState::IDLE && calState == CalState::INACTIVE && !autoClose)
        return;

    if (userTriggered && state != MotorState::IDLE)
        lastDir = (position > 0.01f && position < 0.99f) ? state : MotorState::IDLE;

    _deenergise();

    blinkMs = 0;
    state   = MotorState::IDLE;
    gateNet.requestUpdate();
    LOG_PRINTLN("[Motor] STOP");

    if (userTriggered) {
        _target   = -1.0f;
        calState  = CalState::INACTIVE;
        autoClose = false;
    }
}

void Motor::moveTo(float target)
{
    if (calState != CalState::INACTIVE) return;
    if (state != MotorState::IDLE) stop(false);
    if (target < position && barrierTriggered()) { LOG_PRINTLN("[Motor] moveTo blocked: barrier"); return; }
    if (fabsf(position - target) < 0.01f) return;

    _target = target;
    if (target > position) _startOpen();
    else                   _startClose();
}

void Motor::startCalibration()
{
    if (state != MotorState::IDLE || calState != CalState::INACTIVE) return;
    storage.log("[Motor] Calibration started.");
    calState  = CalState::HOMING;
    blinkMs   = BLINK_CALIBRATING;
    _calStart = millis();
}

void Motor::cancelCalibration()
{
    if (calState == CalState::INACTIVE) return;
    stop(false);
    calState = CalState::INACTIVE;
    blinkMs  = 0;
    storage.log("[Motor] Calibration cancelled.");
}

// ─────────────────────────────────────────────────────────────────────────────
// Diagnostics
// ─────────────────────────────────────────────────────────────────────────────
void Motor::printIO()
{
    int m1 = 0, m2 = 0;
    m1 = digitalRead(PIN_RELAY_1);
    m2 = digitalRead(PIN_RELAY_2);
    LOG_PRINTF("[IO] OL:%d CL:%d BAR:%d | M1:%d M2:%d LED:%d | POS:%.2f\n",
               digitalRead(PIN_LIM_OPEN), digitalRead(PIN_LIM_CLOSE),
               digitalRead(PIN_BARRIER),
               m1, m2, digitalRead(PIN_INDICATOR), position);
}

// ─────────────────────────────────────────────────────────────────────────────
// Private — relay sequencing
// ─────────────────────────────────────────────────────────────────────────────
bool Motor::_relayIdle() const
{
    return _relayPhase == RelayPhase::IDLE;
}

void Motor::_deenergise()
{
    if (xSemaphoreTake(relayMtx, pdMS_TO_TICKS(10)) == pdTRUE) {
        digitalWrite(PIN_RELAY_1, LOW);
        digitalWrite(PIN_RELAY_2, LOW);
        _relayPhase    = RelayPhase::IDLE;
        _nextDirection = MotorState::IDLE;
        xSemaphoreGive(relayMtx);
    }
}

void Motor::_startOpen()
{
    if (openLimit()) { position = 1.0f; stop(false); return; }
    if (state == MotorState::OPENING) return;

    state   = MotorState::OPENING;
    blinkMs = BLINK_OPENING;
    gateNet.requestUpdate();

    if (xSemaphoreTake(relayMtx, pdMS_TO_TICKS(10)) == pdTRUE) {
        // Cut power on both relays first for absolute safety
        digitalWrite(PIN_RELAY_1, LOW);
        digitalWrite(PIN_RELAY_2, LOW);

        if (storage.cfg.motorMode == 1) digitalWrite(PIN_RELAY_1, LOW); // DIR=Open

        _relayPhase    = (storage.cfg.preBlinkDelay > 0) ? RelayPhase::PRE_BLINK : RelayPhase::WAIT_ENGAGE;
        _nextDirection = MotorState::OPENING;
        _relayTimer = millis();
        xSemaphoreGive(relayMtx);
    }
}

void Motor::_startClose()
{
    if (closeLimit()) { position = 0.0f; stop(false); return; }
    if (state == MotorState::CLOSING) return;

    state   = MotorState::CLOSING;
    blinkMs = BLINK_CLOSING;
    gateNet.requestUpdate();

    if (xSemaphoreTake(relayMtx, pdMS_TO_TICKS(10)) == pdTRUE) {
        // Cut power on both relays first for absolute safety
        digitalWrite(PIN_RELAY_1, LOW);
        digitalWrite(PIN_RELAY_2, LOW);

        if (storage.cfg.motorMode == 1) digitalWrite(PIN_RELAY_1, HIGH); // DIR=Close

        _relayPhase    = (storage.cfg.preBlinkDelay > 0) ? RelayPhase::PRE_BLINK : RelayPhase::WAIT_ENGAGE;
        _nextDirection = MotorState::CLOSING;
        _relayTimer = millis();
        xSemaphoreGive(relayMtx);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Private — relay engage after motor_delay
// ─────────────────────────────────────────────────────────────────────────────
void Motor::_handleRelays()
{
    const unsigned long now = millis();

    if (_relayPhase == RelayPhase::PRE_BLINK && now - _relayTimer >= storage.cfg.preBlinkDelay) {
        _relayPhase = RelayPhase::WAIT_ENGAGE;
        _relayTimer = now; // restart timer for the motor engage delay
    }

    if (_relayPhase == RelayPhase::WAIT_ENGAGE &&
        now - _relayTimer >= storage.cfg.motorDelay)
    {
        if (xSemaphoreTake(relayMtx, pdMS_TO_TICKS(1)) == pdTRUE) {
            if (storage.cfg.motorMode == 1) {
                digitalWrite(PIN_RELAY_2, HIGH); // Enable Relay
            } else {
                if (_nextDirection == MotorState::OPENING)
                    digitalWrite(PIN_RELAY_1, HIGH); // Open Relay
                else if (_nextDirection == MotorState::CLOSING)
                    digitalWrite(PIN_RELAY_2, HIGH); // Close Relay
            }
            xSemaphoreGive(relayMtx);
        }
        _relayPhase    = RelayPhase::IDLE;
        _nextDirection = MotorState::IDLE;
        _moveStart     = now;
        _moveStartPos  = position;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Private — position tracking
// ─────────────────────────────────────────────────────────────────────────────
void Motor::_updatePosition()
{
    if (state == MotorState::IDLE || !_relayIdle()) return;

    const unsigned long elapsed = millis() - _moveStart;
    const float ratio = storage.cfg.travelTime > 0
                        ? (float(elapsed) / float(storage.cfg.travelTime))
                        : 1.0f;

    position = (state == MotorState::OPENING)
               ? _moveStartPos + ratio
               : _moveStartPos - ratio;
    position = constrain(position, 0.0f, 1.0f);

    // Positional target reached?
    if (_target >= 0.0f) {
        const bool done = (state == MotorState::OPENING && position >= _target)
                       || (state == MotorState::CLOSING && position <= _target);
        if (done) { stop(false); position = _target; _target = -1.0f; return; }
    }

    // Travel timeout safety net
    if (storage.cfg.travelTime > 0 && elapsed > storage.cfg.travelTime + T_TRAVEL_OVERTIME) {
        LOG_PRINTLN("[Motor] Travel timeout — stopping.");
        stop(false);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Private — hardware safety checks
// ─────────────────────────────────────────────────────────────────────────────
void Motor::_checkSafety()
{
    if (calState != CalState::INACTIVE) return;

    if (state == MotorState::OPENING && openLimit()) {
        stop(false);
        position = 1.0f;
        return;
    }

    if (state == MotorState::CLOSING) {
        if (closeLimit()) { stop(false); position = 0.0f; return; }

        if (barrierTriggered()) {
            LOG_PRINTLN("[Motor] Barrier — reversing.");
            stop(false);
            _target   = -1.0f;
            autoClose = true;
            _startOpen();
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Private — calibration state machine
// ─────────────────────────────────────────────────────────────────────────────
void Motor::_runCalibration()
{
    if (millis() - _calStart > T_CAL_SAFETY) {
        storage.log("[Motor] Calibration timeout — aborted.");
        stop(false); calState = CalState::INACTIVE; blinkMs = 0;
        return;
    }

    switch (calState) {
    case CalState::HOMING:
        if (closeLimit()) {
            stop(false); delay(500);
            calState = CalState::MEASURING;
            _startOpen();
        } else if (state != MotorState::CLOSING) {
            _startClose();
        }
        break;

    case CalState::MEASURING:
        if (openLimit()) {
            storage.cfg.travelTime = millis() - _moveStart;
            storage.log("[Motor] travel_time = " + String(storage.cfg.travelTime) + " ms");
            stop(false); delay(500);
            calState = CalState::VERIFYING;
            _startClose();
        }
        break;

    case CalState::VERIFYING:
        if (closeLimit()) {
            stop(false);
            storage.save();
            storage.log("[Motor] Calibration complete.");
            calState = CalState::DONE;
        }
        break;

    case CalState::DONE:
        calState = CalState::INACTIVE;
        blinkMs  = 0;
        position = 0.0f;
        break;

    default: break;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Private — auto-close after barrier reversal
// ─────────────────────────────────────────────────────────────────────────────
void Motor::_handleAutoClose()
{
    // 1. Barrier-triggered auto-close
    if (autoClose && state == MotorState::IDLE && calState == CalState::INACTIVE) {
        if (acTimer == 0) {
            acTimer = millis();
            storage.log("[Motor] Auto-close: waiting…");
        }
        if (millis() - acTimer >= storage.cfg.acDelay) {
            storage.log("[Motor] Auto-close: closing now.");
            autoClose = false;
            acTimer   = 0;
            close();
        }
    } else if (!autoClose) {
        acTimer = 0;
    }

    // 2. Timer-triggered auto-close from fully open
    if (storage.cfg.acOpenEnabled && state == MotorState::IDLE && calState == CalState::INACTIVE && position >= 0.99f) {
        if (_openTimer == 0) _openTimer = millis();
        if (millis() - _openTimer >= storage.cfg.acOpenDelay) {
            storage.log("[Motor] Timer Auto-close: closing now.");
            _openTimer = 0;
            close();
        }
    } else {
        _openTimer = 0;
    }
}
