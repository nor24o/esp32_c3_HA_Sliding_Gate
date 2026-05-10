#pragma once
/**
 * @file  Motor.h
 * @brief Sliding gate motor controller.
 *
 * Manages relay sequencing, position tracking, limit/barrier safety,
 * calibration state-machine, auto-close after barrier, and indicator LED.
 *
 * All public methods are safe to call from behind stateMtx.
 * Relay I/O is additionally protected by relayMtx (separate mutex so the
 * high-priority motor task never blocks the indicator LED task).
 */

#include "Config.h"
#include "Types.h"
#include "Storage.h"

class Motor
{
public:
    // ── Observable state (read by other tasks under stateMtx) ────────────────
    float       position    = 0.0f;           ///< 0.0 = closed, 1.0 = fully open
    MotorState  state       = MotorState::IDLE;
    MotorState  lastDir     = MotorState::IDLE; ///< last direction before user stop
    CalState    calState    = CalState::INACTIVE;

    unsigned long blinkMs   = 0;              ///< 0 = LED off, >0 = period in ms

    bool          autoClose = false;          ///< armed after barrier reversal
    bool          holdOpen  = false;          ///< overrides all auto-close timers
    unsigned long acTimer   = 0;              ///< millis() when auto-close arm started

    // ── Lifecycle ─────────────────────────────────────────────────────────────
    void begin();

    // ── Commands ─────────────────────────────────────────────────────────────
    void open();
    void close();
    void stop(bool userTriggered);
    void moveTo(float target);
    void togglePedestrian();
    void startCalibration();
    void cancelCalibration();

    // ── Periodic (called from vMotorTask every 5 ms) ─────────────────────────
    void tick();             ///< runs safety + relay + position + calibration + autoClose

    // ── Periodic (called from vLedTask every 100 ms) ─────────────────────────
    void updateLed();

    // ── Sensor reads (const — no side-effects) ───────────────────────────────
    bool barrierTriggered() const;
    bool openLimit()        const;
    bool closeLimit()       const;

    // ── Diagnostics ──────────────────────────────────────────────────────────
    void printIO();

    // ── Timers Info ──────────────────────────────────────────────────────────
    String getTimerStatus() const;
    bool   hasActiveTimer() const;

private:
    // Motion bookkeeping
    unsigned long _moveStart    = 0;
    float         _moveStartPos = 0.0f;
    float         _target       = -1.0f;      ///< -1 = no positional target

    // Calibration
    unsigned long _calStart = 0;

    // LED
    unsigned long _ledLast  = 0;
    bool          _ledState = false;

    // ── Relay state-machine ───────────────────────────────────────────────────
    enum class RelayPhase : uint8_t { IDLE, PRE_BLINK, WAIT_ENGAGE };
    RelayPhase _relayPhase    = RelayPhase::IDLE;
    MotorState _nextDirection = MotorState::IDLE;
    unsigned long _relayTimer = 0;
    unsigned long _openTimer  = 0;

    // ── Private helpers ───────────────────────────────────────────────────────
    void _startOpen();
    void _startClose();
    void _deenergise();
    void _handleRelays();
    void _updatePosition();
    void _checkSafety();
    void _runCalibration();
    void _handleAutoClose();

    bool _relayIdle() const;
};

extern Motor motor;
