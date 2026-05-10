#pragma once
/**
 * @file  RF.h
 * @brief 433 MHz RF remote learning, decoding, and command dispatch.
 *
 * In normal mode: received codes are matched against the key list and the
 * mapped gate command is posted to cmdQueue.
 *
 * In learn mode: up to four codes are captured in sequence (Open, Close,
 * Stop, Pedestrian) and written to NVS.
 *
 * In web-scan mode: the last received code is stored in `scannedCode` for
 * the web UI to display while the user decides which function to assign.
 */

#include "Config.h"
#include "Types.h"
#include "Storage.h"
#include <RCSwitch.h>
#include <Button2.h>

class RF
{
public:
    std::vector<RFEntry> keys;

    RFLearnState  learnState   = RFLearnState::INACTIVE;
    unsigned long scannedCode  = 0;   ///< last code seen in SCANNING_WEB mode
    unsigned long lastCode     = 0;   ///< last matched code (for HA sensor)

    void begin();
    void tick();   ///< called from vRFTask — polls RCSwitch

    void startLearning();
    void startWebScan();
    void stopWebScan();
    void setMaintButton(Button2 *btn);

    void addKey(unsigned long code, uint8_t func);
    void deleteKey(int index);

private:
    RCSwitch      _sw;
    unsigned long _learnStart = 0;
    unsigned long _lastTime   = 0;    ///< debounce timestamp
    Button2      *_maintBtn   = nullptr;

    void _onSignal(unsigned long code);
    void _onLearning(unsigned long code);
    void _dispatch(GateCommand cmd, float pos = -1.0f);
};

extern RF rf;
