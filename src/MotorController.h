#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "Definitions.h"
#include "ConfigManager.h"
#include <ArduinoHA.h> // for cover states

class MotorController
{
public:
    float currentPosition = 0.0;
    CoverOperation currentOperation = IDLE;
    CoverOperation lastOperationBeforeStop = IDLE;
    CalibrationState calState = CAL_INACTIVE;
    unsigned long blinkInterval = 0;
    unsigned long lastBlinkTime = 0;
    bool indicatorLightState = false;

    // Safety & Auto Close
    bool autoResumeArmed = false;
    unsigned long acTimer = 0;

    void begin();

    // Actions
    void open();
    void close();
    void stop(bool userTriggered);
    void moveTo(float target);
    void startCalibration();
    void cancelCalibration();

    // Loop Logic (Call in Task)
    void handleRelays();
    void updatePosition();
    void checkSafety();
    void runCalibration();
    void handleIndicator();

    // IO Checks
    bool isBarrierTriggered();
    bool isOpenLimit();
    bool isCloseLimit();
    void logIO();

private:
    unsigned long movementStartTime = 0;
    float movementStartPosition = 0.0;
    float targetPosition = -1.0;
    unsigned long calibrationStartTime = 0;

    // Relay State Machine
#if MOTOR_CONTROL_MODE == 1
    enum MotorRelayState
    {
        R_OFF,
        R_WAIT_ENABLE
    };
    MotorRelayState relayState = R_OFF;
#elif MOTOR_CONTROL_MODE == 2
    enum MotorChangeState
    {
        M_IDLE,
        M_WAIT_FOR_ENGAGE
    };
    MotorChangeState motorChangeState = M_IDLE;
    CoverOperation nextOperation = IDLE;
#endif
    unsigned long relayTimer = 0;

    void executeOpenSequence();
    void executeCloseSequence();
};

extern MotorController gateMotor;

#endif