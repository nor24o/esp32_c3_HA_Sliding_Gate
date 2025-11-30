#include "MotorController.h"
#include "GateNetwork.h"

MotorController gateMotor;

void MotorController::begin()
{
#if MOTOR_CONTROL_MODE == 1
    pinMode(RELAY_MOTOR_DIRECTION_PIN, OUTPUT);
    pinMode(RELAY_MOTOR_ENABLE_PIN, OUTPUT);
    digitalWrite(RELAY_MOTOR_ENABLE_PIN, LOW);
    digitalWrite(RELAY_MOTOR_DIRECTION_PIN, LOW);
#elif MOTOR_CONTROL_MODE == 2
    pinMode(RELAY_MOTOR_OPEN_PIN, OUTPUT);
    pinMode(RELAY_MOTOR_CLOSE_PIN, OUTPUT);
    digitalWrite(RELAY_MOTOR_OPEN_PIN, LOW);
    digitalWrite(RELAY_MOTOR_CLOSE_PIN, LOW);
#endif

    pinMode(RELAY_INDICATOR_LIGHT_PIN, OUTPUT);
    digitalWrite(RELAY_INDICATOR_LIGHT_PIN, LOW);
    pinMode(LIMIT_OPEN_PIN, INPUT_PULLUP);
    pinMode(LIMIT_CLOSE_PIN, INPUT_PULLUP);
    pinMode(PHOTO_BARRIER_PIN, INPUT_PULLUP);
}

bool MotorController::isBarrierTriggered()
{
    return sysConfig.config.bar_active_high ? (digitalRead(PHOTO_BARRIER_PIN) == HIGH) : (digitalRead(PHOTO_BARRIER_PIN) == LOW);
}

bool MotorController::isOpenLimit()
{
    return sysConfig.config.lim_active_high ? (digitalRead(LIMIT_OPEN_PIN) == HIGH) : (digitalRead(LIMIT_OPEN_PIN) == LOW);
}

bool MotorController::isCloseLimit()
{
    return sysConfig.config.lim_active_high ? (digitalRead(LIMIT_CLOSE_PIN) == HIGH) : (digitalRead(LIMIT_CLOSE_PIN) == LOW);
}

void MotorController::handleIndicator()
{
    if (blinkInterval == 0)
    {
        if (indicatorLightState)
        {
            digitalWrite(RELAY_INDICATOR_LIGHT_PIN, LOW);
            indicatorLightState = false;
        }
        return;
    }
    if (millis() - lastBlinkTime > blinkInterval)
    {
        lastBlinkTime = millis();
        indicatorLightState = !indicatorLightState;
        digitalWrite(RELAY_INDICATOR_LIGHT_PIN, indicatorLightState);
    }
}

void MotorController::executeOpenSequence()
{
    if (currentOperation == OPENING)
        return;
    currentOperation = OPENING;
    netManager.updateRequest = true; // <--- Force UI update to show "OPENING" immediately
    blinkInterval = BLINK_INTERVAL_OPENING;

    if (xSemaphoreTake(xMotorRelayMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
#if MOTOR_CONTROL_MODE == 1
        digitalWrite(RELAY_MOTOR_DIRECTION_PIN, LOW);
        relayTimer = millis();
        relayState = R_WAIT_ENABLE;
#elif MOTOR_CONTROL_MODE == 2
        digitalWrite(RELAY_MOTOR_OPEN_PIN, LOW);
        digitalWrite(RELAY_MOTOR_CLOSE_PIN, LOW);
        motorChangeState = M_WAIT_FOR_ENGAGE;
        relayTimer = millis();
        nextOperation = OPENING;
#endif
        xSemaphoreGive(xMotorRelayMutex);
    }
}

void MotorController::executeCloseSequence()
{
    if (currentOperation == CLOSING)
        return;
    currentOperation = CLOSING;
    netManager.updateRequest = true; // <--- Force UI update to show "CLOSING" immediately
    blinkInterval = BLINK_INTERVAL_CLOSING;

    if (xSemaphoreTake(xMotorRelayMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
#if MOTOR_CONTROL_MODE == 1
        digitalWrite(RELAY_MOTOR_DIRECTION_PIN, HIGH);
        relayTimer = millis();
        relayState = R_WAIT_ENABLE;
#elif MOTOR_CONTROL_MODE == 2
        digitalWrite(RELAY_MOTOR_OPEN_PIN, LOW);
        digitalWrite(RELAY_MOTOR_CLOSE_PIN, LOW);
        motorChangeState = M_WAIT_FOR_ENGAGE;
        relayTimer = millis();
        nextOperation = CLOSING;
#endif
        xSemaphoreGive(xMotorRelayMutex);
    }
}

void MotorController::handleRelays()
{
#if MOTOR_CONTROL_MODE == 1
    if (relayState == R_WAIT_ENABLE && (millis() - relayTimer >= sysConfig.config.motor_delay))
    {
        if (xSemaphoreTake(xMotorRelayMutex, pdMS_TO_TICKS(1)) == pdTRUE)
        {
            digitalWrite(RELAY_MOTOR_ENABLE_PIN, HIGH);
            xSemaphoreGive(xMotorRelayMutex);
        }
        relayState = R_OFF;
        movementStartTime = millis();
        movementStartPosition = currentPosition;
    }
#elif MOTOR_CONTROL_MODE == 2
    if (motorChangeState == M_WAIT_FOR_ENGAGE && (millis() - relayTimer >= sysConfig.config.motor_delay))
    {
        if (xSemaphoreTake(xMotorRelayMutex, pdMS_TO_TICKS(1)) == pdTRUE)
        {
            if (nextOperation == OPENING)
                digitalWrite(RELAY_MOTOR_OPEN_PIN, HIGH);
            else if (nextOperation == CLOSING)
                digitalWrite(RELAY_MOTOR_CLOSE_PIN, HIGH);
            xSemaphoreGive(xMotorRelayMutex);
        }
        motorChangeState = M_IDLE;
        nextOperation = IDLE;
        movementStartTime = millis();
        movementStartPosition = currentPosition;
    }
#endif
}

void MotorController::open()
{
    lastOperationBeforeStop = IDLE;
    if (currentOperation != IDLE || calState != CAL_INACTIVE)
        return;
    if (currentPosition >= 0.99f || isOpenLimit())
    {
        LOG_PRINTLN("Cannot open: Limit/Pos");
        return;
    }
    LOG_PRINTLN("CMD: OPEN");
    executeOpenSequence();
}

void MotorController::close()
{
    lastOperationBeforeStop = IDLE;
    if (currentOperation != IDLE || calState != CAL_INACTIVE)
        return;
    if (currentPosition <= 0.01f || isCloseLimit())
    {
        LOG_PRINTLN("Cannot close: Limit/Pos");
        return;
    }
    if (isBarrierTriggered())
    {
        LOG_PRINTLN("Cannot close: Barrier");
        return;
    }
    LOG_PRINTLN("CMD: CLOSE");
    executeCloseSequence();
}

void MotorController::stop(bool userTriggered)
{
    if (currentOperation == IDLE && calState == CAL_INACTIVE && !autoResumeArmed)
        return;
    if (userTriggered && currentOperation != IDLE)
        lastOperationBeforeStop = (currentPosition > 0.01f && currentPosition < 0.99f) ? currentOperation : IDLE;

    if (xSemaphoreTake(xMotorRelayMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
#if MOTOR_CONTROL_MODE == 1
        digitalWrite(RELAY_MOTOR_ENABLE_PIN, LOW);
        digitalWrite(RELAY_MOTOR_DIRECTION_PIN, LOW);
        relayState = R_OFF;
#elif MOTOR_CONTROL_MODE == 2
        digitalWrite(RELAY_MOTOR_OPEN_PIN, LOW);
        digitalWrite(RELAY_MOTOR_CLOSE_PIN, LOW);
        motorChangeState = M_IDLE;
        nextOperation = IDLE;
#endif
        xSemaphoreGive(xMotorRelayMutex);
    }

    blinkInterval = 0;
    currentOperation = IDLE;
    netManager.updateRequest = true; // <--- Force UI update to show "STOPPED" immediately
    LOG_PRINTLN("STOP Executed");

    if (userTriggered)
    {
        targetPosition = -1.0;
        if (calState != CAL_INACTIVE)
            calState = CAL_INACTIVE;
        autoResumeArmed = false;
    }
}

void MotorController::moveTo(float target)
{
    if (calState != CAL_INACTIVE)
        return;
    if (currentOperation != IDLE)
        stop(false);
    if (abs(currentPosition - target) < 0.01)
        return;
    targetPosition = target;
    if (target > currentPosition)
        executeOpenSequence();
    else
        executeCloseSequence();
}

void MotorController::updatePosition()
{
    if (currentOperation == IDLE)
        return;
// Wait for relay engage
#if MOTOR_CONTROL_MODE == 1
    if (relayState != R_OFF)
        return;
#elif MOTOR_CONTROL_MODE == 2
    if (motorChangeState != M_IDLE)
        return;
#endif

    unsigned long elapsed = millis() - movementStartTime;
    float ratio = (sysConfig.config.travel_time > 0) ? (float(elapsed) / sysConfig.config.travel_time) : 1.0f;

    if (currentOperation == OPENING)
        currentPosition = movementStartPosition + ratio;
    else
        currentPosition = movementStartPosition - ratio;

    currentPosition = constrain(currentPosition, 0.0f, 1.0f);

    // Target Check
    if (targetPosition >= 0.0)
    {
        if ((currentOperation == OPENING && currentPosition >= targetPosition) ||
            (currentOperation == CLOSING && currentPosition <= targetPosition))
        {
            stop(false);
            currentPosition = targetPosition;
            targetPosition = -1.0;
        }
    }
    // Timeout
    if (sysConfig.config.travel_time > 0 && elapsed > (sysConfig.config.travel_time + 3000))
    {
        LOG_PRINTLN("Travel Timeout!");
        stop(false);
    }
}

void MotorController::checkSafety()
{
    if (calState != CAL_INACTIVE)
        return;

    if (currentOperation == OPENING && isOpenLimit())
    {
        stop(false);
        currentPosition = 1.0f;
    }
    if (currentOperation == CLOSING)
    {
        if (isCloseLimit())
        {
            stop(false);
            currentPosition = 0.0f;
        }
        if (isBarrierTriggered())
        {
            LOG_PRINTLN("Barrier Triggered!");
            stop(false);
            autoResumeArmed = true;
            executeOpenSequence();
        }
    }
}

void MotorController::startCalibration()
{
    if (currentOperation != IDLE || calState != CAL_INACTIVE)
        return;
    sysConfig.logInfo("Starting Calibration...");
    calState = CAL_HOMING_CLOSE;
    blinkInterval = CALIBRATION_BLINK_INTERVAL;
    calibrationStartTime = millis();
}

void MotorController::cancelCalibration()
{
    if (calState != CAL_INACTIVE)
    {
        stop(false);
        calState = CAL_INACTIVE;
        blinkInterval = 0;
        sysConfig.logInfo("Calibration Cancelled.");
    }
}

void MotorController::runCalibration()
{
    if (millis() - calibrationStartTime > CALIBRATION_SAFETY_TIMEOUT)
    {
        sysConfig.logInfo("CAL: Timeout Failed");
        stop(false);
        calState = CAL_INACTIVE;
        blinkInterval = 0;
        return;
    }

    switch (calState)
    {
    case CAL_HOMING_CLOSE:
        if (isCloseLimit())
        {
            stop(false);
            delay(500);
            calState = CAL_MEASURING_OPEN;
            executeOpenSequence();
        }
        else if (currentOperation != CLOSING)
        {
            executeCloseSequence();
        }
        break;
    case CAL_MEASURING_OPEN:
        if (isOpenLimit())
        {
            sysConfig.config.travel_time = millis() - movementStartTime;
            sysConfig.logInfo("CAL: Time Measured: " + String(sysConfig.config.travel_time));
            stop(false);
            delay(500);
            calState = CAL_VERIFYING_CLOSE;
            executeCloseSequence();
        }
        break;
    case CAL_VERIFYING_CLOSE:
        if (isCloseLimit())
        {
            stop(false);
            sysConfig.logInfo("CAL: Complete.");
            sysConfig.save();
            calState = CAL_DONE;
        }
        break;
    case CAL_DONE:
        calState = CAL_INACTIVE;
        blinkInterval = 0;
        currentPosition = 0.0f;
        break;
    }
}

void MotorController::logIO()
{
    int m1 = 0, m2 = 0;
#if MOTOR_CONTROL_MODE == 1
    m1 = digitalRead(RELAY_MOTOR_DIRECTION_PIN);
    m2 = digitalRead(RELAY_MOTOR_ENABLE_PIN);
#elif MOTOR_CONTROL_MODE == 2
    m1 = digitalRead(RELAY_MOTOR_OPEN_PIN);
    m2 = digitalRead(RELAY_MOTOR_CLOSE_PIN);
#endif
    char buf[128];
    snprintf(buf, sizeof(buf), "[IO] OP:%d CL:%d PH:%d | M1:%d M2:%d LGT:%d | POS:%.2f",
             digitalRead(LIMIT_OPEN_PIN), digitalRead(LIMIT_CLOSE_PIN), digitalRead(PHOTO_BARRIER_PIN),
             m1, m2, digitalRead(RELAY_INDICATOR_LIGHT_PIN), currentPosition);
    LOG_PRINTLN(buf);
}