#include "RFManager.h"
#include "MotorController.h" // <--- ADDED THIS LINE TO FIX 'gateMotor' ERROR
#include "GateNetwork.h"

RFManager rfHandler;

void RFManager::begin()
{
    mySwitch.enableReceive(RF_RECEIVER_PIN);
}

void RFManager::setMaintenanceButton(Button2 *btn)
{
    maintBtn = btn;
}

void RFManager::sendCmd(GateCommand cmd, float pos)
{
    CommandMessage msg = {cmd, pos, 0, 0};
    xQueueSend(xCommandQueue, &msg, 0);
}

void RFManager::startLearning()
{
    learnState = RF_LEARN_WAIT_OPEN;
    learnStartTime = millis();
    gateMotor.blinkInterval = RF_LEARN_BLINK_INTERVAL;
    if (maintBtn)
        maintBtn->setLongClickTime(RF_LEARN_SAVE_LONG_PRESS_TIME);
    sysConfig.logInfo("RF Learn Started");
}

void RFManager::addKey(unsigned long code, uint8_t func)
{
    keyList.push_back({code, func});
    sysConfig.saveRF(keyList);
    sysConfig.logInfo("Added RF: " + String(code));
}

void RFManager::deleteKey(int index)
{
    if (index >= 0 && index < keyList.size())
    {
        keyList.erase(keyList.begin() + index);
        sysConfig.saveRF(keyList);
        sysConfig.logInfo("Deleted RF Index: " + String(index));
    }
}

void RFManager::handleSignal(unsigned long code)
{
    if (code == lastCode && (millis() - lastProcessTime < sysConfig.config.rf_debounce))
        return;
    lastCode = code;
    lastProcessTime = millis();

    // TRIGGER IMMEDIATE UPDATE HERE
    netManager.updateRequest = true; // <--- ADD THIS LINE

    for (const auto &key : keyList)
    {
        if (key.code == code)
        {
            sysConfig.logInfo("RF Match: " + String(code));
            switch (key.function)
            {
            case 0:
                sendCmd(CMD_OPEN);
                break;
            case 1:
                sendCmd(CMD_CLOSE);
                break;
            case 2:
                sendCmd(CMD_STOP_ONLY);
                break;
            case 3:
            {
                float target = (float)sysConfig.config.pedestrian_percent / 100.0f;
                sendCmd(CMD_MOVE_TO_POSITION, target);
            }
            break;
            case 4:
                sendCmd(CMD_TOGGLE);
                break;
            }
            return;
        }
    }
}

void RFManager::handleLearning(unsigned long code)
{
    if (learnState == RF_SCANNING_WEB)
    {
        scannedCode = code;
        learnStartTime = millis(); // Refresh timeout
        netManager.updateRequest = true;
        return;
    }

    sysConfig.logInfo("RF Learn Code: " + String(code));
    netManager.updateRequest = true;
    switch (learnState)
    {
    case RF_LEARN_WAIT_OPEN:
        keyList.push_back({code, 0});
        learnState = RF_LEARN_WAIT_CLOSE;
        learnStartTime = millis();
        break;
    case RF_LEARN_WAIT_CLOSE:
        keyList.push_back({code, 1});
        learnState = RF_LEARN_WAIT_STOP;
        learnStartTime = millis();
        break;
    case RF_LEARN_WAIT_STOP:
        keyList.push_back({code, 2});
        learnState = RF_LEARN_WAIT_POS50;
        learnStartTime = millis();
        break;
    case RF_LEARN_WAIT_POS50:
        keyList.push_back({code, 3});
        sysConfig.saveRF(keyList);
        learnState = RF_LEARN_INACTIVE;
        gateMotor.blinkInterval = 0;
        if (maintBtn)
            maintBtn->setLongClickTime(CALIBRATION_LONG_PRESS_TIME);
        sysConfig.logInfo("RF Learn Done");
        break;
    default:
        break;
    }
}

void RFManager::loop()
{
    if (learnState != RF_LEARN_INACTIVE)
    {
        if (millis() - learnStartTime > RF_LEARN_TIMEOUT)
        {
            learnState = RF_LEARN_INACTIVE;
            gateMotor.blinkInterval = 0;
            if (maintBtn)
                maintBtn->setLongClickTime(CALIBRATION_LONG_PRESS_TIME);
            sysConfig.logInfo("RF Timeout");
            return;
        }
    }

    if (mySwitch.available())
    {
        unsigned long value = mySwitch.getReceivedValue();
        mySwitch.resetAvailable();
        if (value == 0)
            return;

        if (learnState != RF_LEARN_INACTIVE)
            handleLearning(value);
        else
            handleSignal(value);
    }
}