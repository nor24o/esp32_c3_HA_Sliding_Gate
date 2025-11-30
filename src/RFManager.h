#ifndef RF_MANAGER_H
#define RF_MANAGER_H

#include "Definitions.h"
#include <RCSwitch.h>
#include <vector>
#include "ConfigManager.h"
#include <Button2.h> // Needed to adjust long press times

class RFManager
{
public:
    std::vector<RFEntry> keyList;
    RFLearningState learnState = RF_LEARN_INACTIVE;
    unsigned long scannedCode = 0;
    unsigned long lastCode = 0;
    unsigned long lastProcessTime = 0;

    void begin();
    void loop();
    void startLearning();
    void stopLearning();
    void setMaintenanceButton(Button2 *btn);

    // Helpers
    void addKey(unsigned long code, uint8_t func);
    void deleteKey(int index);

private:
    RCSwitch mySwitch;
    unsigned long learnStartTime = 0;
    Button2 *maintBtn = nullptr;

    void handleSignal(unsigned long code);
    void handleLearning(unsigned long code);
    void sendCmd(GateCommand cmd, float pos = -1.0);
};

extern RFManager rfHandler;

#endif