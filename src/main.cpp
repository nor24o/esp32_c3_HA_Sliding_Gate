#include <Arduino.h>
#include <esp_task_wdt.h>
#include <Button2.h>
#include "Definitions.h"
#include "ConfigManager.h"
#include "MotorController.h"
#include "RFManager.h"
#include "GateNetwork.h"

// --- GLOBALS ---
QueueHandle_t xCommandQueue;
SemaphoreHandle_t xStateMutex;
SemaphoreHandle_t xMotorRelayMutex;
SemaphoreHandle_t xLogMutex;
WiFiClient telnetClient;

// --- WATCHDOGS ---
volatile unsigned long last_checkin_motor = 0;
volatile unsigned long last_checkin_net = 0;
volatile unsigned long last_checkin_gate = 0;
volatile unsigned long last_checkin_rf = 0;
volatile unsigned long last_checkin_btn = 0;

// Simulations
bool simCrashMotor = false;
bool simCrashGate = false;
bool simCrashRF = false;
bool simCrashBtn = false;

// Buttons
Button2 mainButton;
Button2 maintButton;
Button2 wifiButton;
Button2 pedestrianButton;

// --- TASK FUNCTIONS ---

void vSupervisorTask(void *pvParameters)
{
    if (ENABLE_HW_WATCHDOG)
        esp_task_wdt_add(NULL);
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (ENABLE_HW_WATCHDOG)
            esp_task_wdt_reset();
        unsigned long now = millis();
        if (millis() > 25000)
        {
            if (now - last_checkin_net > SOFT_WDT_TIMEOUT)
            {
                sysConfig.logError("CRASH: Network");
                delay(500);
                ESP.restart();
            }
            if (now - last_checkin_motor > SOFT_WDT_TIMEOUT)
            {
                sysConfig.logError("CRASH: Motor");
                delay(500);
                ESP.restart();
            }
            if (now - last_checkin_gate > SOFT_WDT_TIMEOUT)
            {
                sysConfig.logError("CRASH: Logic");
                delay(500);
                ESP.restart();
            }
        }
    }
}

void vMotorTask(void *pvParameters)
{
    if (ENABLE_HW_WATCHDOG)
        esp_task_wdt_add(NULL);
    while (1)
    {
        last_checkin_motor = millis();
        if (ENABLE_HW_WATCHDOG)
            esp_task_wdt_reset();
        if (simCrashMotor)
            while (1)
                vTaskDelay(1);
        vTaskDelay(pdMS_TO_TICKS(5));

        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(1)))
        {
            gateMotor.checkSafety();
            gateMotor.handleRelays();
            gateMotor.updatePosition();
            if (gateMotor.calState != CAL_INACTIVE)
                gateMotor.runCalibration();
            xSemaphoreGive(xStateMutex);
        }
    }
}

void vGateLogicTask(void *pvParameters)
{
    if (ENABLE_HW_WATCHDOG)
        esp_task_wdt_add(NULL);
    CommandMessage msg;
    while (1)
    {
        last_checkin_gate = millis();
        if (ENABLE_HW_WATCHDOG)
            esp_task_wdt_reset();
        if (simCrashGate)
            while (1)
                vTaskDelay(1);

        if (xQueueReceive(xCommandQueue, &msg, pdMS_TO_TICKS(100)) == pdPASS)
        {
            if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(10)))
            {
                // Command Routing
                if (msg.cmd == CMD_WIFI_CONFIG_START)
                    netManager.triggerWifiConfig();
                else if (msg.cmd == CMD_RF_LEARN_SAVE_EXIT)
                {
                    rfHandler.learnState = RF_LEARN_INACTIVE;
                    sysConfig.saveRF(rfHandler.keyList);
                    gateMotor.blinkInterval = 0;
                    maintButton.setLongClickTime(CALIBRATION_LONG_PRESS_TIME);
                }
                else if (msg.cmd == CMD_RF_ADD_CODE)
                {
                    rfHandler.addKey(msg.code, msg.aux_data);
                    netManager.sendRFListToWeb();
                }

                else if (msg.cmd == CMD_RF_DELETE_CODE)
                {
                    rfHandler.deleteKey(msg.aux_data);
                    netManager.sendRFListToWeb();
                }

                else if (msg.cmd == CMD_RF_SCAN_MODE)
                {
                    rfHandler.learnState = (msg.aux_data == 1) ? RF_SCANNING_WEB : RF_LEARN_INACTIVE;
                    rfHandler.scannedCode = 0;
                }
                else if (msg.cmd == CMD_CALIBRATE_CANCEL)
                    gateMotor.cancelCalibration();
                else if (gateMotor.calState == CAL_INACTIVE && rfHandler.learnState == RF_LEARN_INACTIVE)
                {
                    switch (msg.cmd)
                    {
                    case CMD_OPEN:
                        gateMotor.open();
                        break;
                    case CMD_CLOSE:
                        gateMotor.close();
                        break;
                    case CMD_STOP_ONLY:
                        gateMotor.stop(true);
                        break;
                    case CMD_TOGGLE:
                        if (gateMotor.currentOperation != IDLE)
                            gateMotor.stop(true);
                        else if (gateMotor.lastOperationBeforeStop == OPENING)
                            gateMotor.open();
                        else if (gateMotor.lastOperationBeforeStop == CLOSING)
                            gateMotor.close();
                        else if (gateMotor.currentPosition >= 0.99)
                            gateMotor.close();
                        else
                            gateMotor.open();
                        break;
                    case CMD_REVERSE:
                        if (gateMotor.currentOperation == OPENING)
                            gateMotor.close();
                        else
                            gateMotor.open();
                        break;
                    case CMD_CALIBRATE_START:
                        gateMotor.startCalibration();
                        break;
                    case CMD_RF_LEARN_START:
                        rfHandler.startLearning();
                        break;
                    case CMD_MOVE_TO_POSITION:
                        gateMotor.moveTo(msg.position);
                        break;
                    case CMD_RF_LEARN_SKIP:
                        if (rfHandler.learnState != RF_LEARN_INACTIVE)
                        {
                            rfHandler.learnState = (RFLearningState)(rfHandler.learnState + 1);
                            if (rfHandler.learnState > RF_LEARN_WAIT_POS50)
                            {
                                sysConfig.saveRF(rfHandler.keyList);
                                rfHandler.learnState = RF_LEARN_INACTIVE;
                                gateMotor.blinkInterval = 0;
                            }
                        }
                        break;
                    default:
                        break;
                    }
                }
                xSemaphoreGive(xStateMutex);
            }
        }
    }
}

void vRFTask(void *pvParameters)
{
    if (ENABLE_HW_WATCHDOG)
        esp_task_wdt_add(NULL);
    while (1)
    {
        last_checkin_rf = millis();
        if (ENABLE_HW_WATCHDOG)
            esp_task_wdt_reset();
        if (simCrashRF)
            while (1)
                vTaskDelay(1);
        vTaskDelay(pdMS_TO_TICKS(10));

        if (xSemaphoreTake(xStateMutex, 5))
        {
            rfHandler.loop();
            xSemaphoreGive(xStateMutex);
        }
    }
}

void vNetworkTask(void *pvParameters)
{
    if (ENABLE_HW_WATCHDOG)
        esp_task_wdt_add(NULL);
    while (1)
    {
        last_checkin_net = millis();
        if (ENABLE_HW_WATCHDOG)
            esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(25));
        netManager.loop();
    }
}

void vLedTask(void *pvParameters)
{
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
        if (xSemaphoreTake(xStateMutex, 20))
        {
            gateMotor.handleIndicator();
            xSemaphoreGive(xStateMutex);
        }
    }
}

void vLoggerTask(void *pvParameters)
{
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(5000));
        if (xSemaphoreTake(xStateMutex, 100))
        {
            gateMotor.logIO();
            xSemaphoreGive(xStateMutex);
        }
    }
}

void vButtonTask(void *pvParameters)
{
    if (ENABLE_HW_WATCHDOG)
        esp_task_wdt_add(NULL);
    static unsigned long comboStart = 0;
    while (1)
    {
        last_checkin_btn = millis();
        if (ENABLE_HW_WATCHDOG)
            esp_task_wdt_reset();
        if (simCrashBtn)
            while (1)
                vTaskDelay(1);
        vTaskDelay(pdMS_TO_TICKS(10));

        mainButton.loop();
        maintButton.loop();
        wifiButton.loop();
        pedestrianButton.loop();

        if (wifiButton.isPressed() && maintButton.isPressed())
        {
            if (comboStart == 0)
                comboStart = millis();
            if (millis() - comboStart > COMBO_MODE_HOLD_TIME)
            {
                CommandMessage msg = {CMD_RF_LEARN_START, 0, 0, 0};
                xQueueSend(xCommandQueue, &msg, 0);
                comboStart = millis() + 5000; // prevent re-trigger
            }
        }
        else
        {
            comboStart = 0;
        }
    }
}

// Button Callbacks
void cbMainClick(Button2 &b)
{
    CommandMessage msg = {CMD_TOGGLE, 0, 0, 0};
    xQueueSend(xCommandQueue, &msg, 0);
}
void cbMainLong(Button2 &b)
{
    CommandMessage msg = {CMD_REVERSE, 0, 0, 0};
    xQueueSend(xCommandQueue, &msg, 0);
}
void cbMaintClick(Button2 &b)
{
    CommandMessage msg = {CMD_RF_LEARN_SKIP, 0, 0, 0};
    xQueueSend(xCommandQueue, &msg, 0);
}
void cbMaintLong(Button2 &b)
{
    CommandMessage msg = {CMD_RF_LEARN_SAVE_EXIT, 0, 0, 0};
    xQueueSend(xCommandQueue, &msg, 0);
}
void cbWifiLong(Button2 &b)
{
    CommandMessage msg = {CMD_WIFI_CONFIG_START, 0, 0, 0};
    xQueueSend(xCommandQueue, &msg, 0);
}
void cbPedestrianClick(Button2 &b)
{
    CommandMessage msg = {CMD_MOVE_TO_POSITION, (float)sysConfig.config.pedestrian_percent / 100.0f, 0, 0};
    xQueueSend(xCommandQueue, &msg, 0);
}

void setup()
{
    xLogMutex = xSemaphoreCreateMutex();
    xStateMutex = xSemaphoreCreateMutex();
    xMotorRelayMutex = xSemaphoreCreateMutex();
    xCommandQueue = xQueueCreate(10, sizeof(CommandMessage));

    sysConfig.begin();
    sysConfig.load();
    sysConfig.loadRF(rfHandler.keyList);
    sysConfig.checkResetReason();

    if (ENABLE_HW_WATCHDOG)
    {
        esp_task_wdt_deinit();
        esp_task_wdt_config_t cfg = {.timeout_ms = HW_WDT_TIMEOUT * 1000, .idle_core_mask = (1 << 0), .trigger_panic = true};
        esp_task_wdt_init(&cfg);
        esp_task_wdt_add(NULL);
    }

    gateMotor.begin();
    rfHandler.begin();
    netManager.begin();

    pinMode(RF_RECEIVER_PIN, INPUT);

    // Button Setup
    mainButton.begin(MANUAL_MAIN_BUTTON_PIN, INPUT_PULLUP, true);
    mainButton.setReleasedHandler(cbMainClick);
    mainButton.setLongClickHandler(cbMainLong);
    mainButton.setLongClickTime(REVERSE_LONG_PRESS_TIME);

    maintButton.begin(MANUAL_MAINTENANCE_BUTTON_PIN, INPUT_PULLUP, true);
    maintButton.setReleasedHandler(cbMaintClick);
    maintButton.setLongClickHandler(cbMaintLong);
    maintButton.setLongClickTime(CALIBRATION_LONG_PRESS_TIME);

    wifiButton.begin(MANUAL_WIFI_BUTTON_PIN, INPUT_PULLUP, true);
    wifiButton.setLongClickHandler(cbWifiLong);
    wifiButton.setLongClickTime(WIFI_CONFIG_LONG_PRESS_TIME);

    pedestrianButton.begin(PEDESTRIAN_OPPENING_PIN, INPUT_PULLUP, true);
    pedestrianButton.setReleasedHandler(cbPedestrianClick);
    pedestrianButton.setLongClickTime(PEDESTRIAN_LONG_PRESS_TIME);

    rfHandler.setMaintenanceButton(&maintButton);

    // Tasks
    xTaskCreate(vSupervisorTask, "Sup", 3072, NULL, 5, NULL);
    xTaskCreate(vMotorTask, "Mot", 4096, NULL, 4, NULL);
    xTaskCreate(vGateLogicTask, "Log", 4096, NULL, 3, NULL);
    xTaskCreate(vRFTask, "RF", 3072, NULL, 3, NULL);
    xTaskCreate(vButtonTask, "Btn", 3072, NULL, 3, NULL);
    xTaskCreate(vNetworkTask, "Net", 4096, NULL, 2, NULL);
    xTaskCreate(vLedTask, "Led", 2048, NULL, 1, NULL);
    xTaskCreate(vLoggerTask, "Lgr", 4096, NULL, 1, NULL);

    sysConfig.logInfo("System Started.");
}

void loop()
{
    if (ENABLE_HW_WATCHDOG)
        esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(1000));
}