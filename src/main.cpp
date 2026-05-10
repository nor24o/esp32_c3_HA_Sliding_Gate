/**
 * @file  main.cpp
 * @brief Entry point — FreeRTOS task creation, button setup, WDT config.
 *
 * Task map (priority descending):
 *   5  Supervisor  — hardware + software watchdog
 *   4  Motor       — relay sequencing, safety, position tracking
 *   3  Logic       — command dispatcher (receives from cmdQueue)
 *   3  RF          — 433 MHz receiver polling
 *   3  Buttons     — Button2 polling + combo detection
 *   2  Network     — WiFi, WebSocket, MQTT, Telnet
 *   1  LED         — indicator blink
 *   1  IOLogger    — periodic IO dump to serial/telnet
 */

#include <Arduino.h>
#include <esp_idf_version.h>
#include <esp_task_wdt.h>
#include <Button2.h>

#include "Config.h"
#include "Types.h"
#include "Log.hpp"
#include "Storage.h"
#include "Motor.h"
#include "RF.h"
#include "GateNetwork.h"

// ─────────────────────────────────────────────────────────────────────────────
// FreeRTOS handles (declared extern in Types.h)
// ─────────────────────────────────────────────────────────────────────────────
QueueHandle_t     cmdQueue;
SemaphoreHandle_t stateMtx;
SemaphoreHandle_t relayMtx;
SemaphoreHandle_t logMtx;

// ─────────────────────────────────────────────────────────────────────────────
// Software watchdog timestamps — updated by each supervised task
// ─────────────────────────────────────────────────────────────────────────────
static volatile unsigned long wdtMotor = 0;
static volatile unsigned long wdtNet   = 0;
static volatile unsigned long wdtLogic = 0;

// ─────────────────────────────────────────────────────────────────────────────
// Buttons
// ─────────────────────────────────────────────────────────────────────────────
static Button2 btnMain, btnMaint, btnWifi, btnPed;

// ─────────────────────────────────────────────────────────────────────────────
// Button callbacks — post to cmdQueue from any task safely
// ─────────────────────────────────────────────────────────────────────────────
static void onMainClick (Button2 &btn) { Command m={CMD_TOGGLE};                              xQueueSend(cmdQueue,&m,pdMS_TO_TICKS(10)); }
static void onMainLong  (Button2 &btn) { Command m={CMD_REVERSE};                             xQueueSend(cmdQueue,&m,pdMS_TO_TICKS(10)); }
static void onMaintClick(Button2 &btn) { Command m={CMD_RF_LEARN_SKIP};                       xQueueSend(cmdQueue,&m,pdMS_TO_TICKS(10)); }
static void onMaintLong (Button2 &btn) { Command m={CMD_RF_LEARN_SAVE_EXIT};                  xQueueSend(cmdQueue,&m,pdMS_TO_TICKS(10)); }
static void onWifiLong  (Button2 &btn) { Command m={CMD_WIFI_CONFIG_START};                   xQueueSend(cmdQueue,&m,pdMS_TO_TICKS(10)); }
static void onPedClick  (Button2 &btn) {
    Command m = { CMD_TOGGLE_PEDESTRIAN, 0, 0, 0 };
    xQueueSend(cmdQueue, &m, pdMS_TO_TICKS(10));
}

// ─────────────────────────────────────────────────────────────────────────────
// TASKS
// ─────────────────────────────────────────────────────────────────────────────

// ── Supervisor — hardware WDT pet + software check-in monitor ───────────────
static void taskSupervisor(void *)
{
    if (HW_WDT_ENABLED) esp_task_wdt_add(NULL);
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (HW_WDT_ENABLED) esp_task_wdt_reset();
        if (millis() < 25000) continue;   // startup grace period

        const unsigned long now = millis();
        auto check = [&](unsigned long ts, const char *who) {
            if (now - ts > SOFT_WDT_TIMEOUT_MS) {
                storage.logError(("SWDT: " + String(who)).c_str());
                delay(200); ESP.restart();
            }
        };
        check(wdtMotor, "Motor");
        check(wdtNet,   "Network");
        check(wdtLogic, "Logic");
    }
}

// ── Motor — runs all Motor::tick() work at 200 Hz ───────────────────────────
static void taskMotor(void *)
{
    if (HW_WDT_ENABLED) esp_task_wdt_add(NULL);
    for (;;) {
        wdtMotor = millis();
        if (HW_WDT_ENABLED) esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(5));
        if (xSemaphoreTake(stateMtx, pdMS_TO_TICKS(1)) == pdTRUE) {
            motor.tick();
            xSemaphoreGive(stateMtx);
        }
    }
}

// ── Logic — drains cmdQueue and dispatches gate commands ────────────────────
static void taskLogic(void *)
{
    if (HW_WDT_ENABLED) esp_task_wdt_add(NULL);
    Command msg;
    for (;;) {
        wdtLogic = millis();
        if (HW_WDT_ENABLED) esp_task_wdt_reset();

        if (xQueueReceive(cmdQueue, &msg, pdMS_TO_TICKS(100)) != pdPASS) continue;
        
        // Wait patiently for the state mutex without silently dropping the button press!
        while (xSemaphoreTake(stateMtx, pdMS_TO_TICKS(100)) != pdTRUE) {
            wdtLogic = millis();
            if (HW_WDT_ENABLED) esp_task_wdt_reset();
        }

        switch (msg.cmd) {
        // ── Always-handled commands ──────────────────────────────────────────
        case CMD_WIFI_CONFIG_START:
            gateNet.startWifiPortal();
            break;

        case CMD_RF_LEARN_SAVE_EXIT:
            rf.learnState    = RFLearnState::INACTIVE;
            storage.saveRF(rf.keys);
            motor.blinkMs    = 0;
            btnMaint.setLongClickTime(T_CAL_LONG_PRESS);
            break;

        case CMD_RF_ADD_CODE:
            rf.addKey(msg.code, uint8_t(msg.aux));
            gateNet.sendRFList();
            break;

        case CMD_RF_DELETE_CODE:
            rf.deleteKey(msg.aux);
            gateNet.sendRFList();
            break;

        case CMD_RF_SCAN_MODE:
            if (msg.aux == 1) rf.startWebScan();
            else              rf.stopWebScan();
            break;

        case CMD_CALIBRATE_CANCEL:
            motor.cancelCalibration();
            break;

        case CMD_REBOOT:
            vTaskDelay(pdMS_TO_TICKS(500)); // allow final network packets to leave
            ESP.restart();
            break;

        case CMD_TOGGLE_PEDESTRIAN:
            motor.togglePedestrian();
            break;

        case CMD_SET_HOLD_OPEN:
            motor.holdOpen = (msg.aux == 1);
            gateNet.requestUpdate();
            break;

        // ── Motion commands — gate must be idle and not in learn mode ────────
        default:
            if (motor.calState   == CalState::INACTIVE &&
                rf.learnState    == RFLearnState::INACTIVE)
            {
                switch (msg.cmd) {
                case CMD_OPEN:    motor.open();         break;
                case CMD_CLOSE:   motor.close();        break;
                case CMD_STOP_ONLY: motor.stop(true);   break;

                case CMD_TOGGLE:
                    if      (motor.state  != MotorState::IDLE)         motor.stop(true);
                    else if (motor.openLimit())                        motor.close();
                    else if (motor.closeLimit())                       motor.open();
                    else if (motor.lastDir == MotorState::OPENING)     motor.close();
                    else if (motor.lastDir == MotorState::CLOSING)     motor.open();
                    else if (motor.position >= 0.95f)                  motor.close();
                    else                                               motor.open();
                    break;

                case CMD_REVERSE:
                    motor.state == MotorState::OPENING ? motor.close() : motor.open();
                    break;

                case CMD_CALIBRATE_START: motor.startCalibration(); break;
                case CMD_RF_LEARN_START:  rf.startLearning();       break;

                case CMD_MOVE_TO_POSITION:
                    motor.moveTo(msg.position);
                    break;

                case CMD_RF_LEARN_SKIP: {
                    const auto cur  = static_cast<uint8_t>(rf.learnState);
                    const auto next = cur + 1;
                    if (cur == 0) break;   // not in learn mode
                    if (next > static_cast<uint8_t>(RFLearnState::WAIT_PED)) {
                        storage.saveRF(rf.keys);
                        rf.learnState = RFLearnState::INACTIVE;
                        motor.blinkMs = 0;
                    } else {
                        rf.learnState = static_cast<RFLearnState>(next);
                    }
                    break;
                }

                default: break;
                }
            }
            break;
        }

        xSemaphoreGive(stateMtx);
    }
}

// ── RF — polls RCSwitch ─────────────────────────────────────────────────────
static void taskRF(void *)
{
    if (HW_WDT_ENABLED) esp_task_wdt_add(NULL);
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(5)); // Poll faster to catch short RF bursts
        while (xSemaphoreTake(stateMtx, pdMS_TO_TICKS(100)) != pdTRUE) {
            if (HW_WDT_ENABLED) esp_task_wdt_reset();
        }
        rf.tick();
        xSemaphoreGive(stateMtx);
        if (HW_WDT_ENABLED) esp_task_wdt_reset();
    }
}

// ── Network — WiFi, web server, MQTT, Telnet ────────────────────────────────
static void taskNetwork(void *)
{
    if (HW_WDT_ENABLED) esp_task_wdt_add(NULL);
    for (;;) {
        wdtNet = millis();
        if (HW_WDT_ENABLED) esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(25));
        gateNet.loop();
    }
}

// ── LED indicator ────────────────────────────────────────────────────────────
static void taskLED(void *)
{
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(100));
        if (xSemaphoreTake(stateMtx, pdMS_TO_TICKS(20)) == pdTRUE) {
            motor.updateLed();
            xSemaphoreGive(stateMtx);
        }
    }
}

// ── IO logger — periodic IO state to serial/telnet ──────────────────────────
static void taskIOLogger(void *)
{
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        if (xSemaphoreTake(stateMtx, pdMS_TO_TICKS(100)) == pdTRUE) {
            motor.printIO();
            xSemaphoreGive(stateMtx);
        }
    }
}

// ── Button poller + two-button combo detection ───────────────────────────────
static void taskButtons(void *)
{
    if (HW_WDT_ENABLED) esp_task_wdt_add(NULL);
    unsigned long comboStart = 0;
    for (;;) {
        if (HW_WDT_ENABLED) esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(10));

        btnMain.loop(); btnMaint.loop(); btnWifi.loop(); btnPed.loop();

        // WiFi + Maint held together → start RF learn
        if (btnWifi.isPressed() && btnMaint.isPressed()) {
            if (!comboStart) comboStart = millis();
            if (millis() - comboStart > T_COMBO_HOLD) {
                Command m = { CMD_RF_LEARN_START };
                xQueueSend(cmdQueue, &m, 0);
                comboStart = millis() + 5000;   // suppress re-trigger
            }
        } else {
            comboStart = 0;
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// setup()
// ─────────────────────────────────────────────────────────────────────────────
void setup()
{

    Serial.begin(115200);
    delay(2000);  // allow time for serial monitor to connect before logs start
    // Synchronisation primitives must be first — Log.hpp uses logMtx
    logMtx   = xSemaphoreCreateMutex();
    stateMtx = xSemaphoreCreateMutex();
    relayMtx = xSemaphoreCreateMutex();
    cmdQueue = xQueueCreate(10, sizeof(Command));

    storage.begin();
    storage.load();
    storage.loadRF(rf.keys);
    storage.printResetReason();

    // Hardware WDT — replace the Arduino default (5 s, watches loopTask).
    // We do NOT register loopTask: it blocks in network.begin() during WiFi
    // connect for up to 60 s, which exceeds our 30 s timeout.
    // Each critical FreeRTOS task self-registers inside its own function.
    if (HW_WDT_ENABLED) {
        esp_task_wdt_deinit();
        #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
            esp_task_wdt_config_t cfg = {
                .timeout_ms     = HW_WDT_TIMEOUT_S * 1000,
                .idle_core_mask = 0,      // don't auto-watch idle tasks
                .trigger_panic  = true,
            };
            esp_task_wdt_init(&cfg);
        #else
            esp_task_wdt_init(HW_WDT_TIMEOUT_S, true);
        #endif
    }

    motor.begin();
    pinMode(PIN_RF_RX, INPUT); // Explicitly configure GPIO matrix before attaching interrupt
    rf.begin();
    gateNet.begin();

    // Button configuration
    btnMain.begin(PIN_BTN_MAIN,  INPUT_PULLUP, true);
    btnMain.setDebounceTime(50);
    btnMain.setPressedHandler(onMainClick); // Instantaneous reaction!
    btnMain.setLongClickHandler(onMainLong);
    btnMain.setLongClickTime(T_REVERSE_PRESS);

    btnMaint.begin(PIN_BTN_MAINT, INPUT_PULLUP, true);
    btnMaint.setDebounceTime(50);
    btnMaint.setPressedHandler(onMaintClick); // Instantaneous reaction!
    btnMaint.setLongClickHandler(onMaintLong);
    btnMaint.setLongClickTime(T_CAL_LONG_PRESS);

    btnWifi.begin(PIN_BTN_WIFI, INPUT_PULLUP, true);
    btnWifi.setDebounceTime(50);
    btnWifi.setLongClickHandler(onWifiLong);
    btnWifi.setLongClickTime(T_WIFI_LONG_PRESS);

    btnPed.begin(PIN_BTN_PED, INPUT_PULLUP, true);
    btnPed.setDebounceTime(50);
    btnPed.setPressedHandler(onPedClick); // Instantaneous reaction!
    btnPed.setLongClickTime(T_PED_PRESS);

    rf.setMaintButton(&btnMaint);

    // Spawn tasks
    xTaskCreate(taskSupervisor, "Sup",  3072, nullptr, 5, nullptr);
    xTaskCreate(taskMotor,      "Mot",  4096, nullptr, 4, nullptr);
    xTaskCreate(taskLogic,      "Log",  4096, nullptr, 3, nullptr);
    xTaskCreate(taskRF,         "RF",   3072, nullptr, 3, nullptr);
    xTaskCreate(taskButtons,    "Btn",  3072, nullptr, 3, nullptr);
    xTaskCreate(taskNetwork,    "Net",  6144, nullptr, 2, nullptr);
    xTaskCreate(taskLED,        "Led",  2048, nullptr, 1, nullptr);
    xTaskCreate(taskIOLogger,   "IO",   4096, nullptr, 1, nullptr);

    storage.log("[System] Started. Boot #" + String(storage.cfg.bootCount));
}

// ─────────────────────────────────────────────────────────────────────────────
// loop() — intentionally empty; all work is done in FreeRTOS tasks above.
// loopTask is NOT registered with the HW WDT (see setup() comment).
// ─────────────────────────────────────────────────────────────────────────────
void loop()
{
    vTaskDelay(pdMS_TO_TICKS(1000));
}
