#pragma once
/**
 * @file  Types.h
 * @brief Shared enumerations, structures, and FreeRTOS handle declarations.
 *
 * enum class is used for all gate-state enums to avoid name collisions with
 * lwIP's TCP state machine (tcpbase.h declares CLOSING, LISTEN, etc. as
 * unscoped global names via ESPAsyncWebServer → AsyncTCP → lwip).
 */

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <vector>

// ─────────────────────────────────────────────────────────────────────────────
// Command identifiers (CMD_ prefix prevents any global name collision)
// ─────────────────────────────────────────────────────────────────────────────
enum GateCommand : uint8_t {
    CMD_NONE = 0,
    CMD_OPEN,
    CMD_CLOSE,
    CMD_STOP_ONLY,
    CMD_TOGGLE,
    CMD_REVERSE,
    CMD_CALIBRATE_START,
    CMD_CALIBRATE_CANCEL,
    CMD_RF_LEARN_START,
    CMD_RF_LEARN_SKIP,
    CMD_RF_LEARN_SAVE_EXIT,
    CMD_RF_SCAN_MODE,
    CMD_RF_ADD_CODE,
    CMD_RF_DELETE_CODE,
    CMD_MOVE_TO_POSITION,
    CMD_WIFI_CONFIG_START,
    CMD_REBOOT,
};

// ─────────────────────────────────────────────────────────────────────────────
// Gate motion state  (enum class — avoids lwIP's CLOSING in global scope)
// ─────────────────────────────────────────────────────────────────────────────
enum class MotorState : uint8_t { IDLE, OPENING, CLOSING };

// ─────────────────────────────────────────────────────────────────────────────
// Calibration state
// ─────────────────────────────────────────────────────────────────────────────
enum class CalState : uint8_t { INACTIVE, HOMING, MEASURING, VERIFYING, DONE };

// ─────────────────────────────────────────────────────────────────────────────
// RF learning state
// ─────────────────────────────────────────────────────────────────────────────
enum class RFLearnState : uint8_t {
    INACTIVE,
    WAIT_OPEN,
    WAIT_CLOSE,
    WAIT_STOP,
    WAIT_PED,
    SCANNING_WEB,
};

// ─────────────────────────────────────────────────────────────────────────────
// RF remote key entry  (function codes mirror RFLearnState ordering)
//   0 = Open  1 = Close  2 = Stop  3 = Pedestrian  4 = Toggle
// ─────────────────────────────────────────────────────────────────────────────
struct RFEntry {
    unsigned long code;
    uint8_t       function;
};

// ─────────────────────────────────────────────────────────────────────────────
// Inter-task command message
// ─────────────────────────────────────────────────────────────────────────────
struct Command {
    GateCommand   cmd;
    float         position;   // CMD_MOVE_TO_POSITION
    unsigned long code;       // CMD_RF_ADD_CODE
    int           aux;        // CMD_RF_SCAN_MODE, CMD_RF_ADD_CODE function index
};

// ─────────────────────────────────────────────────────────────────────────────
// FreeRTOS handles — defined in main.cpp, used everywhere
// ─────────────────────────────────────────────────────────────────────────────
extern QueueHandle_t     cmdQueue;
extern SemaphoreHandle_t stateMtx;
extern SemaphoreHandle_t relayMtx;
extern SemaphoreHandle_t logMtx;
