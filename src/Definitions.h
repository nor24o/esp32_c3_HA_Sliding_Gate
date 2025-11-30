#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include <Arduino.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <vector>

// --- CONFIGURATION ---
#define ENABLE_HW_WATCHDOG true
#define HW_WDT_TIMEOUT 30      
#define SOFT_WDT_TIMEOUT 20000 
#define MAX_LOG_SIZE 5000
#define MOTOR_CONTROL_MODE 2 

// --- PINS ---
#if MOTOR_CONTROL_MODE == 1
    const int RELAY_MOTOR_DIRECTION_PIN = 1;
    const int RELAY_MOTOR_ENABLE_PIN = 4;
#elif MOTOR_CONTROL_MODE == 2
    const int RELAY_MOTOR_OPEN_PIN = 1;
    const int RELAY_MOTOR_CLOSE_PIN = 4;
#endif

const int RELAY_INDICATOR_LIGHT_PIN = 3;
const int MANUAL_MAIN_BUTTON_PIN = 5;
const int MANUAL_WIFI_BUTTON_PIN = 6;
const int MANUAL_MAINTENANCE_BUTTON_PIN = 7;

const int PEDESTRIAN_OPPENING_PIN = 8;

const int LIMIT_OPEN_PIN = 10;
const int LIMIT_CLOSE_PIN = 20;
const int PHOTO_BARRIER_PIN = 2;
const int RF_RECEIVER_PIN = 21;

// --- TIMINGS ---
const unsigned long WIFI_RETRY_INTERVAL = 30000;
const unsigned long CALIBRATION_LONG_PRESS_TIME = 8000;
const unsigned long WIFI_CONFIG_LONG_PRESS_TIME = 5000;
const unsigned long PEDESTRIAN_LONG_PRESS_TIME = 200;


const unsigned long REVERSE_LONG_PRESS_TIME = 1000;
const unsigned long RF_LEARN_TIMEOUT = 60000;
const unsigned long RF_LEARN_SAVE_LONG_PRESS_TIME = 1500;
const unsigned long COMBO_MODE_HOLD_TIME = 2000;
const unsigned long CALIBRATION_SAFETY_TIMEOUT = 90000;

#define BLINK_INTERVAL_OPENING 1000
#define BLINK_INTERVAL_CLOSING 500
#define CALIBRATION_BLINK_INTERVAL 100
#define RF_LEARN_BLINK_INTERVAL 250

// --- ENUMS ---
enum GateCommand {
    CMD_NONE, CMD_OPEN, CMD_CLOSE, CMD_STOP_ONLY, CMD_TOGGLE, 
    CMD_STOP_INTERNAL, CMD_REVERSE, CMD_CALIBRATE_START, 
    CMD_CALIBRATE_CANCEL, CMD_RF_LEARN_START, CMD_RF_LEARN_SKIP, 
    CMD_RF_LEARN_SAVE_EXIT, CMD_RF_SCAN_MODE, CMD_RF_ADD_CODE, 
    CMD_RF_DELETE_CODE, CMD_MOVE_TO_POSITION, CMD_WIFI_CONFIG_START
};

enum CoverOperation { IDLE, OPENING, CLOSING };
enum CalibrationState { CAL_INACTIVE, CAL_HOMING_CLOSE, CAL_MEASURING_OPEN, CAL_VERIFYING_CLOSE, CAL_DONE };
enum RFLearningState { RF_LEARN_INACTIVE, RF_LEARN_WAIT_OPEN, RF_LEARN_WAIT_CLOSE, RF_LEARN_WAIT_STOP, RF_LEARN_WAIT_POS50, RF_SCANNING_WEB };

struct RFEntry {
    unsigned long code;
    uint8_t function; // 0=Open, 1=Close, 2=Stop, 3=Pedestrian, 4=Toggle
};

typedef struct {
    GateCommand cmd;
    float position;
    unsigned long code;
    int aux_data;
} CommandMessage;

// --- GLOBAL HANDLES ---
extern QueueHandle_t xCommandQueue;
extern SemaphoreHandle_t xStateMutex;
extern SemaphoreHandle_t xMotorRelayMutex;
extern SemaphoreHandle_t xLogMutex;

#endif