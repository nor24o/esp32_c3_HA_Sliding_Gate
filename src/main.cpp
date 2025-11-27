#include <Arduino.h>
#include <RCSwitch.h>
#include <Button2.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <ArduinoHA.h>
#include <LittleFS.h>
#include <stdlib.h>
#include <ArduinoJson.h>
#include <esp_task_wdt.h>
#include <Preferences.h> // [NEW] To store Boot Count

// --- CONFIGURATION ---
#define ENABLE_WATCHDOG true
#define WDT_TIMEOUT_SECONDS 10
#define MAX_LOG_SIZE 5000 // [NEW] Max log size in bytes (5KB)

// Define global handles
SemaphoreHandle_t xLogMutex = NULL;
WiFiClient telnetClient;

#include "SerialMirror.hpp"

// --- FreeRTOS Includes ---
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

// --- Command Structure ---
enum GateCommand
{
    CMD_NONE,
    CMD_OPEN,
    CMD_CLOSE,
    CMD_STOP_USER,
    CMD_STOP_INTERNAL,
    CMD_REVERSE,
    CMD_CALIBRATE_START,
    CMD_RF_LEARN_START,
    CMD_RF_LEARN_SKIP,
    CMD_RF_LEARN_SAVE_EXIT,
    CMD_MOVE_TO_POSITION,
    CMD_WIFI_CONFIG_START
};

typedef struct
{
    GateCommand cmd;
    float position;
} CommandMessage;

// --- RTOS Global Objects ---
QueueHandle_t xCommandQueue;
SemaphoreHandle_t xStateMutex;
SemaphoreHandle_t xMotorRelayMutex;
TaskHandle_t hNetworkTask = NULL;

// --- PREFERENCES (Boot Count) ---
Preferences preferences;
unsigned int bootCount = 0;

// =================================================================
// Pin and Constant Definitions
// =================================================================
#define MOTOR_CONTROL_MODE 2

#if MOTOR_CONTROL_MODE == 1
const int RELAY_MOTOR_DIRECTION_PIN = 1;
const int RELAY_MOTOR_ENABLE_PIN = 4;
#elif MOTOR_CONTROL_MODE == 2
const int RELAY_MOTOR_OPEN_PIN = 1;
const int RELAY_MOTOR_CLOSE_PIN = 4;
#endif

const int RELAY_INDICATOR_LIGHT_PIN = 3;
const int MANUAL_MAIN_BUTTON_PIN = 21;
const int MANUAL_WIFI_BUTTON_PIN = 6;
const int MANUAL_MAINTENANCE_BUTTON_PIN = 7;
const int LIMIT_OPEN_PIN = 10;
const int LIMIT_CLOSE_PIN = 20;
const int PHOTO_BARRIER_PIN = 2;
const int RF_RECEIVER_PIN = 5;

// ——— Timing constants ———
unsigned long MOTOR_DIRECTION_DELAY = 700;
unsigned long CALIBRATION_LONG_PRESS_TIME = 8000;
unsigned long WIFI_CONFIG_LONG_PRESS_TIME = 5000;
unsigned long WIFI_RETRY_INTERVAL = 30000;
const unsigned long REVERSE_LONG_PRESS_TIME = 1000;
const unsigned long RF_LEARN_TIMEOUT = 30000;
const unsigned long RF_LEARN_SAVE_LONG_PRESS_TIME = 1500;
const unsigned long RF_DEBOUNCE_DELAY = 400;
const unsigned long COMBO_MODE_HOLD_TIME = 2000;

// Persistent parameters
unsigned long gate_travel_time = 30000;
char mqtt_server[40] = "192.168.1.12";
char mqtt_port_str[6] = "1883";
char mqtt_user[32] = "admin";
char mqtt_password[64] = "admin";
unsigned long rf_gate_open_code = 1234567;
unsigned long rf_gate_close_code = 7654321;
unsigned long rf_gate_stop_code = 1111111;
unsigned long rf_gate_pos50_code = 2222222;

WiFiManagerParameter custom_mqtt_server("server", "MQTT Server", mqtt_server, sizeof(mqtt_server));
WiFiManagerParameter custom_mqtt_port("port", "MQTT Port", mqtt_port_str, sizeof(mqtt_port_str));
WiFiManagerParameter custom_mqtt_user("user", "MQTT User", mqtt_user, sizeof(mqtt_user));
WiFiManagerParameter custom_mqtt_password("password", "MQTT Password", mqtt_password, sizeof(mqtt_password));

unsigned long last_blink_time = 0;
unsigned long blink_interval = 0;
bool indicator_light_state = false;

#define PARAMS_FILE "/gate_params_data.txt"
#define LOG_FILE "/system_log.txt"
#define BLINK_INTERVAL_OPENING 1000
#define BLINK_INTERVAL_CLOSING 500
#define CALIBRATION_BLINK_INTERVAL 100
#define RF_LEARN_BLINK_INTERVAL 250

// =================================================================
// Global State Variables
// =================================================================
enum CoverOperation
{
    IDLE,
    OPENING,
    CLOSING
};
enum CalibrationState
{
    CAL_INACTIVE,
    CAL_CLOSING_TO_START,
    CAL_OPENING_FOR_TIMING,
    CAL_DONE
};
enum RFLearningState
{
    RF_LEARN_INACTIVE,
    RF_LEARN_WAIT_OPEN,
    RF_LEARN_WAIT_CLOSE,
    RF_LEARN_WAIT_STOP,
    RF_LEARN_WAIT_POS50
};

#if MOTOR_CONTROL_MODE == 1
enum MotorRelayState
{
    R_OFF,
    R_WAIT_ENABLE
};
#elif MOTOR_CONTROL_MODE == 2
enum MotorChangeState
{
    M_IDLE,
    M_WAIT_FOR_ENGAGE
};
#endif

CoverOperation current_operation = IDLE;
CalibrationState cal_state = CAL_INACTIVE;
CoverOperation last_operation_before_stop = IDLE;
RFLearningState rf_learn_state = RF_LEARN_INACTIVE;
unsigned long rf_learn_start_time = 0;

#if MOTOR_CONTROL_MODE == 1
MotorRelayState motor_relay_state = R_OFF;
unsigned long motor_relay_timer = 0;
#elif MOTOR_CONTROL_MODE == 2
MotorChangeState motor_change_state = M_IDLE;
unsigned long motor_change_timer = 0;
CoverOperation next_operation = IDLE;
#endif

float current_position = 0.0;
unsigned long movement_start_time;
float movement_start_position;
float target_position = -1.0;
bool auto_resume_is_armed = false;
bool resume_countdown_is_active = false;
unsigned long path_cleared_time = 0;
unsigned long last_wifi_check = 0;
unsigned long last_rf_code_received = 0;
unsigned long last_rf_process_time = 0;

RCSwitch mySwitch;
Button2 mainButton;
Button2 maintenanceButton;
Button2 wifiButton;
WiFiManager wm;
WiFiServer telnetServer(23);

WiFiClient wifiClient;
HADevice device;
HAMqtt mqtt(wifiClient, device);
HACover cover("sliding_gate_cover");
HASensor rfCodeSensor("sliding_gate_last_rf_code");
HAButton calibrateButton("sliding_gate_calibrate");
HAButton moveTo50Button("sliding_gate_move_to_50");
HASensor gateState("sliding_gate_state");
HASensor gateIP("sliding_gate_IP");
HASensor travelTimeSensor("sliding_gate_travel_time");
HAButton openButtonHA("sliding_gate_open_button");
HAButton closeButtonHA("sliding_gate_close_button");
HAButton stopButtonHA("sliding_gate_stop_button");

// ——— Forward Declarations ———
void execute_open_sequence();
void execute_close_sequence();
void start_opening();
void start_closing();
void stop_movement(bool triggered_by_user);
void update_gate_position();
void handle_safety_sensors();
void handle_rf_signal();
void handle_motor_relays();
void start_calibration();
void handle_calibration();
void handle_rf_learning();
void handle_indicator_light();
void handle_wifi_status();
void move_to_position(float new_target);
void publish_all_states();
void save_params();
void load_params();
void check_reset_reason();
void dump_system_log();
void vConfigPortalTask(void *pvParameters);
void log_io_status();

// =================================================================
// Logging & Maintenance
// =================================================================

void log_system_error(const char *msg)
{
    // [NEW] Check size before appending
    if (LittleFS.exists(LOG_FILE))
    {
        File f = LittleFS.open(LOG_FILE, "r");
        if (f.size() > MAX_LOG_SIZE)
        {
            f.close();
            LittleFS.remove(LOG_FILE); // Delete old file
            File fNew = LittleFS.open(LOG_FILE, "w");
            fNew.println("--- LOG ROTATED (Size Limit) ---");
            fNew.close();
        }
        else
        {
            f.close();
        }
    }

    File logFile = LittleFS.open(LOG_FILE, "a");
    if (logFile)
    {
        // [NEW] Cleaner format: [Boot #45] Reason
        logFile.print("[Boot #");
        logFile.print(bootCount);
        logFile.print("] ");
        logFile.println(msg);
        logFile.close();
    }
}

void dump_system_log()
{
    if (!LittleFS.exists(LOG_FILE))
    {
        LOG_PRINTLN("No logs found.");
        return;
    }
    LOG_PRINTLN("\n--- SYSTEM LOG HISTORY ---");
    File logFile = LittleFS.open(LOG_FILE, "r");
    if (logFile)
    {
        while (logFile.available())
        {
            if (ENABLE_WATCHDOG)
                esp_task_wdt_reset();
            TelnetLogger.write(logFile.read());
        }
        logFile.close();
    }
    LOG_PRINTLN("\n--------------------------\n");
}

void check_reset_reason()
{
    esp_reset_reason_t reason = esp_reset_reason();
    const char *reason_str = "Unknown";
    bool save_log = false;

    switch (reason)
    {
    case ESP_RST_POWERON:
        reason_str = "Power On";
        break;
    case ESP_RST_SW:
        reason_str = "Software Reset";
        break;
    case ESP_RST_PANIC:
        reason_str = "Crash/Panic (Error)";
        save_log = true;
        break;
    case ESP_RST_TASK_WDT:
        reason_str = "Task Watchdog (Hung Task)";
        save_log = true;
        break;
    case ESP_RST_WDT:
        reason_str = "Interrupt Watchdog";
        save_log = true;
        break;
    case ESP_RST_BROWNOUT:
        reason_str = "Brownout (Low Voltage)";
        save_log = true;
        break;
    default:
        break;
    }

    LOG_PRINTF("Boot #%u | Reason: %s\n", bootCount, reason_str);

    if (save_log)
    {
        log_system_error(reason_str);
        LOG_PRINTLN("!! Reset Logged !!");
    }
}

void save_params()
{
    LOG_PRINTLN("Saving config...");
    JsonDocument doc;
    doc["gate_travel_time"] = gate_travel_time;
    doc["mqtt_server"] = mqtt_server;
    doc["mqtt_port"] = mqtt_port_str;
    doc["mqtt_user"] = mqtt_user;
    doc["mqtt_password"] = mqtt_password;
    doc["rf_gate_open_code"] = rf_gate_open_code;
    doc["rf_gate_close_code"] = rf_gate_close_code;
    doc["rf_gate_stop_code"] = rf_gate_stop_code;
    doc["rf_gate_pos50_code"] = rf_gate_pos50_code;

    File configFile = LittleFS.open(PARAMS_FILE, "w");
    if (!configFile)
        return;
    serializeJson(doc, configFile);
    configFile.close();
}

void load_params()
{
    if (LittleFS.exists(PARAMS_FILE))
    {
        File configFile = LittleFS.open(PARAMS_FILE, "r");
        if (configFile)
        {
            JsonDocument doc;
            deserializeJson(doc, configFile);
            gate_travel_time = doc["gate_travel_time"] | gate_travel_time;
            strncpy(mqtt_server, doc["mqtt_server"] | mqtt_server, sizeof(mqtt_server));
            strncpy(mqtt_port_str, doc["mqtt_port"] | mqtt_port_str, sizeof(mqtt_port_str));
            strncpy(mqtt_user, doc["mqtt_user"] | mqtt_user, sizeof(mqtt_user));
            strncpy(mqtt_password, doc["mqtt_password"] | mqtt_password, sizeof(mqtt_password));
            rf_gate_open_code = doc["rf_gate_open_code"] | rf_gate_open_code;
            rf_gate_close_code = doc["rf_gate_close_code"] | rf_gate_close_code;
            rf_gate_stop_code = doc["rf_gate_stop_code"] | rf_gate_stop_code;
            rf_gate_pos50_code = doc["rf_gate_pos50_code"] | rf_gate_pos50_code;
            configFile.close();
        }
    }
    else
    {
        save_params();
    }
}

void send_command(GateCommand cmd, float pos = -1.0f)
{
    CommandMessage msg = {cmd, pos};
    xQueueSend(xCommandQueue, &msg, 0);
}

void log_io_status()
{
    char buf[128];
    int lim_open = digitalRead(LIMIT_OPEN_PIN);
    int lim_close = digitalRead(LIMIT_CLOSE_PIN);
    int photo = digitalRead(PHOTO_BARRIER_PIN);
    int light = digitalRead(RELAY_INDICATOR_LIGHT_PIN);
    int mot1 = 0;
    int mot2 = 0;

#if MOTOR_CONTROL_MODE == 1
    mot1 = digitalRead(RELAY_MOTOR_DIRECTION_PIN);
    mot2 = digitalRead(RELAY_MOTOR_ENABLE_PIN);
#elif MOTOR_CONTROL_MODE == 2
    mot1 = digitalRead(RELAY_MOTOR_OPEN_PIN);
    mot2 = digitalRead(RELAY_MOTOR_CLOSE_PIN);
#endif

    snprintf(buf, sizeof(buf),
             "[IO] OP:%d CL:%d PH:%d | M1:%d M2:%d LGT:%d | POS:%.2f",
             lim_open, lim_close, photo, mot1, mot2, light, current_position);
    LOG_PRINTLN(buf);
}

// =================================================================
// Button Callbacks
// =================================================================
void main_button_short_press(Button2 &btn) { send_command(CMD_STOP_USER); }
void main_button_long_press(Button2 &btn) { send_command(CMD_REVERSE); }
void maintenance_button_short_press(Button2 &btn) { send_command(CMD_RF_LEARN_SKIP); }

void maintenance_button_long_press(Button2 &btn)
{
    if (xSemaphoreTake(xStateMutex, 0) == pdTRUE)
    {
        if (millis() - rf_learn_start_time > 3000)
        {
            if (rf_learn_state != RF_LEARN_INACTIVE)
                send_command(CMD_RF_LEARN_SAVE_EXIT);
            else
                send_command(CMD_CALIBRATE_START);
        }
        xSemaphoreGive(xStateMutex);
    }
}
void wifi_button_short_press(Button2 &btn) {}
void wifi_config_long_press(Button2 &btn) { send_command(CMD_WIFI_CONFIG_START); }

// =================================================================
// HA & Logic
// =================================================================
void onCoverCommand(HACover::CoverCommand cmd, HACover *sender)
{
    switch (cmd)
    {
    case HACover::CommandOpen:
        send_command(CMD_OPEN);
        break;
    case HACover::CommandClose:
        send_command(CMD_CLOSE);
        break;
    case HACover::CommandStop:
        send_command(CMD_STOP_USER);
        break;
    }
}
void onOpenCommand(HAButton *sender) { send_command(CMD_OPEN); }
void onCloseCommand(HAButton *sender) { send_command(CMD_CLOSE); }
void onStopCommand(HAButton *sender) { send_command(CMD_STOP_USER); }
void onCalibrateCommand(HAButton *sender) { send_command(CMD_CALIBRATE_START); }
void onMoveTo50Command(HAButton *sender) { send_command(CMD_MOVE_TO_POSITION, 0.5f); }

void publish_all_states()
{
    cover.setCurrentPosition(current_position * 100);
    char buf[16];
    snprintf(buf, sizeof(buf), "%lu", gate_travel_time / 1000);
    travelTimeSensor.setValue(buf);

    if (cal_state != CAL_INACTIVE)
        gateState.setValue("calibrating");
    else if (rf_learn_state != RF_LEARN_INACTIVE)
        gateState.setValue("rf_learning");
    else if (current_operation == OPENING)
        gateState.setValue("opening");
    else if (current_operation == CLOSING)
        gateState.setValue("closing");
    else if (current_position == 1.0f)
        gateState.setValue("open");
    else if (current_position == 0.0f)
        gateState.setValue("closed");
    else
        gateState.setValue("stopped");
    gateIP.setValue(WiFi.localIP().toString().c_str());
}

void move_to_position(float new_target)
{
    if (cal_state != CAL_INACTIVE)
        return;
    if (current_operation != IDLE)
        stop_movement(false);
    if (abs(current_position - new_target) < 0.01)
        return;
    target_position = new_target;
    if (new_target > current_position)
        execute_open_sequence();
    else
        execute_close_sequence();
}

void handle_indicator_light()
{
    if (blink_interval == 0)
    {
        if (indicator_light_state)
        {
            digitalWrite(RELAY_INDICATOR_LIGHT_PIN, LOW);
            indicator_light_state = false;
        }
        return;
    }
    if (millis() - last_blink_time > blink_interval)
    {
        last_blink_time = millis();
        indicator_light_state = !indicator_light_state;
        digitalWrite(RELAY_INDICATOR_LIGHT_PIN, indicator_light_state);
    }
}

void handle_wifi_status()
{
    if (millis() - last_wifi_check > WIFI_RETRY_INTERVAL)
    {
        last_wifi_check = millis();
        if (WiFi.status() != WL_CONNECTED)
            LOG_PRINTLN("WiFi Reconnecting...");
    }
}

void handle_rf_learning()
{
    if (millis() - rf_learn_start_time > RF_LEARN_TIMEOUT)
    {
        LOG_PRINTLN("RF Learning timed out.");
        rf_learn_state = RF_LEARN_INACTIVE;
        blink_interval = 0;
        maintenanceButton.setLongClickTime(CALIBRATION_LONG_PRESS_TIME);
        return;
    }
    if (!mySwitch.available())
        return;
    unsigned long code = mySwitch.getReceivedValue();
    mySwitch.resetAvailable();
    if (code == 0)
        return;

    LOG_PRINTF("RF LEARN | Code: %lu\n", code);
    switch (rf_learn_state)
    {
    case RF_LEARN_WAIT_OPEN:
        rf_gate_open_code = code;
        LOG_PRINTLN("Learned OPEN. Press CLOSE...");
        rf_learn_state = RF_LEARN_WAIT_CLOSE;
        rf_learn_start_time = millis();
        break;
    case RF_LEARN_WAIT_CLOSE:
        rf_gate_close_code = code;
        LOG_PRINTLN("Learned CLOSE. Press STOP...");
        rf_learn_state = RF_LEARN_WAIT_STOP;
        rf_learn_start_time = millis();
        break;
    case RF_LEARN_WAIT_STOP:
        rf_gate_stop_code = code;
        LOG_PRINTLN("Learned STOP. Press 50%...");
        rf_learn_state = RF_LEARN_WAIT_POS50;
        rf_learn_start_time = millis();
        break;
    case RF_LEARN_WAIT_POS50:
        rf_gate_pos50_code = code;
        LOG_PRINTLN("Learned 50%. Saving...");
        save_params();
        rf_learn_state = RF_LEARN_INACTIVE;
        blink_interval = 0;
        maintenanceButton.setLongClickTime(CALIBRATION_LONG_PRESS_TIME);
        break;
    default:
        break;
    }
}

void handle_rf_signal()
{
    if (!mySwitch.available())
        return;
    unsigned long code = mySwitch.getReceivedValue();
    unsigned long now = millis();
    bool is_repeat = false;

    if (code == last_rf_code_received)
    {
        if (now - last_rf_process_time < RF_DEBOUNCE_DELAY)
            is_repeat = true;
    }
    last_rf_code_received = code;
    last_rf_process_time = now;
    mySwitch.resetAvailable();

    if (code == 0 || is_repeat)
        return;

    char code_str[20];
    sprintf(code_str, "%lu", last_rf_code_received);
    rfCodeSensor.setValue(code_str);
    LOG_PRINTF("RF RX: %lu\n", code);

    if (code == rf_gate_open_code)
        send_command(CMD_OPEN);
    else if (code == rf_gate_close_code)
        send_command(CMD_CLOSE);
    else if (code == rf_gate_stop_code)
        send_command(CMD_STOP_USER);
    else if (code == rf_gate_pos50_code)
        send_command(CMD_MOVE_TO_POSITION, 0.5f);
}

void handle_motor_relays()
{
#if MOTOR_CONTROL_MODE == 1
    if (motor_relay_state == R_OFF)
        return;
    if (millis() - motor_relay_timer >= MOTOR_DIRECTION_DELAY)
    {
        if (motor_relay_state == R_WAIT_ENABLE)
        {
            if (xSemaphoreTake(xMotorRelayMutex, pdMS_TO_TICKS(1)) == pdTRUE)
            {
                digitalWrite(RELAY_MOTOR_ENABLE_PIN, HIGH);
                xSemaphoreGive(xMotorRelayMutex);
            }
            motor_relay_state = R_OFF;
            LOG_PRINTLN("Motor: Enabled");
            movement_start_time = millis();
            movement_start_position = current_position;
        }
    }
#elif MOTOR_CONTROL_MODE == 2
    if (motor_change_state == M_WAIT_FOR_ENGAGE)
    {
        if (millis() - motor_change_timer >= MOTOR_DIRECTION_DELAY)
        {
            if (xSemaphoreTake(xMotorRelayMutex, pdMS_TO_TICKS(1)) == pdTRUE)
            {
                if (next_operation == OPENING)
                {
                    digitalWrite(RELAY_MOTOR_OPEN_PIN, HIGH);
                    LOG_PRINTLN("Motor: Opening");
                }
                else if (next_operation == CLOSING)
                {
                    digitalWrite(RELAY_MOTOR_CLOSE_PIN, HIGH);
                    LOG_PRINTLN("Motor: Closing");
                }
                xSemaphoreGive(xMotorRelayMutex);
            }
            motor_change_state = M_IDLE;
            next_operation = IDLE;
            movement_start_time = millis();
            movement_start_position = current_position;
        }
    }
#endif
}

void execute_open_sequence()
{
    if (current_operation == OPENING)
        return;
    current_operation = OPENING;
    if (WiFi.status() == WL_CONNECTED)
        cover.setState(HACover::StateOpening);
    blink_interval = BLINK_INTERVAL_OPENING;

    if (xSemaphoreTake(xMotorRelayMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
#if MOTOR_CONTROL_MODE == 1
        digitalWrite(RELAY_MOTOR_DIRECTION_PIN, LOW);
        motor_relay_timer = millis();
        motor_relay_state = R_WAIT_ENABLE;
#elif MOTOR_CONTROL_MODE == 2
        digitalWrite(RELAY_MOTOR_OPEN_PIN, LOW);
        digitalWrite(RELAY_MOTOR_CLOSE_PIN, LOW);
        motor_change_state = M_WAIT_FOR_ENGAGE;
        motor_change_timer = millis();
        next_operation = OPENING;
#endif
        xSemaphoreGive(xMotorRelayMutex);
    }
}

void execute_close_sequence()
{
    if (current_operation == CLOSING)
        return;
    current_operation = CLOSING;
    if (WiFi.status() == WL_CONNECTED)
        cover.setState(HACover::StateClosing);
    blink_interval = BLINK_INTERVAL_CLOSING;

    if (xSemaphoreTake(xMotorRelayMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
#if MOTOR_CONTROL_MODE == 1
        digitalWrite(RELAY_MOTOR_DIRECTION_PIN, HIGH);
        motor_relay_timer = millis();
        motor_relay_state = R_WAIT_ENABLE;
#elif MOTOR_CONTROL_MODE == 2
        digitalWrite(RELAY_MOTOR_OPEN_PIN, LOW);
        digitalWrite(RELAY_MOTOR_CLOSE_PIN, LOW);
        motor_change_state = M_WAIT_FOR_ENGAGE;
        motor_change_timer = millis();
        next_operation = CLOSING;
#endif
        xSemaphoreGive(xMotorRelayMutex);
    }
}

void start_opening()
{
    last_operation_before_stop = IDLE;
    if (current_operation != IDLE || cal_state != CAL_INACTIVE)
        return;
    if (current_position >= 0.99f || digitalRead(LIMIT_OPEN_PIN) == LOW)
    {
        LOG_PRINTLN("Cannot open: Limit.");
        return;
    }
    LOG_PRINTLN("CMD: OPEN");
    execute_open_sequence();
}

void start_closing()
{
    last_operation_before_stop = IDLE;
    if (current_operation != IDLE || cal_state != CAL_INACTIVE)
        return;
    if (current_position <= 0.01f || digitalRead(LIMIT_CLOSE_PIN) == LOW)
    {
        LOG_PRINTLN("Cannot close: Limit.");
        return;
    }
    if (digitalRead(PHOTO_BARRIER_PIN) == LOW)
    {
        LOG_PRINTLN("Cannot close: Sensor blocked.");
        return;
    }
    LOG_PRINTLN("CMD: CLOSE");
    execute_close_sequence();
}

void stop_movement(bool triggered_by_user)
{
    if (current_operation == IDLE && cal_state == CAL_INACTIVE && !auto_resume_is_armed && !resume_countdown_is_active)
        return;

    if (triggered_by_user && current_operation != IDLE)
    {
        if (current_position > 0.01f && current_position < 0.99f)
            last_operation_before_stop = current_operation;
        else
            last_operation_before_stop = IDLE;
    }

    if (xSemaphoreTake(xMotorRelayMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
#if MOTOR_CONTROL_MODE == 1
        digitalWrite(RELAY_MOTOR_ENABLE_PIN, LOW);
        digitalWrite(RELAY_MOTOR_DIRECTION_PIN, LOW);
        motor_relay_state = R_OFF;
#elif MOTOR_CONTROL_MODE == 2
        digitalWrite(RELAY_MOTOR_OPEN_PIN, LOW);
        digitalWrite(RELAY_MOTOR_CLOSE_PIN, LOW);
        motor_change_state = M_IDLE;
        next_operation = IDLE;
#endif
        xSemaphoreGive(xMotorRelayMutex);
    }

    blink_interval = 0;
    current_operation = IDLE;
    LOG_PRINTLN("STOP executed.");

    if (WiFi.status() == WL_CONNECTED)
    {
        if (current_position >= 0.99f)
            cover.setState(HACover::StateOpen);
        else if (current_position <= 0.01f)
            cover.setState(HACover::StateClosed);
        else
            cover.setState(HACover::StateStopped);
    }
    cover.setCurrentPosition(current_position * 100);

    if (triggered_by_user)
    {
        target_position = -1.0;
        if (cal_state != CAL_INACTIVE)
            cal_state = CAL_INACTIVE;
        auto_resume_is_armed = false;
        resume_countdown_is_active = false;
    }
}

void update_gate_position()
{
#if MOTOR_CONTROL_MODE == 1
    if (motor_relay_state != R_OFF)
        return;
#elif MOTOR_CONTROL_MODE == 2
    if (motor_change_state != M_IDLE)
        return;
#endif

    unsigned long elapsed = millis() - movement_start_time;
    float ratio = (gate_travel_time > 0) ? (float(elapsed) / float(gate_travel_time)) : 1.0f;

    if (current_operation == OPENING)
        current_position = movement_start_position + ratio;
    else if (current_operation == CLOSING)
        current_position = movement_start_position - ratio;

    current_position = constrain(current_position, 0.0f, 1.0f);

    if (target_position >= 0.0)
    {
        if ((current_operation == OPENING && current_position >= target_position) ||
            (current_operation == CLOSING && current_position <= target_position))
        {
            LOG_PRINTF("Target %.2f reached.\n", target_position);
            stop_movement(false);
            current_position = target_position;
            target_position = -1.0;
        }
    }
    if (gate_travel_time > 0 && elapsed > (gate_travel_time + (gate_travel_time / 10)))
    {
        LOG_PRINTLN("Error: Gate movement timed out!");
        stop_movement(false);
        target_position = -1.0;
    }
}

void handle_safety_sensors()
{
    if (current_operation == OPENING && digitalRead(LIMIT_OPEN_PIN) == LOW)
    {
        LOG_PRINTLN("Limit: OPEN reached");
        stop_movement(false);
        current_position = 1.0f;
        target_position = -1.0;
    }
    if (current_operation == CLOSING && digitalRead(LIMIT_CLOSE_PIN) == LOW)
    {
        LOG_PRINTLN("Limit: CLOSE reached");
        stop_movement(false);
        current_position = 0.0f;
        target_position = -1.0;
    }

    if (digitalRead(PHOTO_BARRIER_PIN) == LOW && current_operation == CLOSING)
    {
        LOG_PRINTLN("Photo Barrier! Reversing.");
        stop_movement(false);
        auto_resume_is_armed = true;
        target_position = -1.0;
        execute_open_sequence();
    }
}

void start_calibration()
{
    if (current_operation != IDLE || cal_state != CAL_INACTIVE || rf_learn_state != RF_LEARN_INACTIVE)
        return;
    LOG_PRINTLN("--- Starting Calibration ---");
    cal_state = CAL_CLOSING_TO_START;
    blink_interval = CALIBRATION_BLINK_INTERVAL;
}

void handle_calibration()
{
    switch (cal_state)
    {
    case CAL_CLOSING_TO_START:
        if (digitalRead(LIMIT_CLOSE_PIN) == LOW)
        {
            cal_state = CAL_OPENING_FOR_TIMING;
            movement_start_time = millis();
            execute_open_sequence();
        }
        else
        {
            execute_close_sequence();
        }
        break;
    case CAL_OPENING_FOR_TIMING:
        if (digitalRead(LIMIT_OPEN_PIN) == LOW)
        {
            stop_movement(false);
            gate_travel_time = millis() - movement_start_time;
            LOG_PRINTF("Calibration Done. Time: %lu ms\n", gate_travel_time);
            save_params();
            cal_state = CAL_DONE;
        }
        break;
    case CAL_DONE:
        cal_state = CAL_INACTIVE;
        blink_interval = 0;
        publish_all_states();
        break;
    default:
        break;
    }
}

// =================================================================
// Tasks
// =================================================================

void vGateStateTask(void *pvParameters)
{
    if (ENABLE_WATCHDOG)
        esp_task_wdt_add(NULL);
    CommandMessage msg;
    while (1)
    {
        if (ENABLE_WATCHDOG)
            esp_task_wdt_reset();
        if (xQueueReceive(xCommandQueue, &msg, pdMS_TO_TICKS(100)) == pdPASS)
        {
            if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
            {
                if (msg.cmd == CMD_WIFI_CONFIG_START)
                    xTaskCreate(vConfigPortalTask, "ConfigPortal", 6144, NULL, 0, NULL);
                else if (msg.cmd == CMD_RF_LEARN_SAVE_EXIT)
                {
                    LOG_PRINTLN("Saving codes.");
                    save_params();
                    rf_learn_state = RF_LEARN_INACTIVE;
                    blink_interval = 0;
                    maintenanceButton.setLongClickTime(CALIBRATION_LONG_PRESS_TIME);
                }
                else if (cal_state == CAL_INACTIVE && rf_learn_state == RF_LEARN_INACTIVE)
                {
                    switch (msg.cmd)
                    {
                    case CMD_OPEN:
                        start_opening();
                        break;
                    case CMD_CLOSE:
                        start_closing();
                        break;
                    case CMD_STOP_USER:
                        if (current_operation != IDLE)
                            stop_movement(true);
                        else if (last_operation_before_stop != IDLE)
                        {
                            if (last_operation_before_stop == OPENING)
                                start_opening();
                            else
                                start_closing();
                        }
                        else if (current_position >= 0.99f)
                            start_closing();
                        else
                            start_opening();
                        break;
                    case CMD_REVERSE:
                        if (current_operation == OPENING || (current_operation == IDLE && current_position >= 0.99f))
                            execute_close_sequence();
                        else
                            execute_open_sequence();
                        break;
                    case CMD_CALIBRATE_START:
                        start_calibration();
                        break;
                    case CMD_RF_LEARN_START:
                        rf_learn_state = RF_LEARN_WAIT_OPEN;
                        rf_learn_start_time = millis();
                        blink_interval = RF_LEARN_BLINK_INTERVAL;
                        maintenanceButton.setLongClickTime(RF_LEARN_SAVE_LONG_PRESS_TIME);
                        LOG_PRINTLN("--- RF Learning Mode ---");
                        break;
                    case CMD_MOVE_TO_POSITION:
                        move_to_position(msg.position);
                        break;
                    case CMD_STOP_INTERNAL:
                        stop_movement(false);
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

void vMotorControlTask(void *pvParameters)
{
    if (ENABLE_WATCHDOG)
        esp_task_wdt_add(NULL);
    const TickType_t xFrequency = pdMS_TO_TICKS(5);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        if (ENABLE_WATCHDOG)
            esp_task_wdt_reset();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(1)) == pdTRUE)
        {
            handle_safety_sensors();
            handle_motor_relays();
            if (current_operation != IDLE)
                update_gate_position();
            if (cal_state != CAL_INACTIVE)
                handle_calibration();
            xSemaphoreGive(xStateMutex);
        }
    }
}

void vInputTask(void *pvParameters)
{
    if (ENABLE_WATCHDOG)
        esp_task_wdt_add(NULL);
    const TickType_t xFrequency = pdMS_TO_TICKS(10);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    static unsigned long last_wifi_print_sec = 999;
    static unsigned long last_maint_print_sec = 999;
    static unsigned long last_combo_print_sec = 999;

    static unsigned long wifi_press_start = 0;
    static unsigned long maint_press_start = 0;
    static unsigned long combo_press_start = 0;
    static bool combo_triggered = false;

    while (1)
    {
        if (ENABLE_WATCHDOG)
            esp_task_wdt_reset();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        mainButton.loop();
        maintenanceButton.loop();
        wifiButton.loop();

        bool wifi_pressed = wifiButton.isPressed();
        bool maint_pressed = maintenanceButton.isPressed();

        // Combo Press
        if (wifi_pressed && maint_pressed)
        {
            if (combo_press_start == 0)
                combo_press_start = millis();
            unsigned long held = millis() - combo_press_start;
            if (held < COMBO_MODE_HOLD_TIME)
            {
                unsigned long rem = (COMBO_MODE_HOLD_TIME - held) / 1000;
                if (rem != last_combo_print_sec)
                {
                    last_combo_print_sec = rem;
                    LOG_PRINTF("Enter RF Learn in %lu s...\n", rem + 1);
                }
            }
            else if (!combo_triggered)
            {
                LOG_PRINTLN("Combo: Entering RF Learn Mode!");
                send_command(CMD_RF_LEARN_START);
                combo_triggered = true;
            }
            wifi_press_start = 0;
            maint_press_start = 0;
        }
        else
        {
            combo_press_start = 0;
            combo_triggered = false;
            last_combo_print_sec = 999;

            // WiFi Button
            if (wifi_pressed)
            {
                if (wifi_press_start == 0)
                    wifi_press_start = millis();
                unsigned long held = millis() - wifi_press_start;
                if (held < WIFI_CONFIG_LONG_PRESS_TIME)
                {
                    unsigned long rem = (WIFI_CONFIG_LONG_PRESS_TIME - held) / 1000;
                    if (rem != last_wifi_print_sec)
                    {
                        last_wifi_print_sec = rem;
                        LOG_PRINTF("WiFi Config in %lu s...\n", rem + 1);
                    }
                }
            }
            else
            {
                wifi_press_start = 0;
                last_wifi_print_sec = 999;
            }

            // Maintenance Button
            if (maint_pressed)
            {
                if (maint_press_start == 0)
                    maint_press_start = millis();
                unsigned long held = millis() - maint_press_start;
                unsigned long target = (rf_learn_state != RF_LEARN_INACTIVE) ? RF_LEARN_SAVE_LONG_PRESS_TIME : CALIBRATION_LONG_PRESS_TIME;
                if (held < target)
                {
                    unsigned long rem = (target - held) / 1000;
                    if (rem != last_maint_print_sec)
                    {
                        last_maint_print_sec = rem;
                        if (rf_learn_state != RF_LEARN_INACTIVE)
                            LOG_PRINTF("Saving RF in %lu s...\n", rem + 1);
                        else
                            LOG_PRINTF("Calibration in %lu s...\n", rem + 1);
                    }
                }
            }
            else
            {
                maint_press_start = 0;
                last_maint_print_sec = 999;
            }
        }

        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(1)) == pdTRUE)
        {
            if (rf_learn_state != RF_LEARN_INACTIVE)
                handle_rf_learning();
            else
                handle_rf_signal();
            handle_indicator_light();
            xSemaphoreGive(xStateMutex);
        }
    }
}

void vNetworkTask(void *pvParameters)
{
    if (ENABLE_WATCHDOG)
        esp_task_wdt_add(NULL);
    const TickType_t xFrequency = pdMS_TO_TICKS(500);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    int logCounter = 0;

    while (1)
    {
        if (ENABLE_WATCHDOG)
            esp_task_wdt_reset();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        if (WiFi.status() == WL_CONNECTED)
        {
            mqtt.loop();
            if (telnetServer.hasClient())
            {
                if (!telnetClient || !telnetClient.connected())
                {
                    if (telnetClient)
                        telnetClient.stop();
                    telnetClient = telnetServer.accept();
                    LOG_PRINTLN("Telnet connected!");
                }
            }
            if (telnetClient && telnetClient.connected() && telnetClient.available())
            {
                String cmd = telnetClient.readStringUntil('\n');
                cmd.trim();
                if (cmd.equalsIgnoreCase("logs"))
                    dump_system_log();
                else if (cmd.equalsIgnoreCase("clear_logs"))
                {
                    LittleFS.remove(LOG_FILE);
                    LOG_PRINTLN("Logs cleared.");
                }
                else if (cmd.equalsIgnoreCase("restart"))
                {
                    LOG_PRINTLN("Restarting...");
                    delay(500);
                    ESP.restart();
                }
                else if (cmd.equalsIgnoreCase("crash")) {
                    LOG_PRINTLN("Force crashing! Watchdog should reset in 10s...");
                    delay(100); 
                    
                    // Infinite loop that blocks EVERYTHING and never sleeps
                    while(true) {
                        // Do NOTHING. Do not delay. Do not reset WDT.
                        // This starves the CPU and triggers the hardware watchdog.
                    }
                }
            }
        }
        handle_wifi_status();
        logCounter++;
        if (logCounter >= 10)
        {
            logCounter = 0;
            if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
            {
                log_io_status();
                publish_all_states();
                xSemaphoreGive(xStateMutex);
            }
        }
    }
}

void vConfigPortalTask(void *pvParameters)
{
    if (hNetworkTask != NULL)
        vTaskSuspend(hNetworkTask);
    LOG_PRINTLN("Starting Config Portal...");
    wm.setConfigPortalTimeout(180);
    wm.startConfigPortal("GateControllerAP");
    LOG_PRINTLN("Restarting...");
    delay(1000);
    ESP.restart();
    vTaskDelete(NULL);
}

// =================================================================
// Setup
// =================================================================
void setup()
{
    xLogMutex = xSemaphoreCreateMutex();
    xStateMutex = xSemaphoreCreateMutex();
    xMotorRelayMutex = xSemaphoreCreateMutex();
    xCommandQueue = xQueueCreate(10, sizeof(CommandMessage));

    Serial.begin(115200);
    delay(1000);
    LOG_PRINTLN("\nSliding Gate Controller Starting...");

    // Init Preferences for Boot Count
    preferences.begin("gate_stats", false);
    bootCount = preferences.getUInt("boot_count", 0);
    bootCount++;
    preferences.putUInt("boot_count", bootCount);
    preferences.end();

    if (ENABLE_WATCHDOG)
    {
        esp_task_wdt_deinit();
        esp_task_wdt_config_t twdt_config = {
            .timeout_ms = WDT_TIMEOUT_SECONDS * 1000,
            .idle_core_mask = (1 << 0),
            .trigger_panic = true};
        esp_task_wdt_init(&twdt_config);
        esp_task_wdt_add(NULL);
        LOG_PRINTLN("Watchdog Enabled (10s).");
    }

    if (!LittleFS.begin(true))
        LOG_PRINTLN("LittleFS Failed!");

    check_reset_reason();
    dump_system_log();
    load_params();

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
    pinMode(RF_RECEIVER_PIN, INPUT);

    byte mac[6];
    WiFi.macAddress(mac);
    device.setUniqueId(mac, sizeof(mac));
    device.setName("Sliding Gate");
    device.setModel("ESP32-C3");
    cover.setName("Sliding Gate");
    cover.setDeviceClass("gate");
    cover.onCommand(onCoverCommand);
    openButtonHA.onCommand(onOpenCommand);
    closeButtonHA.onCommand(onCloseCommand);
    stopButtonHA.onCommand(onStopCommand);
    calibrateButton.onCommand(onCalibrateCommand);
    moveTo50Button.onCommand(onMoveTo50Command);

    mainButton.begin(MANUAL_MAIN_BUTTON_PIN, INPUT_PULLUP, true);
    mainButton.setReleasedHandler(main_button_short_press);
    mainButton.setLongClickHandler(main_button_long_press);
    mainButton.setLongClickTime(REVERSE_LONG_PRESS_TIME);

    maintenanceButton.begin(MANUAL_MAINTENANCE_BUTTON_PIN, INPUT_PULLUP, true);
    maintenanceButton.setReleasedHandler(maintenance_button_short_press);
    maintenanceButton.setLongClickHandler(maintenance_button_long_press);
    maintenanceButton.setLongClickTime(CALIBRATION_LONG_PRESS_TIME);

    wifiButton.begin(MANUAL_WIFI_BUTTON_PIN, INPUT_PULLUP, true);
    wifiButton.setReleasedHandler(wifi_button_short_press);
    wifiButton.setLongClickHandler(wifi_config_long_press);
    wifiButton.setLongClickTime(WIFI_CONFIG_LONG_PRESS_TIME);

    wm.addParameter(&custom_mqtt_server);
    wm.addParameter(&custom_mqtt_port);
    wm.addParameter(&custom_mqtt_user);
    wm.addParameter(&custom_mqtt_password);

    mySwitch.enableReceive(RF_RECEIVER_PIN);
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin();

    uint16_t port = atoi(mqtt_port_str);
    mqtt.begin(mqtt_server, port, mqtt_user, mqtt_password);
    telnetServer.begin();

    xTaskCreate(vMotorControlTask, "MotorControl", 4096, NULL, 4, NULL);
    xTaskCreate(vInputTask, "InputReader", 3072, NULL, 3, NULL);
    xTaskCreate(vGateStateTask, "GateStateManager", 4096, NULL, 2, NULL);
    xTaskCreate(vNetworkTask, "NetworkManager", 4096, NULL, 1, &hNetworkTask);

    LOG_PRINTLN("Scheduler Started.");
}

void loop()
{
    if (ENABLE_WATCHDOG)
        esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(1000));
}