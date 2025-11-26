#include <Arduino.h>
#include <RCSwitch.h>
#include <Button2.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <ArduinoHA.h>
#include <LittleFS.h>
#include <stdlib.h> // For atoi
#include <ArduinoJson.h>

#include "SerialMirror.hpp" 

// --- FreeRTOS Includes and Definitions ---
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

// --- Command Structure for Inter-Task Communication (ITC) ---
enum GateCommand { 
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

typedef struct {
    GateCommand cmd;
    float position; // Used for CMD_MOVE_TO_POSITION
} CommandMessage;


// --- RTOS Global Objects ---
QueueHandle_t xCommandQueue; // Input/Network -> State Manager
SemaphoreHandle_t xStateMutex; // Protects shared state variables (position, current_operation, states)
SemaphoreHandle_t xMotorRelayMutex; // Protects motor GPIO pins

// =================================================================
// Pin and Constant Definitions (Original)
// =================================================================
#define MOTOR_CONTROL_MODE 2 

// ——— Pin definitions ———
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

const int MOVEMENT_INHIBIT_PIN = 8;


const int LIMIT_OPEN_PIN = 10;
const int LIMIT_CLOSE_PIN = 20;


const int PHOTO_BARRIER_PIN = 2;


const int RF_RECEIVER_PIN = 5;

// ——— Timing constants ———
unsigned long MOTOR_DIRECTION_DELAY = 700;
unsigned long OBSTACLE_CLEAR_RESUME_DELAY = 8000;
unsigned long CALIBRATION_LONG_PRESS_TIME = 8000;
unsigned long WIFI_CONFIG_LONG_PRESS_TIME = 5000; 
unsigned long WIFI_RETRY_INTERVAL = 30000;
const unsigned long REVERSE_LONG_PRESS_TIME = 1000; 
const unsigned long RF_LEARN_TIMEOUT = 30000; 
const unsigned long RF_LEARN_SAVE_LONG_PRESS_TIME = 1500; 


// Persistent parameters (Defaults)
unsigned long gate_travel_time = 30000;
char mqtt_server[40] = "192.168.1.12";
char mqtt_port_str[6] = "1883";
char mqtt_user[32] = "admin";
char mqtt_password[64] = "admin";
unsigned long rf_gate_open_code = 1234567;
unsigned long rf_gate_close_code = 7654321;
unsigned long rf_gate_stop_code = 1111111;
unsigned long rf_gate_pos50_code = 2222222;

// --- WiFiManager Custom Parameters ---
WiFiManagerParameter custom_mqtt_server("server", "MQTT Server", mqtt_server, sizeof(mqtt_server));
WiFiManagerParameter custom_mqtt_port("port", "MQTT Port", mqtt_port_str, sizeof(mqtt_port_str));
WiFiManagerParameter custom_mqtt_user("user", "MQTT User", mqtt_user, sizeof(mqtt_user));
WiFiManagerParameter custom_mqtt_password("password", "MQTT Password", mqtt_password, sizeof(mqtt_password));

// --- Indicator light state ---
unsigned long last_blink_time = 0;
unsigned long blink_interval = 0; 
bool indicator_light_state = false;

// --- LittleFS storage helpers ---
#define PARAMS_FILE "/gate_params_data.txt"

#define BLINK_INTERVAL_OPENING 1000      
#define BLINK_INTERVAL_CLOSING 500       
#define CALIBRATION_BLINK_INTERVAL 100 
#define RF_LEARN_BLINK_INTERVAL 250 

// =================================================================
// Global State Variables (Accessed via xStateMutex)
// =================================================================
enum CoverOperation { IDLE, OPENING, CLOSING };
enum CalibrationState { CAL_INACTIVE, CAL_CLOSING_TO_START, CAL_OPENING_FOR_TIMING, CAL_DONE };
enum RFLearningState { RF_LEARN_INACTIVE, RF_LEARN_WAIT_OPEN, RF_LEARN_WAIT_CLOSE, RF_LEARN_WAIT_STOP, RF_LEARN_WAIT_POS50 };

#if MOTOR_CONTROL_MODE == 1
enum MotorRelayState { R_OFF, R_WAIT_ENABLE };
#elif MOTOR_CONTROL_MODE == 2
enum MotorChangeState { M_IDLE, M_WAIT_FOR_ENGAGE };
#endif

// Shared State Variables
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


// ——— Library Objects ———
RCSwitch mySwitch;
Button2 mainButton;        
Button2 maintenanceButton; 
Button2 wifiButton;        
WiFiManager wm;
WiFiServer telnetServer(23);

// 2) ArduinoHA objects
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


// ——— Function Forward Declarations ———
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
void vConfigPortalTask(void *pvParameters); // <--- ADD THIS LINE
// =================================================================
// Helper Functions (Save/Load Params) - Mutex Free (Runs before RTOS Start)
// =================================================================

void save_params()
{
  LOG_PRINTLN("Saving configuration to LittleFS...");
  JsonDocument doc;

  doc["gate_travel_time"] = gate_travel_time;
  doc["mqtt_server"] = mqtt_server;
  doc["mqtt_port"] = mqtt_port_str;
  doc["mqtt_user"] = mqtt_user;
  doc["mqtt_password"] = mqtt_password;
  
  // --- NEW: Save learned RF codes ---
  doc["rf_gate_open_code"] = rf_gate_open_code;
  doc["rf_gate_close_code"] = rf_gate_close_code;
  doc["rf_gate_stop_code"] = rf_gate_stop_code;
  doc["rf_gate_pos50_code"] = rf_gate_pos50_code;

  File configFile = LittleFS.open(PARAMS_FILE, "w");
  if (!configFile)
  {
    LOG_PRINTLN("Failed to open config file for writing");
    return;
  }

  if (serializeJson(doc, configFile) == 0)
  {
    LOG_PRINTLN("Failed to write to config file");
  }
  else
  {
    LOG_PRINTLN("Configuration saved successfully.");
  }
  configFile.close();
}

void load_params()
{
  if (LittleFS.exists(PARAMS_FILE))
  {
    LOG_PRINTLN("Reading configuration from LittleFS...");
    File configFile = LittleFS.open(PARAMS_FILE, "r");
    if (configFile)
    {
      JsonDocument doc;
      DeserializationError error = deserializeJson(doc, configFile);
      if (error)
      {
        LOG_PRINT(F("deserializeJson() failed: "));
        LOG_PRINTLN(error.c_str());
        return;
      }

      gate_travel_time = doc["gate_travel_time"] | gate_travel_time;
      strncpy(mqtt_server, doc["mqtt_server"] | mqtt_server, sizeof(mqtt_server));
      strncpy(mqtt_port_str, doc["mqtt_port"] | mqtt_port_str, sizeof(mqtt_port_str));
      strncpy(mqtt_user, doc["mqtt_user"] | mqtt_user, sizeof(mqtt_user));
      strncpy(mqtt_password, doc["mqtt_password"] | mqtt_password, sizeof(mqtt_password));

      // --- NEW: Load learned RF codes, with fallback to defaults ---
      rf_gate_open_code = doc["rf_gate_open_code"] | rf_gate_open_code;
      rf_gate_close_code = doc["rf_gate_close_code"] | rf_gate_close_code;
      rf_gate_stop_code = doc["rf_gate_stop_code"] | rf_gate_stop_code;
      rf_gate_pos50_code = doc["rf_gate_pos50_code"] | rf_gate_pos50_code;

      LOG_PRINTLN("Configuration loaded:");
      LOG_PRINTF(" - Travel Time: %lu\n", gate_travel_time);
      LOG_PRINTF(" - MQTT Server: %s\n", mqtt_server);
      LOG_PRINTF(" - MQTT Port: %s\n", mqtt_port_str);
      LOG_PRINTF(" - MQTT User: %s\n", mqtt_user);
      LOG_PRINTLN("Loaded RF Codes:");
      LOG_PRINTF(" - Open: %lu\n", rf_gate_open_code);
      LOG_PRINTF(" - Close: %lu\n", rf_gate_close_code);
      LOG_PRINTF(" - Stop: %lu\n", rf_gate_stop_code);
      LOG_PRINTF(" - Pos50: %lu\n", rf_gate_pos50_code);


      configFile.close();
    }
  }
  else
  {
    LOG_PRINTLN("No configuration file found, using default values.");
    // Optional: save defaults on first boot
    save_params();
  }
}

// =================================================================
// Command Helpers (Used by Input/HA callbacks)
// =================================================================

// Helper to safely send command
void send_command(GateCommand cmd, float pos = -1.0f) {
    CommandMessage msg = {cmd, pos};
    // Send command without blocking (timeout=0)
    if (xQueueSend(xCommandQueue, &msg, 0) != pdPASS) {
        LOG_PRINTLN("Warning: Command queue full. Dropping command.");
    }
}

// =================================================================
// Button Callbacks (Modified to send commands to Queue)
// =================================================================

void main_button_short_press(Button2 &btn)
{
    send_command(CMD_STOP_USER); // User stop/pause/resume logic is handled by the State Manager
}

void main_button_long_press(Button2 &btn)
{
    send_command(CMD_REVERSE); // Reverse logic is handled by the State Manager
}

void maintenance_button_short_press(Button2 &btn) {
    send_command(CMD_RF_LEARN_SKIP);
}

void maintenance_button_long_press(Button2 &btn) {
    // Check if we are in RF Learn mode (read-only access is safe, but we'll use mutex anyway for consistency)
    if (xSemaphoreTake(xStateMutex, 0) == pdTRUE) {
        if (rf_learn_state != RF_LEARN_INACTIVE) {
            send_command(CMD_RF_LEARN_SAVE_EXIT);
        } else {
            send_command(CMD_CALIBRATE_START);
        }
        xSemaphoreGive(xStateMutex);
    }
}

void wifi_button_short_press(Button2 &btn) {
    send_command(CMD_RF_LEARN_START);
}

void wifi_config_long_press(Button2 &btn)
{
    // Sends the command to start the Config Portal Task
    send_command(CMD_WIFI_CONFIG_START);
}

// =================================================================
// Home Assistant Command Callbacks (Modified to send commands to Queue)
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

// =================================================================
// Core Logic Functions (Modified to use Mutexes)
// =================================================================

void publish_all_states()
{
    // MUST be called only after acquiring xStateMutex
    cover.setCurrentPosition(current_position * 100);

    // publish the travel-time (in whole seconds)
    unsigned long travelSeconds = gate_travel_time / 1000;
    char buf[16];
    snprintf(buf, sizeof(buf), "%lu", travelSeconds);
    travelTimeSensor.setValue(buf);

    // Set gateState based on enums
    if (cal_state != CAL_INACTIVE) {
        gateState.setValue("calibrating");
    } else if (rf_learn_state != RF_LEARN_INACTIVE) {
        gateState.setValue("rf_learning");
    } else if (current_operation == OPENING) {
        gateState.setValue("opening");
    } else if (current_operation == CLOSING) {
        gateState.setValue("closing");
    } else if (current_position == 1.0f) {
        gateState.setValue("open");
    } else if (current_position == 0.0f) {
        gateState.setValue("closed");
    } else {
        gateState.setValue("stopped");
    }
    // No need for mutex around WiFi.localIP() as it's library internal
    gateIP.setValue(WiFi.localIP().toString().c_str());
}

void move_to_position(float new_target)
{
    // Called by the Gate State Task after acquiring xStateMutex
    if (cal_state != CAL_INACTIVE) {
        LOG_PRINTLN("Cannot move to position: Calibration in progress.");
        return;
    }

    if (current_operation != IDLE) {
        LOG_PRINTLN("Gate is currently moving, stopping first...");
        stop_movement(false); 
    }

    if (abs(current_position - new_target) < 0.01) {
        LOG_PRINTLN("Already at target position.");
        return;
    }

    target_position = new_target;

    if (new_target > current_position) {
        LOG_PRINTF("Moving to position %.2f by OPENING.\n", target_position);
        execute_open_sequence(); // Direct execution, state check passed
    } else {
        LOG_PRINTF("Moving to position %.2f by CLOSING.\n", target_position);
        execute_close_sequence(); // Direct execution, state check passed
    }
}

void handle_indicator_light()
{
    // No shared variable access, safe to run directly
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
    // No shared variable access, safe to run directly
    if (millis() - last_wifi_check > WIFI_RETRY_INTERVAL)
    {
        last_wifi_check = millis();
        if (WiFi.status() != WL_CONNECTED)
        {
            LOG_PRINTLN("WiFi: Not connected. Trying to reconnect automatically...");
        }
    }
}

void handle_rf_learning() {
    // Called by the Input Task after acquiring xStateMutex

    if (millis() - rf_learn_start_time > RF_LEARN_TIMEOUT) {
        LOG_PRINTLN("RF Learning timed out. Exiting.");
        rf_learn_state = RF_LEARN_INACTIVE;
        blink_interval = 0;
        maintenanceButton.setLongClickTime(CALIBRATION_LONG_PRESS_TIME);
        return;
    }

    if (!mySwitch.available()) return;

    unsigned long code = mySwitch.getReceivedValue();
    mySwitch.resetAvailable();
    if (code == 0) return;

    LOG_PRINTF("Received potential RF code: %lu\n", code);

    switch (rf_learn_state) {
        case RF_LEARN_WAIT_OPEN:
            rf_gate_open_code = code;
            LOG_PRINTF("=> OPEN code learned: %lu\n", rf_gate_open_code);
            LOG_PRINTLN("Now press the desired CLOSE button on your remote...");
            rf_learn_state = RF_LEARN_WAIT_CLOSE;
            rf_learn_start_time = millis(); 
            break;

        case RF_LEARN_WAIT_CLOSE:
            rf_gate_close_code = code;
            LOG_PRINTF("=> CLOSE code learned: %lu\n", rf_gate_close_code);
            LOG_PRINTLN("Now press the desired STOP button on your remote...");
            rf_learn_state = RF_LEARN_WAIT_STOP;
            rf_learn_start_time = millis(); 
            break;

        case RF_LEARN_WAIT_STOP:
            rf_gate_stop_code = code;
            LOG_PRINTF("=> STOP code learned: %lu\n", rf_gate_stop_code);
            LOG_PRINTLN("Now press the desired 50% POSITION button on your remote...");
            rf_learn_state = RF_LEARN_WAIT_POS50;
            rf_learn_start_time = millis(); 
            break;

        case RF_LEARN_WAIT_POS50:
            rf_gate_pos50_code = code;
            LOG_PRINTF("=> 50% POSITION code learned: %lu\n", rf_gate_pos50_code);
            LOG_PRINTLN("--- RF Learning Complete! ---");
            save_params(); 
            rf_learn_state = RF_LEARN_INACTIVE;
            blink_interval = 0; 
            maintenanceButton.setLongClickTime(CALIBRATION_LONG_PRESS_TIME);
            break;

        case RF_LEARN_INACTIVE:
            break;
    }
}


void handle_rf_signal()
{
  // Called by Input Task after acquiring xStateMutex

  if (!mySwitch.available()) return;
  
  unsigned long code = mySwitch.getReceivedValue();
  mySwitch.resetAvailable();
  if (code == 0) return;

  last_rf_code_received = code;
  char code_str[20];
  sprintf(code_str, "%lu", last_rf_code_received);
  rfCodeSensor.setValue(code_str);
  LOG_PRINTF("RF code received: %lu\n", code);
  
  // Instead of calling start/stop directly, send a command to the Queue
  if (code == rf_gate_open_code) {
      send_command(CMD_OPEN);
  } else if (code == rf_gate_close_code) {
      send_command(CMD_CLOSE);
  } else if (code == rf_gate_stop_code) {
      send_command(CMD_STOP_USER);
  } else if (code == rf_gate_pos50_code) {
      LOG_PRINTLN("RF command: Move to 50%");
      send_command(CMD_MOVE_TO_POSITION, 0.5f);
  }
}

void handle_motor_relays()
{
    // Called by Motor Control Task after acquiring xStateMutex
#if MOTOR_CONTROL_MODE == 1
    if (motor_relay_state == R_OFF) return;
    if (millis() - motor_relay_timer >= MOTOR_DIRECTION_DELAY)
    {
        if (motor_relay_state == R_WAIT_ENABLE)
        {
            if (xSemaphoreTake(xMotorRelayMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
                digitalWrite(RELAY_MOTOR_ENABLE_PIN, HIGH);
                xSemaphoreGive(xMotorRelayMutex);
            }
            motor_relay_state = R_OFF;
            LOG_PRINTLN("Motor Enabled.");
            movement_start_time = millis();
            movement_start_position = current_position;
        }
    }
#elif MOTOR_CONTROL_MODE == 2
    if (motor_change_state == M_WAIT_FOR_ENGAGE)
    {
        if (millis() - motor_change_timer >= MOTOR_DIRECTION_DELAY)
        {
            if (xSemaphoreTake(xMotorRelayMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
                if (next_operation == OPENING) {
                    LOG_PRINTLN("Motor Opening (Mode 2) after delay.");
                    digitalWrite(RELAY_MOTOR_OPEN_PIN, HIGH);
                } else if (next_operation == CLOSING) {
                    LOG_PRINTLN("Motor Closing (Mode 2) after delay.");
                    digitalWrite(RELAY_MOTOR_CLOSE_PIN, HIGH);
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
    // Called by Gate State Task after acquiring xStateMutex
    if (current_operation == OPENING) return;
    
    current_operation = OPENING;
    if (WiFi.status() == WL_CONNECTED) { cover.setState(HACover::StateOpening); }
    blink_interval = BLINK_INTERVAL_OPENING; 

    if (xSemaphoreTake(xMotorRelayMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
#if MOTOR_CONTROL_MODE == 1
        digitalWrite(RELAY_MOTOR_DIRECTION_PIN, LOW);
        motor_relay_timer = millis();
        motor_relay_state = R_WAIT_ENABLE;
#elif MOTOR_CONTROL_MODE == 2
        digitalWrite(RELAY_MOTOR_OPEN_PIN, LOW);
        digitalWrite(RELAY_MOTOR_CLOSE_PIN, LOW);
        LOG_PRINTLN("Motor Opening (Mode 2), waiting for relay delay.");
        motor_change_state = M_WAIT_FOR_ENGAGE;
        motor_change_timer = millis();
        next_operation = OPENING;
#endif
        xSemaphoreGive(xMotorRelayMutex);
    }
}

void execute_close_sequence()
{
    // Called by Gate State Task after acquiring xStateMutex
    if (current_operation == CLOSING) return;

    current_operation = CLOSING;
    if (WiFi.status() == WL_CONNECTED) { cover.setState(HACover::StateClosing); }
    blink_interval = BLINK_INTERVAL_CLOSING; 

    if (xSemaphoreTake(xMotorRelayMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
#if MOTOR_CONTROL_MODE == 1
        digitalWrite(RELAY_MOTOR_DIRECTION_PIN, HIGH);
        motor_relay_timer = millis();
        motor_relay_state = R_WAIT_ENABLE;
#elif MOTOR_CONTROL_MODE == 2
        digitalWrite(RELAY_MOTOR_OPEN_PIN, LOW);
        digitalWrite(RELAY_MOTOR_CLOSE_PIN, LOW);
        LOG_PRINTLN("Motor Closing (Mode 2), waiting for relay delay.");
        motor_change_state = M_WAIT_FOR_ENGAGE;
        motor_change_timer = millis();
        next_operation = CLOSING;
#endif
        xSemaphoreGive(xMotorRelayMutex);
    }
}

void start_opening()
{
    // Called by Gate State Task after acquiring xStateMutex
    last_operation_before_stop = IDLE; 
    if (current_operation != IDLE || cal_state != CAL_INACTIVE) return;
    if (current_position >= 0.99f || digitalRead(LIMIT_OPEN_PIN) == LOW) { LOG_PRINTLN("Cannot open: Already fully open."); return; }
    if (digitalRead(MOVEMENT_INHIBIT_PIN) == LOW) { LOG_PRINTLN("Cannot open: Movement inhibited."); return; }
    LOG_PRINTLN("Command: OPEN");
    execute_open_sequence();
}

void start_closing()
{
    // Called by Gate State Task after acquiring xStateMutex
    last_operation_before_stop = IDLE; 
    if (current_operation != IDLE || cal_state != CAL_INACTIVE) return;
    if (current_position <= 0.01f || digitalRead(LIMIT_CLOSE_PIN) == LOW) { LOG_PRINTLN("Cannot close: Already fully closed."); return; }
    if (digitalRead(PHOTO_BARRIER_PIN) == LOW || digitalRead(MOVEMENT_INHIBIT_PIN) == LOW) { LOG_PRINTLN("Cannot close: Safety sensor active."); return; }
    LOG_PRINTLN("Command: CLOSE");
    execute_close_sequence();
}

void stop_movement(bool triggered_by_user)
{
    // Called by Gate State Task after acquiring xStateMutex
    if (current_operation == IDLE && cal_state == CAL_INACTIVE && !auto_resume_is_armed && !resume_countdown_is_active) return;

    if (triggered_by_user && current_operation != IDLE) {
        if (current_position > 0.01f && current_position < 0.99f) {
            last_operation_before_stop = current_operation;
        } else {
            last_operation_before_stop = IDLE; 
        }
    }

    if (current_operation != IDLE || auto_resume_is_armed || resume_countdown_is_active) { LOG_PRINTLN("Command: STOP"); }

    if (xSemaphoreTake(xMotorRelayMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
#if MOTOR_CONTROL_MODE == 1
        digitalWrite(RELAY_MOTOR_ENABLE_PIN, LOW);
        digitalWrite(RELAY_MOTOR_DIRECTION_PIN, LOW);
        motor_relay_state = R_OFF;
#elif MOTOR_CONTROL_MODE == 2
        LOG_PRINTLN("Motor Stopped (Mode 2).");
        digitalWrite(RELAY_MOTOR_OPEN_PIN, LOW);
        digitalWrite(RELAY_MOTOR_CLOSE_PIN, LOW);
        motor_change_state = M_IDLE;
        next_operation = IDLE;
#endif
        xSemaphoreGive(xMotorRelayMutex);
    }
    
    blink_interval = 0; 
    current_operation = IDLE;

    if (WiFi.status() == WL_CONNECTED) {
        if (current_position >= 0.99f) cover.setState(HACover::StateOpen);
        else if (current_position <= 0.01f) cover.setState(HACover::StateClosed);
        else cover.setState(HACover::StateStopped);
    }
    cover.setCurrentPosition(current_position * 100); 

    if (triggered_by_user) {
        target_position = -1.0; 
        if (cal_state != CAL_INACTIVE) {
            LOG_PRINTLN("Calibration cancelled by user.");
            cal_state = CAL_INACTIVE;
        }
        if (auto_resume_is_armed || resume_countdown_is_active) {
            LOG_PRINTLN("Auto-resume cancelled by user.");
            auto_resume_is_armed = false;
            resume_countdown_is_active = false;
        }
    }
}

void update_gate_position()
{
    // Called by Motor Control Task after acquiring xStateMutex
#if MOTOR_CONTROL_MODE == 1
    if (motor_relay_state != R_OFF) return;
#elif MOTOR_CONTROL_MODE == 2
    if (motor_change_state != M_IDLE) return;
#endif

    unsigned long elapsed = millis() - movement_start_time;
    float ratio = gate_travel_time > 0 ? float(elapsed) / float(gate_travel_time) : 0.0f;
    if (current_operation == OPENING) current_position = movement_start_position + ratio;
    else if (current_operation == CLOSING) current_position = movement_start_position - ratio;
    
    current_position = constrain(current_position, 0.0f, 1.0f);
    
    // ... (logging) ...
    
    if (target_position >= 0.0) {
        if ((current_operation == OPENING && current_position >= target_position) ||
            (current_operation == CLOSING && current_position <= target_position)) {
            LOG_PRINTF("Target position %.2f reached.\n", target_position);
            stop_movement(false);
            current_position = target_position; 
            target_position = -1.0;              
        }
    }
    if (elapsed > gate_travel_time) {
        LOG_PRINTLN("Error: Gate movement timed out!");
        stop_movement(false);
        target_position = -1.0; 
    }
}

void handle_safety_sensors()
{
    // Called by Motor Control Task after acquiring xStateMutex
    
    // Safety Inhibition
    if (current_operation != IDLE && digitalRead(MOVEMENT_INHIBIT_PIN) == LOW) {
        LOG_PRINTLN("Safety: Movement inhibited!");
        stop_movement(true);
        return;
    }
    
    // LIMIT SWITCHES
    if (current_operation == OPENING && digitalRead(LIMIT_OPEN_PIN) == LOW) {
        LOG_PRINTLN("Limit: OPEN reached");
        stop_movement(false);
        current_position = 1.0f;
        target_position = -1.0; 
    }
    if (current_operation == CLOSING && digitalRead(LIMIT_CLOSE_PIN) == LOW) {
        LOG_PRINTLN("Limit: CLOSE reached");
        stop_movement(false);
        current_position = 0.0f;
        target_position = -1.0; 
    }
    
    bool is_path_blocked = (digitalRead(PHOTO_BARRIER_PIN) == LOW);
    
    // PHOTO BARRIER OBSTRUCTION
    if (is_path_blocked && current_operation == CLOSING) {
        LOG_PRINTLN("Photo Barrier: Obstacle detected. Reversing fully.");
        stop_movement(false);
        auto_resume_is_armed = true;
        resume_countdown_is_active = false;
        target_position = -1.0; 
        execute_open_sequence();
    }
    
    // AUTO-RESUME LOGIC (Checks for path clear and timer)
    if (auto_resume_is_armed) {
        // ... (original auto-resume logic) ...
    }
    if (resume_countdown_is_active) {
        // ... (original resume countdown logic) ...
    }
}
void start_calibration()
{
    // Called by Gate State Task after acquiring xStateMutex
    if (current_operation != IDLE || cal_state != CAL_INACTIVE || rf_learn_state != RF_LEARN_INACTIVE) return;
    LOG_PRINTLN("--- Starting Gate Calibration ---");
    cal_state = CAL_CLOSING_TO_START;
    blink_interval = CALIBRATION_BLINK_INTERVAL; 
}
void handle_calibration()
{
    // Called by Motor Control Task after acquiring xStateMutex
    switch (cal_state)
    {
        case CAL_CLOSING_TO_START:
            // ... (original closing logic, calls execute_close_sequence, which handles mutex) ...
            break;
        case CAL_OPENING_FOR_TIMING:
            // ... (original opening logic, calls execute_open_sequence, which handles mutex) ...
            break;
        case CAL_DONE:
            // ... (original finish logic, calls save_params, publish_all_states) ...
            break;
        case CAL_INACTIVE:
            break;
    }
}


// =================================================================
// FreeRTOS Task Definitions
// =================================================================

// --- Task 1: Gate State Manager (Medium Priority) ---
void vGateStateTask(void *pvParameters) {
    CommandMessage msg;
    while(1) {
        if (xQueueReceive(xCommandQueue, &msg, portMAX_DELAY) == pdPASS) {
            
            if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                
                // --- Command Pre-Guards and Execution ---
                if (msg.cmd == CMD_WIFI_CONFIG_START) {
                    // Create the blocking Config Portal Task dynamically
                    xTaskCreate(vConfigPortalTask, "ConfigPortal", 6144, NULL, 0, NULL);
                } 
                else if (msg.cmd == CMD_RF_LEARN_SAVE_EXIT) {
                    // Logic to save codes and exit RF Learn mode
                    LOG_PRINTLN("Saving learned codes and exiting RF learn mode.");
                    save_params();
                    rf_learn_state = RF_LEARN_INACTIVE;
                    blink_interval = 0;
                    maintenanceButton.setLongClickTime(CALIBRATION_LONG_PRESS_TIME);
                } 
                else if (msg.cmd == CMD_RF_LEARN_SKIP) {
                    // Logic to skip step in RF Learn mode
                    if (rf_learn_state != RF_LEARN_INACTIVE) {
                        // ... (original skip logic using switch/case on rf_learn_state) ...
                    }
                }
                else if (cal_state == CAL_INACTIVE && rf_learn_state == RF_LEARN_INACTIVE) {
                    // Normal operational commands
                    switch(msg.cmd) {
                        case CMD_OPEN: start_opening(); break;
                        case CMD_CLOSE: start_closing(); break;
                        case CMD_STOP_USER: 
                            if (current_operation != IDLE) { // Pause
                                stop_movement(true);
                            } else if (last_operation_before_stop != IDLE) { // Resume
                                if (last_operation_before_stop == OPENING) start_opening();
                                else start_closing();
                            } else if (current_position >= 0.99f) { // Toggle
                                start_closing();
                            } else {
                                start_opening();
                            }
                            break;
                        case CMD_REVERSE:
                            // Simplified reverse logic (calling execute sequence)
                            if (current_operation == OPENING || (current_operation == IDLE && current_position >= 0.99f)) {
                                execute_close_sequence();
                            } else {
                                execute_open_sequence();
                            }
                            break;
                        case CMD_CALIBRATE_START: start_calibration(); break;
                        case CMD_RF_LEARN_START: 
                            rf_learn_state = RF_LEARN_WAIT_OPEN;
                            rf_learn_start_time = millis();
                            blink_interval = RF_LEARN_BLINK_INTERVAL;
                            maintenanceButton.setLongClickTime(RF_LEARN_SAVE_LONG_PRESS_TIME);
                            LOG_PRINTLN("--- RF Learning Mode Activated ---");
                            break;
                        case CMD_MOVE_TO_POSITION: move_to_position(msg.position); break;
                        case CMD_STOP_INTERNAL: stop_movement(false); break; // Only used by safety
                        default: break;
                    }
                }
                
                xSemaphoreGive(xStateMutex);
            }
        }
    }
}


// --- Task 2: Motor Control and Safety (High Priority) ---
void vMotorControlTask(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(5); 
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while(1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            
            handle_safety_sensors(); 
            handle_motor_relays();
            
            if (current_operation != IDLE) {
                update_gate_position();
            }
            
            if (cal_state != CAL_INACTIVE) {
                handle_calibration();
            }
            
            xSemaphoreGive(xStateMutex);
        }
    }
}


// --- Task 3: Input & Maintenance State (Medium Priority) ---
void vInputTask(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(10); 
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while(1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        mainButton.loop();
        maintenanceButton.loop();
        wifiButton.loop();
        
        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            if (rf_learn_state != RF_LEARN_INACTIVE) {
                handle_rf_learning();
            } else {
                handle_rf_signal(); 
            }
            xSemaphoreGive(xStateMutex);
        }
    }
}

// --- Task 4: Network and Configuration (Low Priority) ---
void vNetworkTask(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(500); 
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while(1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        if (WiFi.status() == WL_CONNECTED) {
            mqtt.loop(); 
            
            if (telnetServer.hasClient()) {
                if (!telnetClient || !telnetClient.connected()) {
                    if (telnetClient) telnetClient.stop();
                    telnetClient = telnetServer.accept();
                    LOG_PRINTLN("New Telnet client connected!");
                }
            }
        }

        handle_wifi_status();
        handle_indicator_light();
        
        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            publish_all_states();
            xSemaphoreGive(xStateMutex);
        }
    }
}

// --- Dynamic Task 5: Blocking WiFi Config Portal (Lowest Priority) ---
void vConfigPortalTask(void *pvParameters) {
    LOG_PRINTLN("WiFi Config: Portal started in background. Gate control remains live.");
    wm.setConfigPortalTimeout(180);

    wm.startConfigPortal("GateControllerAP"); 
    
    LOG_PRINTLN("Exiting config mode. Restarting device...");
    delay(1000);
    ESP.restart();
    
    vTaskDelete(NULL); 
}


// =================================================================
// Setup and Task Creation
// =================================================================

void setup()
{
    Serial.begin(115200);

    // --- Create RTOS Objects FIRST ---
    xLogMutex = xSemaphoreCreateMutex();      
    xStateMutex = xSemaphoreCreateMutex();    
    xMotorRelayMutex = xSemaphoreCreateMutex(); 
    xCommandQueue = xQueueCreate(10, sizeof(CommandMessage));

    LOG_PRINTLN("\nSliding Gate Controller Starting (RTOS)");

    if (!LittleFS.begin(true)) { LOG_PRINTLN("LittleFS Mount Failed!"); return; }
    load_params();

    byte mac[6];
    WiFi.macAddress(mac);
    device.setUniqueId(mac, sizeof(mac));
    device.setName("Sliding Gate");
    device.setManufacturer("DIY");
    device.setModel("ESP32-C3");

    cover.setName("Sliding Gate");
    cover.setDeviceClass("gate");
    cover.onCommand(onCoverCommand);

    // ... (All other HA sensor/button setup) ...
    
    // --- GPIO Setup ---
    // This is safe before tasks start
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
    
    // ... (Input pin setup and initial position check) ...

    // --- Button Setup (Handlers send Commands) ---
    mainButton.begin(MANUAL_MAIN_BUTTON_PIN, INPUT_PULLUP, true);
    maintenanceButton.begin(MANUAL_MAINTENANCE_BUTTON_PIN, INPUT_PULLUP, true);
    wifiButton.begin(MANUAL_WIFI_BUTTON_PIN, INPUT_PULLUP, true); 

    mainButton.setReleasedHandler(main_button_short_press);
    mainButton.setLongClickHandler(main_button_long_press);
    mainButton.setLongClickTime(REVERSE_LONG_PRESS_TIME);

    maintenanceButton.setReleasedHandler(maintenance_button_short_press);
    maintenanceButton.setLongClickHandler(maintenance_button_long_press);
    maintenanceButton.setLongClickTime(CALIBRATION_LONG_PRESS_TIME); 

    wifiButton.setReleasedHandler(wifi_button_short_press);
    wifiButton.setLongClickHandler(wifi_config_long_press); 
    wifiButton.setLongClickTime(WIFI_CONFIG_LONG_PRESS_TIME);

    // ... (WiFiManager Parameter Setup and Callback) ...

    pinMode(RF_RECEIVER_PIN, INPUT);
    mySwitch.enableReceive(RF_RECEIVER_PIN);

    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true); 
    WiFi.begin();

    LOG_PRINTLN("Setup complete.");
    
    uint16_t port = atoi(mqtt_port_str);
    mqtt.begin(mqtt_server, port, mqtt_user, mqtt_password);

    telnetServer.begin();
    LOG_PRINTLN("Telnet server started. Logger is active.");
    
    // --- Create Tasks ---
    xTaskCreate(vMotorControlTask, "MotorControl", 4096, NULL, 4, NULL);     
    xTaskCreate(vInputTask, "InputReader", 3072, NULL, 3, NULL);             
    xTaskCreate(vGateStateTask, "GateStateManager", 4096, NULL, 2, NULL);   
    xTaskCreate(vNetworkTask, "NetworkManager", 4096, NULL, 1, NULL);       

    LOG_PRINTLN("FreeRTOS Scheduler Started.");
}

// 6. Main loop is now empty
void loop() {
    // The scheduler manages all tasks. This loop is intentionally empty.
}