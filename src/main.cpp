#include <Arduino.h>
#include <RCSwitch.h>
#include <Button2.h>

// --- WiFi Libraries ---
#include <WiFi.h>
#include <WiFiManager.h>

#include <ArduinoHA.h>
#include <LittleFS.h>
#include <stdlib.h> // For atoi
#include <ArduinoJson.h>

#include "SerialMirror.hpp" // Include SerialMirror class and LOG_PRINT macros

// --- Build-time Configuration ---
// Set the motor control mode.
// 1: Direction relay + Enable relay (Original mode)
// 2: Open relay + Close relay (New mode)
#define MOTOR_CONTROL_MODE 2 // <--- SET THIS TO 1 or 2 AT BUILD TIME

// ——— Pin definitions ———
#if MOTOR_CONTROL_MODE == 1
// Mode 1: One relay for direction, one for power
const int RELAY_MOTOR_DIRECTION_PIN = 1;
const int RELAY_MOTOR_ENABLE_PIN = 4;
#elif MOTOR_CONTROL_MODE == 2
// Mode 2: One relay for opening, one for closing
const int RELAY_MOTOR_OPEN_PIN = 1;
const int RELAY_MOTOR_CLOSE_PIN = 4;
#endif

const int RELAY_INDICATOR_LIGHT_PIN = 3;
// --- NEW: Simplified Button Configuration ---
// The main button now handles Open/Close/Stop toggle functionality.
const int MANUAL_MAIN_BUTTON_PIN = 5; // Was MANUAL_OPEN_BUTTON_PIN
const int MANUAL_WIFI_BUTTON_PIN = 6; // <-- NEW: Dedicated button for WiFi config
// A separate button is kept for the calibration long-press. Its short-press is disabled.
const int MANUAL_MAINTENANCE_BUTTON_PIN = 7; // Was MANUAL_STOP_BUTTON_PIN
// Pin 6 is now dedicated to WiFi setup.

const int MOVEMENT_INHIBIT_PIN = 8;
const int LIMIT_OPEN_PIN = 10;
const int LIMIT_CLOSE_PIN = 20;
const int PHOTO_BARRIER_PIN = 2;
const int RF_RECEIVER_PIN = 21;

// ——— Timing constants ———
unsigned long MOTOR_DIRECTION_DELAY = 700;
unsigned long OBSTACLE_CLEAR_RESUME_DELAY = 8000;
unsigned long CALIBRATION_LONG_PRESS_TIME = 8000;
unsigned long WIFI_CONFIG_LONG_PRESS_TIME = 5000; // Changed from 10000 to 5000 (5 seconds)
unsigned long WIFI_RETRY_INTERVAL = 30000;
const unsigned long REVERSE_LONG_PRESS_TIME = 1000; // 1 second for long press to reverse
const unsigned long RF_LEARN_TIMEOUT = 30000; // 30 seconds timeout for learning mode
const unsigned long RF_LEARN_SAVE_LONG_PRESS_TIME = 1500; // 1.5s to save and exit learning mode


// Persistent parameters
// These are the default values. They will be overwritten by values from LittleFS if available.
unsigned long gate_travel_time = 30000;
char mqtt_server[40] = "192.168.1.8";
char mqtt_port_str[6] = "1883";
char mqtt_user[32] = "admin";
char mqtt_password[64] = "admin";

// --- RF codes are now variables, not constants, to allow for learning ---
// These are the default codes, which will be overwritten by values from LittleFS if available.
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
unsigned long blink_interval = 0; // 0 means light is off
bool indicator_light_state = false;

// --- LittleFS storage helpers ---
#define PARAMS_FILE "/gate_params_data.txt"

#define BLINK_INTERVAL_OPENING 1000      // 1 second
#define BLINK_INTERVAL_CLOSING 500       // 0.5 second
#define CALIBRATION_BLINK_INTERVAL 100 // 0.2 second for calibration
#define RF_LEARN_BLINK_INTERVAL 250 // Fast blink for RF learning mode


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

// ——— Library Objects ———
RCSwitch mySwitch;
Button2 mainButton;        // Replaces separate open/close/stop buttons
Button2 maintenanceButton; // Used for calibration long-press
Button2 wifiButton;        // <-- NEW: Button for WiFi config
WiFiManager wm;
WiFiServer telnetServer(23);

// 2) ArduinoHA objects
WiFiClient wifiClient;
HADevice device;
HAMqtt mqtt(wifiClient, device);
HACover cover("sliding_gate_cover");
HASensor rfCodeSensor("sliding_gate_last_rf_code");
HAButton calibrateButton("sliding_gate_calibrate");
HAButton moveTo50Button("sliding_gate_move_to_50"); // <-- NEW: HA Button for 50% position
HASensor gateState("sliding_gate_state");
HASensor gateIP("sliding_gate_IP");
// ← new!
HASensor travelTimeSensor("sliding_gate_travel_time");

HAButton openButtonHA("sliding_gate_open_button");
HAButton closeButtonHA("sliding_gate_close_button");
HAButton stopButtonHA("sliding_gate_stop_button");

// ——— State machine enums ———
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

#if MOTOR_CONTROL_MODE == 1
// This state is only for Mode 1 (Direction + Enable relays)
enum MotorRelayState
{
  R_OFF,
  R_WAIT_ENABLE
};
#elif MOTOR_CONTROL_MODE == 2
// State machine for Mode 2 to ensure a delay when changing direction
enum MotorChangeState
{
  M_IDLE,
  M_WAIT_FOR_ENGAGE
};
#endif

// --- NEW: Enum and state for RF Learning Mode ---
enum RFLearningState {
    RF_LEARN_INACTIVE,
    RF_LEARN_WAIT_OPEN,
    RF_LEARN_WAIT_CLOSE,
    RF_LEARN_WAIT_STOP,
    RF_LEARN_WAIT_POS50
};


// ——— Global state variables ———
CoverOperation current_operation = IDLE;
CalibrationState cal_state = CAL_INACTIVE;
CoverOperation last_operation_before_stop = IDLE; // Remembers last direction for pause/resume

RFLearningState rf_learn_state = RF_LEARN_INACTIVE;
unsigned long rf_learn_start_time = 0;


#if MOTOR_CONTROL_MODE == 1
// These variables are only used in Mode 1
MotorRelayState motor_relay_state = R_OFF;
unsigned long motor_relay_timer = 0;
#elif MOTOR_CONTROL_MODE == 2
// These variables are only used in Mode 2
MotorChangeState motor_change_state = M_IDLE;
unsigned long motor_change_timer = 0;
CoverOperation next_operation = IDLE; // To store which direction to engage after delay
#endif

float current_position = 0.0;
unsigned long movement_start_time;
float movement_start_position;

float target_position = -1.0; // <-- NEW: -1.0 means no specific target

bool auto_resume_is_armed = false;
bool resume_countdown_is_active = false;
unsigned long path_cleared_time = 0;

unsigned long last_wifi_check = 0;
unsigned long last_rf_code_received = 0;

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
void handle_wifi_status(); // Changed from handle_wifi
void onCalibrateCommand(HAButton *sender);
void handle_indicator_light();
void onMoveTo50Command(HAButton *sender); // <-- NEW
void move_to_position(float new_target);   // <-- NEW
void onOpenCommand(HAButton *sender);
void onCloseCommand(HAButton *sender);
void onStopCommand(HAButton *sender);
void main_button_short_press(Button2 &btn); // For pause/resume
void main_button_long_press(Button2 &btn);  // For reverse
void wifi_button_short_press(Button2 &btn); // <-- NEW: For RF Learning
void handle_rf_learning();                  // <-- NEW: Handler for the learning process
void maintenance_button_short_press(Button2 &btn); // For skipping RF learn steps
void maintenance_button_long_press(Button2 &btn); // For saving RF learn / calibration


void publish_all_states()
{
  cover.setCurrentPosition(current_position * 100);

  // publish the travel-time (in whole seconds)
  unsigned long travelSeconds = gate_travel_time / 1000;
  char buf[16];
  snprintf(buf, sizeof(buf), "%lu", travelSeconds);
  travelTimeSensor.setValue(buf);

  // Set gateState based on enums
  if (cal_state != CAL_INACTIVE)
  {
    gateState.setValue("calibrating");
  }
  else if (rf_learn_state != RF_LEARN_INACTIVE) // <-- NEW
  {
      gateState.setValue("rf_learning");
  }
  else if (current_operation == OPENING)
  {
    gateState.setValue("opening");
  }
  else if (current_operation == CLOSING)
  {
    gateState.setValue("closing");
  }
  else if (current_position == 1.0f)
  {
    gateState.setValue("open");
  }
  else if (current_position == 0.0f)
  {
    gateState.setValue("closed");
  }
  else
  {
    gateState.setValue("stopped");
  }
  gateIP.setValue(WiFi.localIP().toString().c_str());
}

// ——— Button Callback Functions ———

// NEW: Handlers for main button with advanced pause, resume, and reverse logic.
void main_button_short_press(Button2 &btn)
{
    if (cal_state != CAL_INACTIVE || rf_learn_state != RF_LEARN_INACTIVE) return;

    if (current_operation != IDLE)
    {
        // If gate is moving, a short press will PAUSE it.
        LOG_PRINTLN("Main button (short press): Pausing movement.");
        stop_movement(true);
    }
    else // Gate is stopped (IDLE)
    {
        if (last_operation_before_stop != IDLE)
        {
            // If gate was paused, RESUME in the same direction.
            LOG_PRINTLN("Main button (short press): Resuming movement.");
            if (last_operation_before_stop == OPENING) {
                start_opening();
            } else { // last_operation_before_stop == CLOSING
                start_closing();
            }
        }
        else // Gate is stopped at a limit or was stopped programmatically.
        {
            // Standard toggle behavior.
            if (current_position >= 0.99f) {
                LOG_PRINTLN("Main button (short press): Gate is open, starting to close.");
                start_closing();
            } else {
                LOG_PRINTLN("Main button (short press): Gate is not open, starting to open.");
                start_opening();
            }
        }
    }
}

void main_button_long_press(Button2 &btn)
{
    if (cal_state != CAL_INACTIVE || rf_learn_state != RF_LEARN_INACTIVE) return;

    LOG_PRINTLN("Main button (long press): Reversing direction.");

    // Determine the last or current direction of movement.
    CoverOperation effective_direction = current_operation;
    if (effective_direction == IDLE) {
        effective_direction = last_operation_before_stop;
    }
    
    // By calling the execute sequence directly, we avoid the abrupt stop_movement()
    // and create a smoother transition from one direction to the other.
    if (effective_direction == OPENING || (effective_direction == IDLE && current_position >= 0.99f)) {
        // If it was opening, or it is fully open, the reverse action is to CLOSE.
        execute_close_sequence();
    } else {
        // If it was closing, is fully closed, or was paused while closing, the reverse action is to OPEN.
        execute_open_sequence();
    }
}

// --- NEW: Context-aware handlers for the maintenance button ---
void maintenance_button_short_press(Button2 &btn) {
    if (rf_learn_state == RF_LEARN_INACTIVE) return; // Do nothing if not in learning mode

    LOG_PRINTLN("Maintenance button (short press): Skipping current RF code.");
    switch (rf_learn_state) {
        case RF_LEARN_WAIT_OPEN:
            LOG_PRINTLN("Skipped OPEN. Now press the desired CLOSE button...");
            rf_learn_state = RF_LEARN_WAIT_CLOSE;
            rf_learn_start_time = millis(); // Reset timeout
            break;
        case RF_LEARN_WAIT_CLOSE:
            LOG_PRINTLN("Skipped CLOSE. Now press the desired STOP button...");
            rf_learn_state = RF_LEARN_WAIT_STOP;
            rf_learn_start_time = millis(); // Reset timeout
            break;
        case RF_LEARN_WAIT_STOP:
            LOG_PRINTLN("Skipped STOP. Now press the desired 50% POSITION button...");
            rf_learn_state = RF_LEARN_WAIT_POS50;
            rf_learn_start_time = millis(); // Reset timeout
            break;
        case RF_LEARN_WAIT_POS50:
            LOG_PRINTLN("Skipped 50% POSITION. Learning finished.");
            // This is the end of the process, save and exit.
            LOG_PRINTLN("--- RF Learning Complete! ---");
            save_params();
            rf_learn_state = RF_LEARN_INACTIVE;
            blink_interval = 0;
            maintenanceButton.setLongClickTime(CALIBRATION_LONG_PRESS_TIME); // Restore original time
            break;
        case RF_LEARN_INACTIVE:
            break;
    }
}

void maintenance_button_long_press(Button2 &btn) {
    if (rf_learn_state != RF_LEARN_INACTIVE) {
        // In learning mode, this is Save & Exit
        LOG_PRINTLN("Maintenance button (long press): Saving learned codes and exiting RF learn mode.");
        save_params();
        rf_learn_state = RF_LEARN_INACTIVE;
        blink_interval = 0;
        maintenanceButton.setLongClickTime(CALIBRATION_LONG_PRESS_TIME); // Restore original time
    } else {
        // In normal mode, this is Calibrate
        start_calibration();
    }
}

void wifi_config_long_press(Button2 &btn)
{
  LOG_PRINTLN("WiFi Config: Long press detected. Starting portal.");
  LOG_PRINTLN("Gate will be unresponsive for up to 3 minutes.");

  wm.setConfigPortalTimeout(180);

  if (!wm.startConfigPortal("GateControllerAP"))
  {
    LOG_PRINTLN("WiFi Config: Portal timed out. No new WiFi connection.");
  }
  else
  {
    LOG_PRINTLN("WiFi Config: Portal exited successfully (new credentials may have been saved).");
  }
  
  LOG_PRINTLN("Exiting config mode. Restarting device...");
  delay(1000);
  ESP.restart();
}

// --- NEW: Short press on WiFi button to start RF learning ---
void wifi_button_short_press(Button2 &btn) {
    if (current_operation != IDLE || cal_state != CAL_INACTIVE) {
        LOG_PRINTLN("Cannot start RF learning while gate is active.");
        return;
    }

    rf_learn_state = RF_LEARN_WAIT_OPEN;
    rf_learn_start_time = millis();
    blink_interval = RF_LEARN_BLINK_INTERVAL;
    maintenanceButton.setLongClickTime(RF_LEARN_SAVE_LONG_PRESS_TIME); // Set shorter time for save/exit
    LOG_PRINTLN("--- RF Learning Mode Activated ---");
    LOG_PRINTLN("Press the desired OPEN button on your remote...");
    LOG_PRINTLN("Or, tap Maintenance button to skip, hold to save and exit.");
}


// =================================================================
// Home Assistant Command Callbacks
// =================================================================
void onCoverCommand(HACover::CoverCommand cmd, HACover *sender)
{
  if (cal_state != CAL_INACTIVE || rf_learn_state != RF_LEARN_INACTIVE)
  {
    LOG_PRINTLN("Ignoring command: Maintenance/Learning in progress.");
    return;
  }
  target_position = -1.0;
  switch (cmd)
  {
  case HACover::CommandOpen:
    LOG_PRINTLN("Received OPEN command from HA");
    start_opening();
    break;
  case HACover::CommandClose:
    LOG_PRINTLN("Received CLOSE command from HA");
    start_closing();
    break;
  case HACover::CommandStop:
    LOG_PRINTLN("Received STOP command from HA");
    stop_movement(true);
    break;
  }
}

// ——— Setup ———
void setup()
{
  Serial.begin(115200);

  LOG_PRINTLN("\nSliding Gate Controller Starting");

  if (!LittleFS.begin(true))
  {
    LOG_PRINTLN("LittleFS Mount Failed!");
    return;
  }

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

  rfCodeSensor.setName("Last RF Code");
  rfCodeSensor.setIcon("mdi:remote-control");

#if MOTOR_CONTROL_MODE == 1
  LOG_PRINTLN("Configured for Motor Control Mode 1 (Direction + Enable Relays).");
  pinMode(RELAY_MOTOR_DIRECTION_PIN, OUTPUT);
  pinMode(RELAY_MOTOR_ENABLE_PIN, OUTPUT);
  digitalWrite(RELAY_MOTOR_ENABLE_PIN, LOW);
  digitalWrite(RELAY_MOTOR_DIRECTION_PIN, LOW);
#elif MOTOR_CONTROL_MODE == 2
  LOG_PRINTLN("Configured for Motor Control Mode 2 (Open + Close Relays).");
  pinMode(RELAY_MOTOR_OPEN_PIN, OUTPUT);
  pinMode(RELAY_MOTOR_CLOSE_PIN, OUTPUT);
  digitalWrite(RELAY_MOTOR_OPEN_PIN, LOW);
  digitalWrite(RELAY_MOTOR_CLOSE_PIN, LOW);
#endif
  pinMode(RELAY_INDICATOR_LIGHT_PIN, OUTPUT);
  digitalWrite(RELAY_INDICATOR_LIGHT_PIN, LOW);

  pinMode(MOVEMENT_INHIBIT_PIN, INPUT_PULLUP);
  pinMode(LIMIT_OPEN_PIN, INPUT_PULLUP);
  pinMode(LIMIT_CLOSE_PIN, INPUT_PULLUP);
  pinMode(PHOTO_BARRIER_PIN, INPUT_PULLUP);

  delay(50);
  if (digitalRead(LIMIT_OPEN_PIN) == LOW)
  {
    current_position = 1.0;
    LOG_PRINTLN("Initial state: Gate is OPEN.");
  }
  else if (digitalRead(LIMIT_CLOSE_PIN) == LOW)
  {
    current_position = 0.0;
    LOG_PRINTLN("Initial state: Gate is CLOSED.");
  }
  else
  {
    current_position = 0.0;
    LOG_PRINTLN("Initial state: Gate position UNKNOWN (assuming CLOSED).");
  }
  
  mainButton.begin(MANUAL_MAIN_BUTTON_PIN, INPUT_PULLUP, true);
  maintenanceButton.begin(MANUAL_MAINTENANCE_BUTTON_PIN, INPUT_PULLUP, true);
  wifiButton.begin(MANUAL_WIFI_BUTTON_PIN, INPUT_PULLUP, true); 


  wm.addParameter(&custom_mqtt_server);
  wm.addParameter(&custom_mqtt_port);
  wm.addParameter(&custom_mqtt_user);
  wm.addParameter(&custom_mqtt_password);

  wm.setSaveConfigCallback([]()
                           {
    LOG_PRINTLN("SaveConfigCallback triggered. Saving custom parameters...");
    strncpy(mqtt_server, custom_mqtt_server.getValue(), sizeof(mqtt_server));
    strncpy(mqtt_port_str, custom_mqtt_port.getValue(), sizeof(mqtt_port_str));
    strncpy(mqtt_user, custom_mqtt_user.getValue(), sizeof(mqtt_user));
    strncpy(mqtt_password, custom_mqtt_password.getValue(), sizeof(mqtt_password));

    save_params(); });

  mainButton.setReleasedHandler(main_button_short_press);
  mainButton.setLongClickHandler(main_button_long_press);
  mainButton.setLongClickTime(REVERSE_LONG_PRESS_TIME);

  // --- NEW: Maintenance button is now context-aware ---
  maintenanceButton.setReleasedHandler(maintenance_button_short_press);
  maintenanceButton.setLongClickHandler(maintenance_button_long_press);
  maintenanceButton.setLongClickTime(CALIBRATION_LONG_PRESS_TIME); // Default time for calibration

  wifiButton.setReleasedHandler(wifi_button_short_press);
  wifiButton.setLongClickHandler(wifi_config_long_press);
  wifiButton.setLongClickTime(WIFI_CONFIG_LONG_PRESS_TIME);


  calibrateButton.setName("Calibrate");
  calibrateButton.setIcon("mdi:wrench");
  calibrateButton.onCommand(onCalibrateCommand);

  gateState.setName("Gate State");
  gateState.setIcon("mdi:gate");

  gateIP.setName("Gate IP");
  gateIP.setUnitOfMeasurement("IP");
  gateIP.setIcon("mdi:ip");

  travelTimeSensor.setName("Gate Travel Time");
  travelTimeSensor.setUnitOfMeasurement("s");
  travelTimeSensor.setIcon("mdi:timer");

  moveTo50Button.setName("Move Gate to 50%");
  moveTo50Button.setIcon("mdi:arrow-split-vertical");
  moveTo50Button.onCommand(onMoveTo50Command);

  openButtonHA.setName("Open Gate Button");
  openButtonHA.setIcon("mdi:arrow-up-box");
  openButtonHA.onCommand(onOpenCommand);

  closeButtonHA.setName("Close Gate Button");
  closeButtonHA.setIcon("mdi:arrow-down-box");
  closeButtonHA.onCommand(onCloseCommand);

  stopButtonHA.setName("Stop Gate Button");
  stopButtonHA.setIcon("mdi:stop-circle-outline");
  stopButtonHA.onCommand(onStopCommand);

  pinMode(RF_RECEIVER_PIN, INPUT);
  mySwitch.enableReceive(RF_RECEIVER_PIN);

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true); 
  WiFi.begin();

  LOG_PRINTLN("Setup complete.");
  LOG_PRINTLN("Hold STOP/MAINTENANCE button for 8s to calibrate.");
  LOG_PRINTLN("Hold WIFI button (Pin 6) for 5s for WiFi setup.");
  LOG_PRINTLN("Tap WIFI button (Pin 6) to enter RF Learning Mode.");

  LOG_PRINTLN("MQTT Configuration:");
  LOG_PRINTF(" - Server: %s\n", mqtt_server);
  LOG_PRINTF(" - Port: %s\n", mqtt_port_str);
  LOG_PRINTF(" - User: %s\n", mqtt_user);

  uint16_t port = atoi(mqtt_port_str);
  mqtt.begin(mqtt_server, port, mqtt_user, mqtt_password);

  telnetServer.begin();
  LOG_PRINTLN("Telnet server started. Logger is active.");
  gateIP.setValue(WiFi.localIP().toString().c_str());
}

// ——— Main loop ———
void loop()
{
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
        LOG_PRINTLN("New Telnet client connected!");
      }
    }
  }

  static unsigned long last_publish_time = 0;
  if (millis() - last_publish_time > 3000)
  {
    last_publish_time = millis();
    publish_all_states();
  }

  mainButton.loop();
  maintenanceButton.loop();
  wifiButton.loop();

  
  handle_motor_relays();
  handle_wifi_status();
  handle_indicator_light();
  
  if (rf_learn_state != RF_LEARN_INACTIVE)
  {
      handle_rf_learning();
  }
  else if (cal_state != CAL_INACTIVE)
  {
    handle_calibration();
  }
  else
  {
    handle_rf_signal();
    handle_safety_sensors();
    if (current_operation != IDLE)
    {
      update_gate_position();
    }
  }
}

void onOpenCommand(HAButton *sender)
{
  LOG_PRINTLN("Received OPEN command from HA Button");
  target_position = -1.0;
  start_opening();
}

void onCloseCommand(HAButton *sender)
{
  LOG_PRINTLN("Received CLOSE command from HA Button");
  target_position = -1.0;
  start_closing();
}

void onStopCommand(HAButton *sender)
{
  LOG_PRINTLN("Received STOP command from HA Button");
  stop_movement(true);
}

void onCalibrateCommand(HAButton *sender)
{
  LOG_PRINTLN("Received CALIBRATE command from HA");
  start_calibration();
}

void onMoveTo50Command(HAButton *sender)
{
  LOG_PRINTLN("Received Move to 50% command from HA");
  move_to_position(0.5f);
}

void move_to_position(float new_target)
{
  if (cal_state != CAL_INACTIVE)
  {
    LOG_PRINTLN("Cannot move to position: Calibration in progress.");
    return;
  }

  if (current_operation != IDLE)
  {
    LOG_PRINTLN("Gate is currently moving, stopping first...");
    stop_movement(false); 
  }

  if (abs(current_position - new_target) < 0.01)
  {
    LOG_PRINTLN("Already at target position.");
    return;
  }

  target_position = new_target;

  if (new_target > current_position)
  {
    LOG_PRINTF("Moving to position %.2f by OPENING.\n", target_position);
    start_opening();
  }
  else
  {
    LOG_PRINTF("Moving to position %.2f by CLOSING.\n", target_position);
    start_closing();
  }
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
    {
      LOG_PRINTLN("WiFi: Not connected. Trying to reconnect automatically...");
    }
  }
}

void handle_rf_learning() {
    if (millis() - rf_learn_start_time > RF_LEARN_TIMEOUT) {
        LOG_PRINTLN("RF Learning timed out. Exiting.");
        rf_learn_state = RF_LEARN_INACTIVE;
        blink_interval = 0;
        maintenanceButton.setLongClickTime(CALIBRATION_LONG_PRESS_TIME); // Restore original time
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
            maintenanceButton.setLongClickTime(CALIBRATION_LONG_PRESS_TIME); // Restore original time
            break;

        case RF_LEARN_INACTIVE:
            break;
    }
}


void handle_rf_signal()
{
  if (!mySwitch.available())
    return;
  unsigned long code = mySwitch.getReceivedValue();
  mySwitch.resetAvailable();
  if (code != 0)
  {
    last_rf_code_received = code;
    char code_str[20];
    sprintf(code_str, "%lu", last_rf_code_received);
    rfCodeSensor.setValue(code_str);
    LOG_PRINTF("RF code received: %lu\n", code);
  }

  if (code == rf_gate_open_code) {
      target_position = -1.0;
      start_opening();
  } else if (code == rf_gate_close_code) {
      target_position = -1.0;
      start_closing();
  } else if (code == rf_gate_stop_code) {
      stop_movement(true);
  } else if (code == rf_gate_pos50_code) {
      LOG_PRINTLN("RF command: Move to 50%");
      move_to_position(0.5f);
  }
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
      digitalWrite(RELAY_MOTOR_ENABLE_PIN, HIGH);
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
      if (next_operation == OPENING)
      {
        LOG_PRINTLN("Motor Opening (Mode 2) after delay.");
        digitalWrite(RELAY_MOTOR_OPEN_PIN, HIGH);
      }
      else if (next_operation == CLOSING)
      {
        LOG_PRINTLN("Motor Closing (Mode 2) after delay.");
        digitalWrite(RELAY_MOTOR_CLOSE_PIN, HIGH);
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
  if (current_operation == OPENING) return;

  current_operation = OPENING;
  if (WiFi.status() == WL_CONNECTED)
  {
    cover.setState(HACover::StateOpening);
  }
  blink_interval = BLINK_INTERVAL_OPENING; 

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
}

void execute_close_sequence()
{
  if (current_operation == CLOSING) return;

  current_operation = CLOSING;
  if (WiFi.status() == WL_CONNECTED)
  {
    cover.setState(HACover::StateClosing);
  }
  blink_interval = BLINK_INTERVAL_CLOSING; 

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
}

void start_opening()
{
  last_operation_before_stop = IDLE; 
  if (current_operation != IDLE || cal_state != CAL_INACTIVE)
    return;
  if (current_position >= 0.99f || digitalRead(LIMIT_OPEN_PIN) == LOW)
  {
    LOG_PRINTLN("Cannot open: Already fully open.");
    return;
  }
  if (digitalRead(MOVEMENT_INHIBIT_PIN) == LOW)
  {
    LOG_PRINTLN("Cannot open: Movement inhibited.");
    return;
  }
  LOG_PRINTLN("Command: OPEN");
  execute_open_sequence();
}

void start_closing()
{
  last_operation_before_stop = IDLE; 
  if (current_operation != IDLE || cal_state != CAL_INACTIVE)
    return;
  if (current_position <= 0.01f || digitalRead(LIMIT_CLOSE_PIN) == LOW)
  {
    LOG_PRINTLN("Cannot close: Already fully closed.");
    return;
  }
  if (digitalRead(PHOTO_BARRIER_PIN) == LOW || digitalRead(MOVEMENT_INHIBIT_PIN) == LOW)
  {
    LOG_PRINTLN("Cannot close: Safety sensor active.");
    return;
  }
  LOG_PRINTLN("Command: CLOSE");
  execute_close_sequence();
}

void stop_movement(bool triggered_by_user)
{
  if (current_operation == IDLE && cal_state == CAL_INACTIVE && !auto_resume_is_armed && !resume_countdown_is_active)
    return;

  if (triggered_by_user && current_operation != IDLE) {
      if (current_position > 0.01f && current_position < 0.99f) {
          last_operation_before_stop = current_operation;
      } else {
          last_operation_before_stop = IDLE; 
      }
  }

  if (current_operation != IDLE || auto_resume_is_armed || resume_countdown_is_active)
  {
    LOG_PRINTLN("Command: STOP");
  }

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

  blink_interval = 0; 
  current_operation = IDLE;

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
    {
      LOG_PRINTLN("Calibration cancelled by user.");
      cal_state = CAL_INACTIVE;
    }
    if (auto_resume_is_armed || resume_countdown_is_active)
    {
      LOG_PRINTLN("Auto-resume cancelled by user.");
      auto_resume_is_armed = false;
      resume_countdown_is_active = false;
    }
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
  float ratio = gate_travel_time > 0 ? float(elapsed) / float(gate_travel_time) : 0.0f;
  if (current_operation == OPENING)
    current_position = movement_start_position + ratio;
  else if (current_operation == CLOSING)
    current_position = movement_start_position - ratio;
  current_position = constrain(current_position, 0.0f, 1.0f);
  if (current_operation != IDLE)
  {
    const char *stateStr = (current_operation == OPENING) ? "OPENING" : "CLOSING";
    LOG_PRINTF("Position: %.2f  State: %s\n", current_position, stateStr);
  }
  if (target_position >= 0.0)
  {
    if ((current_operation == OPENING && current_position >= target_position) ||
        (current_operation == CLOSING && current_position <= target_position))
    {

      LOG_PRINTF("Target position %.2f reached.\n", target_position);
      stop_movement(false);
      current_position = target_position; 
      target_position = -1.0;              
    }
  }
  if (elapsed > gate_travel_time)
  {
    LOG_PRINTLN("Error: Gate movement timed out!");
    stop_movement(false);
    target_position = -1.0; 
  }
}

void handle_safety_sensors()
{
  if (current_operation != IDLE && digitalRead(MOVEMENT_INHIBIT_PIN) == LOW)
  {
    LOG_PRINTLN("Safety: Movement inhibited!");
    stop_movement(true);
    return;
  }
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
  bool is_path_blocked = (digitalRead(PHOTO_BARRIER_PIN) == LOW);
  if (is_path_blocked && current_operation == CLOSING)
  {
    LOG_PRINTLN("Photo Barrier: Obstacle detected. Reversing fully.");
    stop_movement(false);
    auto_resume_is_armed = true;
    resume_countdown_is_active = false;
    target_position = -1.0; 
    execute_open_sequence();
  }
  if (auto_resume_is_armed)
  {
    if (current_operation == IDLE && current_position >= 1.0)
    {
      if (!is_path_blocked)
      {
        LOG_PRINTF("Photo Barrier: Path is clear. Starting %lu sec auto-close timer.\n", OBSTACLE_CLEAR_RESUME_DELAY / 1000);
        resume_countdown_is_active = true;
        path_cleared_time = millis();
        auto_resume_is_armed = false;
      }
    }
  }
  if (resume_countdown_is_active)
  {
    if (current_operation != IDLE)
    {
      resume_countdown_is_active = false;
      return;
    }
    if (millis() - path_cleared_time >= OBSTACLE_CLEAR_RESUME_DELAY)
    {
      if (is_path_blocked)
      {
        LOG_PRINTLN("Photo Barrier: Path re-blocked at last moment. Resetting timer.");
        path_cleared_time = millis();
      }
      else
      {
        LOG_PRINTLN("Photo Barrier: Timer expired. Resuming close operation.");
        resume_countdown_is_active = false;
        start_closing();
      }
    }
  }
}
void start_calibration()
{
  if (current_operation != IDLE || cal_state != CAL_INACTIVE || rf_learn_state != RF_LEARN_INACTIVE)
    return;
  LOG_PRINTLN("--- Starting Gate Calibration ---");
  cal_state = CAL_CLOSING_TO_START;
  blink_interval = CALIBRATION_BLINK_INTERVAL; 
}
void handle_calibration()
{
  switch (cal_state)
  {
  case CAL_CLOSING_TO_START:
    if (current_operation == IDLE)
    {
      if (digitalRead(LIMIT_CLOSE_PIN) == LOW)
      {
        LOG_PRINTLN("Step 1 Complete: Already at close limit.");
        stop_movement(false);
        current_position = 0.0f;
        cal_state = CAL_OPENING_FOR_TIMING;
        return;
      }
      else
      {
        LOG_PRINTLN("Step 1: Closing gate to find zero position...");
        execute_close_sequence();
      }
    }
    if (current_operation == CLOSING && digitalRead(LIMIT_CLOSE_PIN) == LOW)
    {
      LOG_PRINTLN("Step 1 Complete: Close limit found.");
      stop_movement(false);
      current_position = 0.0f;
      cal_state = CAL_OPENING_FOR_TIMING;
    }
    break;
  case CAL_OPENING_FOR_TIMING:
    if (current_operation == IDLE)
    {
      LOG_PRINTLN("Step 2: Opening gate to measure travel time...");
      execute_open_sequence();
    }
    if (current_operation == OPENING && digitalRead(LIMIT_OPEN_PIN) == LOW)
    {
      LOG_PRINTLN("Step 2 Complete: Open limit found.");
      stop_movement(false);
      gate_travel_time = millis() - movement_start_time;
      current_position = 1.0f;
      cal_state = CAL_DONE;
    }
    break;
  case CAL_DONE:
    if (current_operation == IDLE)
    {
      LOG_PRINTLN("--- Calibration Complete ---");
      LOG_PRINTF("New gate travel time: %lu ms\n", gate_travel_time);
      LOG_PRINTLN("Position has been set to OPEN (1.0).");
      save_params();
      cal_state = CAL_INACTIVE;
      publish_all_states();
    }
    break;
  case CAL_INACTIVE:
    break;
  }
}

