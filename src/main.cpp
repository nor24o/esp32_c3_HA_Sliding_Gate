#include <Arduino.h>
#include <RCSwitch.h>
#include <Button2.h>
#include "secrets.h"

// --- WiFi Libraries ---
#include <WiFi.h>
#include <WiFiManager.h>

#include <ArduinoHA.h>
#include <LittleFS.h>
#include <stdlib.h> // For atoi
#include <ArduinoJson.h>

// ——— Pin definitions ———
const int RELAY_MOTOR_DIRECTION_PIN = 1;
const int RELAY_MOTOR_ENABLE_PIN = 3;
const int RELAY_INDICATOR_LIGHT_PIN = 4;
const int MANUAL_OPEN_BUTTON_PIN = 5;
const int MANUAL_CLOSE_BUTTON_PIN = 6;
const int MANUAL_STOP_BUTTON_PIN = 7;
const int MOVEMENT_INHIBIT_PIN = 8;
const int LIMIT_OPEN_PIN = 10;
const int LIMIT_CLOSE_PIN = 20;
const int PHOTO_BARRIER_PIN = 2;
const int RF_RECEIVER_PIN = 21;

// ——— Timing constants ———
unsigned long MOTOR_DIRECTION_DELAY = 700;
unsigned long OBSTACLE_CLEAR_RESUME_DELAY = 8000;
unsigned long CALIBRATION_LONG_PRESS_TIME = 8000;
unsigned long WIFI_CONFIG_LONG_PRESS_TIME = 10000;
unsigned long WIFI_RETRY_INTERVAL = 30000;

// Persistent parameters
// These are the default values. They will be overwritten by values from LittleFS if available.
unsigned long gate_travel_time = 30000;
char mqtt_server[40] = "192.168.100.18";
char mqtt_port_str[6] = "1883";
char mqtt_user[32] = "hassio";
char mqtt_password[64] = "hassio";

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

#define BLINK_INTERVAL_OPENING 1000    // 1 second
#define BLINK_INTERVAL_CLOSING 500     // 0.5 second
#define CALIBRATION_BLINK_INTERVAL 100 // 0.2 second for calibration

void save_params()
{
  Serial.println("Saving configuration to LittleFS...");
  JsonDocument doc;

  doc["gate_travel_time"] = gate_travel_time;
  doc["mqtt_server"] = mqtt_server;
  doc["mqtt_port"] = mqtt_port_str;
  doc["mqtt_user"] = mqtt_user;
  doc["mqtt_password"] = mqtt_password;

  File configFile = LittleFS.open(PARAMS_FILE, "w");
  if (!configFile)
  {
    Serial.println("Failed to open config file for writing");
    return;
  }

  if (serializeJson(doc, configFile) == 0)
  {
    Serial.println("Failed to write to config file");
  }
  else
  {
    Serial.println("Configuration saved successfully.");
  }
  configFile.close();
}

void load_params()
{
  if (LittleFS.exists(PARAMS_FILE))
  {
    Serial.println("Reading configuration from LittleFS...");
    File configFile = LittleFS.open(PARAMS_FILE, "r");
    if (configFile)
    {
      JsonDocument doc;
      DeserializationError error = deserializeJson(doc, configFile);
      if (error)
      {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.c_str());
        return;
      }

      gate_travel_time = doc["gate_travel_time"] | gate_travel_time;
      strncpy(mqtt_server, doc["mqtt_server"] | mqtt_server, sizeof(mqtt_server));
      strncpy(mqtt_port_str, doc["mqtt_port"] | mqtt_port_str, sizeof(mqtt_port_str));
      strncpy(mqtt_user, doc["mqtt_user"] | mqtt_user, sizeof(mqtt_user));
      strncpy(mqtt_password, doc["mqtt_password"] | mqtt_password, sizeof(mqtt_password));

      Serial.println("Configuration loaded:");
      Serial.printf(" - Travel Time: %lu\n", gate_travel_time);
      Serial.printf(" - MQTT Server: %s\n", mqtt_server);
      Serial.printf(" - MQTT Port: %s\n", mqtt_port_str);
      Serial.printf(" - MQTT User: %s\n", mqtt_user);
      // Do not print password for security

      configFile.close();
    }
  }
  else
  {
    Serial.println("No configuration file found, using default values.");
    // Optional: save defaults on first boot
    save_params();
  }
}

// ——— Library Objects ———
RCSwitch mySwitch;
Button2 buttonOpen;
Button2 buttonClose;
Button2 buttonStop;
WiFiManager wm;

// 2) ArduinoHA objects
WiFiClient wifiClient;
HADevice device;
HAMqtt mqtt(wifiClient, device);
HACover cover("sliding_gate_cover");
HASensor rfCodeSensor("sliding_gate_last_rf_code");
HAButton calibrateButton("sliding_gate_calibrate");
HAButton moveTo50Button("sliding_gate_move_to_50"); // <-- NEW: HA Button for 50% position
HASensor gateState("sliding_gate_state");
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
enum MotorRelayState
{
  R_OFF,
  R_WAIT_ENABLE
};

// ——— Global state variables ———
CoverOperation current_operation = IDLE;
CalibrationState cal_state = CAL_INACTIVE;
MotorRelayState motor_relay_state = R_OFF;
unsigned long motor_relay_timer = 0;

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
void move_to_position(float new_target);  // <-- NEW
void onOpenCommand(HAButton *sender);
void onCloseCommand(HAButton *sender);
void onStopCommand(HAButton *sender);

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

  // limitOpen.setState(digitalRead(LIMIT_OPEN_PIN) == LOW);
  // limitClose.setState(digitalRead(LIMIT_CLOSE_PIN) == LOW);
  // photoBarrier.setState(digitalRead(PHOTO_BARRIER_PIN) == LOW);
  // movementInhibit.setState(digitalRead(MOVEMENT_INHIBIT_PIN) == LOW);

  // char code_str[20];
  // sprintf(code_str, "%lu", last_rf_code_received);
  // rfCodeSensor.setValue(code_str);
}

// ——— Button Callback Functions ———
void open_button_pressed(Button2 &btn)
{
  if (cal_state == CAL_INACTIVE)
    start_opening();
}
void close_button_pressed(Button2 &btn)
{
  if (cal_state == CAL_INACTIVE)
    start_closing();
}
void stop_button_pressed(Button2 &btn) { stop_movement(true); }
void calibration_long_press(Button2 &btn) { start_calibration(); }
// --- CORRECTED WiFi Config Portal Trigger ---
void wifi_config_long_press(Button2 &btn) {
    Serial.println("WiFi Config: Long press detected. Starting portal.");
    Serial.println("Gate will be unresponsive for up to 3 minutes.");

    wm.setConfigPortalTimeout(180);

    // This call is blocking. The save callback will handle saving the parameters.
    if (!wm.startConfigPortal("GateControllerAP")) {
        Serial.println("WiFi Config: Portal timed out. No new WiFi connection.");
    } else {
        Serial.println("WiFi Config: Portal exited successfully (new credentials may have been saved).");
    }

    // After the portal is done, a restart is always the safest way to ensure a clean state
    // and apply any new WiFi credentials or saved parameters.
    Serial.println("Exiting config mode. Restarting device...");
    delay(1000);
    ESP.restart();
}

// =================================================================
// Home Assistant Command Callbacks
// =================================================================
void onCoverCommand(HACover::CoverCommand cmd, HACover *sender)
{
  // FIX: Add guard to prevent normal commands during calibration
  if (cal_state != CAL_INACTIVE)
  {
    Serial.println("Ignoring command: Calibration in progress.");
    return;
  }
  // Clear any specific position target when using standard open/close/stop
  target_position = -1.0;
  switch (cmd)
  {
  case HACover::CommandOpen:
    Serial.println("Received OPEN command from HA");
    start_opening();
    break;
  case HACover::CommandClose:
    Serial.println("Received CLOSE command from HA");
    start_closing();
    break;
  case HACover::CommandStop:
    Serial.println("Received STOP command from HA");
    stop_movement(true);
    break;
  }
}

// ——— Setup ———
void setup()
{
  Serial.begin(115200);

  Serial.println("\nSliding Gate Controller Starting");

  // Mount filesystem & init pins
  // Mount filesystem
  if (!LittleFS.begin(true))
  {
    Serial.println("LittleFS Mount Failed!");
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

  pinMode(RELAY_MOTOR_DIRECTION_PIN, OUTPUT);
  pinMode(RELAY_MOTOR_ENABLE_PIN, OUTPUT);
  pinMode(RELAY_INDICATOR_LIGHT_PIN, OUTPUT);
  digitalWrite(RELAY_MOTOR_ENABLE_PIN, LOW);
  digitalWrite(RELAY_MOTOR_DIRECTION_PIN, LOW);
  digitalWrite(RELAY_INDICATOR_LIGHT_PIN, LOW);

  pinMode(MOVEMENT_INHIBIT_PIN, INPUT_PULLUP);
  pinMode(LIMIT_OPEN_PIN, INPUT_PULLUP);
  pinMode(LIMIT_CLOSE_PIN, INPUT_PULLUP);
  pinMode(PHOTO_BARRIER_PIN, INPUT_PULLUP);

  delay(50);
  if (digitalRead(LIMIT_OPEN_PIN) == LOW)
  {
    current_position = 1.0;
    Serial.println("Initial state: Gate is OPEN.");
  }
  else if (digitalRead(LIMIT_CLOSE_PIN) == LOW)
  {
    current_position = 0.0;
    Serial.println("Initial state: Gate is CLOSED.");
  }
  else
  {
    current_position = 0.0;
    Serial.println("Initial state: Gate position UNKNOWN (assuming CLOSED).");
  }

  buttonOpen.begin(MANUAL_OPEN_BUTTON_PIN, INPUT_PULLUP, true);
  buttonClose.begin(MANUAL_CLOSE_BUTTON_PIN, INPUT_PULLUP, true);
  buttonStop.begin(MANUAL_STOP_BUTTON_PIN, INPUT_PULLUP, true);

  wm.addParameter(&custom_mqtt_server);
  wm.addParameter(&custom_mqtt_port);
  wm.addParameter(&custom_mqtt_user);
  wm.addParameter(&custom_mqtt_password);

    // --- (FIX) Define a callback that is triggered when WiFiManager saves configuration ---
    // This is the key to reliably saving parameters, as it runs *before* the device restarts.
    wm.setSaveConfigCallback([]() {
        Serial.println("SaveConfigCallback triggered. Saving custom parameters...");
        // Retrieve values from the parameter objects and store them in our global variables
        strncpy(mqtt_server, custom_mqtt_server.getValue(), sizeof(mqtt_server));
        strncpy(mqtt_port_str, custom_mqtt_port.getValue(), sizeof(mqtt_port_str));
        strncpy(mqtt_user, custom_mqtt_user.getValue(), sizeof(mqtt_user));
        strncpy(mqtt_password, custom_mqtt_password.getValue(), sizeof(mqtt_password));

        // Now, explicitly save these global variables to LittleFS
        save_params();
    });

  buttonOpen.setPressedHandler(open_button_pressed);
  buttonClose.setPressedHandler(close_button_pressed);
  buttonStop.setPressedHandler(stop_button_pressed);

  buttonStop.setLongClickHandler(calibration_long_press);
  buttonStop.setLongClickTime(CALIBRATION_LONG_PRESS_TIME);
  buttonOpen.setLongClickHandler(wifi_config_long_press);
  buttonOpen.setLongClickTime(WIFI_CONFIG_LONG_PRESS_TIME);

  calibrateButton.setName("Calibrate");
  calibrateButton.setIcon("mdi:wrench");
  calibrateButton.onCommand(onCalibrateCommand);

  gateState.setName("Gate State");
  gateState.setIcon("mdi:gate");

  // ← configure the new travel‐time sensor:
  travelTimeSensor.setName("Gate Travel Time");
  travelTimeSensor.setUnitOfMeasurement("s");
  travelTimeSensor.setIcon("mdi:timer");

  // --- NEW: Configure HA Button entity for 50% position ---
  moveTo50Button.setName("Move Gate to 50%");
  moveTo50Button.setIcon("mdi:arrow-split-vertical");
  moveTo50Button.onCommand(onMoveTo50Command);

  // Configure separate HA buttons
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

  // Don't connect automatically in setup, to keep it non-blocking
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true); // This is the key!
  WiFi.begin();

  Serial.println("Setup complete.");
  Serial.println("Hold STOP for 2s to calibrate.");
  Serial.println("Hold OPEN for 2s for WiFi setup.");

  // 1) Configure MQTT → broker parameters & credentials
  //    This does *not* block; first connect attempt happens in mqtt.loop().

  //Serial print mqtt parameters
  Serial.println("MQTT Configuration:");
  Serial.printf(" - Server: %s\n", mqtt_server);
  Serial.printf(" - Port: %s\n", mqtt_port_str);
  Serial.printf(" - User: %s\n", mqtt_user);  
  
  uint16_t port = atoi(mqtt_port_str);
  mqtt.begin(mqtt_server, port, mqtt_user, mqtt_password);
}

// ——— Main loop ———
void loop()
{

  // --- MQTT layer ---
  if (WiFi.status() == WL_CONNECTED)
  {
    // mqtt.loop() will:
    //  • keep the keep-alive ping alive
    //  • notice if the socket dropped
    //  • automatically reconnect every 10 s (ReconnectInterval) if needed
    mqtt.loop(); // ← critical for auto-reconnect :contentReference[oaicite:0]{index=0}
  }

  // send every 3 seconds
  static unsigned long last_publish_time = 0;
  if (millis() - last_publish_time > 3000)
  {
    last_publish_time = millis();
    publish_all_states();
  }

  buttonOpen.loop();
  buttonClose.loop();
  buttonStop.loop();

  handle_rf_signal();
  handle_motor_relays();

  // Periodically check WiFi status in the background
  handle_wifi_status();
  handle_indicator_light(); // New handler for blinking light

  if (cal_state != CAL_INACTIVE)
  {
    handle_calibration();
  }
  else
  {
    handle_safety_sensors();
    if (current_operation != IDLE)
    {
      update_gate_position();
    }
  }
}

void onOpenCommand(HAButton *sender)
{
  Serial.println("Received OPEN command from HA Button");
  target_position = -1.0;
  start_opening();
}

void onCloseCommand(HAButton *sender)
{
  Serial.println("Received CLOSE command from HA Button");
  target_position = -1.0;
  start_closing();
}

void onStopCommand(HAButton *sender)
{
  Serial.println("Received STOP command from HA Button");
  stop_movement(true);
}

void onCalibrateCommand(HAButton *sender)
{
  Serial.println("Received CALIBRATE command from HA");
  start_calibration();
}

// --- NEW: Callback for the HA Button ---
void onMoveTo50Command(HAButton *sender)
{
  Serial.println("Received Move to 50% command from HA");
  move_to_position(0.5f);
}

// --- NEW: Logic to initiate movement to a specific position ---
void move_to_position(float new_target)
{
  if (cal_state != CAL_INACTIVE)
  {
    Serial.println("Cannot move to position: Calibration in progress.");
    return;
  }

  // --- FIX: Stop any current movement before starting a new one. ---
  // This resets the state to IDLE, allowing a reversal of direction.
  if (current_operation != IDLE)
  {
    Serial.println("Gate is currently moving, stopping first...");
    stop_movement(false); // Programmatic stop, doesn't cancel user-intent
  }

  // Check if we are already there (with a small tolerance)
  if (abs(current_position - new_target) < 0.01)
  {
    Serial.println("Already at target position.");
    return;
  }

  // Set the global target
  target_position = new_target;

  // Decide whether to open or close
  if (new_target > current_position)
  {
    Serial.printf("Moving to position %.2f by OPENING.\n", target_position);
    start_opening();
  }
  else
  {
    Serial.printf("Moving to position %.2f by CLOSING.\n", target_position);
    start_closing();
  }
}

// --- NEW ---
// This function handles the blinking logic for the indicator light.
void handle_indicator_light()
{
  // If blink_interval is 0, the gate is stopped, so make sure the light is off.
  if (blink_interval == 0)
  {
    if (indicator_light_state)
    { // Only write if state needs to change
      Serial.println("Indicator light OFF");
      digitalWrite(RELAY_INDICATOR_LIGHT_PIN, LOW);
      indicator_light_state = false;
    }
    return;
  }

  // If the blink interval has elapsed, toggle the light state.
  if (millis() - last_blink_time > blink_interval)
  {
    last_blink_time = millis();
    if (!indicator_light_state)
    {
      Serial.println("Indicator light ON");
    }
    else
    {
      Serial.println("Indicator light OFF");
    }
    indicator_light_state = !indicator_light_state;
    digitalWrite(RELAY_INDICATOR_LIGHT_PIN, indicator_light_state);
  }
}

// --- CORRECTED Non-Blocking WiFi Status Handler ---
void handle_wifi_status()
{
  // This function just checks and prints the status. It does not block.
  if (millis() - last_wifi_check > WIFI_RETRY_INTERVAL)
  {
    last_wifi_check = millis();
    if (WiFi.status() == WL_CONNECTED)
    {
      // Serial.print("WiFi: Connected. IP Address: ");
      // Serial.println(WiFi.localIP());
      //  Future MQTT/Web server code can be triggered from here
    }
    else
    {
      Serial.println("WiFi: Not connected. Trying to reconnect automatically...");
      // ESP32's WiFi stack will handle reconnection attempts automatically
      // If it was previously configured.
    }
  }
}

// ALL OTHER FUNCTIONS FROM THE PREVIOUS "BASE" CODE REMAIN UNCHANGED
// (Pasted below for completeness)

void handle_rf_signal()
{ /* ... unchanged ... */
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
  }
  switch (code)
  {
  case RF_GATE_OPEN_CODE:
    target_position = -1.0;
    start_opening();
    break;
  case RF_GATE_CLOSE_CODE:
    target_position = -1.0;
    start_closing();
    break;
  case RF_GATE_STOP_CODE:
    stop_movement(true);
    break;
  case RF_GATE_POS50_CODE:
    Serial.println("RF command: Move to 50%");
    move_to_position(0.5f);
    break;
  }
}
void handle_motor_relays()
{ /* ... unchanged ... */
  if (motor_relay_state == R_OFF)
    return;
  if (millis() - motor_relay_timer >= MOTOR_DIRECTION_DELAY)
  {
    if (motor_relay_state == R_WAIT_ENABLE)
    {
      digitalWrite(RELAY_MOTOR_ENABLE_PIN, HIGH);
      motor_relay_state = R_OFF;
      Serial.println("Motor Enabled.");
      movement_start_time = millis();
      movement_start_position = current_position;
    }
  }
}
void execute_open_sequence()
{
  if (current_operation != IDLE || motor_relay_state != R_OFF)
    return;
  current_operation = OPENING;
  if (WiFi.status() == WL_CONNECTED)
  {
    cover.setState(HACover::StateOpening);
  }
  blink_interval = BLINK_INTERVAL_OPENING; // Set slow blink
  digitalWrite(RELAY_MOTOR_DIRECTION_PIN, LOW);
  motor_relay_timer = millis();
  motor_relay_state = R_WAIT_ENABLE;
}
void execute_close_sequence()
{ /* ... unchanged ... */
  if (current_operation != IDLE || motor_relay_state != R_OFF)
    return;
  current_operation = CLOSING;
  if (WiFi.status() == WL_CONNECTED)
  {
    cover.setState(HACover::StateClosing);
  }
  blink_interval = BLINK_INTERVAL_CLOSING; // Set fast blink
  digitalWrite(RELAY_MOTOR_DIRECTION_PIN, HIGH);
  motor_relay_timer = millis();
  motor_relay_state = R_WAIT_ENABLE;
}
void start_opening()
{

  if (current_operation != IDLE || cal_state != CAL_INACTIVE)
    return;
  if (current_position >= 0.99f || digitalRead(LIMIT_OPEN_PIN) == LOW)
  {
    Serial.println("Cannot open: Already fully open.");
    return;
  }
  if (digitalRead(MOVEMENT_INHIBIT_PIN) == LOW)
  {
    Serial.println("Cannot open: Movement inhibited.");
    return;
  }
  Serial.println("Command: OPEN");
  execute_open_sequence();
}
void start_closing()
{ /* ... unchanged ... */
  if (current_operation != IDLE || cal_state != CAL_INACTIVE)
    return;
  if (current_position <= 0.01f || digitalRead(LIMIT_CLOSE_PIN) == LOW)
  {
    Serial.println("Cannot close: Already fully closed.");
    return;
  }
  if (digitalRead(PHOTO_BARRIER_PIN) == LOW || digitalRead(MOVEMENT_INHIBIT_PIN) == LOW)
  {
    Serial.println("Cannot close: Safety sensor active.");
    return;
  }
  Serial.println("Command: CLOSE");
  execute_close_sequence();
}
void stop_movement(bool triggered_by_user)
{ /* ... unchanged ... */
  if (current_operation == IDLE && cal_state == CAL_INACTIVE && !auto_resume_is_armed && !resume_countdown_is_active)
    return;
  if (current_operation != IDLE || auto_resume_is_armed || resume_countdown_is_active)
  {
    Serial.println("Command: STOP");
  }
  digitalWrite(RELAY_MOTOR_ENABLE_PIN, LOW);
  digitalWrite(RELAY_MOTOR_DIRECTION_PIN, LOW);
  blink_interval = 0; // Stop blinking
  current_operation = IDLE;
  motor_relay_state = R_OFF;

  if (WiFi.status() == WL_CONNECTED)
  {
    if (current_position >= 0.99f)
      cover.setState(HACover::StateOpen);
    else if (current_position <= 0.01f)
      cover.setState(HACover::StateClosed);
    else
      cover.setState(HACover::StateStopped);
  }
  cover.setCurrentPosition(current_position * 100); // Immediately publish position with state

  if (triggered_by_user)
  {
    target_position = -1.0; // <-- NEW: Cancel target on manual stop
    if (cal_state != CAL_INACTIVE)
    {
      Serial.println("Calibration cancelled by user.");
      cal_state = CAL_INACTIVE;
    }
    if (auto_resume_is_armed || resume_countdown_is_active)
    {
      Serial.println("Auto-resume cancelled by user.");
      auto_resume_is_armed = false;
      resume_countdown_is_active = false;
    }
  }
}
void update_gate_position()
{ /* ... unchanged ... */
  if (motor_relay_state != R_OFF)
    return;
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
    Serial.printf("Position: %.2f  State: %s\n", current_position, stateStr);
  }
  // --- FIXED: Check if we have a specific target and have reached or passed it ---
  if (target_position >= 0.0)
  {
    if ((current_operation == OPENING && current_position >= target_position) ||
        (current_operation == CLOSING && current_position <= target_position))
    {

      Serial.printf("Target position %.2f reached.\n", target_position);
      stop_movement(false);
      current_position = target_position; // Snap to the exact position for accuracy
      target_position = -1.0;             // Reset the target
    }
  }
  if (elapsed > gate_travel_time)
  {
    Serial.println("Error: Gate movement timed out!");
    stop_movement(false);
    target_position = -1.0; // <-- NEW: Clear target at hard limit
  }
}

void handle_safety_sensors()
{ /* ... unchanged ... */
  if (current_operation != IDLE && digitalRead(MOVEMENT_INHIBIT_PIN) == LOW)
  {
    Serial.println("Safety: Movement inhibited!");
    stop_movement(true);
    return;
  }
  if (current_operation == OPENING && digitalRead(LIMIT_OPEN_PIN) == LOW)
  {
    Serial.println("Limit: OPEN reached");
    stop_movement(false);
    current_position = 1.0f;
    target_position = -1.0; // <-- NEW: Clear target at hard limit
  }
  if (current_operation == CLOSING && digitalRead(LIMIT_CLOSE_PIN) == LOW)
  {
    Serial.println("Limit: CLOSE reached");
    stop_movement(false);
    current_position = 0.0f;
    target_position = -1.0; // <-- NEW: Clear target at hard limit
  }
  bool is_path_blocked = (digitalRead(PHOTO_BARRIER_PIN) == LOW);
  static bool was_path_blocked = false;
  if (is_path_blocked && !was_path_blocked && current_operation == CLOSING)
  {
    Serial.println("Photo Barrier: Obstacle detected. Reversing fully.");
    stop_movement(false);
    auto_resume_is_armed = true;
    resume_countdown_is_active = false;
    target_position = -1.0; // Cancel target on photo barrier trigger
    execute_open_sequence();
  }
  if (auto_resume_is_armed)
  {
    if (current_operation == IDLE && current_position >= 1.0)
    {
      if (!is_path_blocked)
      {
        Serial.printf("Photo Barrier: Path is clear. Starting %lu sec auto-close timer.\n", OBSTACLE_CLEAR_RESUME_DELAY / 1000);
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
        Serial.println("Photo Barrier: Path re-blocked at last moment. Resetting timer.");
        path_cleared_time = millis();
      }
      else
      {
        Serial.println("Photo Barrier: Timer expired. Resuming close operation.");
        resume_countdown_is_active = false;
        start_closing();
      }
    }
  }
  was_path_blocked = is_path_blocked;
}
void start_calibration()
{ /* ... unchanged ... */
  if (current_operation != IDLE || cal_state != CAL_INACTIVE)
    return;
  Serial.println("--- Starting Gate Calibration ---");
  cal_state = CAL_CLOSING_TO_START;
  blink_interval = CALIBRATION_BLINK_INTERVAL; // Set fast blink for calibration
}
void handle_calibration()
{ /* ... unchanged ... */
  switch (cal_state)
  {
  case CAL_CLOSING_TO_START:
    if (current_operation == IDLE)
    {
      if (digitalRead(LIMIT_CLOSE_PIN) == LOW)
      {
        Serial.println("Step 1 Complete: Already at close limit.");
        stop_movement(false);
        current_position = 0.0f;
        cal_state = CAL_OPENING_FOR_TIMING;
        return;
      }
      else
      {
        Serial.println("Step 1: Closing gate to find zero position...");
        execute_close_sequence();
      }
    }
    if (current_operation == CLOSING && digitalRead(LIMIT_CLOSE_PIN) == LOW)
    {
      Serial.println("Step 1 Complete: Close limit found.");
      stop_movement(false);
      current_position = 0.0f;
      cal_state = CAL_OPENING_FOR_TIMING;
    }
    break;
  case CAL_OPENING_FOR_TIMING:
    if (current_operation == IDLE)
    {
      Serial.println("Step 2: Opening gate to measure travel time...");
      execute_open_sequence();
    }
    if (current_operation == OPENING && digitalRead(LIMIT_OPEN_PIN) == LOW)
    {
      Serial.println("Step 2 Complete: Open limit found.");
      stop_movement(false);
      gate_travel_time = millis() - movement_start_time;
      current_position = 1.0f;
      cal_state = CAL_DONE;
    }
    break;
  case CAL_DONE:
    if (current_operation == IDLE)
    {
      Serial.println("--- Calibration Complete ---");
      Serial.printf("New gate travel time: %lu ms\n", gate_travel_time);
      Serial.println("Position has been set to OPEN (1.0).");
      save_params();
      cal_state = CAL_INACTIVE;
      publish_all_states();
    }
    break;
  case CAL_INACTIVE:
    break;
  }
}