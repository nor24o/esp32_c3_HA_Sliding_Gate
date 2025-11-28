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
#include <Preferences.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <WebSocketsServer.h>
#include <vector>

// [CRITICAL] Disable Serial to prevent Pin 1 (TX) interference
#define USE_SERIAL_DEBUG false

// --- CONFIGURATION ---
#define ENABLE_HW_WATCHDOG true
#define HW_WDT_TIMEOUT 30      // 30s Hardware WDT
#define SOFT_WDT_TIMEOUT 20000 // 20s Software Monitor Timeout
#define MAX_LOG_SIZE 5000

SemaphoreHandle_t xLogMutex = NULL;
WiFiClient telnetClient;
WebServer server(80);           
WebSocketsServer webSocket(81); 

#include "SerialMirror.hpp"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

// --- WATCHDOG TRACKERS ---
volatile unsigned long last_checkin_motor = 0;
volatile unsigned long last_checkin_network = 0;
volatile unsigned long last_checkin_gate = 0;
volatile unsigned long last_checkin_logger = 0;
volatile unsigned long last_checkin_rf = 0;
volatile unsigned long last_checkin_button = 0;
volatile unsigned long last_checkin_led = 0;

// Flags
volatile bool wifi_config_request = false;
bool web_server_started = false;

// Simulation Flags
bool simulate_crash_motor = false;
bool simulate_crash_network = false;
bool simulate_crash_rf = false;
bool simulate_crash_gate = false;
bool simulate_crash_button = false;
bool simulate_crash_logger = false;

// --- Command Structure ---
enum GateCommand {
    CMD_NONE, CMD_OPEN, CMD_CLOSE, CMD_STOP_ONLY, CMD_TOGGLE,
    CMD_STOP_INTERNAL, CMD_REVERSE, CMD_CALIBRATE_START,
    CMD_RF_LEARN_START, CMD_RF_LEARN_SKIP, CMD_RF_LEARN_SAVE_EXIT, // Restored these
    CMD_RF_SCAN_MODE, CMD_RF_ADD_CODE, CMD_RF_DELETE_CODE, 
    CMD_MOVE_TO_POSITION, CMD_WIFI_CONFIG_START
};

typedef struct {
    GateCommand cmd;
    float position;
    unsigned long code; 
    int aux_data;       
} CommandMessage;

// RF Structure
struct RFEntry {
    unsigned long code;
    uint8_t function; // 0=Open, 1=Close, 2=Stop, 3=Pedestrian, 4=Toggle
};
std::vector<RFEntry> rfKeyList;
unsigned long scanned_rf_code = 0; 

QueueHandle_t xCommandQueue;
SemaphoreHandle_t xStateMutex;
SemaphoreHandle_t xMotorRelayMutex;
TaskHandle_t hNetworkTask = NULL;

Preferences preferences; 
Preferences gatePrefs;   
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

unsigned long MOTOR_DIRECTION_DELAY = 700;
unsigned long CALIBRATION_LONG_PRESS_TIME = 8000;
unsigned long WIFI_CONFIG_LONG_PRESS_TIME = 5000;
unsigned long WIFI_RETRY_INTERVAL = 30000;
const unsigned long REVERSE_LONG_PRESS_TIME = 1000;
const unsigned long RF_LEARN_TIMEOUT = 60000; 
const unsigned long RF_LEARN_SAVE_LONG_PRESS_TIME = 1500;
const unsigned long RF_DEBOUNCE_DELAY = 400;
const unsigned long COMBO_MODE_HOLD_TIME = 2000;
const unsigned long CALIBRATION_SAFETY_TIMEOUT = 90000; 

unsigned long gate_travel_time = 30000;
char mqtt_server[40] = "192.168.1.12";
char mqtt_port_str[6] = "1883";
char mqtt_user[32] = "admin";
char mqtt_password[64] = "admin";

unsigned long last_blink_time = 0;
unsigned long blink_interval = 0;
bool indicator_light_state = false;

#define LOG_FILE "/system_log.txt"
#define BLINK_INTERVAL_OPENING 1000
#define BLINK_INTERVAL_CLOSING 500
#define CALIBRATION_BLINK_INTERVAL 100
#define RF_LEARN_BLINK_INTERVAL 250

enum CoverOperation { IDLE, OPENING, CLOSING };
enum CalibrationState { CAL_INACTIVE, CAL_HOMING_CLOSE, CAL_MEASURING_OPEN, CAL_VERIFYING_CLOSE, CAL_DONE };
enum RFLearningState { RF_LEARN_INACTIVE, RF_LEARN_WAIT_OPEN, RF_LEARN_WAIT_CLOSE, RF_LEARN_WAIT_STOP, RF_LEARN_WAIT_POS50, RF_SCANNING_WEB };

#if MOTOR_CONTROL_MODE == 1
enum MotorRelayState { R_OFF, R_WAIT_ENABLE };
#elif MOTOR_CONTROL_MODE == 2
enum MotorChangeState { M_IDLE, M_WAIT_FOR_ENGAGE };
#endif

CoverOperation current_operation = IDLE;
CalibrationState cal_state = CAL_INACTIVE;
CoverOperation last_operation_before_stop = IDLE;
RFLearningState rf_learn_state = RF_LEARN_INACTIVE;
unsigned long rf_learn_start_time = 0;
unsigned long calibration_start_time = 0;

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

HABinarySensor limitOpenSensor("sliding_gate_lim_open");
HABinarySensor limitCloseSensor("sliding_gate_lim_close");
HABinarySensor barrierSensor("sliding_gate_barrier");

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
void dump_config();
void log_io_status();
void log_system_error(const char *msg);
void log_info(String msg);
void send_command(GateCommand cmd, float pos = -1.0f);
void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length);
void broadcastStatus();
void sendRFListToWeb();

// =================================================================
// LOGGING & WATCHDOG
// =================================================================

void log_info(String msg) {
    LOG_PRINTLN(msg.c_str()); 
    File logFile = LittleFS.open(LOG_FILE, "a");
    if (logFile) {
        logFile.print("[");
        logFile.print(millis()/1000);
        logFile.print("s] ");
        logFile.println(msg);
        logFile.close();
    }
}

void log_system_error(const char *msg)
{
    log_info("ERROR: " + String(msg));
}

void dump_system_log()
{
    if (!LittleFS.exists(LOG_FILE))
    {
        LOG_PRINTLN("No logs.");
        return;
    }
    LOG_PRINTLN("\n--- LOGS ---");
    File logFile = LittleFS.open(LOG_FILE, "r");
    if (logFile)
    {
        while (logFile.available())
        {
            if (ENABLE_HW_WATCHDOG)
                esp_task_wdt_reset();
            TelnetLogger.write(logFile.read());
        }
        logFile.close();
    }
    LOG_PRINTLN("\n------------\n");
}

void dump_config()
{
    LOG_PRINTLN("\n--- CONFIGURATION ---");
    LOG_PRINTF("Travel Time: %lu ms\n", gate_travel_time);
    LOG_PRINTF("MQTT Server: %s\n", mqtt_server);
    LOG_PRINTF("Stored RF Keys: %d\n", rfKeyList.size());
    LOG_PRINTLN("\n--- STATE ---");
    LOG_PRINTF("Current POS: %.2f\n", current_position);
    LOG_PRINTLN("---------------------\n");
}

void check_reset_reason()
{
    esp_reset_reason_t reason = esp_reset_reason();
    const char *reason_str = "Unknown";
    bool save_log = false;
    switch (reason)
    {
    case ESP_RST_POWERON: reason_str = "Power On"; break;
    case ESP_RST_SW: reason_str = "Software Reset"; break;
    case ESP_RST_PANIC: reason_str = "Crash/Panic"; save_log = true; break;
    case ESP_RST_TASK_WDT: reason_str = "HW Watchdog (Hang)"; save_log = true; break;
    case ESP_RST_WDT: reason_str = "Interrupt WDT"; save_log = true; break;
    case ESP_RST_BROWNOUT: reason_str = "Brownout"; save_log = true; break;
    default: break;
    }
    LOG_PRINTF("Boot #%u | Reason: %s\n", bootCount, reason_str);
    if (save_log) log_system_error(reason_str);
}

void vSupervisorTask(void *pvParameters)
{
    if (ENABLE_HW_WATCHDOG) esp_task_wdt_add(NULL);
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (ENABLE_HW_WATCHDOG) esp_task_wdt_reset();
        unsigned long now = millis();

        if (millis() > 25000)
        {
            if (!wifi_config_request && (now - last_checkin_network > SOFT_WDT_TIMEOUT))
            {
                log_system_error("CRASH: Network Task Hung");
                delay(500);
                ESP.restart();
            }
            if (now - last_checkin_motor > SOFT_WDT_TIMEOUT)
            {
                log_system_error("CRASH: Motor Task Hung");
                delay(500);
                ESP.restart();
            }
            if (now - last_checkin_gate > SOFT_WDT_TIMEOUT)
            {
                log_system_error("CRASH: Gate Logic Hung");
                delay(500);
                ESP.restart();
            }
        }
    }
}

// =================================================================
// Parameters & IO
// =================================================================

void save_params()
{
    log_info("Saving to Preferences...");
    gatePrefs.begin("gate_conf", false); 
    
    gatePrefs.putULong("travel_time", gate_travel_time);
    gatePrefs.putString("mqtt_server", mqtt_server);
    gatePrefs.putString("mqtt_port", mqtt_port_str);
    gatePrefs.putString("mqtt_user", mqtt_user);
    gatePrefs.putString("mqtt_pass", mqtt_password);
    
    // Save RF List
    if (rfKeyList.size() > 0) {
        gatePrefs.putBytes("rf_list", rfKeyList.data(), rfKeyList.size() * sizeof(RFEntry));
    } else {
        gatePrefs.remove("rf_list");
    }
    
    gatePrefs.end();
    log_info("Config Saved.");
}

void load_params()
{
    LOG_PRINTLN("LOADING FROM PREFERENCES...");
    gatePrefs.begin("gate_conf", false); 
    
    gate_travel_time = gatePrefs.getULong("travel_time", 30000);
    
    String s = gatePrefs.getString("mqtt_server", "192.168.1.12");
    s.toCharArray(mqtt_server, 40);
    s = gatePrefs.getString("mqtt_port", "1883");
    s.toCharArray(mqtt_port_str, 6);
    s = gatePrefs.getString("mqtt_user", "admin");
    s.toCharArray(mqtt_user, 32);
    s = gatePrefs.getString("mqtt_pass", "admin");
    s.toCharArray(mqtt_password, 64);
    
    size_t len = gatePrefs.getBytesLength("rf_list");
    if (len > 0 && (len % sizeof(RFEntry) == 0)) {
        rfKeyList.resize(len / sizeof(RFEntry));
        gatePrefs.getBytes("rf_list", rfKeyList.data(), len);
        LOG_PRINTF("Loaded %d RF keys from list.\n", rfKeyList.size());
    } else {
        // Migration
        unsigned long old_open = gatePrefs.getULong("rf_open", 0);
        unsigned long old_close = gatePrefs.getULong("rf_close", 0);
        unsigned long old_stop = gatePrefs.getULong("rf_stop", 0);
        unsigned long old_pos50 = gatePrefs.getULong("rf_pos50", 0);
        
        if (old_open != 0 || old_close != 0 || old_stop != 0) {
            log_info("Migrating old RF codes...");
            if (old_open != 0) rfKeyList.push_back({old_open, 0});
            if (old_close != 0) rfKeyList.push_back({old_close, 1});
            if (old_stop != 0) rfKeyList.push_back({old_stop, 2});
            if (old_pos50 != 0) rfKeyList.push_back({old_pos50, 3});
            
            gatePrefs.remove("rf_open");
            gatePrefs.remove("rf_close");
            gatePrefs.remove("rf_stop");
            gatePrefs.remove("rf_pos50");
            
            gatePrefs.putBytes("rf_list", rfKeyList.data(), rfKeyList.size() * sizeof(RFEntry));
        }
    }
    
    gatePrefs.end();
    LOG_PRINTF("Loaded MQTT IP: %s\n", mqtt_server);
}

void send_command_full(GateCommand cmd, float pos = -1.0f, unsigned long code = 0, int aux = 0)
{
    CommandMessage msg = {cmd, pos, code, aux};
    xQueueSend(xCommandQueue, &msg, pdMS_TO_TICKS(100));
}

void send_command(GateCommand cmd, float pos) {
    send_command_full(cmd, pos, 0, 0);
}

void log_io_status()
{
    char buf[128];
    int mot1 = 0;
    int mot2 = 0;
#if MOTOR_CONTROL_MODE == 1
    mot1 = digitalRead(RELAY_MOTOR_DIRECTION_PIN);
    mot2 = digitalRead(RELAY_MOTOR_ENABLE_PIN);
#elif MOTOR_CONTROL_MODE == 2
    mot1 = digitalRead(RELAY_MOTOR_OPEN_PIN);
    mot2 = digitalRead(RELAY_MOTOR_CLOSE_PIN);
#endif
    snprintf(buf, sizeof(buf), "[IO] OP:%d CL:%d PH:%d | M1:%d M2:%d LGT:%d | POS:%.2f",
             digitalRead(LIMIT_OPEN_PIN), digitalRead(LIMIT_CLOSE_PIN), digitalRead(PHOTO_BARRIER_PIN),
             mot1, mot2, digitalRead(RELAY_INDICATOR_LIGHT_PIN), current_position);
    LOG_PRINTLN(buf);
}

// =================================================================
// WEB SERVER HANDLERS
// =================================================================

void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
    switch (type) {
        case WStype_DISCONNECTED: break;
        case WStype_CONNECTED:
            broadcastStatus();
            sendRFListToWeb();
            break;
        case WStype_TEXT:
            String text = String((char *)payload);
            if (text == "OPEN") send_command(CMD_OPEN);
            else if (text == "CLOSE") send_command(CMD_CLOSE);
            else if (text == "STOP") send_command(CMD_STOP_ONLY);
            else if (text == "CALIBRATE") send_command(CMD_CALIBRATE_START);
            else if (text == "SCAN_START") send_command_full(CMD_RF_SCAN_MODE, -1, 0, 1);
            else if (text == "SCAN_STOP") send_command_full(CMD_RF_SCAN_MODE, -1, 0, 0);
            
            if (text.startsWith("ADD:")) {
                int firstColon = text.indexOf(':');
                int secondColon = text.lastIndexOf(':');
                if (firstColon > 0 && secondColon > firstColon) {
                    String codeStr = text.substring(firstColon + 1, secondColon);
                    String funcStr = text.substring(secondColon + 1);
                    send_command_full(CMD_RF_ADD_CODE, -1, strtoul(codeStr.c_str(), NULL, 10), funcStr.toInt());
                }
            }
            if (text.startsWith("DEL:")) {
                String idxStr = text.substring(4);
                send_command_full(CMD_RF_DELETE_CODE, -1, 0, idxStr.toInt());
            }
            break;
    }
}

void broadcastStatus() {
    if (webSocket.connectedClients() > 0) {
        char json[350]; 
        String status = "STOPPED";
        String subStatus = "";

        if (current_operation == OPENING) status = "OPENING";
        else if (current_operation == CLOSING) status = "CLOSING";
        
        if (cal_state != CAL_INACTIVE) {
            status = "CALIBRATING";
            if(cal_state==CAL_HOMING_CLOSE) subStatus="Homing...";
            else if(cal_state==CAL_MEASURING_OPEN) subStatus="Measuring...";
            else if(cal_state==CAL_VERIFYING_CLOSE) subStatus="Verifying...";
        }
        
        if (rf_learn_state == RF_SCANNING_WEB) {
            status = "SCANNING";
            subStatus = (scanned_rf_code > 0) ? String(scanned_rf_code) : "Waiting...";
        } else if (rf_learn_state != RF_LEARN_INACTIVE) {
            status = "PHYSICAL LEARN";
        }
        
        snprintf(json, sizeof(json), 
            "{\"type\":\"status\",\"s\":\"%s\",\"ss\":\"%s\",\"p\":%d,\"lo\":%d,\"lc\":%d,\"pb\":%d,\"rf\":%lu}", 
            status.c_str(), subStatus.c_str(),
            (int)(current_position * 100),
            digitalRead(LIMIT_OPEN_PIN), digitalRead(LIMIT_CLOSE_PIN), digitalRead(PHOTO_BARRIER_PIN),
            last_rf_code_received
        );
        webSocket.broadcastTXT(json);
    }
}

void sendRFListToWeb() {
    String json = "{\"type\":\"rf_list\",\"data\":[";
    for (size_t i = 0; i < rfKeyList.size(); i++) {
        json += "{\"c\":" + String(rfKeyList[i].code) + ",\"f\":" + String(rfKeyList[i].function) + "}";
        if (i < rfKeyList.size() - 1) json += ",";
    }
    json += "]}";
    webSocket.broadcastTXT(json);
}

void handleLogs() {
    if (LittleFS.exists(LOG_FILE)) {
        File file = LittleFS.open(LOG_FILE, "r");
        server.streamFile(file, "text/plain");
        file.close();
    } else {
        server.send(200, "text/plain", "No logs found.");
    }
}

void handleClearLogs() {
    LittleFS.remove(LOG_FILE);
    log_info("Logs Cleared via Web.");
    server.send(200, "text/plain", "Logs cleared.");
}

void handleRoot() {
  String html = "<!DOCTYPE html><html><head><title>Gate</title><meta name='viewport' content='width=device-width, initial-scale=1'><style>";
  html += "body{font-family:sans-serif;margin:10px;text-align:center;background:#f4f4f4}";
  html += ".card{background:white;padding:15px;border-radius:10px;box-shadow:0 2px 5px rgba(0,0,0,0.1);max-width:500px;margin:10px auto}";
  html += ".btn{display:inline-block;padding:12px;color:white;text-decoration:none;border-radius:5px;margin:5px;font-weight:bold;cursor:pointer;border:none;font-size:16px;width:30%}";
  html += ".grn{background:#28a745}.red{background:#dc3545}.gry{background:#6c757d}.blu{background:#007bff}";
  html += ".sensor-row{display:flex;justify-content:space-between;padding:8px;background:#eee;border-bottom:1px solid #ddd}";
  html += "table{width:100%;border-collapse:collapse;margin-top:10px} th,td{border:1px solid #ddd;padding:8px}";
  html += "textarea{width:100%;height:100px;font-family:monospace;font-size:12px;margin-bottom:10px}";
  html += "</style></head><body>";

  html += "<div class='card'>";
  html += "<h1>Gate Control</h1>";
  html += "<p>Status: <b id='status'>...</b> <span id='sub_status' style='color:#007bff'></span></p>";
  html += "<p>Position: <b id='pos'>...</b>%</p>";
  html += "<button onclick=\"send('OPEN')\" class='btn grn'>OPEN</button>";
  html += "<button onclick=\"send('STOP')\" class='btn gry'>STOP</button>";
  html += "<button onclick=\"send('CLOSE')\" class='btn red'>CLOSE</button>";
  html += "</div>";

  html += "<div class='card'>";
  html += "<h3>Sensors</h3>";
  html += "<div class='sensor-row'><span>Open Limit:</span><span id='lo_ind'>-</span></div>";
  html += "<div class='sensor-row'><span>Close Limit:</span><span id='lc_ind'>-</span></div>";
  html += "<div class='sensor-row'><span>Barrier:</span><span id='pb_ind'>-</span></div>";
  html += "<div class='sensor-row'><span>Last RF:</span><b id='rf_val'>-</b></div>";
  html += "</div>";

  html += "<div class='card'>";
  html += "<h3>RF Remotes</h3>";
  html += "<button id='scanBtn' onclick=\"toggleScan()\" class='btn blu' style='width:90%'>Scan New Remote</button>";
  html += "<div id='scanPanel' style='display:none;background:#e9ecef;padding:10px;margin-top:10px'>";
  html += "<p>Press remote button...</p>";
  html += "<b>Code: <span id='scannedCode'>Waiting...</span></b><br><br>";
  html += "<select id='funcSel'>";
  html += "<option value='0'>OPEN</option><option value='1'>CLOSE</option>";
  html += "<option value='2'>STOP</option><option value='3'>PEDESTRIAN</option>";
  html += "<option value='4'>TOGGLE</option></select>";
  html += "<button onclick=\"addCode()\" class='btn grn'>ADD</button></div>";
  html += "<table id='rfTable'><thead><tr><th>Code</th><th>Func</th><th>Action</th></tr></thead><tbody></tbody></table>";
  html += "</div>";

  html += "<div class='card'>";
  html += "<h3>System</h3>";
  html += "<button onclick=\"send('CALIBRATE')\" class='btn blu' style='width:45%'>Calibrate</button>";
  html += "<a href='/config'><button class='btn gry' style='width:45%'>WiFi/MQTT</button></a><br><br>";
  html += "<textarea id='logArea' readonly>Loading...</textarea>";
  html += "<button onclick='fetchLogs()' class='btn gry'>Refresh</button>";
  html += "<button onclick='clearLogs()' class='btn red'>Clear</button>";
  html += "</div>";

  html += "<script>";
  html += "var ws = new WebSocket('ws://' + location.hostname + ':81/');";
  html += "var scanning = false; var lastScanned = 0;";
  html += "ws.onmessage = function(e) {";
  html += "  var d = JSON.parse(e.data);";
  html += "  if(d.type === 'status') {";
  html += "    document.getElementById('status').innerHTML = d.s;";
  html += "    document.getElementById('sub_status').innerHTML = d.ss;";
  html += "    document.getElementById('pos').innerHTML = d.p;";
  html += "    document.getElementById('lo_ind').innerHTML = d.lo==0?'<b style=\"color:red\">HIT</b>':'<span style=\"color:green\">FREE</span>';";
  html += "    document.getElementById('lc_ind').innerHTML = d.lc==0?'<b style=\"color:red\">HIT</b>':'<span style=\"color:green\">FREE</span>';";
  html += "    document.getElementById('pb_ind').innerHTML = d.pb==0?'<b style=\"color:red\">BLOCKED</b>':'<span style=\"color:green\">CLEAR</span>';";
  html += "    document.getElementById('rf_val').innerHTML = d.rf;";
  html += "    if(scanning && d.s === 'SCANNING' && d.ss !== 'Waiting...') {";
  html += "       lastScanned = d.ss; document.getElementById('scannedCode').innerHTML = lastScanned;";
  html += "    }";
  html += "  } else if (d.type === 'rf_list') { renderTable(d.data); }";
  html += "};";
  html += "function renderTable(list) {";
  html += "  var tb = document.querySelector('#rfTable tbody'); tb.innerHTML = '';";
  html += "  var funcs = ['OPEN', 'CLOSE', 'STOP', 'PED', 'TOGGLE'];";
  html += "  list.forEach((item, idx) => {";
  html += "    tb.innerHTML += '<tr><td>'+item.c+'</td><td>'+funcs[item.f]+'</td><td><button onclick=\"delCode('+idx+')\" style=\"background:red;color:white;border:none;border-radius:3px\">X</button></td></tr>';";
  html += "  });";
  html += "}";
  html += "function send(cmd) { ws.send(cmd); }";
  html += "function toggleScan() { scanning = !scanning;";
  html += "  document.getElementById('scanPanel').style.display = scanning ? 'block' : 'none';";
  html += "  document.getElementById('scanBtn').innerText = scanning ? 'Stop Scanning' : 'Scan New Remote';";
  html += "  ws.send(scanning ? 'SCAN_START' : 'SCAN_STOP'); }";
  html += "function addCode() { if(lastScanned != 0) { var f = document.getElementById('funcSel').value; ws.send('ADD:' + lastScanned + ':' + f); toggleScan(); } }";
  html += "function delCode(idx) { if(confirm('Delete?')) ws.send('DEL:' + idx); }";
  html += "function fetchLogs() { fetch('/logs').then(r => r.text()).then(t => document.getElementById('logArea').value = t); }";
  html += "function clearLogs() { fetch('/clear_logs').then(r => fetchLogs()); }";
  html += "fetchLogs();";
  html += "</script></body></html>";
  
  server.send(200, "text/html", html);
}

void handleConfigPage() {
  String html = "<!DOCTYPE html><html><head><title>Config</title><meta name='viewport' content='width=device-width, initial-scale=1'><style>";
  html += "body{font-family:sans-serif;margin:20px;background-color:#f4f4f4}";
  html += ".card{background:white;padding:20px;border-radius:10px;box-shadow:0 2px 5px rgba(0,0,0,0.1);max-width:400px;margin:auto}";
  html += "input{width:100%;padding:10px;margin:8px 0;box-sizing:border-box;border:1px solid #ccc;border-radius:4px}";
  html += "input[type=submit]{background-color:#007bff;color:white;cursor:pointer;padding:12px;border:none;border-radius:4px;width:100%}";
  html += "</style></head><body><div class='card'>";
  html += "<h2>Configuration</h2><form action='/save' method='POST'>";
  
  html += "<label>MQTT IP:</label><input type='text' name='mq_ip' value='" + String(mqtt_server) + "'>";
  html += "<label>MQTT Port:</label><input type='text' name='mq_pt' value='" + String(mqtt_port_str) + "'>";
  html += "<label>MQTT User:</label><input type='text' name='mq_us' value='" + String(mqtt_user) + "'>";
  html += "<label>MQTT Pass:</label><input type='password' name='mq_pw' value='" + String(mqtt_password) + "'>";
  html += "<label>Travel Time (ms):</label><input type='number' name='tt' value='" + String(gate_travel_time) + "'>";
  
  html += "<input type='submit' value='SAVE & REBOOT'>";
  html += "</form><br><center><a href='/'>Back to Controls</a></center></div></body></html>";
  server.send(200, "text/html", html);
}

void handleSavePage() {
  if (server.hasArg("mq_ip")) {
      String ip = server.arg("mq_ip");
      ip.trim();
      strncpy(mqtt_server, ip.c_str(), 40);
  }
  if (server.hasArg("mq_pt")) {
      strncpy(mqtt_port_str, server.arg("mq_pt").c_str(), 6);
  }
  if (server.hasArg("mq_us")) {
      strncpy(mqtt_user, server.arg("mq_us").c_str(), 32);
  }
  if (server.hasArg("mq_pw")) {
      strncpy(mqtt_password, server.arg("mq_pw").c_str(), 64);
  }
  if (server.hasArg("tt")) {
      gate_travel_time = server.arg("tt").toInt();
  }
  
  save_params();
  
  String html = "<html><head><meta http-equiv='refresh' content='5;url=/'></head><body>";
  html += "<h1>Settings Saved!</h1><p>Device is rebooting...</p></body></html>";
  server.send(200, "text/html", html);
  
  delay(500);
  ESP.restart();
}

void handleWebCommand() {
    String path = server.uri();
    if (path == "/open") send_command(CMD_OPEN);
    else if (path == "/close") send_command(CMD_CLOSE);
    else if (path == "/stop") send_command(CMD_STOP_ONLY);
    
    server.sendHeader("Location", "/");
    server.send(303);
}

// =================================================================
// Button & HA Callbacks
// =================================================================
void main_button_short_press(Button2 &btn) { send_command(CMD_TOGGLE); }
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

void onCoverCommand(HACover::CoverCommand cmd, HACover *sender)
{
    switch (cmd)
    {
    case HACover::CommandOpen: send_command(CMD_OPEN); break;
    case HACover::CommandClose: send_command(CMD_CLOSE); break;
    case HACover::CommandStop: send_command(CMD_STOP_ONLY); break;
    }
}
void onOpenCommand(HAButton *sender) { send_command(CMD_OPEN); }
void onCloseCommand(HAButton *sender) { send_command(CMD_CLOSE); }
void onStopCommand(HAButton *sender) { send_command(CMD_STOP_ONLY); }
void onCalibrateCommand(HAButton *sender) { send_command(CMD_CALIBRATE_START); }
void onMoveTo50Command(HAButton *sender) { send_command(CMD_MOVE_TO_POSITION, 0.5f); }

void publish_all_states()
{
    cover.setCurrentPosition(current_position * 100);
    char buf[16];
    snprintf(buf, sizeof(buf), "%lu", gate_travel_time / 1000);
    travelTimeSensor.setValue(buf);
    if (cal_state != CAL_INACTIVE) gateState.setValue("calibrating");
    else if (rf_learn_state != RF_LEARN_INACTIVE) gateState.setValue("rf_learning");
    else if (current_operation == OPENING) gateState.setValue("opening");
    else if (current_operation == CLOSING) gateState.setValue("closing");
    else if (current_position >= 0.99f) gateState.setValue("open");
    else if (current_position <= 0.01f) gateState.setValue("closed");
    else gateState.setValue("stopped");
    gateIP.setValue(WiFi.localIP().toString().c_str());

    limitOpenSensor.setState(digitalRead(LIMIT_OPEN_PIN) == LOW);
    limitCloseSensor.setState(digitalRead(LIMIT_CLOSE_PIN) == LOW);
    barrierSensor.setState(digitalRead(PHOTO_BARRIER_PIN) == LOW);
}

void move_to_position(float new_target)
{
    if (cal_state != CAL_INACTIVE) return;
    if (current_operation != IDLE) stop_movement(false);
    if (abs(current_position - new_target) < 0.01) return;
    target_position = new_target;
    if (new_target > current_position) execute_open_sequence();
    else execute_close_sequence();
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
        if (WiFi.status() != WL_CONNECTED) LOG_PRINTLN("WiFi Reconnecting...");
    }
}

void handle_rf_learning()
{
    if (millis() - rf_learn_start_time > RF_LEARN_TIMEOUT)
    {
        if (rf_learn_state == RF_SCANNING_WEB) {
            log_info("Web Scan Timeout");
            scanned_rf_code = 0;
        }
        rf_learn_state = RF_LEARN_INACTIVE;
        blink_interval = 0;
        maintenanceButton.setLongClickTime(CALIBRATION_LONG_PRESS_TIME);
        return;
    }
    
    if (!mySwitch.available()) return;
    unsigned long code = mySwitch.getReceivedValue();
    mySwitch.resetAvailable();
    if (code == 0) return;
    
    if (rf_learn_state == RF_SCANNING_WEB) {
        scanned_rf_code = code;
        log_info("Scanned Code: " + String(code));
        rf_learn_start_time = millis(); 
        return;
    }
    
    log_info("RF LEARN | Code: " + String(code));
    switch (rf_learn_state)
    {
    case RF_LEARN_WAIT_OPEN:
        rfKeyList.push_back({code, 0}); 
        log_info("Learned OPEN. Press CLOSE...");
        rf_learn_state = RF_LEARN_WAIT_CLOSE;
        rf_learn_start_time = millis();
        break;
    case RF_LEARN_WAIT_CLOSE:
        rfKeyList.push_back({code, 1}); 
        log_info("Learned CLOSE. Press STOP...");
        rf_learn_state = RF_LEARN_WAIT_STOP;
        rf_learn_start_time = millis();
        break;
    case RF_LEARN_WAIT_STOP:
        rfKeyList.push_back({code, 2}); 
        log_info("Learned STOP. Press 50%...");
        rf_learn_state = RF_LEARN_WAIT_POS50;
        rf_learn_start_time = millis();
        break;
    case RF_LEARN_WAIT_POS50:
        rfKeyList.push_back({code, 3}); 
        log_info("Learned 50%. Saving...");
        save_params();
        rf_learn_state = RF_LEARN_INACTIVE;
        blink_interval = 0;
        maintenanceButton.setLongClickTime(CALIBRATION_LONG_PRESS_TIME);
        break;
    default: break;
    }
}

void handle_rf_signal()
{
    if (!mySwitch.available()) return;
    unsigned long code = mySwitch.getReceivedValue();
    unsigned long now = millis();
    bool is_repeat = false;
    if (code == last_rf_code_received)
    {
        if (now - last_rf_process_time < RF_DEBOUNCE_DELAY) is_repeat = true;
    }
    last_rf_code_received = code;
    last_rf_process_time = now;
    mySwitch.resetAvailable();
    if (code == 0 || is_repeat) return;

    char code_str[20];
    sprintf(code_str, "%lu", last_rf_code_received);
    rfCodeSensor.setValue(code_str);
    
    for (const auto& key : rfKeyList) {
        if (key.code == code) {
            log_info("RF Match: " + String(code) + " Func: " + String(key.function));
            switch (key.function) {
                case 0: send_command(CMD_OPEN); break;
                case 1: send_command(CMD_CLOSE); break;
                case 2: send_command(CMD_STOP_ONLY); break;
                case 3: send_command(CMD_MOVE_TO_POSITION, 0.5f); break;
                case 4: send_command(CMD_TOGGLE); break;
            }
            return;
        }
    }
}

void handle_motor_relays()
{
#if MOTOR_CONTROL_MODE == 1
    if (motor_relay_state == R_OFF) return;
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
                if (next_operation == OPENING) digitalWrite(RELAY_MOTOR_OPEN_PIN, HIGH);
                else if (next_operation == CLOSING) digitalWrite(RELAY_MOTOR_CLOSE_PIN, HIGH);
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
    if (current_operation == OPENING) return;
    current_operation = OPENING;
    if (WiFi.status() == WL_CONNECTED) cover.setState(HACover::StateOpening);
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
    if (current_operation == CLOSING) return;
    current_operation = CLOSING;
    if (WiFi.status() == WL_CONNECTED) cover.setState(HACover::StateClosing);
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
    if (current_operation != IDLE || cal_state != CAL_INACTIVE) return;
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
    if (current_operation != IDLE || cal_state != CAL_INACTIVE) return;
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

void start_calibration()
{
    if (current_operation != IDLE || cal_state != CAL_INACTIVE || rf_learn_state != RF_LEARN_INACTIVE) return;
    log_info("--- Starting Calibration ---");
    cal_state = CAL_HOMING_CLOSE;
    blink_interval = CALIBRATION_BLINK_INTERVAL;
    calibration_start_time = millis();
}

void handle_calibration()
{
    if (millis() - calibration_start_time > CALIBRATION_SAFETY_TIMEOUT)
    {
        log_info("CAL: FAILED (Timeout)");
        stop_movement(false);
        cal_state = CAL_INACTIVE;
        blink_interval = 0;
        return;
    }

    switch (cal_state)
    {
    case CAL_HOMING_CLOSE:
        if (digitalRead(LIMIT_CLOSE_PIN) == LOW)
        {
            log_info("CAL: Homing Done. Opening...");
            stop_movement(false);
            delay(500); 
            cal_state = CAL_MEASURING_OPEN;
            movement_start_time = millis();
            execute_open_sequence();
        }
        else if (current_operation != CLOSING)
        {
            execute_close_sequence();
        }
        break;

    case CAL_MEASURING_OPEN:
        if (digitalRead(LIMIT_OPEN_PIN) == LOW)
        {
            gate_travel_time = millis() - movement_start_time;
            log_info("CAL: Measured Time: " + String(gate_travel_time) + "ms");
            stop_movement(false);
            delay(500);
            cal_state = CAL_VERIFYING_CLOSE;
            log_info("CAL: Verifying Close...");
            execute_close_sequence();
        }
        break;

    case CAL_VERIFYING_CLOSE:
        if (digitalRead(LIMIT_CLOSE_PIN) == LOW)
        {
            stop_movement(false);
            log_info("CAL: Calibration Complete & Saved.");
            save_params();
            cal_state = CAL_DONE;
        }
        break;

    case CAL_DONE:
        cal_state = CAL_INACTIVE;
        blink_interval = 0;
        current_position = 0.0f;
        publish_all_states();
        break;

    default: break;
    }
}

void stop_movement(bool triggered_by_user)
{
    if (current_operation == IDLE && cal_state == CAL_INACTIVE && !auto_resume_is_armed) return;
    if (triggered_by_user && current_operation != IDLE)
        last_operation_before_stop = (current_position > 0.01f && current_position < 0.99f) ? current_operation : IDLE;
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
        if (current_position >= 0.99f) cover.setState(HACover::StateOpen);
        else if (current_position <= 0.01f) cover.setState(HACover::StateClosed);
        else cover.setState(HACover::StateStopped);
    }
    cover.setCurrentPosition(current_position * 100);
    if (triggered_by_user)
    {
        target_position = -1.0;
        if (cal_state != CAL_INACTIVE) cal_state = CAL_INACTIVE;
        auto_resume_is_armed = false;
    }
}

void update_gate_position()
{
#if MOTOR_CONTROL_MODE == 1
    if (motor_relay_state != R_OFF) return;
#elif MOTOR_CONTROL_MODE == 2
    if (motor_change_state != M_IDLE) return;
#endif
    if (cal_state != CAL_INACTIVE) return;

    unsigned long elapsed = millis() - movement_start_time;
    float ratio = (gate_travel_time > 0) ? (float(elapsed) / float(gate_travel_time)) : 1.0f;
    if (current_operation == OPENING) current_position = movement_start_position + ratio;
    else if (current_operation == CLOSING) current_position = movement_start_position - ratio;
    current_position = constrain(current_position, 0.0f, 1.0f);
    if (target_position >= 0.0)
    {
        if ((current_operation == OPENING && current_position >= target_position) || (current_operation == CLOSING && current_position <= target_position))
        {
            stop_movement(false);
            current_position = target_position;
            target_position = -1.0;
        }
    }
    if (gate_travel_time > 0 && elapsed > (gate_travel_time + 3000))
    {
        LOG_PRINTLN("Timeout!");
        stop_movement(false);
        target_position = -1.0;
    }
}

void handle_safety_sensors()
{
    if (cal_state != CAL_INACTIVE) return; 
    if (current_operation == OPENING && digitalRead(LIMIT_OPEN_PIN) == LOW)
    {
        stop_movement(false);
        current_position = 1.0f;
    }
    if (current_operation == CLOSING && digitalRead(LIMIT_CLOSE_PIN) == LOW)
    {
        stop_movement(false);
        current_position = 0.0f;
    }
    if (digitalRead(PHOTO_BARRIER_PIN) == LOW && current_operation == CLOSING)
    {
        LOG_PRINTLN("Barrier!");
        stop_movement(false);
        auto_resume_is_armed = true;
        execute_open_sequence();
    }
}

// =================================================================
// DEDICATED TASKS
// =================================================================

void vLedTask(void *pvParameters)
{
    if (ENABLE_HW_WATCHDOG) esp_task_wdt_add(NULL);
    const TickType_t xFrequency = pdMS_TO_TICKS(100);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        last_checkin_led = millis();
        if (ENABLE_HW_WATCHDOG) esp_task_wdt_reset();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(20)) == pdTRUE)
        {
            handle_indicator_light();
            xSemaphoreGive(xStateMutex);
        }
    }
}

void vLoggerTask(void *pvParameters)
{
    if (ENABLE_HW_WATCHDOG) esp_task_wdt_add(NULL);
    const TickType_t xFrequency = pdMS_TO_TICKS(5000);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        last_checkin_logger = millis();
        if (ENABLE_HW_WATCHDOG) esp_task_wdt_reset();
        if (simulate_crash_logger) while (1) vTaskDelay(1);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            log_io_status();
            xSemaphoreGive(xStateMutex);
        }
    }
}

void vGateLogicTask(void *pvParameters)
{
    if (ENABLE_HW_WATCHDOG) esp_task_wdt_add(NULL);
    CommandMessage msg;
    while (1)
    {
        last_checkin_gate = millis();
        if (ENABLE_HW_WATCHDOG) esp_task_wdt_reset();
        if (simulate_crash_gate) while (1) vTaskDelay(1);
        if (xQueueReceive(xCommandQueue, &msg, pdMS_TO_TICKS(100)) == pdPASS)
        {
            if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
            {
                if (msg.cmd == CMD_WIFI_CONFIG_START)
                {
                    wifi_config_request = true;
                }
                else if (msg.cmd == CMD_RF_LEARN_SAVE_EXIT)
                {
                    log_info("Saving codes.");
                    save_params();
                    rf_learn_state = RF_LEARN_INACTIVE;
                    blink_interval = 0;
                    maintenanceButton.setLongClickTime(CALIBRATION_LONG_PRESS_TIME);
                }
                else if (msg.cmd == CMD_RF_SCAN_MODE)
                {
                    // Start or Stop scan mode
                    rf_learn_state = (msg.aux_data == 1) ? RF_SCANNING_WEB : RF_LEARN_INACTIVE;
                    rf_learn_start_time = millis();
                    scanned_rf_code = 0;
                    broadcastStatus();
                }
                else if (msg.cmd == CMD_RF_ADD_CODE)
                {
                    rfKeyList.push_back({msg.code, (uint8_t)msg.aux_data});
                    save_params();
                    log_info("Added RF Code: " + String(msg.code));
                    sendRFListToWeb();
                }
                else if (msg.cmd == CMD_RF_DELETE_CODE)
                {
                    if (msg.aux_data >= 0 && msg.aux_data < rfKeyList.size()) {
                        rfKeyList.erase(rfKeyList.begin() + msg.aux_data);
                        save_params();
                        log_info("Deleted RF Code at index " + String(msg.aux_data));
                        sendRFListToWeb();
                    }
                }
                else if (msg.cmd == CMD_RF_LEARN_SKIP) 
                {
                    // Logic for skipping or advancing the learning state manually
                    if (rf_learn_state != RF_LEARN_INACTIVE && rf_learn_state != RF_SCANNING_WEB) {
                        rf_learn_state = (RFLearningState)(rf_learn_state + 1);
                        if (rf_learn_state > RF_LEARN_WAIT_POS50) {
                            save_params();
                            rf_learn_state = RF_LEARN_INACTIVE;
                            blink_interval = 0;
                            maintenanceButton.setLongClickTime(CALIBRATION_LONG_PRESS_TIME);
                        }
                        rf_learn_start_time = millis();
                        log_info("Skipped RF step.");
                    }
                }
                else if (cal_state == CAL_INACTIVE && rf_learn_state == RF_LEARN_INACTIVE)
                {
                    switch (msg.cmd)
                    {
                    case CMD_OPEN: start_opening(); break;
                    case CMD_CLOSE: start_closing(); break;
                    case CMD_STOP_ONLY: if (current_operation != IDLE) stop_movement(true); break;
                    case CMD_TOGGLE:
                        if (current_operation != IDLE) stop_movement(true);
                        else if (last_operation_before_stop != IDLE) { if (last_operation_before_stop == OPENING) start_opening(); else start_closing(); }
                        else if (current_position >= 0.99f) start_closing();
                        else start_opening();
                        break;
                    case CMD_REVERSE: if (current_operation == OPENING || (current_operation == IDLE && current_position >= 0.99f)) execute_close_sequence(); else execute_open_sequence(); break;
                    case CMD_CALIBRATE_START: start_calibration(); break;
                    case CMD_RF_LEARN_START: rf_learn_state = RF_LEARN_WAIT_OPEN; rf_learn_start_time = millis(); blink_interval = RF_LEARN_BLINK_INTERVAL; maintenanceButton.setLongClickTime(RF_LEARN_SAVE_LONG_PRESS_TIME); log_info("--- RF Learn ---"); break;
                    case CMD_MOVE_TO_POSITION: move_to_position(msg.position); break;
                    case CMD_STOP_INTERNAL: stop_movement(false); break;
                    default: break;
                    }
                }
                xSemaphoreGive(xStateMutex);
            }
        }
    }
}

void vMotorTask(void *pvParameters)
{
    if (ENABLE_HW_WATCHDOG) esp_task_wdt_add(NULL);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        last_checkin_motor = millis();
        if (ENABLE_HW_WATCHDOG) esp_task_wdt_reset();
        if (simulate_crash_motor) while (1) vTaskDelay(1);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));
        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(1)) == pdTRUE)
        {
            handle_safety_sensors();
            handle_motor_relays();
            if (current_operation != IDLE) update_gate_position();
            if (cal_state != CAL_INACTIVE) handle_calibration();
            xSemaphoreGive(xStateMutex);
        }
    }
}

void vRFTask(void *pvParameters)
{
    if (ENABLE_HW_WATCHDOG) esp_task_wdt_add(NULL);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        last_checkin_rf = millis();
        if (ENABLE_HW_WATCHDOG) esp_task_wdt_reset();
        if (simulate_crash_rf) while (1) vTaskDelay(1);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
        if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(5)) == pdTRUE)
        {
            if (rf_learn_state != RF_LEARN_INACTIVE) handle_rf_learning();
            else handle_rf_signal();
            xSemaphoreGive(xStateMutex);
        }
    }
}

void vButtonTask(void *pvParameters)
{
    if (ENABLE_HW_WATCHDOG) esp_task_wdt_add(NULL);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    static unsigned long wifi_press_start = 0;
    static unsigned long maint_press_start = 0;
    static unsigned long combo_press_start = 0;
    static bool combo_triggered = false;
    static unsigned long last_print_combo = 0;

    while (1)
    {
        last_checkin_button = millis();
        if (ENABLE_HW_WATCHDOG) esp_task_wdt_reset();
        if (simulate_crash_button) while (1) vTaskDelay(1);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));

        mainButton.loop();
        maintenanceButton.loop();
        wifiButton.loop();

        bool w = wifiButton.isPressed();
        bool m = maintenanceButton.isPressed();

        if (w && m)
        {
            if (combo_press_start == 0) combo_press_start = millis();
            unsigned long held = millis() - combo_press_start;
            if (held < COMBO_MODE_HOLD_TIME)
            {
                if (millis() - last_print_combo > 1000)
                {
                    last_print_combo = millis();
                    LOG_PRINTF("RF Learn in %lu s...\n", (COMBO_MODE_HOLD_TIME - held) / 1000 + 1);
                }
            }
            else if (!combo_triggered)
            {
                LOG_PRINTLN("Combo: RF Learn Mode!");
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
            if (w)
            {
                if (wifi_press_start == 0) wifi_press_start = millis();
                unsigned long held = millis() - wifi_press_start;
                if (held < WIFI_CONFIG_LONG_PRESS_TIME)
                {
                    if (millis() - last_print_combo > 1000)
                    {
                        last_print_combo = millis();
                        LOG_PRINTF("WiFi Config in %lu s...\n", (WIFI_CONFIG_LONG_PRESS_TIME - held) / 1000 + 1);
                    }
                }
            }
            else
            {
                if (wifi_press_start > 0 && (millis() - wifi_press_start > WIFI_CONFIG_LONG_PRESS_TIME))
                {
                    send_command(CMD_WIFI_CONFIG_START);
                }
                wifi_press_start = 0;
            }
            if (m) { if (maint_press_start == 0) maint_press_start = millis(); }
            else maint_press_start = 0;
        }
    }
}

void vNetworkTask(void *pvParameters)
{
    if (ENABLE_HW_WATCHDOG) esp_task_wdt_add(NULL);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    int mqttCounter = 0;
    // [NEW] Last broadcast time
    unsigned long last_ws_broadcast = 0;

    wm.setConfigPortalBlocking(false);

    while (1)
    {
        last_checkin_network = millis();
        if (ENABLE_HW_WATCHDOG) esp_task_wdt_reset();
        if (simulate_crash_network) while (1) vTaskDelay(1);

        wm.process();

        if (wifi_config_request)
        {
            LOG_PRINTLN("Manual Config Portal Requested...");
            if (ENABLE_HW_WATCHDOG) esp_task_wdt_delete(NULL);
            wm.setConfigPortalBlocking(true);
            wm.startConfigPortal("GateControllerAP");
            wm.setConfigPortalBlocking(false);
            if (ENABLE_HW_WATCHDOG) esp_task_wdt_add(NULL);
            wifi_config_request = false;
            LOG_PRINTLN("Portal Closed.");
        }

        // Web server tick (approx every 25ms due to delay below)
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(25));

        if (WiFi.status() == WL_CONNECTED)
        {
            if (!web_server_started) {
                server.begin();
                webSocket.begin(); // Start WebSocket
                webSocket.onEvent(onWebSocketEvent);
                MDNS.begin("gate");
                // Register new routes
                server.on("/logs", handleLogs);
                server.on("/clear_logs", handleClearLogs);
                web_server_started = true;
                LOG_PRINTLN("Web+WS Server Started");
            }
            server.handleClient();
            webSocket.loop(); // Handle WS events
            mqtt.loop();
            
            // [NEW] Broadcast Status periodically via WebSocket
            // Updates more frequently (250ms) if moving, less (2000ms) if idle
            unsigned long interval = (current_operation != IDLE) ? 250 : 2000;
            if (millis() - last_ws_broadcast > interval) {
                broadcastStatus();
                last_ws_broadcast = millis();
            }
            
            // Telnet Handling
            if (telnetServer.hasClient())
            {
                if (!telnetClient || !telnetClient.connected())
                {
                    if (telnetClient) telnetClient.stop();
                    telnetClient = telnetServer.accept();
                    LOG_PRINTLN("Telnet connected!");
                }
            }
            if (telnetClient && telnetClient.connected() && telnetClient.available())
            {
                String cmd = telnetClient.readStringUntil('\n');
                cmd.trim();
                if (cmd.equalsIgnoreCase("logs")) dump_system_log();
                else if (cmd.equalsIgnoreCase("conf")) dump_config();
                else if (cmd.equalsIgnoreCase("clear_logs")) { LittleFS.remove(LOG_FILE); LOG_PRINTLN("Logs cleared."); }
                else if (cmd.equalsIgnoreCase("restart")) { LOG_PRINTLN("Restarting..."); delay(500); ESP.restart(); }
                else if (cmd.equalsIgnoreCase("crash motor")) { simulate_crash_motor = true; LOG_PRINTLN("Simulating MOTOR crash..."); }
                else if (cmd.equalsIgnoreCase("crash network")) { simulate_crash_network = true; LOG_PRINTLN("Simulating NETWORK crash..."); }
                else if (cmd.equalsIgnoreCase("crash input")) { simulate_crash_button = true; LOG_PRINTLN("Simulating INPUT crash..."); }
                else if (cmd.equalsIgnoreCase("crash logger")) { simulate_crash_logger = true; LOG_PRINTLN("Simulating LOGGER crash..."); }
                else if (cmd.equalsIgnoreCase("learn_rf")) { send_command(CMD_RF_LEARN_START); }
                else if (cmd.equalsIgnoreCase("start_cal")) { send_command(CMD_CALIBRATE_START); }
                else if (cmd.equalsIgnoreCase("wifi")) { wifi_config_request = true; }
            }
        }
        handle_wifi_status();

        mqttCounter++;
        if (mqttCounter >= 200) // Adjusted for faster loop (25ms * 200 = 5s)
        {
            mqttCounter = 0;
            if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) { publish_all_states(); xSemaphoreGive(xStateMutex); }
        }
    }
}

void setup()
{
    xLogMutex = xSemaphoreCreateMutex();
    xStateMutex = xSemaphoreCreateMutex();
    xMotorRelayMutex = xSemaphoreCreateMutex();
    xCommandQueue = xQueueCreate(10, sizeof(CommandMessage));

#if USE_SERIAL_DEBUG
    Serial.begin(115200);
#endif
    delay(1000);
    LOG_PRINTLN("\nGate Controller Starting...");

    preferences.begin("gate_stats", false);
    bootCount = preferences.getUInt("boot_count", 0);
    bootCount++;
    preferences.putUInt("boot_count", bootCount);
    preferences.end();

    if (ENABLE_HW_WATCHDOG)
    {
        esp_task_wdt_deinit();
        esp_task_wdt_config_t twdt_config = {.timeout_ms = HW_WDT_TIMEOUT * 1000, .idle_core_mask = (1 << 0), .trigger_panic = true};
        esp_task_wdt_init(&twdt_config);
        esp_task_wdt_add(NULL);
    }

    if (!LittleFS.begin(true)) LOG_PRINTLN("LittleFS Failed!");
    check_reset_reason();
    
    // Load Params
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
    device.setManufacturer("Custom");

    // --- CONFIGURE HOME ASSISTANT ENTITIES ---
    
    // Main Cover
    cover.setName("Sliding Gate");
    cover.setDeviceClass("gate");
    cover.onCommand(onCoverCommand);

    // Control Buttons
    openButtonHA.setName("Open Gate");
    openButtonHA.setIcon("mdi:gate-open");
    openButtonHA.onCommand(onOpenCommand);

    closeButtonHA.setName("Close Gate");
    closeButtonHA.setIcon("mdi:gate");
    closeButtonHA.onCommand(onCloseCommand);

    stopButtonHA.setName("Stop Gate");
    stopButtonHA.setIcon("mdi:stop-circle-outline");
    stopButtonHA.onCommand(onStopCommand);

    calibrateButton.setName("Calibrate Gate");
    calibrateButton.setIcon("mdi:ruler");
    calibrateButton.onCommand(onCalibrateCommand);

    moveTo50Button.setName("Pedestrian (50%)");
    moveTo50Button.setIcon("mdi:walk");
    moveTo50Button.onCommand(onMoveTo50Command);

    // Sensors
    rfCodeSensor.setName("Last RF Code");
    rfCodeSensor.setIcon("mdi:remote");
    
    gateState.setName("Gate Logic State");
    gateState.setIcon("mdi:state-machine");

    gateIP.setName("Gate IP Address");
    gateIP.setIcon("mdi:ip-network");

    travelTimeSensor.setName("Travel Duration");
    travelTimeSensor.setIcon("mdi:timer-outline");
    travelTimeSensor.setUnitOfMeasurement("s");

    // NEW: Limit Switches and Barrier Sensors configuration
    limitOpenSensor.setName("Limit Switch (Open)");
    limitOpenSensor.setIcon("mdi:arrow-left-bold-box-outline");
    
    limitCloseSensor.setName("Limit Switch (Closed)");
    limitCloseSensor.setIcon("mdi:arrow-right-bold-box-outline");

    barrierSensor.setName("Safety Barrier");
    barrierSensor.setDeviceClass("safety"); // Shows "Unsafe" when active (true)
    barrierSensor.setIcon("mdi:shield-alert");

    mainButton.begin(MANUAL_MAIN_BUTTON_PIN, INPUT_PULLUP, true);
    mainButton.setReleasedHandler(main_button_short_press);
    mainButton.setLongClickHandler(main_button_long_press);
    mainButton.setLongClickTime(REVERSE_LONG_PRESS_TIME);
    mainButton.setDebounceTime(50); 

    maintenanceButton.begin(MANUAL_MAINTENANCE_BUTTON_PIN, INPUT_PULLUP, true);
    maintenanceButton.setReleasedHandler(maintenance_button_short_press);
    maintenanceButton.setLongClickHandler(maintenance_button_long_press);
    maintenanceButton.setLongClickTime(CALIBRATION_LONG_PRESS_TIME);
    maintenanceButton.setDebounceTime(50);

    wifiButton.begin(MANUAL_WIFI_BUTTON_PIN, INPUT_PULLUP, true);
    wifiButton.setReleasedHandler(wifi_button_short_press);
    wifiButton.setLongClickHandler(wifi_config_long_press);
    wifiButton.setLongClickTime(WIFI_CONFIG_LONG_PRESS_TIME);
    wifiButton.setDebounceTime(50);

    // Setup Custom Web Server Routes
    server.on("/", handleRoot);
    server.on("/config", handleConfigPage);
    server.on("/save", HTTP_POST, handleSavePage);
    server.onNotFound(handleWebCommand); // Handles /open, /close, /stop

    mySwitch.enableReceive(RF_RECEIVER_PIN);
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);

    wm.setConfigPortalBlocking(false);
    // [CHANGE] No custom params added to WM anymore
    wm.setDebugOutput(false);

    if (wm.autoConnect("GateControllerAP"))
    {
        LOG_PRINTLN("Connected...yeey :)");
    }
    else
    {
        LOG_PRINTLN("Not connected, Config Portal running in background...");
    }

    uint16_t port = atoi(mqtt_port_str);
    mqtt.begin(mqtt_server, port, mqtt_user, mqtt_password);
    telnetServer.begin();

    unsigned long now = millis();
    last_checkin_motor = now;
    last_checkin_network = now;
    last_checkin_rf = now;
    last_checkin_button = now;
    last_checkin_gate = now;
    last_checkin_logger = now;
    last_checkin_led = now;

    xTaskCreate(vSupervisorTask, "Supervisor", 3072, NULL, 5, NULL);
    xTaskCreate(vMotorTask, "Motor", 4096, NULL, 4, NULL);
    xTaskCreate(vGateLogicTask, "GateLogic", 4096, NULL, 3, NULL);
    xTaskCreate(vRFTask, "RF", 3072, NULL, 3, NULL);
    xTaskCreate(vButtonTask, "Button", 3072, NULL, 3, NULL);
    xTaskCreate(vNetworkTask, "Network", 4096, NULL, 2, &hNetworkTask);
    xTaskCreate(vLedTask, "LED", 2048, NULL, 1, NULL);
    xTaskCreate(vLoggerTask, "Logger", 4096, NULL, 1, NULL);

    LOG_PRINTLN("Setup Complete. 8 Tasks Running.");
}

void loop()
{
    if (ENABLE_HW_WATCHDOG) esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(1000));
}