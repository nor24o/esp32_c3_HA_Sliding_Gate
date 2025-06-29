#pragma once

// --- WiFi Credentials ---
#define WIFI_SSID "dlink"
#define WIFI_PASSWORD "horvat2017"

// --- RF Remote Codes ---
// Replace these with the actual codes from your remote
#define RF_GATE_OPEN_CODE 8671458
#define RF_GATE_CLOSE_CODE 8671457
#define RF_GATE_STOP_CODE 8671460
#define RF_GATE_POS50_CODE 8671459 // New code for moving to 50% position

// --- MQTT Broker Configuration ---
#define MQTT_SERVER "192.168.1.11"
#define MQTT_PORT 1883
#define MQTT_USER "admin"
#define MQTT_PASSWORD "admin"
// A unique name for this device in MQTT, used for all topics
#define MQTT_CLIENT_ID "sliding_gate_controller"

// --- Home Assistant API ---
// This key is for API communication, if you connect to Home Assistant
const char* api_encryption_key = "wExrjPMnKzjXGbfT/kh7TTwGPclmWk9povfOy9dEKDM=";

// --- OTA Update ---
const char* ota_password = "57c9aa71812b6d793d95831a0dcb8594";