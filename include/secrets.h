#pragma once
// ============================================================
//  secrets.h  –  Credentials (DO NOT commit to version control)
// ============================================================

// WiFi (used by WiFiManager as fallback only – normally loaded from portal)
#define WIFI_SSID     "Link2"
#define WIFI_PASSWORD "Horvat2017"

// Default MQTT broker (overridden by NVS after first /save)
#define MQTT_SERVER   "192.168.1.11"
#define MQTT_PORT     1883
#define MQTT_USER     "admin"
#define MQTT_PASSWORD "admin"

// Home Assistant long-lived access token (optional, API use only)
static const char *api_encryption_key =
    "wExrjPMnKzjXGbfT/kh7TTwGPclmWk9povfOy9dEKDM=";

// OTA password
static const char *ota_password = "57c9aa71812b6d793d95831a0dcb8594";
