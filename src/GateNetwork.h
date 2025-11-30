#ifndef NETWORK_MANAGER_H
#define NETWORK_MANAGER_H

#include "Definitions.h"
#include "ConfigManager.h"
#include "MotorController.h"
#include "RFManager.h"
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoHA.h>
#include <WiFiManager.h>
#include <ESPmDNS.h>

class GateNetwork {
// ---------------------------------------------------------
// CRITICAL: DEVICE & MQTT MUST BE DEFINED FIRST
// ---------------------------------------------------------
private: 
    // They must be initialized before any Button/Sensor tries to use them
    HADevice device;
    HAMqtt mqtt;

public:
    GateNetwork();
    void begin();
    void loop();
    void broadcastStatus();
    void triggerWifiConfig();
    void sendRFListToWeb();
    
    bool simCrashNetwork = false;
    volatile bool updateRequest = false; 

    // Public Entities (Must be defined AFTER 'device')
    HAButton btnOpen;
    HAButton btnClose;
    HAButton btnStop;
    HAButton btnCalibrate;
    HAButton btnCancelCal;
    HAButton btnMove50;
    HANumber pedWidthNumber; 

private:
    WebServer server;
    WebSocketsServer webSocket;
    WiFiManager wm;
    WiFiClient wifiClient;
    WiFiServer telnetServer;
    
    // Other Entities
    HACover haCover;
    HASensor rfCodeSensor;
    HASensor gateState;
    HASensor gateIP;
    HASensor travelTime;
    HABinarySensor limOpen;
    HABinarySensor limClose;
    HABinarySensor barrier;

    bool webServerStarted = false;
    bool configPortalRequested = false;
    unsigned long lastBroadcast = 0;
    unsigned long lastWifiCheck = 0;
    int mqttCounter = 0;

    void setupWebRoutes();
    void handleRoot();
    void handleConfig();
    void handleSave();
    void handleLogs();
    void handleClearLogs();
    void handleWebCommand();
    
    void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);

    static void onCoverCommand(HACover::CoverCommand cmd, HACover *sender);
    static void onButtonCommand(HAButton* sender);
    static void onPedWidthChange(HANumeric number, HANumber* sender);
};

extern GateNetwork netManager;
extern WiFiClient telnetClient;

#endif