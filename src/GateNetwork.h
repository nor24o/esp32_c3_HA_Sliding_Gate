#pragma once
/**
 * @file  Network.h
 * @brief WiFi, async HTTP + WebSocket server, Home Assistant MQTT integration,
 *        Telnet debug console.
 */

#include "Config.h"
#include "Types.h"
#include "Storage.h"
#include "Motor.h"
#include "RF.h"

#include <ESPAsyncWebServer.h>    // 2nd — skips its conflicting typedef
#include <AsyncTCP.h>
#include <WiFi.h>
#include <ArduinoHA.h>
#include <ESPmDNS.h>

// ─────────────────────────────────────────────────────────────────────────────
class GateNetwork
{
    // Construction order is critical for ArduinoHA:
    // wifiClient must be fully constructed before mqtt references it.
private:
    WiFiClient _wifiClient;   // 1st
    HADevice   _device;       // 2nd
    HAMqtt     _mqtt;         // 3rd
    byte       _mac[6];       // Persistent MAC array for HA

public:
    GateNetwork();
    void begin();
    void loop();              ///< called from vNetworkTask every 25 ms

    void requestUpdate() { _update = true; }   ///< force immediate broadcast
    void startWifiPortal()   { _portalReq = true; }

    void sendRFList();
    void broadcastStatus();

    // Public HA entities (must be declared after _device / _mqtt are constructed)
    HAButton btnOpen;
    HAButton btnClose;
    HAButton btnStop;
    HAButton btnCalibrate;
    HAButton btnCancelCal;
    HAButton btnPed;
    HASwitch swHoldOpen;
    HANumber pedWidth;

private:
    // Web stack — single port 80, WebSocket at /ws
    AsyncWebServer _server;
    AsyncWebSocket _ws;
    WiFiServer     _telnet;

    // HA entities
    HACover        _cover;
    HASensor       _sLastRF;
    HASensor       _sState;
    HASensor       _sIP;
    HASensor       _sTravelTime;
    HASensor       _sPosition;
    HABinarySensor _sLimOpen;
    HABinarySensor _sLimClose;
    HABinarySensor _sBarrier;

    volatile bool _update    = false;
    bool          _wsStarted = false;
    bool          _portalReq = false;
    unsigned long _wifiCheck = 0;
    int           _tick      = 0;
    
    RFLearnState  _lastRfState = RFLearnState::INACTIVE;
    unsigned long _rfStateMs   = 0;

    // Web
    void _startWebServer();
    void _handleRoot      (AsyncWebServerRequest *req);
    void _handleSettings  (AsyncWebServerRequest *req);
    void _handleSave      (AsyncWebServerRequest *req);
    void _handleLogs      (AsyncWebServerRequest *req);
    void _handleClearLogs (AsyncWebServerRequest *req);
    void _onWsEvent(AsyncWebSocket *srv, AsyncWebSocketClient *client,
                    AwsEventType type, void *arg, uint8_t *data, size_t len);
    void _handleWsText(const String &text);

    // Telnet
    void _pollTelnet();

    // HA static callbacks
    static void _onCover (HACover::CoverCommand cmd, HACover *s);
    static void _onButton(HAButton *s);
    static void _onSwitch(bool state, HASwitch *s);
    static void _onPedWidth(HANumeric n, HANumber *s);
};

extern GateNetwork gateNet;
extern WiFiClient telnetClient;   // shared with Log.hpp
