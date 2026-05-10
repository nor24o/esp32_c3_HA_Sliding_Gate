/**
 * @file Network.cpp
 */

#include "GateNetwork.h"
#include "Motor.h"
#include "RF.h"
#include "Storage.h"
#include "Log.hpp"
#include "WifiWrapper.h"
#include <LittleFS.h>
#include <WiFi.h>
#include <Update.h>
#include <esp_timer.h>

GateNetwork gateNet;
WiFiClient  telnetClient;

// ── Helper: post a command to the gate logic queue ───────────────────────────
static void postCmd(GateCommand cmd, float pos = -1.0f,
                    unsigned long code = 0, int aux = 0)
{
    Command msg = { cmd, pos, code, aux };
    xQueueSend(cmdQueue, &msg, 0);
}

// ─────────────────────────────────────────────────────────────────────────────
// Constructor — initialiser list order must match declaration order in .h
// ─────────────────────────────────────────────────────────────────────────────
GateNetwork::GateNetwork()
    : _mqtt(_wifiClient, _device)
    , btnOpen("gate_btn_open"),     btnClose("gate_btn_close")
    , btnStop("gate_btn_stop"),     btnCalibrate("gate_btn_cal")
    , btnCancelCal("gate_btn_cal_cancel"), btnPed("gate_btn_ped")
    , swHoldOpen("gate_hold_open")
    , pedWidth("gate_ped_width")
    , _server(80)
    , _ws("/ws")
    , _telnet(23)
    , _cover("gate_cover")
    , _sLastRF("gate_last_rf"),     _sState("gate_state")
    , _sIP("gate_ip"),              _sTravelTime("gate_travel_time")
    , _sPosition("gate_position")
    , _sLimOpen("gate_lim_open"),   _sLimClose("gate_lim_close")
    , _sBarrier("gate_barrier")
{
    _device.setName("Sliding Gate");
    _device.setModel("ESP32-C3");
    _device.setManufacturer("H_N");
    _device.setSoftwareVersion("1.0.0");

    _cover.setName("Sliding Gate"); _cover.setDeviceClass("gate");
    _cover.onCommand(_onCover);

    auto btn = [](HAButton &b, const char *name, const char *icon) {
        b.setName(name); b.setIcon(icon); b.onCommand(_onButton);
    };
    btn(btnOpen,      "Open Gate",   "mdi:gate-open");
    btn(btnClose,     "Close Gate",  "mdi:gate");
    btn(btnStop,      "Stop Gate",   "mdi:stop-circle-outline");
    btn(btnCalibrate, "Calibrate",   "mdi:ruler");
    btn(btnCancelCal, "Cancel Cal",  "mdi:cancel");
    btn(btnPed,       "Pedestrian",  "mdi:walk");

    swHoldOpen.setName("Hold Open (Party Mode)");
    swHoldOpen.setIcon("mdi:lock-open-variant");
    swHoldOpen.onCommand(_onSwitch);

    pedWidth.setName("Pedestrian Width %");
    pedWidth.setIcon("mdi:arrow-expand-horizontal");
    pedWidth.setMin(10); pedWidth.setMax(90); pedWidth.setStep(10);
    pedWidth.setUnitOfMeasurement("%");
    pedWidth.onCommand(_onPedWidth);

    _sLastRF.setName("Last RF");        _sLastRF.setIcon("mdi:remote");
    _sState.setName("Gate State");      _sState.setIcon("mdi:state-machine");
    _sIP.setName("IP Address");         _sIP.setIcon("mdi:ip-network");
    _sTravelTime.setName("Travel Time");_sTravelTime.setIcon("mdi:timer-outline");
    _sTravelTime.setUnitOfMeasurement("s");
    _sPosition.setName("Gate Position");_sPosition.setIcon("mdi:gate-arrow-left-right");
    _sPosition.setUnitOfMeasurement("%");
    _sLimOpen.setName("Open Limit");    _sLimOpen.setIcon("mdi:arrow-left-bold-box-outline");
    _sLimClose.setName("Close Limit");  _sLimClose.setIcon("mdi:arrow-right-bold-box-outline");
    _sBarrier.setName("Barrier");       _sBarrier.setDeviceClass("safety");
    _sBarrier.setIcon("mdi:shield-alert");
}

// ─────────────────────────────────────────────────────────────────────────────
void GateNetwork::begin()
{
    wifiWrapperBegin();

    WiFi.macAddress(_mac);
    _device.setUniqueId(_mac, sizeof(_mac));

    _device.enableSharedAvailability();
    _mqtt.begin(storage.cfg.mqttHost,
                static_cast<uint16_t>(atoi(storage.cfg.mqttPort)),
                storage.cfg.mqttUser, storage.cfg.mqttPass);

    pedWidth.setCurrentState(storage.cfg.pedPercent);
    _telnet.begin();
}

// ─────────────────────────────────────────────────────────────────────────────
// HA callbacks
// ─────────────────────────────────────────────────────────────────────────────
void GateNetwork::_onCover(HACover::CoverCommand cmd, HACover * /*s*/)
{
    if      (cmd == HACover::CommandOpen)  postCmd(CMD_OPEN);
    else if (cmd == HACover::CommandClose) postCmd(CMD_CLOSE);
    else if (cmd == HACover::CommandStop)  postCmd(CMD_STOP_ONLY);
}

void GateNetwork::_onButton(HAButton *s)
{
    auto &n = gateNet;
    if      (s == &n.btnOpen)      postCmd(CMD_OPEN);
    else if (s == &n.btnClose)     postCmd(CMD_CLOSE);
    else if (s == &n.btnStop)      postCmd(CMD_STOP_ONLY);
    else if (s == &n.btnCalibrate) postCmd(CMD_CALIBRATE_START);
    else if (s == &n.btnCancelCal) postCmd(CMD_CALIBRATE_CANCEL);
    else if (s == &n.btnPed)
        postCmd(CMD_TOGGLE_PEDESTRIAN);
}

void GateNetwork::_onSwitch(bool state, HASwitch *s)
{
    if (s == &gateNet.swHoldOpen) postCmd(CMD_SET_HOLD_OPEN, -1.0f, 0, state ? 1 : 0);
}

void GateNetwork::_onPedWidth(HANumeric n, HANumber *s)
{
    const uint8_t pct = n.toUInt8();
    if (pct == storage.cfg.pedPercent) return;
    storage.cfg.pedPercent = pct;
    storage.save();
    s->setState(static_cast<int32_t>(pct));
}

// ─────────────────────────────────────────────────────────────────────────────
// broadcastStatus() — WebSocket JSON + HA MQTT
// ─────────────────────────────────────────────────────────────────────────────
void GateNetwork::broadcastStatus()
{
    // ── WebSocket ────────────────────────────────────────────────────────────
    if (_ws.count() > 0) {
        const char *st  = "STOPPED";
        String      sub = "";

        if      (motor.state == MotorState::OPENING) st = "OPENING";
        else if (motor.state == MotorState::CLOSING) st = "CLOSING";

        if (motor.calState != CalState::INACTIVE) {
            st  = "CALIBRATING";
            sub = (motor.calState == CalState::HOMING)    ? "Homing…"    :
                  (motor.calState == CalState::MEASURING) ? "Measuring…" :
                                                            "Verifying…";
        }

        if (rf.learnState == RFLearnState::SCANNING_WEB) {
            st  = "SCANNING";
            sub = rf.scannedCode ? String(rf.scannedCode) : String("Waiting…");
        }

        String ts = motor.getTimerStatus();
        if (rf.learnState != RFLearnState::INACTIVE) {
            long rem = (storage.cfg.tRfLearnTimeout - (millis() - _rfStateMs) + 999) / 1000;
            ts = "RF Timeout in " + String(rem > 0 ? rem : 0) + "s";
        }

        char json[384];
        snprintf(json, sizeof(json),
                 "{\"type\":\"status\",\"s\":\"%s\",\"ss\":\"%s\",\"p\":%d,\"lo\":%d,"
                 "\"lc\":%d,\"pb\":%d,\"rf\":%lu,\"pw\":%d,\"mq\":%d,\"ho\":%d,\"ts\":\"%s\",\"up\":%lu}",
                 st, sub.c_str(),
                 int(motor.position * 100),
                 motor.openLimit()        ? 0 : 1,
                 motor.closeLimit()       ? 0 : 1,
                 motor.barrierTriggered() ? 0 : 1,
                 rf.lastCode, storage.cfg.pedPercent,
                 _mqtt.isConnected()      ? 1 : 0,
                 motor.holdOpen           ? 1 : 0,
                 ts.c_str(),
                 (unsigned long)(esp_timer_get_time() / 1000000ULL));
        _ws.textAll(json);
    }

    // ── Home Assistant ───────────────────────────────────────────────────────
    const int posPct = int(motor.position * 100);
    _cover.setCurrentPosition(uint8_t(posPct));

    char buf[16];
    snprintf(buf, sizeof(buf), "%d", posPct);       _sPosition.setValue(buf);
    snprintf(buf, sizeof(buf), "%lu", storage.cfg.travelTime / 1000UL);
    _sTravelTime.setValue(buf);

    if      (motor.state == MotorState::OPENING) { _cover.setState(HACover::StateOpening); _sState.setValue("opening"); }
    else if (motor.state == MotorState::CLOSING) { _cover.setState(HACover::StateClosing); _sState.setValue("closing"); }
    else if (motor.position >= 0.99f)            { _cover.setState(HACover::StateOpen);    _sState.setValue("open");    }
    else if (motor.position <= 0.01f)            { _cover.setState(HACover::StateClosed);  _sState.setValue("closed");  }
    else                                         { _cover.setState(HACover::StateStopped); _sState.setValue("stopped"); }

    _sIP.setValue(WiFi.localIP().toString().c_str());
    _sLimOpen.setState(motor.openLimit());
    _sLimClose.setState(motor.closeLimit());
    _sBarrier.setState(motor.barrierTriggered());
    swHoldOpen.setState(motor.holdOpen);

    char code[20]; snprintf(code, sizeof(code), "%lu", rf.lastCode);
    _sLastRF.setValue(code);
}

// ─────────────────────────────────────────────────────────────────────────────
void GateNetwork::sendRFList()
{
    if (_ws.count() == 0) return;
    String j;
    j.reserve(64 + rf.keys.size() * 28);
    j = "{\"type\":\"rf_list\",\"data\":[";
    for (size_t i = 0; i < rf.keys.size(); i++) {
        j += "{\"c\":"; j += rf.keys[i].code;
        j += ",\"f\":"; j += rf.keys[i].function; j += "}";
        if (i + 1 < rf.keys.size()) j += ',';
    }
    j += "]}";
    _ws.textAll(j);
}

// ─────────────────────────────────────────────────────────────────────────────
// WebSocket events
// ─────────────────────────────────────────────────────────────────────────────
void GateNetwork::_onWsEvent(AsyncWebSocket * /*s*/, AsyncWebSocketClient *client,
                          AwsEventType type, void * /*arg*/,
                          uint8_t *data, size_t len)
{
    if (type == WS_EVT_CONNECT) {
        LOG_PRINTF("[Net] WS #%lu connected\n", (unsigned long)client->id());
        broadcastStatus(); sendRFList();
    } else if (type == WS_EVT_DISCONNECT) {
        LOG_PRINTF("[Net] WS #%lu disconnected\n", (unsigned long)client->id());
    } else if (type == WS_EVT_DATA) {
        char buf[64] = {};
        size_t copyLen = len < (sizeof(buf) - 1) ? len : (sizeof(buf) - 1);
        memcpy(buf, data, copyLen);
        _handleWsText(String(buf));
    } else if (type == WS_EVT_ERROR) {
        LOG_PRINTF("[Net] WS #%lu error\n", (unsigned long)client->id());
    }
}

void GateNetwork::_handleWsText(const String &t)
{
    if      (t == "OPEN")             postCmd(CMD_OPEN);
    else if (t == "CLOSE")            postCmd(CMD_CLOSE);
    else if (t == "STOP")             postCmd(CMD_STOP_ONLY);
    else if (t == "CALIBRATE")        postCmd(CMD_CALIBRATE_START);
    else if (t == "CALIBRATE_CANCEL") postCmd(CMD_CALIBRATE_CANCEL);
    else if (t == "WIFI_CONFIG")      startWifiPortal();
    else if (t == "SCAN_START")       postCmd(CMD_RF_SCAN_MODE, -1, 0, 1);
    else if (t == "SCAN_STOP")        postCmd(CMD_RF_SCAN_MODE, -1, 0, 0);
    else if (t == "PEDESTRIAN")       postCmd(CMD_TOGGLE_PEDESTRIAN);
    else if (t.startsWith("HOLD:")) {
        postCmd(CMD_SET_HOLD_OPEN, -1.0f, 0, t.substring(5).toInt());
    }
    else if (t.startsWith("ADD:")) {
        const int a = t.indexOf(':'), b = t.lastIndexOf(':');
        if (a > 0 && b > a)
            postCmd(CMD_RF_ADD_CODE, -1,
                    (unsigned long)t.substring(a + 1, b).toInt(),
                    t.substring(b + 1).toInt());
    } else if (t.startsWith("DEL:")) {
        postCmd(CMD_RF_DELETE_CODE, -1, 0, t.substring(4).toInt());
    } else if (t.startsWith("MOVE:")) {
        postCmd(CMD_MOVE_TO_POSITION, t.substring(5).toFloat() / 100.0f);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// HTTP route handlers
// ─────────────────────────────────────────────────────────────────────────────
void GateNetwork::_handleRoot(AsyncWebServerRequest *req)
{
    LittleFS.exists("/index.html")
        ? req->send(LittleFS, "/index.html", "text/html")
        : req->send(404, "text/plain", "index.html missing from LittleFS");
}

void GateNetwork::_handleSettings(AsyncWebServerRequest *req)
{
    auto &c = storage.cfg;
    char json[512];
    snprintf(json, sizeof(json),
        "{\"mq_ip\":\"%s\",\"mq_pt\":\"%s\",\"mq_us\":\"%s\",\"mq_pw\":\"%s\","
        "\"tt\":%lu,\"mdd\":%lu,\"rfd\":%lu,\"p_bd\":%lu,\"ac_od\":%lu,\"ac_opn\":%d,"
        "\"acd\":%lu,\"ac_bar\":%d,\"bh\":%d,\"lh\":%d,\"motor_mode\":%d}",
        c.mqttHost, c.mqttPort, c.mqttUser, c.mqttPass,
        c.travelTime, c.motorDelay, c.rfDebounce, c.preBlinkDelay, c.acOpenDelay,
        c.acOpenEnabled ? 1 : 0, c.acDelay, c.acEnabled ? 1 : 0, 
        c.barrierActiveHigh ? 1 : 0, c.limitsActiveHigh ? 1 : 0, c.motorMode);
    req->send(200, "application/json", json);
}

void GateNetwork::_handleSave(AsyncWebServerRequest *req)
{
    auto &c = storage.cfg;
    auto copyArg = [&](const char *k, char *dst, size_t n) {
        if (req->hasArg(k)) { strncpy(dst, req->arg(k).c_str(), n-1); dst[n-1]='\0'; }
    };
    copyArg("mq_ip", c.mqttHost, sizeof(c.mqttHost));
    copyArg("mq_pt", c.mqttPort, sizeof(c.mqttPort));
    copyArg("mq_us", c.mqttUser, sizeof(c.mqttUser));
    copyArg("mq_pw", c.mqttPass, sizeof(c.mqttPass));
    if (req->hasArg("tt"))  c.travelTime = req->arg("tt").toInt();
    if (req->hasArg("mdd")) c.motorDelay = req->arg("mdd").toInt();
    if (req->hasArg("rfd")) c.rfDebounce = req->arg("rfd").toInt();
    if (req->hasArg("p_bd"))  c.preBlinkDelay = req->arg("p_bd").toInt();
    if (req->hasArg("ac_od")) c.acOpenDelay   = req->arg("ac_od").toInt();
    c.acOpenEnabled     = req->hasArg("ac_opn");
    if (req->hasArg("acd")) c.acDelay    = req->arg("acd").toInt();
    c.acEnabled         = req->hasArg("ac_bar");
    c.barrierActiveHigh = req->hasArg("bh");
    c.limitsActiveHigh  = req->hasArg("lh");
    if (req->hasArg("motor_mode")) c.motorMode = req->arg("motor_mode").toInt();
    storage.save();

    req->send(200, "application/json", "{\"status\":\"ok\",\"msg\":\"Saved! Applied instantly.\"}");
}

void GateNetwork::_handleAdvanced(AsyncWebServerRequest *req)
{
    auto &c = storage.cfg;
    char json[768];
    snprintf(json, sizeof(json),
        "{\"p_r1\":%d,\"p_r2\":%d,\"p_ind\":%d,\"p_bm\":%d,\"p_bw\":%d,\"p_bmt\":%d,\"p_bp\":%d,"
        "\"p_lo\":%d,\"p_lc\":%d,\"p_bar\":%d,\"p_rf\":%d,"
        "\"t_cs\":%lu,\"t_to\":%lu,\"t_rfl\":%lu,\"t_clp\":%lu,\"t_wlp\":%lu,\"t_pp\":%lu,\"t_rp\":%lu,\"t_rfsp\":%lu,\"t_ch\":%lu,\"t_wr\":%lu}",
        c.pinRelay1, c.pinRelay2, c.pinIndicator, c.pinBtnMain, c.pinBtnWifi, c.pinBtnMaint, c.pinBtnPed,
        c.pinLimOpen, c.pinLimClose, c.pinBarrier, c.pinRfRx,
        c.tCalSafety, c.tTravelOvertime, c.tRfLearnTimeout, c.tCalLongPress, c.tWifiLongPress, c.tPedPress, c.tReversePress, c.tRfSavePress, c.tComboHold, c.tWifiRetry
    );
    req->send(200, "application/json", json);
}

void GateNetwork::_handleAdvancedSave(AsyncWebServerRequest *req)
{
    auto &c = storage.cfg;
    if(req->hasArg("p_r1")) c.pinRelay1 = req->arg("p_r1").toInt();
    if(req->hasArg("p_r2")) c.pinRelay2 = req->arg("p_r2").toInt();
    if(req->hasArg("p_ind")) c.pinIndicator = req->arg("p_ind").toInt();
    if(req->hasArg("p_bm")) c.pinBtnMain = req->arg("p_bm").toInt();
    if(req->hasArg("p_bw")) c.pinBtnWifi = req->arg("p_bw").toInt();
    if(req->hasArg("p_bmt")) c.pinBtnMaint = req->arg("p_bmt").toInt();
    if(req->hasArg("p_bp")) c.pinBtnPed = req->arg("p_bp").toInt();
    if(req->hasArg("p_lo")) c.pinLimOpen = req->arg("p_lo").toInt();
    if(req->hasArg("p_lc")) c.pinLimClose = req->arg("p_lc").toInt();
    if(req->hasArg("p_bar")) c.pinBarrier = req->arg("p_bar").toInt();
    if(req->hasArg("p_rf")) c.pinRfRx = req->arg("p_rf").toInt();

    if(req->hasArg("t_cs")) c.tCalSafety = req->arg("t_cs").toInt();
    if(req->hasArg("t_to")) c.tTravelOvertime = req->arg("t_to").toInt();
    if(req->hasArg("t_rfl")) c.tRfLearnTimeout = req->arg("t_rfl").toInt();
    if(req->hasArg("t_clp")) c.tCalLongPress = req->arg("t_clp").toInt();
    if(req->hasArg("t_wlp")) c.tWifiLongPress = req->arg("t_wlp").toInt();
    if(req->hasArg("t_pp")) c.tPedPress = req->arg("t_pp").toInt();
    if(req->hasArg("t_rp")) c.tReversePress = req->arg("t_rp").toInt();
    if(req->hasArg("t_rfsp")) c.tRfSavePress = req->arg("t_rfsp").toInt();
    if(req->hasArg("t_ch")) c.tComboHold = req->arg("t_ch").toInt();
    if(req->hasArg("t_wr")) c.tWifiRetry = req->arg("t_wr").toInt();

    storage.save();
    req->send(200, "application/json", "{\"status\":\"ok\",\"msg\":\"Hardware settings saved! Rebooting to apply...\"}");
    postCmd(CMD_REBOOT);
}

void GateNetwork::_handleLogs(AsyncWebServerRequest *req)
{
    LittleFS.exists("/system_log.txt")
        ? req->send(LittleFS, "/system_log.txt", "text/plain")
        : req->send(200, "text/plain", "No log file.");
}

void GateNetwork::_handleClearLogs(AsyncWebServerRequest *req)
{
    storage.clearLog();
    req->send(200, "text/plain", "Log cleared.");
}

// ─────────────────────────────────────────────────────────────────────────────
// _startWebServer()
// ─────────────────────────────────────────────────────────────────────────────
void GateNetwork::_startWebServer()
{
    _ws.onEvent([this](AsyncWebSocket *s, AsyncWebSocketClient *c,
                       AwsEventType t, void *a, uint8_t *d, size_t l)
    { _onWsEvent(s, c, t, a, d, l); });
    _server.addHandler(&_ws);

    _server.on("/",           HTTP_GET,  [this](AsyncWebServerRequest *r){ _handleRoot(r);      });
    _server.on("/settings",   HTTP_GET,  [this](AsyncWebServerRequest *r){ _handleSettings(r);  });
    _server.on("/save",       HTTP_POST, [this](AsyncWebServerRequest *r){ _handleSave(r);      });
    _server.on("/advanced",   HTTP_GET,  [this](AsyncWebServerRequest *r){ _handleAdvanced(r);  });
    _server.on("/advanced_save", HTTP_POST, [this](AsyncWebServerRequest *r){ _handleAdvancedSave(r); });
    _server.on("/logs",       HTTP_GET,  [this](AsyncWebServerRequest *r){ _handleLogs(r);      });
    _server.on("/clear_logs", HTTP_GET,  [this](AsyncWebServerRequest *r){ _handleClearLogs(r); });
    _server.on("/open",       HTTP_GET,  [](AsyncWebServerRequest *r){ postCmd(CMD_OPEN);      r->redirect("/"); });
    _server.on("/close",      HTTP_GET,  [](AsyncWebServerRequest *r){ postCmd(CMD_CLOSE);     r->redirect("/"); });
    _server.on("/stop",       HTTP_GET,  [](AsyncWebServerRequest *r){ postCmd(CMD_STOP_ONLY); r->redirect("/"); });
    _server.onNotFound([](AsyncWebServerRequest *r){ r->send(404, "text/plain", "Not found"); });

    // OTA Firmware Update
    _server.on("/update", HTTP_POST, [](AsyncWebServerRequest *req){
        AsyncWebServerResponse *response = req->beginResponse(200, "text/plain", (Update.hasError()) ? "OTA FAIL" : "OTA SUCCESS! Rebooting...");
        response->addHeader("Connection", "close");
        req->send(response);
    }, [](AsyncWebServerRequest *req, String filename, size_t index, uint8_t *data, size_t len, bool final){
        if(!index){
            LOG_PRINTF("[OTA] Update Start: %s\n", filename.c_str());
            if(!Update.begin(UPDATE_SIZE_UNKNOWN)){
                Update.printError(Serial);
            }
        }
        if(!Update.hasError()){
            if(Update.write(data, len) != len){
                Update.printError(Serial);
            }
        }
        if(final){
            if(Update.end(true)){
                LOG_PRINTF("[OTA] Update Success: %uB\n", index+len);
                postCmd(CMD_REBOOT);
            } else {
                Update.printError(Serial);
            }
        }
    });

    _server.begin();
    MDNS.begin("gate");
    _wsStarted = true;
    LOG_PRINTLN("[Net] HTTP+WS server started on port 80 (/ws).");
}

// ─────────────────────────────────────────────────────────────────────────────
// _pollTelnet()
// ─────────────────────────────────────────────────────────────────────────────
void GateNetwork::_pollTelnet()
{
    if (_telnet.hasClient()) {
        WiFiClient newClient = _telnet.accept();
        if (xSemaphoreTake(logMtx, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (telnetClient) telnetClient.stop();
            telnetClient = newClient;
            xSemaphoreGive(logMtx);
        }
    }
    if (!telnetClient || !telnetClient.connected() || !telnetClient.available()) return;

    String cmd = telnetClient.readStringUntil('\n');
    cmd.trim();
    if      (cmd == "logs")         storage.dumpLog();
    else if (cmd == "clearlogs")    storage.clearLog();
    else if (cmd == "status")       broadcastStatus();
    else if (cmd == "wificonfig")   startWifiPortal();
    else if (cmd == "restart")      postCmd(CMD_REBOOT);
    else {
        telnetClient.println("Commands: logs | clearlogs | status | wificonfig | restart");
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// loop() — called from vNetworkTask every 25 ms
// ─────────────────────────────────────────────────────────────────────────────
void GateNetwork::loop()
{
    wifiWrapperProcess();

    if (_portalReq) {
        if (_wsStarted) { _ws.closeAll(); _server.end(); _wsStarted = false; }
        wifiWrapperStartPortal();
        _portalReq = false;
    }

    if (wifiWrapperIsPortalActive()) return;

    if (WiFi.status() == WL_CONNECTED) {
        if (!_wsStarted) _startWebServer();
        _ws.cleanupClients();
        _mqtt.loop();
        _pollTelnet();
    }

    if (millis() - _wifiCheck > storage.cfg.tWifiRetry) {
        _wifiCheck = millis();
        if (WiFi.status() != WL_CONNECTED) LOG_PRINTLN("[Net] WiFi reconnecting…");
    }

    // Track when RF state changes to calculate timeout accurately
    if (rf.learnState != _lastRfState) {
        if (rf.learnState != RFLearnState::INACTIVE) _rfStateMs = millis();
        _lastRfState = rf.learnState;
    }

    // Adaptive broadcast rate: fast while moving, slow while idle
    const bool busy = (motor.state != MotorState::IDLE ||
                       motor.calState != CalState::INACTIVE ||
                       motor.hasActiveTimer() ||
                       rf.learnState != RFLearnState::INACTIVE);
    const int  rate = busy ? 10 : 120;   // ticks at 25 ms each → 250 ms / 3 s

    if (_update || ++_tick >= rate) {
        _tick   = 0;
        _update = false;
        if (xSemaphoreTake(stateMtx, pdMS_TO_TICKS(10)) == pdTRUE) {
            broadcastStatus();
            xSemaphoreGive(stateMtx);
        }
    }
}
