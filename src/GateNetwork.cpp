#include "GateNetwork.h"
#include <LittleFS.h>

GateNetwork netManager;

// Helper to bridge C-style callbacks to the Queue
void sendGlobalCmd(GateCommand cmd, float pos = -1.0)
{
    CommandMessage msg = {cmd, pos, 0, 0};
    xQueueSend(xCommandQueue, &msg, 0);
}

GateNetwork::GateNetwork() : server(80), webSocket(81), telnetServer(23), mqtt(wifiClient, device),
                             haCover("sliding_gate_cover"), rfCodeSensor("sliding_gate_last_rf_code"),
                             btnCalibrate("sliding_gate_calibrate"), btnCancelCal("sliding_gate_calibrate_cancel"),
                             btnMove50("sliding_gate_move_to_50"), pedWidthNumber("sliding_gate_ped_width"), gateState("sliding_gate_state"), gatePosition("sliding_gate_position"),
                             gateIP("sliding_gate_IP"), travelTime("sliding_gate_travel_time"),
                             btnOpen("sliding_gate_open_button"), btnClose("sliding_gate_close_button"),
                             btnStop("sliding_gate_stop_button"), limOpen("sliding_gate_lim_open"),
                             limClose("sliding_gate_lim_close"), barrier("sliding_gate_barrier")
{
    // ID Setup
    byte mac[6];
    WiFi.macAddress(mac);
    device.setUniqueId(mac, sizeof(mac));
    device.setName("Sliding Gate");

    device.setModel("ESP32-C3");
    device.setManufacturer("H_N");

    // HA Config
    haCover.setName("Sliding Gate");
    haCover.setDeviceClass("gate");
    haCover.onCommand(onCoverCommand);
    btnOpen.setName("Open Gate");
    btnOpen.setIcon("mdi:gate-open");
    btnOpen.onCommand(onButtonCommand);
    btnClose.setName("Close Gate");
    btnClose.setIcon("mdi:gate");
    btnClose.onCommand(onButtonCommand);
    btnStop.setName("Stop Gate");
    btnStop.setIcon("mdi:stop-circle-outline");
    btnStop.onCommand(onButtonCommand);
    btnCalibrate.setName("Calibrate");
    btnCalibrate.setIcon("mdi:ruler");
    btnCalibrate.onCommand(onButtonCommand);
    btnCancelCal.setName("Cancel Cal");
    btnCancelCal.setIcon("mdi:cancel");
    btnCancelCal.onCommand(onButtonCommand);
    btnMove50.setName("Pedestrian");
    btnMove50.setIcon("mdi:walk");
    btnMove50.onCommand(onButtonCommand);

    pedWidthNumber.setName("Pedestrian Width %");
    pedWidthNumber.setIcon("mdi:arrow-expand-horizontal");
    pedWidthNumber.setMin(10); // Minimum 10%
    pedWidthNumber.setMax(90); // Maximum 90%
    pedWidthNumber.setStep(10);
    pedWidthNumber.setUnitOfMeasurement("%");
    pedWidthNumber.onCommand(onPedWidthChange);

    rfCodeSensor.setName("Last RF");
    rfCodeSensor.setIcon("mdi:remote");
    gateState.setName("Gate State");
    gateState.setIcon("mdi:state-machine");
    gateIP.setName("IP Address");
    gateIP.setIcon("mdi:ip-network");
    travelTime.setName("Travel Time");
    travelTime.setIcon("mdi:timer-outline");
    travelTime.setUnitOfMeasurement("s");

    gatePosition.setName("Gate Position");
    gatePosition.setIcon("mdi:gate-arrow-left-right");
    gatePosition.setUnitOfMeasurement("%");

    limOpen.setName("Open Limit");
    limOpen.setIcon("mdi:arrow-left-bold-box-outline");
    limClose.setName("Close Limit");
    limClose.setIcon("mdi:arrow-right-bold-box-outline");

    barrier.setName("Barrier");
    barrier.setDeviceClass("safety");
    barrier.setIcon("mdi:shield-alert");
}

void GateNetwork::begin()
{
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    // Disable Debug output to prevent Serial interference on Pin 1
    wm.setDebugOutput(false);
    wm.setConfigPortalBlocking(false);

    if (wm.autoConnect("GateControllerAP"))
        LOG_PRINTLN("WiFi Connected");
    else
        LOG_PRINTLN("WiFi Not Connected (Background AP)");
    device.enableSharedAvailability(); // <--- Reduces network traffic significantly
    mqtt.begin(sysConfig.config.mqtt_server, atoi(sysConfig.config.mqtt_port), sysConfig.config.mqtt_user, sysConfig.config.mqtt_pass);
    pedWidthNumber.setCurrentState(sysConfig.config.pedestrian_percent);
    telnetServer.begin();
}

void GateNetwork::triggerWifiConfig() { configPortalRequested = true; }

void GateNetwork::onCoverCommand(HACover::CoverCommand cmd, HACover *sender)
{
    if (cmd == HACover::CommandOpen)
        sendGlobalCmd(CMD_OPEN);
    else if (cmd == HACover::CommandClose)
        sendGlobalCmd(CMD_CLOSE);
    else if (cmd == HACover::CommandStop)
        sendGlobalCmd(CMD_STOP_ONLY);
}

void GateNetwork::onButtonCommand(HAButton *sender)
{
    if (sender == &netManager.btnOpen)
        sendGlobalCmd(CMD_OPEN);
    else if (sender == &netManager.btnClose)
        sendGlobalCmd(CMD_CLOSE);
    else if (sender == &netManager.btnStop)
        sendGlobalCmd(CMD_STOP_ONLY);
    else if (sender == &netManager.btnCalibrate)
        sendGlobalCmd(CMD_CALIBRATE_START);
    else if (sender == &netManager.btnCancelCal)
        sendGlobalCmd(CMD_CALIBRATE_CANCEL);
    else if (sender == &netManager.btnMove50)
    {
        float target = (float)sysConfig.config.pedestrian_percent / 100.0f;
        sendGlobalCmd(CMD_MOVE_TO_POSITION, target);
    }
}

void GateNetwork::broadcastStatus()
{
    if (webSocket.connectedClients() > 0)
    {
        char json[400];
        String status = "STOPPED";
        String subStatus = "";

        if (gateMotor.currentOperation == OPENING)
            status = "OPENING";
        else if (gateMotor.currentOperation == CLOSING)
            status = "CLOSING";

        if (gateMotor.calState != CAL_INACTIVE)
        {
            status = "CALIBRATING";
            if (gateMotor.calState == CAL_HOMING_CLOSE)
                subStatus = "Homing...";
            else if (gateMotor.calState == CAL_MEASURING_OPEN)
                subStatus = "Measuring...";
            else
                subStatus = "Verifying...";
        }

        if (rfHandler.learnState == RF_SCANNING_WEB)
        {
            status = "SCANNING";
            subStatus = (rfHandler.scannedCode > 0) ? String(rfHandler.scannedCode) : "Waiting...";
        }

        // Added "pw" (Pedestrian Width) to the JSON output below
        snprintf(json, sizeof(json),
                 "{\"type\":\"status\",\"s\":\"%s\",\"ss\":\"%s\",\"p\":%d,\"lo\":%d,\"lc\":%d,\"pb\":%d,\"rf\":%lu,\"pw\":%d}",
                 status.c_str(), subStatus.c_str(), (int)(gateMotor.currentPosition * 100),
                 gateMotor.isOpenLimit() ? 0 : 1, gateMotor.isCloseLimit() ? 0 : 1,
                 gateMotor.isBarrierTriggered() ? 0 : 1, rfHandler.lastCode,
                 sysConfig.config.pedestrian_percent); // <--- Sending the config value

        webSocket.broadcastTXT(json);
    }

    // HA Updates
    haCover.setCurrentPosition(gateMotor.currentPosition * 100);

    // Create a small buffer to hold the number string
    char posBuf[8];
    // Convert the position (0-100) into that buffer
    snprintf(posBuf, sizeof(posBuf), "%d", (int)(gateMotor.currentPosition * 100));
    // Send the string buffer
    gatePosition.setValue(posBuf);

    char buf[16];
    snprintf(buf, sizeof(buf), "%lu", sysConfig.config.travel_time / 1000);
    travelTime.setValue(buf);

    if (gateMotor.currentOperation == OPENING)
    {
        haCover.setState(HACover::StateOpening);
        gateState.setValue("opening");
    }
    else if (gateMotor.currentOperation == CLOSING)
    {
        haCover.setState(HACover::StateClosing);
        gateState.setValue("closing");
    }
    else if (gateMotor.currentPosition >= 0.99)
    {
        haCover.setState(HACover::StateOpen);
        gateState.setValue("open");
    }
    else if (gateMotor.currentPosition <= 0.01)
    {
        haCover.setState(HACover::StateClosed);
        gateState.setValue("closed");
    }
    else
    {
        haCover.setState(HACover::StateStopped);
        gateState.setValue("stopped");
    }

    gateIP.setValue(WiFi.localIP().toString().c_str());
    limOpen.setState(gateMotor.isOpenLimit());
    limClose.setState(gateMotor.isCloseLimit());
    barrier.setState(gateMotor.isBarrierTriggered());

    char codeStr[20];
    sprintf(codeStr, "%lu", rfHandler.lastCode);
    rfCodeSensor.setValue(codeStr);
}

void GateNetwork::onWebSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
    if (type == WStype_CONNECTED)
    {
        netManager.broadcastStatus();
        netManager.sendRFListToWeb();
    }
    else if (type == WStype_TEXT)
    {
        String text = String((char *)payload);
        if (text == "OPEN")
            sendGlobalCmd(CMD_OPEN);
        else if (text == "CLOSE")
            sendGlobalCmd(CMD_CLOSE);
        else if (text == "STOP")
            sendGlobalCmd(CMD_STOP_ONLY);
        else if (text == "CALIBRATE")
            sendGlobalCmd(CMD_CALIBRATE_START);
        else if (text == "CALIBRATE_CANCEL")
            sendGlobalCmd(CMD_CALIBRATE_CANCEL);
        else if (text == "SCAN_START")
        {
            CommandMessage msg = {CMD_RF_SCAN_MODE, 0, 0, 1};
            xQueueSend(xCommandQueue, &msg, 0);
        }
        else if (text == "SCAN_STOP")
        {
            CommandMessage msg = {CMD_RF_SCAN_MODE, 0, 0, 0};
            xQueueSend(xCommandQueue, &msg, 0);
        }
        else if (text == "WIFI_CONFIG")
        {
            netManager.triggerWifiConfig();
        }

        if (text.startsWith("ADD:"))
        {
            int fst = text.indexOf(':');
            int sec = text.lastIndexOf(':');
            if (fst > 0 && sec > fst)
            {
                String c = text.substring(fst + 1, sec);
                String f = text.substring(sec + 1);
                CommandMessage msg = {CMD_RF_ADD_CODE, 0, strtoul(c.c_str(), NULL, 10), f.toInt()};
                xQueueSend(xCommandQueue, &msg, 0);
            }
        }
        if (text.startsWith("DEL:"))
        {
            CommandMessage msg = {CMD_RF_DELETE_CODE, 0, 0, text.substring(4).toInt()};
            xQueueSend(xCommandQueue, &msg, 0);
        }
    }
}

void GateNetwork::sendRFListToWeb()
{
    String json = "{\"type\":\"rf_list\",\"data\":[";
    for (size_t i = 0; i < rfHandler.keyList.size(); i++)
    {
        json += "{\"c\":" + String(rfHandler.keyList[i].code) + ",\"f\":" + String(rfHandler.keyList[i].function) + "}";
        if (i < rfHandler.keyList.size() - 1)
            json += ",";
    }
    json += "]}";
    webSocket.broadcastTXT(json);
}

void GateNetwork::setupWebRoutes()
{
    server.on("/", [this]()
              { handleRoot(); });
    server.on("/config", [this]()
              { handleConfig(); });
    server.on("/save", HTTP_POST, [this]()
              { handleSave(); });
    server.on("/logs", [this]()
              { handleLogs(); });
    server.on("/clear_logs", [this]()
              { handleClearLogs(); });
    server.onNotFound([this]()
                      { handleWebCommand(); });
    server.begin();
    webSocket.begin();
    webSocket.onEvent([this](uint8_t n, WStype_t t, uint8_t *p, size_t l)
                      { onWebSocketEvent(n, t, p, l); });
    MDNS.begin("gate");
    webServerStarted = true;
    LOG_PRINTLN("Web Started");
}

void GateNetwork::loop()
{
    wm.process();
    if (simCrashNetwork)
        while (1)
            vTaskDelay(1);

    if (configPortalRequested)
    {
        if (webServerStarted)
        {
            server.stop();
            webServerStarted = false;
        }
        wm.startConfigPortal("GateControllerAP");
        configPortalRequested = false;
    }

    if (wm.getConfigPortalActive())
        return;

    if (WiFi.status() == WL_CONNECTED)
    {
        if (!webServerStarted)
            setupWebRoutes();
        server.handleClient();
        webSocket.loop();
        mqtt.loop();

        if (telnetServer.hasClient())
        {
            if (telnetClient)
                telnetClient.stop();
            telnetClient = telnetServer.accept();
        }
        if (telnetClient && telnetClient.connected() && telnetClient.available())
        {
            String cmd = telnetClient.readStringUntil('\n');
            cmd.trim();
            if (cmd == "logs")
                sysConfig.dumpLog();
            else if (cmd == "restart")
                ESP.restart();
            else if (cmd == "clearlogs")
                sysConfig.clearLog();
            else if (cmd == "status")
                broadcastStatus();
            else if (cmd == "wificonfig")
                triggerWifiConfig();
            else if (cmd == "help")
            {
                telnetClient.println("Available commands:");
                telnetClient.println("logs - Dump system logs");
                telnetClient.println("clearlogs - Clear system logs");
                telnetClient.println("status - Broadcast current status");
                telnetClient.println("wificonfig - Start WiFi config portal");
                telnetClient.println("restart - Restart the device");
            }
        }
    }

    if (millis() - lastWifiCheck > WIFI_RETRY_INTERVAL)
    {
        lastWifiCheck = millis();
        if (WiFi.status() != WL_CONNECTED)
            LOG_PRINTLN("WiFi Reconnecting...");
    }

    // --- NEW "SNAPPY" LOGIC START ---

    // 1. Determine how often we SHOULD update automatically
    // If gate is moving or calibrating: Update fast (every 250ms)
    // If idle: Update slow (every 3000ms)
    int requiredInterval = (gateMotor.currentOperation != IDLE || gateMotor.calState != CAL_INACTIVE) ? 10 : 120;

    mqttCounter++;

    // 2. Broadcast if: Time is up OR someone requested an immediate update
    if (updateRequest || mqttCounter > requiredInterval)
    {
        mqttCounter = 0;
        updateRequest = false; // Reset the flag

        if (xSemaphoreTake(xStateMutex, 10))
        {
            broadcastStatus();
            xSemaphoreGive(xStateMutex);
        }
    }
}

// --- FILE SYSTEM HANDLERS ---

void GateNetwork::handleRoot()
{
    if (LittleFS.exists("/index.html"))
    {
        File file = LittleFS.open("/index.html", "r");
        server.streamFile(file, "text/html");
        file.close();
    }
    else
    {
        server.send(404, "text/plain", "Error: index.html missing");
    }
}

void GateNetwork::handleConfig()
{
    if (!LittleFS.exists("/config.html"))
    {
        server.send(404, "text/plain", "Error: config.html missing");
        return;
    }

    File file = LittleFS.open("/config.html", "r");
    String html = file.readString();
    file.close();

    html.replace("%MQ_IP%", String(sysConfig.config.mqtt_server));
    html.replace("%MQ_PT%", String(sysConfig.config.mqtt_port));
    html.replace("%MQ_US%", String(sysConfig.config.mqtt_user));
    html.replace("%MQ_PW%", String(sysConfig.config.mqtt_pass));

    html.replace("%TT%", String(sysConfig.config.travel_time));
    html.replace("%MDD%", String(sysConfig.config.motor_delay));
    html.replace("%RFD%", String(sysConfig.config.rf_debounce));
    html.replace("%ACD%", String(sysConfig.config.ac_delay));

    html.replace("%CHK_AC%", sysConfig.config.ac_barrier ? "checked" : "");
    html.replace("%CHK_BH%", sysConfig.config.bar_active_high ? "checked" : "");
    html.replace("%CHK_LH%", sysConfig.config.lim_active_high ? "checked" : "");

    server.send(200, "text/html", html);
}

void GateNetwork::handleLogs()
{
    if (LittleFS.exists("/system_log.txt"))
    {
        File f = LittleFS.open("/system_log.txt", "r");
        server.streamFile(f, "text/plain");
        f.close();
    }
    else
        server.send(200, "text/plain", "No logs.");
}

void GateNetwork::handleClearLogs()
{
    sysConfig.clearLog();
    server.send(200, "text/plain", "Cleared");
}

void GateNetwork::handleWebCommand()
{
    String p = server.uri();
    if (p == "/open")
        sendGlobalCmd(CMD_OPEN);
    else if (p == "/close")
        sendGlobalCmd(CMD_CLOSE);
    else if (p == "/stop")
        sendGlobalCmd(CMD_STOP_ONLY);
    server.sendHeader("Location", "/");
    server.send(303);
}

void GateNetwork::handleSave()
{
    if (server.hasArg("mq_ip"))
        strncpy(sysConfig.config.mqtt_server, server.arg("mq_ip").c_str(), 40);
    if (server.hasArg("mq_pt"))
        strncpy(sysConfig.config.mqtt_port, server.arg("mq_pt").c_str(), 6);
    if (server.hasArg("mq_us"))
        strncpy(sysConfig.config.mqtt_user, server.arg("mq_us").c_str(), 32);
    if (server.hasArg("mq_pw"))
        strncpy(sysConfig.config.mqtt_pass, server.arg("mq_pw").c_str(), 64);
    if (server.hasArg("tt"))
        sysConfig.config.travel_time = server.arg("tt").toInt();
    if (server.hasArg("mdd"))
        sysConfig.config.motor_delay = server.arg("mdd").toInt();
    if (server.hasArg("rfd"))
        sysConfig.config.rf_debounce = server.arg("rfd").toInt();
    if (server.hasArg("acd"))
        sysConfig.config.ac_delay = server.arg("acd").toInt();

    sysConfig.config.ac_barrier = server.hasArg("ac_bar");
    sysConfig.config.bar_active_high = server.hasArg("bh");
    sysConfig.config.lim_active_high = server.hasArg("lh");

    sysConfig.save();
    server.send(200, "text/html", "Saved. Rebooting...");
    delay(500);
    ESP.restart();
}

void GateNetwork::onPedWidthChange(HANumeric number, HANumber *sender)
{
    // 1. Extract value using .toUInt8() since we store it as uint8_t
    uint8_t newPercent = number.toUInt8();

    if (newPercent != sysConfig.config.pedestrian_percent)
    {
        sysConfig.config.pedestrian_percent = newPercent;
        sysConfig.save();

        // 2. Cast to (int32_t) to resolve ambiguity in setState overloads
        sender->setState((int32_t)newPercent);
    }
}