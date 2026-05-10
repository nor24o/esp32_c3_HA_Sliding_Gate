#include "WifiWrapper.h"
#include <WiFiManager.h>
#include <WiFi.h>
#include "Log.hpp"

static WiFiManager wm;

void wifiWrapperBegin() {
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    wm.setDebugOutput(false);
    wm.setConfigPortalBlocking(false);

    if (wm.autoConnect("GateControllerAP")) {
        LOG_PRINTLN("[Net] WiFi connected.");
    } else {
        LOG_PRINTLN("[Net] WiFi pending — AP running.");
    }
}

void wifiWrapperProcess() {
    wm.process();
}

void wifiWrapperStartPortal() {
    wm.startConfigPortal("GateControllerAP");
}

bool wifiWrapperIsPortalActive() {
    return wm.getConfigPortalActive();
}