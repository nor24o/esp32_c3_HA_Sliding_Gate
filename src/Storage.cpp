/**
 * @file Storage.cpp
 */

#include "Storage.h"
#include "Log.hpp"
#include <esp_task_wdt.h>
#include <esp_system.h>
#include <esp_idf_version.h>

Storage storage;

// ─────────────────────────────────────────────────────────────────────────────
// Default values (constructor-style initialisation via begin())
// ─────────────────────────────────────────────────────────────────────────────
static void applyDefaults(Storage::Settings &c)
{
    c.travelTime       = 30000;
    c.motorDelay       = 700;
    c.rfDebounce       = 400;
    c.acDelay          = 5000;
    c.pedPercent       = 30;
    c.motorMode        = 2;
    c.preBlinkDelay    = 0;
    c.acOpenDelay      = 30000;
    c.acOpenEnabled    = false;
    c.acEnabled        = false;
    c.barrierActiveHigh = false;
    c.limitsActiveHigh  = false;
    strncpy(c.mqttHost, "192.168.1.12", sizeof(c.mqttHost) - 1);
    strncpy(c.mqttPort, "1883",         sizeof(c.mqttPort) - 1);
    strncpy(c.mqttUser, "admin",        sizeof(c.mqttUser) - 1);
    strncpy(c.mqttPass, "admin",        sizeof(c.mqttPass) - 1);

    c.pinRelay1 = 1;  c.pinRelay2 = 4;  c.pinIndicator = 3;
    c.pinBtnMain = 5; c.pinBtnWifi = 6; c.pinBtnMaint = 7; c.pinBtnPed = 8;
    c.pinLimOpen = 10; c.pinLimClose = 20; c.pinBarrier = 2; c.pinRfRx = 21;

    c.tCalSafety = 90000; c.tTravelOvertime = 3000; c.tRfLearnTimeout = 60000;
    c.tCalLongPress = 8000; c.tWifiLongPress = 5000; c.tReversePress = 1000;
    c.tPedPress = 200; c.tRfSavePress = 1500; c.tComboHold = 2000; c.tWifiRetry = 30000;

    c.bootCount = 0;
}

// ─────────────────────────────────────────────────────────────────────────────
void Storage::begin()
{
    applyDefaults(cfg);

    // LittleFS partition label MUST be "spiffs" — the Arduino driver looks
    // for that label by name regardless of the filesystem format used.
    _lfsOk = LittleFS.begin(true);
    if (_lfsOk)
        LOG_PRINTLN("[Storage] LittleFS mounted.");
    else
        LOG_PRINTLN("[Storage] ERROR: LittleFS failed — check partition label = 'spiffs'.");

    prefs.begin(K_STATS, false);
    cfg.bootCount = prefs.getUInt("boot_count", 0) + 1;
    prefs.putUInt("boot_count", cfg.bootCount);
    prefs.end();
}

// ─────────────────────────────────────────────────────────────────────────────
void Storage::load()
{
    prefs.begin(K_CONF, true);

    cfg.travelTime       = prefs.getULong("travel_ms",  30000UL);
    cfg.motorDelay       = prefs.getULong("motor_ms",   700UL);
    cfg.rfDebounce       = prefs.getULong("rf_deb_ms",  400UL);
    cfg.acDelay          = prefs.getULong("ac_delay_ms",5000UL);
    cfg.pedPercent       = prefs.getUChar("ped_pct",    30);
    cfg.motorMode        = prefs.getUChar("motor_mode", 2);
    cfg.preBlinkDelay    = prefs.getULong("pre_blk_ms", 0UL);
    cfg.acOpenDelay      = prefs.getULong("ac_opn_ms",  30000UL);
    cfg.acOpenEnabled    = prefs.getBool ("ac_opn_en",  false);
    cfg.acEnabled        = prefs.getBool ("ac_en",      false);
    cfg.barrierActiveHigh = prefs.getBool("bar_high",   false);
    cfg.limitsActiveHigh  = prefs.getBool("lim_high",   false);

    loadStr("mqtt_host", cfg.mqttHost, sizeof(cfg.mqttHost), "192.168.1.12");
    loadStr("mqtt_port", cfg.mqttPort, sizeof(cfg.mqttPort), "1883");
    loadStr("mqtt_user", cfg.mqttUser, sizeof(cfg.mqttUser), "admin");
    loadStr("mqtt_pass", cfg.mqttPass, sizeof(cfg.mqttPass), "admin");

    cfg.pinRelay1    = prefs.getUChar("p_r1", 1);
    cfg.pinRelay2    = prefs.getUChar("p_r2", 4);
    cfg.pinIndicator = prefs.getUChar("p_ind", 3);
    cfg.pinBtnMain   = prefs.getUChar("p_bm", 5);
    cfg.pinBtnWifi   = prefs.getUChar("p_bw", 6);
    cfg.pinBtnMaint  = prefs.getUChar("p_bmt", 7);
    cfg.pinBtnPed    = prefs.getUChar("p_bp", 8);
    cfg.pinLimOpen   = prefs.getUChar("p_lo", 10);
    cfg.pinLimClose  = prefs.getUChar("p_lc", 20);
    cfg.pinBarrier   = prefs.getUChar("p_bar", 2);
    cfg.pinRfRx      = prefs.getUChar("p_rf", 21);

    cfg.tCalSafety      = prefs.getULong("t_cs", 90000UL);
    cfg.tTravelOvertime = prefs.getULong("t_to", 3000UL);
    cfg.tRfLearnTimeout = prefs.getULong("t_rfl", 60000UL);
    cfg.tCalLongPress   = prefs.getULong("t_clp", 8000UL);
    cfg.tWifiLongPress  = prefs.getULong("t_wlp", 5000UL);
    cfg.tReversePress   = prefs.getULong("t_rp", 1000UL);
    cfg.tPedPress       = prefs.getULong("t_pp", 200UL);
    cfg.tRfSavePress    = prefs.getULong("t_rfsp", 1500UL);
    cfg.tComboHold      = prefs.getULong("t_ch", 2000UL);
    cfg.tWifiRetry      = prefs.getULong("t_wr", 30000UL);

    prefs.end();
    LOG_PRINTLN("[Storage] Config loaded.");
}

// ─────────────────────────────────────────────────────────────────────────────
void Storage::save()
{
    log("[Storage] Saving config…");
    prefs.begin(K_CONF, false);
    prefs.putULong("travel_ms",   cfg.travelTime);
    prefs.putULong("motor_ms",    cfg.motorDelay);
    prefs.putULong("rf_deb_ms",   cfg.rfDebounce);
    prefs.putULong("ac_delay_ms", cfg.acDelay);
    prefs.putUChar("ped_pct",     cfg.pedPercent);
    prefs.putUChar("motor_mode",  cfg.motorMode);
    prefs.putULong("pre_blk_ms",  cfg.preBlinkDelay);
    prefs.putULong("ac_opn_ms",   cfg.acOpenDelay);
    prefs.putBool ("ac_opn_en",   cfg.acOpenEnabled);
    prefs.putBool ("ac_en",       cfg.acEnabled);
    prefs.putBool ("bar_high",    cfg.barrierActiveHigh);
    prefs.putBool ("lim_high",    cfg.limitsActiveHigh);
    prefs.putString("mqtt_host",  cfg.mqttHost);
    prefs.putString("mqtt_port",  cfg.mqttPort);
    prefs.putString("mqtt_user",  cfg.mqttUser);
    prefs.putString("mqtt_pass",  cfg.mqttPass);

    prefs.putUChar("p_r1", cfg.pinRelay1);
    prefs.putUChar("p_r2", cfg.pinRelay2);
    prefs.putUChar("p_ind", cfg.pinIndicator);
    prefs.putUChar("p_bm", cfg.pinBtnMain);
    prefs.putUChar("p_bw", cfg.pinBtnWifi);
    prefs.putUChar("p_bmt", cfg.pinBtnMaint);
    prefs.putUChar("p_bp", cfg.pinBtnPed);
    prefs.putUChar("p_lo", cfg.pinLimOpen);
    prefs.putUChar("p_lc", cfg.pinLimClose);
    prefs.putUChar("p_bar", cfg.pinBarrier);
    prefs.putUChar("p_rf", cfg.pinRfRx);

    prefs.putULong("t_cs", cfg.tCalSafety);
    prefs.putULong("t_to", cfg.tTravelOvertime);
    prefs.putULong("t_rfl", cfg.tRfLearnTimeout);
    prefs.putULong("t_clp", cfg.tCalLongPress);
    prefs.putULong("t_wlp", cfg.tWifiLongPress);
    prefs.putULong("t_rp", cfg.tReversePress);
    prefs.putULong("t_pp", cfg.tPedPress);
    prefs.putULong("t_rfsp", cfg.tRfSavePress);
    prefs.putULong("t_ch", cfg.tComboHold);
    prefs.putULong("t_wr", cfg.tWifiRetry);

    prefs.end();
}

// ─────────────────────────────────────────────────────────────────────────────
void Storage::loadRF(std::vector<RFEntry> &list)
{
    list.clear();
    prefs.begin(K_CONF, true);
    const size_t len = prefs.getBytesLength("rf_list");
    if (len > 0 && (len % sizeof(RFEntry)) == 0) {
        list.resize(len / sizeof(RFEntry));
        prefs.getBytes("rf_list", list.data(), len);
        LOG_PRINTF("[Storage] Loaded %u RF keys.\n", (unsigned)list.size());
    }
    prefs.end();
}

void Storage::saveRF(const std::vector<RFEntry> &list)
{
    prefs.begin(K_CONF, false);
    if (list.empty())
        prefs.remove("rf_list");
    else
        prefs.putBytes("rf_list", list.data(), list.size() * sizeof(RFEntry));
    prefs.end();
}

// ─────────────────────────────────────────────────────────────────────────────
void Storage::log(const String &msg)
{
    LOG_PRINTLN(msg.c_str());
    if (!_lfsOk) return;

    File f = LittleFS.open(LOG_PATH, "a");
    if (!f) return;

    if (f.size() > MAX_LOG_BYTES) {     // keep log bounded
        f.close();
        LittleFS.remove(LOG_PATH);
        f = LittleFS.open(LOG_PATH, "w");
        if (!f) return;
    }
    f.printf("[%lus] %s\n", (unsigned long)(millis() / 1000UL), msg.c_str());
    f.close();
}

void Storage::dumpLog()
{
    if (!_lfsOk) { LOG_PRINTLN("[Storage] LittleFS not mounted."); return; }
    if (!LittleFS.exists(LOG_PATH)) { LOG_PRINTLN("[Storage] No log."); return; }

    File f = LittleFS.open(LOG_PATH, "r");
    if (!f) return;
    while (f.available()) {
        if (HW_WDT_ENABLED) esp_task_wdt_reset();
        Log.write(static_cast<uint8_t>(f.read()));
    }
    f.close();
}

void Storage::clearLog()
{
    if (!_lfsOk) return;
    LittleFS.remove(LOG_PATH);
    LOG_PRINTLN("[Storage] Log cleared.");
}

// ─────────────────────────────────────────────────────────────────────────────
void Storage::printResetReason()
{
    struct Entry { esp_reset_reason_t r; const char *label; bool save; };
    static constexpr Entry kReasons[] = {
        { ESP_RST_POWERON, "Power On",           false },
        { ESP_RST_SW,      "Software Reset",     false },
        { ESP_RST_PANIC,   "Crash / Panic",      true  },
        { ESP_RST_TASK_WDT,"Task WDT (hang)",    true  },
        { ESP_RST_WDT,     "Interrupt WDT",      true  },
        { ESP_RST_BROWNOUT,"Brownout",            true  },
    };

    const char *label = "Unknown";
    bool        fatal = false;
    const auto  r     = esp_reset_reason();

    for (const auto &e : kReasons) {
        if (e.r == r) { label = e.label; fatal = e.save; break; }
    }

    LOG_PRINTF("[Boot #%lu] Reset reason: %s\n", (unsigned long)cfg.bootCount, label);
    if (fatal) logError(label);
}

// ─────────────────────────────────────────────────────────────────────────────
void Storage::loadStr(const char *key, char *dst, size_t maxLen, const char *def)
{
    const String s = prefs.getString(key, def);
    strncpy(dst, s.c_str(), maxLen - 1);
    dst[maxLen - 1] = '\0';
}
