#pragma once
/**
 * @file  Storage.h
 * @brief NVS-backed configuration, LittleFS-backed log file, RF key list.
 *
 * All public state lives in the `cfg` struct; callers read/write it directly
 * and call save() to persist.  The log is append-only and auto-trimmed when
 * it exceeds MAX_LOG_BYTES.  All file operations are guarded by _lfsOk so
 * a missing/corrupt LittleFS partition never crashes the system.
 */

#include "Config.h"
#include "Types.h"
#include "Log.hpp"
#include <Preferences.h>
#include <LittleFS.h>

// ─────────────────────────────────────────────────────────────────────────────
class Storage
{
public:
    // ── Runtime-configurable settings ────────────────────────────────────────
    struct Settings {
        unsigned long travelTime;          // ms — full open→close travel
        unsigned long motorDelay;          // ms — relay engage delay
        unsigned long rfDebounce;          // ms — RF repeat suppression window
        unsigned long acDelay;             // ms — auto-close delay after barrier
        uint8_t       pedPercent;          // %  — pedestrian opening width
        uint8_t       motorMode;           // 1 = Dir+En, 2 = Open+Close
        unsigned long preBlinkDelay;       // ms — blink light before moving
        unsigned long acOpenDelay;         // ms — auto close after fully open
        bool          acOpenEnabled;       // auto-close from fully open
        bool          acEnabled;           // auto-close after barrier event
        bool          barrierActiveHigh;   // photo-barrier logic polarity
        bool          limitsActiveHigh;    // limit-switch logic polarity
        char          mqttHost[40];
        char          mqttPort[6];
        char          mqttUser[32];
        char          mqttPass[64];

        // Advanced - Pins
        uint8_t       pinRelay1;
        uint8_t       pinRelay2;
        uint8_t       pinIndicator;
        uint8_t       pinBtnMain;
        uint8_t       pinBtnWifi;
        uint8_t       pinBtnMaint;
        uint8_t       pinBtnPed;
        uint8_t       pinLimOpen;
        uint8_t       pinLimClose;
        uint8_t       pinBarrier;
        uint8_t       pinRfRx;

        // Advanced - Internal Timers (ms)
        unsigned long tCalSafety;
        unsigned long tTravelOvertime;
        unsigned long tRfLearnTimeout;
        unsigned long tCalLongPress;
        unsigned long tWifiLongPress;
        unsigned long tReversePress;
        unsigned long tPedPress;
        unsigned long tRfSavePress;
        unsigned long tComboHold;
        unsigned long tWifiRetry;

        uint32_t      bootCount;
    } cfg;

    void begin();           ///< Mount LittleFS, increment boot counter
    void load();            ///< Read all settings from NVS
    void save();            ///< Write all settings to NVS

    void loadRF(std::vector<RFEntry> &list);
    void saveRF(const std::vector<RFEntry> &list);

    void log(const String &msg);
    void logError(const char *msg) { log("ERROR: " + String(msg)); }
    void dumpLog();
    void clearLog();
    void printResetReason();

private:
    Preferences prefs;
    bool        _lfsOk = false;

    static constexpr const char *K_CONF   = "gate_conf";
    static constexpr const char *K_STATS  = "gate_stats";
    static constexpr const char *LOG_PATH = "/system_log.txt";

    // Safe null-terminated string copy from Preferences
    void loadStr(const char *key, char *dst, size_t maxLen, const char *def);
};

extern Storage storage;
