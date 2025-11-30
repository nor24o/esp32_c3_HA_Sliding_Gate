#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include "Definitions.h"
#include <Preferences.h>
#include <LittleFS.h>
#include "SerialMirror.hpp"

class ConfigManager
{
public:
    // Settings Container
    struct Settings
    {
        unsigned long travel_time;
        unsigned long motor_delay;
        unsigned long rf_debounce;
        unsigned long ac_delay;
        uint8_t pedestrian_percent; // <--- ADD THIS
        bool ac_barrier;
        bool bar_active_high;
        bool lim_active_high;
        char mqtt_server[40];
        char mqtt_port[6];
        char mqtt_user[32];
        char mqtt_pass[64];
        unsigned int bootCount;
    } config;

    ConfigManager();
    void begin();
    void load();
    void save();

    // RF Persistence
    void saveRF(const std::vector<RFEntry> &list);
    void loadRF(std::vector<RFEntry> &list);

    // Logging
    void logInfo(String msg);
    void logError(const char *msg);
    void dumpLog();
    void clearLog();
    void checkResetReason();

private:
    Preferences prefs;
    const char *LOG_FILE = "/system_log.txt";
};

extern ConfigManager sysConfig;

#endif