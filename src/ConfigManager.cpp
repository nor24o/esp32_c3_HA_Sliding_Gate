#include "ConfigManager.h"
#include <esp_task_wdt.h>

ConfigManager sysConfig;

ConfigManager::ConfigManager()
{
    // Defaults
    config.travel_time = 30000;
    config.motor_delay = 700;
    config.rf_debounce = 400;
    config.ac_delay = 5000;
    config.pedestrian_percent = 30; // <--- Default to 30%
    config.ac_barrier = false;
    config.bar_active_high = false;
    config.lim_active_high = false;
    strcpy(config.mqtt_server, "192.168.1.12");
    strcpy(config.mqtt_port, "1883");
    strcpy(config.mqtt_user, "admin");
    strcpy(config.mqtt_pass, "admin");
    config.bootCount = 0;
}

void ConfigManager::begin()
{
    if (!LittleFS.begin(true))
        LOG_PRINTLN("LittleFS Failed!");

    prefs.begin("gate_stats", false);
    config.bootCount = prefs.getUInt("boot_count", 0);
    config.bootCount++;
    prefs.putUInt("boot_count", config.bootCount);
    prefs.end();
}

void ConfigManager::checkResetReason()
{
    esp_reset_reason_t reason = esp_reset_reason();
    const char *reason_str = "Unknown";
    bool save_log = false;
    switch (reason)
    {
    case ESP_RST_POWERON:
        reason_str = "Power On";
        break;
    case ESP_RST_SW:
        reason_str = "Software Reset";
        break;
    case ESP_RST_PANIC:
        reason_str = "Crash/Panic";
        save_log = true;
        break;
    case ESP_RST_TASK_WDT:
        reason_str = "HW Watchdog (Hang)";
        save_log = true;
        break;
    case ESP_RST_WDT:
        reason_str = "Interrupt WDT";
        save_log = true;
        break;
    case ESP_RST_BROWNOUT:
        reason_str = "Brownout";
        save_log = true;
        break;
    default:
        break;
    }
    LOG_PRINTF("Boot #%u | Reason: %s\n", config.bootCount, reason_str);
    if (save_log)
        logError(reason_str);
}

void ConfigManager::load()
{
    LOG_PRINTLN("LOADING PREFS...");
    prefs.begin("gate_conf", true);
    config.travel_time = prefs.getULong("travel_time", 30000);
    config.ac_barrier = prefs.getBool("ac_bar", false);
    config.bar_active_high = prefs.getBool("bar_high", false);
    config.lim_active_high = prefs.getBool("lim_high", false);
    config.motor_delay = prefs.getULong("mot_delay", 700);
    config.rf_debounce = prefs.getULong("rf_deb", 400);
    config.ac_delay = prefs.getULong("ac_delay", 5000);
    config.pedestrian_percent = prefs.getUChar("ped_pct", 30);

    String s = prefs.getString("mqtt_server", "192.168.1.12");
    strncpy(config.mqtt_server, s.c_str(), 40);
    s = prefs.getString("mqtt_port", "1883");
    strncpy(config.mqtt_port, s.c_str(), 6);
    s = prefs.getString("mqtt_user", "admin");
    strncpy(config.mqtt_user, s.c_str(), 32);
    s = prefs.getString("mqtt_pass", "admin");
    strncpy(config.mqtt_pass, s.c_str(), 64);
    prefs.end();
}

void ConfigManager::save()
{
    logInfo("Saving Params...");
    prefs.begin("gate_conf", false);
    prefs.putULong("travel_time", config.travel_time);
    prefs.putBool("ac_bar", config.ac_barrier);
    prefs.putBool("bar_high", config.bar_active_high);
    prefs.putBool("lim_high", config.lim_active_high);
    prefs.putULong("mot_delay", config.motor_delay);
    prefs.putULong("rf_deb", config.rf_debounce);
    prefs.putULong("ac_delay", config.ac_delay);
    prefs.putUChar("ped_pct", config.pedestrian_percent);
    prefs.putString("mqtt_server", config.mqtt_server);
    prefs.putString("mqtt_port", config.mqtt_port);
    prefs.putString("mqtt_user", config.mqtt_user);
    prefs.putString("mqtt_pass", config.mqtt_pass);
    prefs.end();
}

void ConfigManager::loadRF(std::vector<RFEntry> &list)
{
    prefs.begin("gate_conf", true);
    size_t len = prefs.getBytesLength("rf_list");
    if (len > 0 && (len % sizeof(RFEntry) == 0))
    {
        list.resize(len / sizeof(RFEntry));
        prefs.getBytes("rf_list", list.data(), len);
        LOG_PRINTF("Loaded %d RF keys.\n", list.size());
    }
    prefs.end();
}

void ConfigManager::saveRF(const std::vector<RFEntry> &list)
{
    prefs.begin("gate_conf", false);
    if (!list.empty())
    {
        prefs.putBytes("rf_list", list.data(), list.size() * sizeof(RFEntry));
    }
    else
    {
        prefs.remove("rf_list");
    }
    prefs.end();
}

void ConfigManager::logInfo(String msg)
{
    LOG_PRINTLN(msg.c_str());
    File logFile = LittleFS.open(LOG_FILE, "a");
    if (logFile)
    {
        logFile.print("[");
        logFile.print(millis() / 1000);
        logFile.print("s] ");
        logFile.println(msg);
        logFile.close();
    }
}

void ConfigManager::logError(const char *msg)
{
    logInfo("ERROR: " + String(msg));
}

void ConfigManager::dumpLog()
{
    if (!LittleFS.exists(LOG_FILE))
    {
        LOG_PRINTLN("No logs.");
        return;
    }
    File logFile = LittleFS.open(LOG_FILE, "r");
    if (logFile)
    {
        while (logFile.available())
        {
            if (ENABLE_HW_WATCHDOG)
                esp_task_wdt_reset();
            TelnetLogger.write(logFile.read());
        }
        logFile.close();
    }
}

void ConfigManager::clearLog()
{
    LittleFS.remove(LOG_FILE);
}