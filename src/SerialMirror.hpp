#ifndef TELNET_LOGGER_HPP
#define TELNET_LOGGER_HPP

#include <Arduino.h>
#include <WiFi.h>
#include <cstdarg>
#include <cstdio>
#include <freertos/semphr.h>

// Global external reference (defined in main.cpp)
extern SemaphoreHandle_t xLogMutex;
extern WiFiClient telnetClient;

class SerialMirror : public Print
{
public:
    size_t write(uint8_t c) override
    {
        // Safe guard: If mutex isn't created yet (e.g. early setup), just use Serial
        if (xLogMutex == NULL)
        {
            Serial.write(c);
            return 1;
        }

        // Try to take mutex to prevent interleaved characters from different tasks
        if (xLogMutex != NULL && xSemaphoreTake(xLogMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
#if USE_SERIAL_DEBUG
            Serial.write(c); // Only write if safe
#endif
            if (telnetClient && telnetClient.connected())
            {
                telnetClient.write(c);
            }
            xSemaphoreGive(xLogMutex);
        }
        else
        {
            // If timeout, write to Serial anyway so we don't lose data
            Serial.write(c);
        }
        return 1;
    }

    void printf(const char *format, ...)
    {
        char buf[512]; // Increased buffer size for safety
        va_list args;
        va_start(args, format);
        vsnprintf(buf, sizeof(buf), format, args);
        va_end(args);
        print(buf);
    }
};

inline SerialMirror TelnetLogger;

// Macros
#define LOG_PRINT(...) TelnetLogger.print(__VA_ARGS__)
#define LOG_PRINTLN(...) TelnetLogger.println(__VA_ARGS__)
#define LOG_PRINTF(...) TelnetLogger.printf(__VA_ARGS__)

#endif // TELNET_LOGGER_HPP