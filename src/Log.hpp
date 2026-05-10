#pragma once
/**
 * @file  Log.hpp
 * @brief Thread-safe dual-output logger: Serial + active Telnet client.
 *
 * Derives from Print so the LOG_* macros work with any Arduino type.
 * Uses logMtx (a FreeRTOS mutex) to prevent interleaved characters from
 * different tasks.  Falls back to direct Serial.write() if the mutex is
 * not yet created or times out, so no log line is ever silently dropped.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <freertos/semphr.h>
#include <cstdarg>
#include <cstdio>

extern SemaphoreHandle_t logMtx;
extern WiFiClient        telnetClient;

class Logger : public Print
{
public:
    size_t write(uint8_t c) override
    {
        if (!logMtx) {               // called before RTOS is up
            Serial.write(c);
            return 1;
        }
        if (xSemaphoreTake(logMtx, pdMS_TO_TICKS(10)) == pdTRUE) {
            Serial.write(c);
            if (telnetClient && telnetClient.connected())
                telnetClient.write(c);
            xSemaphoreGive(logMtx);
        } else {
            Serial.write(c);         // mutex timeout — never drop
        }
        return 1;
    }

    // Variadic helper avoids ambiguity with Print::printf on some toolchains
    void logf(const char *fmt, ...) __attribute__((format(printf, 2, 3)))
    {
        char buf[512];
        va_list ap;
        va_start(ap, fmt);
        vsnprintf(buf, sizeof(buf), fmt, ap);
        va_end(ap);
        print(buf);
    }
};

// Single global instance (inline — valid in C++17, zero-cost)
inline Logger Log;

#define LOG_PRINT(...)   Log.print(__VA_ARGS__)
#define LOG_PRINTLN(...) Log.println(__VA_ARGS__)
#define LOG_PRINTF(...)  Log.logf(__VA_ARGS__)
