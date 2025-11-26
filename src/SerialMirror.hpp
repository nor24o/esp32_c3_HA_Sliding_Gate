#ifndef TELNET_LOGGER_HPP
#define TELNET_LOGGER_HPP

#include <Arduino.h>
#include <WiFi.h>  
#include <cstdarg> 
#include <cstdio>  
#include <freertos/semphr.h> // Include FreeRTOS for Mutex

// --- Global RTOS/Network Declarations ---
inline WiFiClient telnetClient;
inline SemaphoreHandle_t xLogMutex; // Mutex to protect Serial/Telnet

// --- Class Definition & Implementation ---
class SerialMirror : public Print {
public:
    size_t write(uint8_t c) override {
        // Attempt to acquire the mutex for a short period (10ms)
        if (xLogMutex != NULL && xSemaphoreTake(xLogMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            
            Serial.write(c);
            if (telnetClient && telnetClient.connected()) {
                telnetClient.write(c);
            }
            
            xSemaphoreGive(xLogMutex); // Release the mutex
        } else {
            // Fallback to Serial if mutex fails
            Serial.write(c); 
        }
        return 1;
    }

    void printf(const char *format, ...) {
        char buf[256];
        va_list args;
        va_start(args, format);
        vsnprintf(buf, sizeof(buf), format, args);
        va_end(args);
        print(buf);
    }
};

// --- Global Logger Instance ---
inline SerialMirror TelnetLogger;

// --- Logging Macros (Unchanged) ---
#define LOG_PRINT(...)    TelnetLogger.print(__VA_ARGS__)
#define LOG_PRINTLN(...)  TelnetLogger.println(__VA_ARGS__)
#define LOG_PRINTF(...)   TelnetLogger.printf(__VA_ARGS__)

#endif // TELNET_LOGGER_HPP