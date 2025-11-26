#ifndef TELNET_LOGGER_HPP
#define TELNET_LOGGER_HPP

#include <Arduino.h>
#include <WiFi.h>  // For ESP32. Use <ESP8266WiFi.h> for ESP8266
#include <cstdarg> // Required for variadic arguments (printf)
#include <cstdio>  // Required for vsnprintf

// --- Global WiFiClient Declaration ---
// The 'inline' keyword is crucial. It tells the compiler to create only a
// single instance of this object, even if this header is included in
// multiple files. This prevents "multiple definition" linker errors and
// makes the header truly self-contained and reusable.
inline WiFiClient telnetClient;

// --- Class Definition & Implementation ---
class SerialMirror : public Print {
public:
    // Methods defined inside the class are automatically 'inline'.
    size_t write(uint8_t c) override {
        Serial.write(c);
        if (telnetClient && telnetClient.connected()) {
            telnetClient.write(c);
        }
        return 1;
    }

    // Custom printf method for formatted logging.
    void printf(const char *format, ...) {
        char buf[256];
        va_list args;
        va_start(args, format);
        vsnprintf(buf, sizeof(buf), format, args);
        va_end(args);
        print(buf); // This calls the write() method above.
    }
};

// --- Global Logger Instance ---
// The 'inline' keyword here serves the same purpose as above, ensuring
// only one TelnetLogger object exists for your entire project.
inline SerialMirror TelnetLogger;

// --- Logging Macros ---
// These macros provide a simple way to call the logger.
#define LOG_PRINT(...)    TelnetLogger.print(__VA_ARGS__)
#define LOG_PRINTLN(...)  TelnetLogger.println(__VA_ARGS__)
#define LOG_PRINTF(...)   TelnetLogger.printf(__VA_ARGS__)

#endif // TELNET_LOGGER_HPP