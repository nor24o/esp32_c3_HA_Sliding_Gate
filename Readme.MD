# ESP32 Sliding Gate Controller for Home Assistant

This project transforms a standard sliding gate motor into a smart, WiFi-enabled device that integrates seamlessly with Home Assistant. It provides remote control, real-time status monitoring, and enhanced safety features using an ESP32 microcontroller.

## Features

- **Home Assistant Integration**: Full integration via MQTT for controlling and monitoring the gate as a "cover" entity.
- **Multiple Control Methods**:
    - **Physical Buttons**: Dedicated hardware buttons for Open, Close, and Stop operations.
    - **RF Remote**: Supports 433MHz RF remote controls for open, close, stop, and move-to-50% commands.
    - **Home Assistant UI**: Control the gate from your HA dashboard, including open, close, stop, and move to a specific position.
- **State and Position Tracking**: The controller knows the gate's state (opening, closing, stopped, open, closed) and its precise position (0-100%).
- **Automatic Calibration**: A one-touch calibration sequence automatically measures and saves the gate's full travel time for accurate position tracking.
- **Safety Features**:
    - **Photo Barrier**: Detects obstacles during closing, immediately stops and re-opens the gate. Includes an auto-resume feature after the obstacle is cleared.
    - **Limit Switches**: Uses open and close limit switches for reliable end-of-travel detection.
    - **Movement Inhibit**: An input to disable all gate movement for maintenance or safety.
- **Persistent Storage**: Saves the calibrated gate travel time to non-volatile memory (LittleFS), so it survives reboots.
- **WiFi Manager**: On first boot or via a long-press, it creates a WiFi Access Point ("GateControllerAP") for easy on-site configuration of your WiFi credentials.
- **Indicator Light**: A connected light blinks to provide a visual indication of gate movement (slow for opening, fast for closing).

## Hardware Requirements

- **Microcontroller**: ESP32 (the code is written for an ESP32-C3 but should be adaptable).
- **Relays**:
    - 1x Relay for Motor Direction
    - 1x Relay for Motor Enable/Power
    - 1x Relay for Indicator Light
- **Buttons**:
    - 3x Push buttons (for Manual Open, Close, Stop)
- **Sensors**:
    - 2x Limit switches (for fully open and fully closed positions)
    - 1x Photoelectric barrier sensor
    - 1x Movement inhibit switch (optional)
- **RF Receiver**: A 433MHz RF receiver module (e.g., MX-RM-5V).
- **Power Supply**: Appropriate power supply for the ESP32 and relays.

## Software Dependencies (Arduino Libraries)

- `Arduino.h`
- `RCSwitch.h` (for RF remote control)
- `Button2.h` (for handling button presses and long-presses)
- `WiFi.h`
- `WiFiManager.h` (for easy WiFi configuration)
- `ArduinoHA.h` (for Home Assistant integration)
- `LittleFS.h` (for saving configuration)

## Configuration

### 1. `secrets.h`

You must create a `secrets.h` file in the same directory as your main sketch. This file stores your sensitive information.

```cpp
// secrets.h

#ifndef SECRETS_H
#define SECRETS_H

// --- WiFi & MQTT Credentials ---
#define MQTT_SERVER "192.168.1.100" // IP address of your MQTT broker
#define MQTT_USER "your_mqtt_username"
#define MQTT_PASSWORD "your_mqtt_password"

// --- RF Remote Codes ---
// Replace these with the codes from your 433MHz remote
#define RF_GATE_OPEN_CODE   1234567
#define RF_GATE_CLOSE_CODE  7654321
#define RF_GATE_STOP_CODE   1111111
#define RF_GATE_POS50_CODE  2222222

#endif // SECRETS_H