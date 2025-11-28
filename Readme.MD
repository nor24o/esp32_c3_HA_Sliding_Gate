# ESP32 Smart Sliding Gate Controller

This project transforms a standard sliding gate motor into a smart, WiFi-enabled device that integrates seamlessly with Home Assistant. It provides remote control, real-time status monitoring, and enhanced safety features using an ESP32 microcontroller.

Unlike basic controllers, this firmware features a **self-contained Web Dashboard** with real-time WebSocket updates, allowing full configuration and control directly from your browser without needing a hub.

## 🚀 Features

* **Home Assistant Integration**: Automatic discovery via MQTT. Creates covers, buttons, and binary sensors for full control and monitoring.
* **Real-Time Web Dashboard**: A responsive, on-device interface (hosted on Port 80) to control the gate, view sensor status, and manage settings via WebSockets (Port 81).
* **Advanced RF Management**:
    * **Unlimited Remotes**: Stores an unlimited number of RF codes.
    * **Scan & Learn UI**: Capture codes directly from the Web Interface and assign specific functions (Open, Close, Stop, Pedestrian, Toggle).
    * **Management**: View and delete individual remotes from a list.
* **Smart Control Methods**:
    * **Physical Buttons**: Inputs for Manual Open, Close, and Stop switches.
    * **Web & HA**: Full control via network.
* **State and Position Tracking**: Accurate position tracking (0-100%) and state reporting (Opening, Closing, Stopped).
* **Automatic Calibration**: A one-touch sequence that measures the gate's travel time to ensure precise positioning. Includes a "Cancel" safety feature.
* **Safety Features**:
    * **Photo Barrier**: Detects obstacles during closing, stops, and reverses.
    * **Auto-Close**: Configurable timer to automatically close the gate after an obstacle is cleared.
    * **Limit Switches**: Input support for Open and Close limit switches.
    * **Motor Protection**: Configurable delay before reversing motor direction to save gears.
* **Persistent Storage**: Uses ESP32 **Preferences (NVS)** for crash-proof saving of configuration and RF codes.
* **WiFi Manager**: Creates a captive portal hotspot (`GateControllerAP`) for easy initial setup of WiFi credentials.

## 🧠 Software Architecture (FreeRTOS)

The firmware leverages **FreeRTOS** to handle multiple tasks simultaneously, ensuring that network lag never affects safety-critical motor operations.

| Task Name | Priority | Description |
| :--- | :--- | :--- |
| **Supervisor** | 5 (High) | **Watchdog Monitor.** constantly checks if other tasks are responsive. If a task hangs (e.g., WiFi freezes), it reboots the system to restore functionality. |
| **Motor** | 4 | **Safety Core.** Runs continuously to monitor Limit Switches and Photo Barriers. Directly controls relays to ensure immediate stop reaction (<10ms) regardless of WiFi status. |
| **GateLogic** | 3 | **State Machine.** Processes high-level commands (Open/Close) and manages the complex state transitions (Opening -> Stopped -> Closing). |
| **RF** | 3 | **Signal Decoder.** dedicated listener for 433MHz signals to ensure remote presses are never missed. |
| **Button** | 3 | **Input Handler.** Debounces physical push buttons to prevent phantom triggers. |
| **Network** | 2 | **Comms Hub.** Handles WiFi connection, MQTT messaging, Web Server, and WebSocket broadcasts. |
| **LED** | 1 | **Status Indicator.** Manages blink patterns for feedback. |
| **Logger** | 1 | **Diagnostics.** Periodically dumps system stats to internal logs without blocking critical paths. |

## 🔌 Hardware Requirements & Pinout

**Microcontroller:** ESP32-C3 (Code is adaptable to other ESP32 variants).

### GPIO Pin Mapping

| Component | GPIO Pin | Type | Logic / Notes |
| :--- | :--- | :--- | :--- |
| **Motor Open Relay** | `GPIO 1` | Output | Active HIGH (3.3V = Relay ON) |
| **Motor Close Relay** | `GPIO 4` | Output | Active HIGH (3.3V = Relay ON) |
| **Warning Light** | `GPIO 3` | Output | Active HIGH (Blinks during movement) |
| **RF Receiver** | `GPIO 5` | Input | Data Pin (433MHz Receiver) |
| **Open Limit Switch** | `GPIO 10` | Input | **Active LOW** (Connect to GND when hit) |
| **Close Limit Switch** | `GPIO 20` | Input | **Active LOW** (Connect to GND when hit) |
| **Photo Barrier** | `GPIO 2` | Input | **Active LOW** (Connect to GND when blocked) |
| **Manual Button (Main)** | `GPIO 21` | Input | Push Button to GND |
| **Manual Button (WiFi)** | `GPIO 6` | Input | Push Button to GND (Long press for Config) |
| **Manual Button (Maint)**| `GPIO 7` | Input | Push Button to GND |

> **Note:** All inputs use internal **Pull-Up** resistors. Wire your sensors/switches to connect the GPIO pin to **Ground (GND)** when activated.

## 📦 Software Dependencies

This project is built using **PlatformIO**. Ensure the following libraries are installed:

* `Arduino.h`
* `RCSwitch` (RF decoding)
* `Button2` (Debouncing physical buttons)
* `WiFi` & `WiFiManager` (Network connection)
* `ArduinoHA` (MQTT for Home Assistant)
* `ArduinoJson` (Data parsing)
* `WebSockets` (Real-time Web UI communication)
* `WebServer` & `ESPmDNS`
* `Preferences` (Settings storage)
* `LittleFS` (System logs)

## ⚙️ Initial Setup

1. **Flash Firmware:** Upload the code to your ESP32-C3.
2. **Connect to WiFi:**
    * On first boot, connect to the Hotspot: **`GateControllerAP`**.
    * A portal will open (or visit `192.168.4.1`).
    * Select your home WiFi and enter the password.
3. **Access Dashboard:**
    * Find the device IP in your router.
    * Navigate to `http://<DEVICE_IP>` in your browser.

## 🖥️ Web Dashboard Guide

The device hosts a complete control panel divided into four sections:

### 1. Gate Control
Controls the motor and displays live status.
* **Status:** Current operation (OPENING, CLOSING, STOPPED).
* **Position:** Real-time percentage.
* **Buttons:** Manual control overrides.

### 2. Sensors
Diagnostic panel for hardware inputs.
* **Green:** Input is inactive (Safe/Free).
* **Red:** Input is active (Limit hit or Barrier blocked).

### 3. RF Remotes
* **Scan New Remote:** Click this, then press a button on your keyfob. The code will appear instantly. Select a function (Open/Close/Stop/Pedestrian/Toggle) and click **ADD**.
* **List:** View all saved codes and delete unwanted ones.

### 4. System & Maintenance
* **Calibrate:** Starts the auto-calibration run (Close -> Open -> Close).
* **WiFi/MQTT:** Redirects to the configuration page to change broker settings or timings.
* **Logs:** Shows internal debug logs for troubleshooting.

## 🕹️ Hardware Controls & Hidden Modes

You can access critical configuration modes using the physical buttons on the board without needing WiFi access.

| Button Combo | Duration | Action | LED / Behavior |
| :--- | :--- | :--- | :--- |
| **WiFi Button** | Hold > 5s | **Reset WiFi** | Starts Config Portal (`GateControllerAP`) |
| **Maint Button** | Hold > 8s | **Start Calibration** | Gate starts closing to find limits |
| **Main Button** | Hold > 1s | **Force Reverse** | Reverses direction immediately |
| **WiFi + Maint** | Hold > 2s | **RF Learn Mode** | Enters Manual RF Learning Sequence |

### Manual RF Learning Sequence
If you trigger RF Learning via buttons (WiFi + Maint), the system enters a sequential learning mode. The LED will blink to indicate waiting status.
1. **Wait for OPEN:** Press the remote button you want to use for OPEN.
2. **Wait for CLOSE:** Press the remote button for CLOSE.
3. **Wait for STOP:** Press the remote button for STOP.
4. **Wait for PEDESTRIAN:** Press the remote button for 50% Opening.
*Note: This overwrites the first 4 slots in the memory.*

## 📡 Telnet Interface

For advanced debugging and integration, the device runs a Telnet server on **Port 23**.

**Connection:** `telnet <DEVICE_IP>`

### Available Commands
* `logs` - Dumps the current system log buffer.
* `conf` - Prints current configuration (MQTT, Timings, RF count).
* `wifi` - Forces the WiFi Config Portal to open.
* `restart` - Reboots the ESP32.
* `start_cal` - Triggers the calibration sequence.
* `learn_rf` - Triggers the RF Learning sequence.
* `clear_logs` - Wipes the log file from internal storage.

## 🔧 Configuration Parameters

Accessible via the `/config` page:

* **MQTT Settings:** Broker IP, Port, Username, Password.
* **Travel Time:** Duration (ms) for full gate movement (set automatically by calibration).
* **Motor Direction Delay:** Delay (ms) before switching direction to protect mechanical gears.
* **RF Debounce:** Time (ms) to ignore repeated signals.
* **Auto-Close Delay:** Time (ms) to wait before auto-closing (if feature is enabled).
* **Auto-Close after Barrier:** Checkbox to enable automatic closing 5 seconds after an obstacle is cleared.

## 🏠 Home Assistant Integration

Once MQTT is configured, the device auto-discovers in Home Assistant.

**Entities:**
* `cover.sliding_gate`
* `button.sliding_gate_open`, `close`, `stop`
* `button.sliding_gate_calibrate`, `cancel`
* `binary_sensor.sliding_gate_barrier` (Safety)
* `binary_sensor.sliding_gate_lim_open`, `lim_close`
* `sensor.sliding_gate_rf` (Last code received)

## ⚠️ Troubleshooting

* **Gate moves wrong way:** Swap wires on Pin 1 and Pin 4.
* **Gate stops immediately:** Verify Limit Switches are N.O. (Normally Open) and wired to GND.
* **Settings won't save:** Ensure you wait for the "Rebooting..." message in the logs/web UI after saving.
* **WiFi Reset:** Long-press the **WiFi Button (GPIO 6)** for 5 seconds to reset network settings.