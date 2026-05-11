# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ESP32-C3 firmware for a Home Assistant-integrated sliding gate controller. Supports 433 MHz RF remotes, physical buttons, limit switches, photo-barrier safety sensor, and a web UI. Built with Arduino framework via PlatformIO.

## Build Commands

```bash
# Build
platformio run -e esp32-c3

# Build + upload
platformio run -e esp32-c3 --target upload

# Upload filesystem (web UI files)
platformio run -e esp32-c3 --target uploadfs

# Serial monitor (115200 baud)
platformio device monitor -e esp32-c3

# Telnet debug console (after WiFi connects)
telnet <gate-ip> 23
```

There is also an `esp32-s3` environment in `platformio.ini` for a Waveshare ESP32-S3-Zero target.

## Architecture

### Multi-Task Design (FreeRTOS)

Eight tasks run concurrently. **Never block a high-priority task** — motor and RF tasks run at 5ms intervals. All inter-task communication goes through `cmdQueue` (a 10-slot FreeRTOS queue); never call motor methods directly from network callbacks.

| Priority | Task | Interval | Responsibility |
|----------|------|----------|----------------|
| 5 | Supervisor | 1s | Hardware WDT petting; software WDT check-in monitoring |
| 4 | Motor | 5ms | Relay sequencing, limit/barrier safety, position tracking |
| 3 | Logic | 100ms | Dequeues commands, dispatches to Motor |
| 3 | RF | 5ms | RCSwitch polling, learn mode, debounce |
| 3 | Buttons | 10ms | Button2 polling, combo detection |
| 2 | Network | 25ms | WiFi, WebSocket, MQTT, Telnet, HA entity updates |
| 1 | LED | 100ms | Indicator blink state |
| 1 | IOLogger | 5s | Periodic IO state dump |

### Synchronization Primitives

- `cmdQueue` — commands from button/RF/network → Logic task
- `stateMtx` — guards `MotorState`, `CalState`, `RFLearnState` (Motor, Logic, RF, Network all share)
- `relayMtx` — guards GPIO relay writes (separate from stateMtx to prevent blocking motor task)
- `logMtx` — guards Serial + Telnet output in `Log.hpp`

Always acquire mutexes with a timeout and kick the watchdog if you need to wait in a loop (see `taskLogic` for the pattern).

### Watchdog Strategy

- **Hardware WDT (30s):** Supervisor pets it every 1s only if all tasks checked in
- **Software WDT (20s per task):** Each task updates `wdtMotor`/`wdtNet`/`wdtLogic` timestamps; Supervisor triggers restart if any are stale
- The Arduino `loop()` task is excluded from WDT because WiFiManager blocks it for up to 60s during captive portal

### Key Source Files

- `src/main.cpp` — Entry point; creates all 8 tasks and sets up 4 physical buttons
- `src/Config.h` — Compile-time constants (WDT timeouts, LED blink intervals)
- `src/Types.h` — All enums: `MotorState`, `CalState`, `RFLearnState`, `GateCommand`, `RFEntry`
- `src/Log.hpp` — Thread-safe logger; use `LOG(...)` everywhere instead of `Serial.print`
- `src/Motor.h/cpp` — Relay sequencing, calibration state machine, auto-close logic, position tracking
- `src/RF.h/cpp` — RCSwitch decoder, learn mode (captures 4 codes: Open/Close/Stop/Ped), web scan mode
- `src/Storage.h/cpp` — NVS (Preferences) for settings + RF keys; LittleFS for rotating log file
- `src/GateNetwork.h/cpp` — AsyncWebServer, WebSocket `/ws`, MQTT HA integration, Telnet on port 23
- `src/WifiWrapper.h/cpp` — WiFiManager captive portal abstraction
- `include/secrets.h` — WiFi SSID/pass + MQTT credentials (not committed to repo)

### Command Flow

```
Button / RF / WebSocket / MQTT
         ↓
  GateCommand struct → cmdQueue (10 slots)
         ↓
  taskLogic() — acquires stateMtx, dispatches to Motor methods
         ↓
  taskMotor() — executes relay sequencing at 5ms intervals
```

### Motor Control Modes

Configured via `settings.motorMode`:
- **Mode 1 (Dir+Enable):** Relay1 = direction, Relay2 = enable
- **Mode 2 (Open+Close):** Relay1 = open relay, Relay2 = close relay

### Calibration State Machine

`HOMING → MEASURING → VERIFYING → DONE`

Triggered by web UI or by holding WiFi+Maint buttons together for 2s. Timeout safety net at 90s.

### Storage

- **NVS:** All settings (30+ fields in `Settings` struct) and RF key list (up to ~30 codes)
- **LittleFS (320 KB):** Web UI static files + rotating 5000-byte system log
- Settings have defaults in `Storage.cpp`; NVS overrides apply on subsequent boots

### Flash Layout

Custom `partitions.csv` — no OTA partition. App gets ~1.62 MB; LittleFS gets 320 KB. To change layout, edit `partitions.csv` and clean-build.

## Important Constraints

- **No OTA support** — firmware updates require physical USB connection
- **MQTT max packet 2048 bytes** — set via build flag; increasing requires rebuild
- `AsyncTCP` WDT is disabled via build flag (`-D CONFIG_ASYNC_TCP_USE_WDT=0`) — do not remove this flag or the network task will trigger false WDT resets during WiFi operations
- RF debounce is 400ms — repeated codes within this window are silently dropped; account for this in learn-mode UX
- `secrets.h` is gitignored — new clones need this file created manually
