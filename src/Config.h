#pragma once
/**
 * @file  Config.h
 * @brief Compile-time configuration — the single file a user edits.
 *
 * Hardware mode, pin numbers, timing constants, and feature flags
 * all live here.  Nothing in this file produces any code.
 */

#include <Arduino.h>

// ─────────────────────────────────────────────────────────────────────────────
// Watchdog
// ─────────────────────────────────────────────────────────────────────────────
static constexpr bool          HW_WDT_ENABLED  = true;
static constexpr uint32_t      HW_WDT_TIMEOUT_S  = 30;       // hardware WDT (seconds)
static constexpr unsigned long SOFT_WDT_TIMEOUT_MS = 20000;  // software check-in limit

// ─────────────────────────────────────────────────────────────────────────────
// Pins
// ─────────────────────────────────────────────────────────────────────────────
static constexpr int PIN_RELAY_1      = 1; // DIR (Mode 1) or OPEN (Mode 2)
static constexpr int PIN_RELAY_2      = 4; // ENABLE (Mode 1) or CLOSE (Mode 2)

static constexpr int PIN_INDICATOR    = 3;
static constexpr int PIN_BTN_MAIN     = 21;
static constexpr int PIN_BTN_WIFI     = 6;
static constexpr int PIN_BTN_MAINT    = 7;
static constexpr int PIN_BTN_PED      = 8;
static constexpr int PIN_LIM_OPEN     = 10;
static constexpr int PIN_LIM_CLOSE    = 20;
static constexpr int PIN_BARRIER      = 2;
static constexpr int PIN_RF_RX        = 5;
// ─────────────────────────────────────────────────────────────────────────────
// Timing (all in milliseconds unless noted)
// ─────────────────────────────────────────────────────────────────────────────
static constexpr unsigned long T_WIFI_RETRY       = 30000;
static constexpr unsigned long T_CAL_LONG_PRESS   = 8000;
static constexpr unsigned long T_WIFI_LONG_PRESS  = 5000;
static constexpr unsigned long T_PED_PRESS        = 200;
static constexpr unsigned long T_REVERSE_PRESS    = 1000;
static constexpr unsigned long T_RF_LEARN_TIMEOUT = 60000;
static constexpr unsigned long T_RF_SAVE_PRESS    = 1500;
static constexpr unsigned long T_COMBO_HOLD       = 2000;
static constexpr unsigned long T_CAL_SAFETY       = 90000;
static constexpr unsigned long T_TRAVEL_OVERTIME  = 3000;   // margin past travel_time

// LED blink intervals
static constexpr unsigned long BLINK_OPENING      = 1000;
static constexpr unsigned long BLINK_CLOSING      = 500;
static constexpr unsigned long BLINK_CALIBRATING  = 100;
static constexpr unsigned long BLINK_RF_LEARN     = 250;

// ─────────────────────────────────────────────────────────────────────────────
// Limits
// ─────────────────────────────────────────────────────────────────────────────
static constexpr size_t MAX_LOG_BYTES = 5000;
