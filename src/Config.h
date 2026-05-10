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

// LED blink intervals
static constexpr unsigned long BLINK_OPENING      = 1000;
static constexpr unsigned long BLINK_CLOSING      = 500;
static constexpr unsigned long BLINK_CALIBRATING  = 100;
static constexpr unsigned long BLINK_RF_LEARN     = 250;

// ─────────────────────────────────────────────────────────────────────────────
// Limits
// ─────────────────────────────────────────────────────────────────────────────
static constexpr size_t MAX_LOG_BYTES = 5000;
