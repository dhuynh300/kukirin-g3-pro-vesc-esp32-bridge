#pragma once

#include <Arduino.h>
#include "VESCUARTBridge.h"
#include "VESCSafety.h"

/**
 * @file VESCBridge.h
 * @brief Single include for the VESC UART link (VESCUARTBridge) and the safety supervisor (VESCSafetySupervisor).
 *
 * Example: send a control frame every 20 ms without blocking the loop.
 * @code
 * #include <Arduino.h>
 * #include "VESCBridge.h"
 *
 * VESCBridgeDriver g_vesc(&Serial1, 230400, 34, 32);   // RX GPIO 34, TX GPIO 32
 * VESCBridge::VESCSafetySupervisor g_safety(2);         // killswitch output on GPIO 2
 *
 * void setup() {
 *     g_vesc.begin();
 *     g_safety.begin();
 * }
 *
 * void loop() {
 *     g_vesc.update();
 *
 *     static uint32_t last_ms = 0;
 *     const uint32_t now = millis();
 *     if (now - last_ms < 20) return;
 *     last_ms = now;
 *
 *     const bool brake = digitalRead(21) == HIGH;
 *     const uint16_t throttle = g_safety.compute_mode_scaled_throttle(0.5f, 2, 0.0f, 0.02f);
 *     g_safety.update_b2s(brake, throttle, now);
 *     g_vesc.send_control(g_safety.compute_safe_control(throttle, 0, brake, true, false, true, 0.02f));
 * }
 * @endcode
 */

namespace VESCBridge {
    /** @brief The UART driver under the library namespace. */
    using VESCBridgeDriver = VESCUARTBridge;
}

/** @brief Global alias for the UART driver. */
using VESCBridgeDriver = VESCBridge::VESCBridgeDriver;
