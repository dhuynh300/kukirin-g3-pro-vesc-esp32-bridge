//! KNOWN PITFALLS & FAILURE MODES:
//! - P01 [VESC UART Baud Rate]: Must strictly operate at 230,400 bps. 115,200 bps causes framing corruption and link loss.
//! - P02 [Hardware Killswitch Level]: GPIO 2 connects to VESC ADC2 (KILL_SW_MODE_ADC2_LOW). LOW (<1.65V) = killed/fail-closed, HIGH (>1.65V) = armed.
//! - P03 [Automotive Brake-to-Start]: B2S requires brake held >= 200ms with neutral throttle (<100 counts) and vehicle stationary (<0.6 mph / <300 ERPM).
//! - P04 [Active Brake Priority]: Brake actuation must unconditionally override and zero throttle with 0ms phase lag.
//! - P05 [Throttle Hysteresis & Plausibility]: Throttle ADC values >1050 counts indicate rail short/hardware fault. Schmitt trigger prevents jitter at deadband.
//! - P06 [Zero-Throttle Lockout]: Drivetrain will not engage if booted or recovered with throttle open (Whiskey-Throttle Lockout).
//! - P07 [Transient Fault Recovery]: Comm/FOC faults cut torque without dropping physical killswitch; recovery requires neutral throttle in traffic.
//! - P08 [CAN Desync & Single-Motor Limp]: On front CAN loss (>100ms), master asserts E-017 and grants 100% rear power authority governed by rider gear.
//! - P09 [Passive Braking Mandate]: Electronic braking strictly commands CONTROL_MODE_CURRENT_BRAKE (0.0A at 0 RPM); CONTROL_MODE_HANDBRAKE is forbidden.
//! - P10 [Dead-man Watchdog]: 5 missed frames (100ms) trips failsafe: Ticks 1-4 Zero-Order Hold (ZOH), Tick 5 cuts torque to 0.0A freewheel.
//! - P11 [Multi-Packet Pacing]: Secondary debug/telemetry packets must not be dispatched back-to-back on same tick (enforce >= 2ms wire gap).
//! - P12 [Packet Framing & CRC]: Frames use 0xAA 0x55 header with CCITT CRC16. Sliding-window resync via memmove required on CRC error.
//! - P13 [Zero Heap Allocation]: Control path must never invoke malloc/new/free in real-time loops.

#pragma once

#include <Arduino.h>
#include "VESCUARTBridge.h"
#include "VESCSafety.h"

/**
 * @file VESCBridge.h
 * @brief Standalone single-header aggregator for the VESC motor controller bridge and safety supervision engine.
 *
 * @details
 * This header aggregates the high-speed UART transport driver (`VESCUARTBridge`) and the fail-closed
 * powertrain safety supervisor (`VESCSafetySupervisor`). It provides a turnkey interface for controlling
 * single or dual VESC motor controllers over a 230,400 bps serial link with deterministic safety interlocks.
 *
 * Applicable across diverse vehicle topologies including electric scooters, e-bikes, skateboards, and robotics.
 *
 * ### Quickstart Usage Pattern:
 * @code
 * #include <Arduino.h>
 * #include "VESCBridge.h"
 *
 * // Instantiate bridge driver on Serial1 (RX=GPIO 34, TX=GPIO 32, 230,400 bps)
 * VESCBridgeDriver g_vesc_bridge(&Serial1, 230400, 34, 32);
 *
 * // Instantiate safety supervisor with hardware killswitch on GPIO 2
 * VESCBridge::VESCSafetySupervisor g_safety(2);
 *
 * void setup() {
 *     g_vesc_bridge.begin();
 *     g_safety.begin();
 * }
 *
 * void loop() {
 *     // Service UART framing and telemetry watchdogs
 *     g_vesc_bridge.update();
 *
 *     // Sample inputs and update safety state machine (B2S, killswitch, faults)
 *     const uint32_t now = millis();
 *     const bool brake = digitalRead(21) == HIGH;
 *     const float norm_throttle = 0.5f; // [0.0f .. 1.0f]
 *     const uint8_t gear = 2;            // Gear 1..3
 *
 *     const uint16_t throttle_req = g_safety.compute_mode_scaled_throttle(norm_throttle, gear, 0.0f, 0.02f);
 *     g_safety.update_b2s(brake, throttle_req, now);
 *
 *     // Compute safe, rate-limited control payload and dispatch at 50 Hz
 *     TXPayloadControl payload = g_safety.compute_safe_control(
 *         throttle_req, 0, brake, true, false, true, 0.02f
 *     );
 *     g_vesc_bridge.send_control(payload);
 *
 *     delay(20);
 * }
 * @endcode
 */

namespace VESCBridge {
    /**
     * @brief Canonical type alias exposing VESCUARTBridge driver under the VESCBridge namespace.
     */
    using VESCBridgeDriver = VESCUARTBridge;
}

/**
 * @brief Global alias for streamlined top-level access.
 */
using VESCBridgeDriver = VESCBridge::VESCBridgeDriver;
