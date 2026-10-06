//! KNOWN PITFALLS & FAILURE MODES:
//! - P14 [5V Display TX Level]: Display TX line is 5V TTL. ESP32 GPIOs are NOT 5V tolerant. Mandatory voltage divider (R1=1.0k, R2=1.8k) with 2.2nF..4.7nF parallel cap.
//! - P15 [Pure Bitwise XOR Checksum]: TFM13-FEIMI-1 protocol uses pure 12-byte bitwise XOR checksum across Bytes 0..11. E-006 receiver error is asserted by corrupting/inverting this checksum during test injection.
//! - P16 [Display Framing & Cadence]: Display transmits single 20-byte frames (0x01 0x14) at continuous 8.0 Hz (125.2ms period, zero bursts, zero packet duplication). Buffer recovery requires sliding-window memmove.
//! - P17 [Fixed-Point Speed Rendering]: LCD latches directly into 7-segment display without firmware averaging. Requires discrete 2-stage fixed-point digital twin.
//! - P18 [P-Menu Staging]: Live P-menu modifications while riding can cause sudden torque spikes; settings must quarantine until stationary (<= 3.0 mph) and neutral.
//! - P19 [Switch Mutual Exclusion]: Handlebar turn switches are mechanically mutually exclusive; setting both simultaneously violates hardware plausibility.
//! - P20 [Horn Thermal Protection]: 12V horn relay requires 3.0s auto-cutoff watchdog and 100ms re-arm quiet window to prevent coil burnout.
//! - P21 [Synchronous Response Mandate]: Controller response must transmit synchronously within the frame transaction window to satisfy display timeout (<2.5s).
//! - P22 [Brake Release Quorum]: Asymmetric debounce (1-frame instant trip, debounced release) guarantees zero lag on emergency braking.

#pragma once

#include <Arduino.h>
#include "KukirinG3ProProtocol.h"
#include "KukirinG3ProDisplay.h"

/**
 * @file KukirinDisplay.h
 * @brief Standalone single-header aggregator for the Kukirin G3 Pro (TFM13-FEIMI-1) dashboard emulator and accessory driver.
 *
 * @details
 * This header aggregates the complete protocol definitions (`KukirinG3ProProtocol.h`) and real-time
 * display driver engine (`KukirinG3ProDisplay.h`). It provides frame-synchronized ingestion of handlebar
 * inputs (throttle ADC, brake, lights, turn signals, horn, dual motor toggle) and produces 0ms phase-lag
 * binary responses to drive the LCD speedometer, battery gauge, and diagnostic error codes.
 *
 * Configurable for any vehicle (scooter, e-bike, skateboard, robot) with arbitrary GPIO pinouts and baud rates.
 *
 * ### Quickstart Usage Pattern:
 * @code
 * #include <Arduino.h>
 * #include "KukirinDisplay.h"
 *
 * // Instantiate display driver on Serial2 (RX=GPIO 35 via divider, TX=GPIO 33, ButtonDs=GPIO 39)
 * KukirinDisplay g_display(&Serial2, 35, 33, 39);
 *
 * void setup() {
 *     // Initialize UART at 9600 8N1
 *     g_display.begin(9600);
 *
 *     // Initialize accessory GPIOs (lights, turn signals, horn relay)
 *     g_display.initGPIO();
 *
 *     // Optional: Register atomic transaction hook (Pattern B)
 *     g_display.onTransaction([](const KukirinInputs& in, KukirinResponse& resp) {
 *         // in.rawThrottle, in.normalizedThrottle, in.brakeActive, in.gear, etc.
 *         resp.speedMph = 18.5f; // Update dashboard speedometer
 *     });
 * }
 *
 * void loop() {
 *     // Process incoming UART frames and service accessory GPIOs non-blocking
 *     g_display.update();
 * }
 * @endcode
 */

namespace KukirinG3Pro {
    /**
     * @brief Canonical type alias exposing KukirinG3ProDisplay driver under KukirinG3Pro namespace.
     */
    using KukirinDisplay = ::KukirinG3ProDisplay;
}

/**
 * @brief Global alias for streamlined top-level access.
 */
using KukirinDisplay = ::KukirinG3ProDisplay;
