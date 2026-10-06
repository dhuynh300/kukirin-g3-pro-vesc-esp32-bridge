#pragma once

#include <Arduino.h>
#include "KukirinG3ProProtocol.h"
#include "KukirinG3ProDisplay.h"

/**
 * @file KukirinDisplay.h
 * @brief Single include for the Kukirin G3 Pro display library: protocol (KukirinG3ProProtocol.h) and
 *        driver (KukirinG3ProDisplay.h).
 *
 * The driver acts as the scooter's controller towards the display: it reads the display's command frames
 * (throttle, brake, switches, mode, P-menu settings), answers each one with a status frame (speed, error code,
 * dual drive), and drives the lights, turn signals and horn.
 *
 * Example:
 * @code
 * #include <Arduino.h>
 * #include "KukirinDisplay.h"
 *
 * KukirinDisplay g_display(&Serial2, 35, 33, 39);   // RX (through the 5 V divider), TX, dual/single button
 *
 * void setup() {
 *     g_display.begin(9600);
 *     g_display.initGPIO();
 *     g_display.onTransaction([](const KukirinInputs& in, KukirinResponse& resp) {
 *         resp.speedMph = 18.5f;   // speed to show; called once per display frame
 *     });
 * }
 *
 * void loop() {
 *     g_display.update();   // reads frames, answers them, updates lights; does not block
 * }
 * @endcode
 */

namespace KukirinG3Pro {
    /** @brief The driver under the protocol namespace. */
    using KukirinDisplay = ::KukirinG3ProDisplay;
}

/** @brief Global alias for the driver. */
using KukirinDisplay = ::KukirinG3ProDisplay;
