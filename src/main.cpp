/**
 * =============================================================================
 * @file main.cpp
 * @brief Kukirin G3 Pro ESP32 Hardware Bridge Main Production Firmware.
 * @details Core 1 real-time vehicle bridging firmware:
 *          - Interfaces with Kukirin TFM13-FEIMI-1 dual-motor display using KukirinG3ProDisplay driver.
 *          - Implements Automotive Brake-to-Start (Ready-to-Drive) and GPIO 2 Hardware Killswitch.
 *          - Bridges throttle/brake requests and telemetry over CRC16 UART to Master VESC @ 230,400 bps.
 *          - Handles CAN desync fail-safes and E-017 / E-003 / E-002 error arbitration.
 *          - Enforces PA-governed acceleration slew rate (20k..100k counts/s) and current deadband (50 counts).
 *          - Drives vehicle lighting, turn signals, brake strobe, and horn relay via hardware engine.
 *
 * //! KNOWN PITFALLS & FAILURE MODES:
 * //! - P12 [GPI-Only Restriction]: GPIO 34 (VescRx) and GPIO 35 (DispRx) are input-only
 * //!   without internal pull-ups/pull-downs. Never configure as OUTPUT or enable internal pull resistors.
 * //! - P13 [5V Display TX Level]: Display TX sends 5V logic. It MUST pass through an
 * //!   attenuating voltage divider (R1=1.0k, R2=1.8k) to ESP32 RX (3.3V) to prevent MCU destruction.
 * //! - P11 [PSRAM Bus Conflict]: GPIO 16 and GPIO 17 are strictly reserved for ESP32-WROVER PSRAM.
 * //!   Never reassign these pins to GPIO peripherals or UART functions.
 * //! - P02 [Fail-Closed Killswitch]: GPIO 2 is hardwired to VESC Master ADC2 (LOW <1.65V = killed).
 * //!   GPIO 2 boots LOW and MUST remain LOW until B2S interlock passes (stationary standstill, brake held >= 200ms).
 * //! - P07 [Transient Single-Frame Immunity]: Transient UART corruption (1-24 frames) MUST NOT trip link drop.
 * //!   Link loss trips strictly after 500ms timeout (25 consecutive frames).
 * //! - P14 [Event-Driven Control Dispatch]: Core 1 control loop strictly forbids delay(), dynamic
 * //!   memory allocation (malloc/new/std::string), and blocking waits. Primary control frames dispatch
 * //!   immediately (<50us phase lag) upon display packet arrival.
 * //! - P08 [Passive Braking Mandate]: Passive current braking (set-brake-rel) only; active handbrake mode
 * //!   (set-handbrake) is forbidden to prevent coil overheating and static battery drain at 0 RPM.
 * =============================================================================
 */

#include <Arduino.h>
#include <esp_system.h>
#include "build_version.h"
#include "KukirinDisplay.h"
#include "VESCBridge.h"

namespace PowertrainConfig = VESCBridge::PowertrainConfig;

// =============================================================================
// Authoritative System & Hardware Configuration (Zero Hardcoded Magic Numbers)
// =============================================================================

namespace HardwareConfig {
    // CAN Bus Topology Node Identifiers (Aliased from canonical VESCConfig)
    struct CanBus {
        static constexpr uint8_t CAN_ID_REAR_VESC  = VESCConfig::CAN_ID_REAR_VESC;  /**< Master Rear VESC controller connected via UART1. */
        static constexpr uint8_t CAN_ID_FRONT_VESC = VESCConfig::CAN_ID_FRONT_VESC; /**< Slave Front VESC controller connected via CAN bus. */
    };

    // Serial & Bus Baud Rates
    struct BaudRate {
        static constexpr uint32_t SerialMonitor = 115200; /**< USB-UART console monitor and CLI. */
        static constexpr uint32_t VescUart      = 230400; /**< High-speed real-time VESC UART1 transport. */
        static constexpr uint32_t DisplayUart   = 9600;   /**< Kukirin TFM13-FEIMI-1 display UART2 interface. */
    };

    // Powertrain & VESC Hardware Pin Assignments
    struct VescPins {
        static constexpr int8_t Rx         = 34; /**< GPIO 34 RX (Master VESC TX, Input-Only GPI). */
        static constexpr int8_t Tx         = 32; /**< GPIO 32 TX (Master VESC RX). */
        static constexpr int8_t Killswitch = 2;  /**< GPIO 2 Hardware Killswitch to Master VESC ADC2 (Safe LOW). */
    };

    // Display & Handlebar Hardware Pin Assignments
    struct DisplayPins {
        static constexpr int8_t Rx       = 35; /**< GPIO 35 RX (Display TX via voltage divider, Input-Only GPI). */
        static constexpr int8_t Tx       = 33; /**< GPIO 33 TX (Display RX). */
        static constexpr int8_t ButtonDs = 39; /**< GPIO 39 Handlebar D/S Pushbutton (Input-Only GPI). */
    };

    // Vehicle Accessory & Lighting Pin Assignments
    struct AccessoryPins {
        static constexpr int8_t LeftLed      = 18; /**< GPIO 18 Left Turn Signal Blinker MOSFET. */
        static constexpr int8_t RightLed     = 19; /**< GPIO 19 Right Turn Signal Blinker MOSFET. */
        static constexpr int8_t BrakeLed     = 21; /**< GPIO 21 Rear Brake Light MOSFET. */
        static constexpr int8_t HeadlightLed = 22; /**< GPIO 22 Front Headlight & Taillight MOSFET. */
        static constexpr int8_t Horn         = 23; /**< GPIO 23 12V Horn Relay Driver. */
        static constexpr int8_t AuxLed       = 4;  /**< GPIO 4 Auxiliary Lighting (Safe LOW). */
        static constexpr int8_t Argb1        = 25; /**< GPIO 25 ARGB Channel 1 (Safe LOW). */
        static constexpr int8_t Argb2        = 26; /**< GPIO 26 ARGB Channel 2 (Safe LOW). */
        static constexpr int8_t Argb3        = 27; /**< GPIO 27 ARGB Channel 3 (Safe LOW). */
        static constexpr int8_t Argb4        = 14; /**< GPIO 14 ARGB Channel 4 (Safe LOW). */
    };
}

static constexpr uint32_t VESC_COMMS_TIMEOUT_MS  = PowertrainConfig::VESC_COMMS_TIMEOUT_MS;
static constexpr uint8_t  DEFAULT_PB_REGEN_LEVEL = 3;

// Telemetry fault flag masks
static constexpr uint8_t FAULT_FLAG_CAN_DESYNC   = 0x01;
static constexpr uint8_t FAULT_FLAG_HW_MASK      = ~FAULT_FLAG_CAN_DESYNC;

// =============================================================================
// Fault Translation Adapter
// =============================================================================

/**
 * @brief Translates internal VESCBridge::SafetyFault classification to KukirinG3Pro display ErrorCode.
 * @param[in] fault Supervisory safety fault enum.
 * @return Canonical KukirinG3Pro::ErrorCode enum for display rendering.
 */
inline KukirinG3Pro::ErrorCode toKukirinErrorCode(VESCBridge::SafetyFault fault) {
    switch (fault) {
        case VESCBridge::SafetyFault::KillswitchDisarmed:
        case VESCBridge::SafetyFault::FocFaultLockout:
        case VESCBridge::SafetyFault::CommsRecoveryLockout:
        case VESCBridge::SafetyFault::DisplayStalled:
            return KukirinG3Pro::ErrorCode::E02_Diagnostic2;
        case VESCBridge::SafetyFault::VescOffline:
            return KukirinG3Pro::ErrorCode::E03_MainController;
        case VESCBridge::SafetyFault::CanDesync:
            return KukirinG3Pro::ErrorCode::E17_SubControllerRecv;
        case VESCBridge::SafetyFault::None:
        default:
            return KukirinG3Pro::ErrorCode::None;
    }
}

// =============================================================================
// Boot Reason Mapping (Zero Dynamic Memory Allocation)
// =============================================================================

static const char* map_reset_reason(esp_reset_reason_t reason) {
    switch (reason) {
        case ESP_RST_POWERON:   return "PowerOn";
        case ESP_RST_BROWNOUT:  return "Brownout";
        case ESP_RST_SW:        return "Software";
        case ESP_RST_PANIC:     return "CrashPanic";
        case ESP_RST_INT_WDT:
        case ESP_RST_TASK_WDT:
        case ESP_RST_WDT:       return "Watchdog";
        default:                return "Other";
    }
}

// =============================================================================
// Global Subsystem Instances (Static Zero-Dynamic-Allocation at Boot)
// =============================================================================

static VESCUARTBridge                  g_vesc(&Serial1, HardwareConfig::BaudRate::VescUart, HardwareConfig::VescPins::Rx, HardwareConfig::VescPins::Tx);
static KukirinG3ProDisplay             g_display(&Serial2, HardwareConfig::DisplayPins::Rx, HardwareConfig::DisplayPins::Tx, HardwareConfig::DisplayPins::ButtonDs);
static VESCBridge::VESCSafetySupervisor g_safety(HardwareConfig::VescPins::Killswitch);

// =============================================================================
// Vehicle State & Safety Guardrails
// =============================================================================

static const char*          s_reset_reason_str = "Other";
static bool                 s_vesc_link_established = false;
static bool                 s_verbose_packet_loss_log = false;
static volatile float       g_vehicle_speed_mph = 0.0f;
static volatile bool        g_dual_motor_mode = true;
static volatile bool        g_can_desync = false;
static volatile uint8_t     g_pmenu_regen_level = DEFAULT_PB_REGEN_LEVEL;

// Static switch state tracking for batched TDM secondary telemetry
static uint8_t              s_sw_gear = 1;
static bool                 s_sw_lights = false;
static const char*          s_sw_trn = "0";
static bool                 s_sw_horn = false;
static bool                 s_switches_dirty = true; // Forces single initial dispatch at boot

static TXPayloadControl     g_vesc_control_payload;

// Lightweight zero-copy Flash pointer queued for 10ms TDM Slot B
static const char*          s_pending_vesc_log = nullptr;

static inline void queue_vesc_log(const char* msg) {
    s_pending_vesc_log = msg;
}

// Empirical Cadence & Telemetry Diagnostic Tracking
static volatile uint32_t    s_vesc_packet_count = 0;
static volatile uint32_t    s_vesc_single_drop_count = 0;
static volatile uint32_t    s_vesc_streak_drop_count = 0;
static volatile uint32_t    s_vesc_max_dt_ms = 0;
static volatile uint32_t    s_vesc_last_pkt_time_ms = 0;

static volatile uint32_t    s_disp_packet_count = 0;
static volatile uint32_t    s_disp_single_drop_count = 0;
static volatile uint32_t    s_disp_streak_drop_count = 0;
static volatile uint32_t    s_disp_min_dt_ms = 0xFFFFFFFF;
static volatile uint32_t    s_disp_max_dt_ms = 0;
static volatile uint32_t    s_disp_last_dt_ms = 0;
static volatile uint32_t    s_disp_last_pkt_time_ms = 0;

// Core 1 Loop Performance Diagnostics
static volatile uint32_t    s_loop_iter_count = 0;
static volatile uint32_t    s_loop_rate_hz = 0;
static volatile uint32_t    s_loop_max_exec_us = 0;
static uint32_t             s_loop_rate_timer_ms = 0;

// =============================================================================
// VESC Telemetry Callback (Zero Dynamic Memory Allocation)
// =============================================================================

/**
 * @brief Dispatcher callback invoked when telemetry frames arrive from the Master VESC.
 * @param[in] type MessageType enum identifier.
 * @param[in] payload Pointer to verified incoming payload bytes.
 * @param[in] len Length in bytes of the payload (accepts 2-byte or 3-byte payloads).
 */
void onVESCMessage(MessageType type, const uint8_t* payload, uint8_t len) {
    if (type == MSG_RX_TELEMETRY && payload != nullptr && len >= 2) {
        const uint32_t now = millis();
        s_vesc_packet_count++;
        if (s_vesc_last_pkt_time_ms > 0) {
            const uint32_t dt = now - s_vesc_last_pkt_time_ms;
            if (dt > s_vesc_max_dt_ms) {
                s_vesc_max_dt_ms = dt;
            }
        }
        s_vesc_last_pkt_time_ms = now;

        if (!s_vesc_link_established) {
            s_vesc_link_established = true;
            Serial.println(F("E:VescOnline"));
            queue_vesc_log("E:VescOnline");
        }

        // Decode Little-Endian int16_t matching RXTelemetryPayload and LispBM format (100x cm/s scaling)
        const int16_t speed_raw = static_cast<int16_t>(payload[0] | (static_cast<uint16_t>(payload[1]) << 8));
        g_vehicle_speed_mph = fabsf(PowertrainConfig::cms_to_mph(speed_raw));

        g_display.setSpeedMph(g_vehicle_speed_mph);

        // Check optional fault_flags byte (Bit 0: CAN Desync, other bits: VESC faults)
        if (len >= sizeof(RXTelemetryPayload)) {
            const uint8_t fault_flags = payload[2];
            const bool desync = (fault_flags & FAULT_FLAG_CAN_DESYNC) != 0;
            const uint8_t hw_faults = (fault_flags & FAULT_FLAG_HW_MASK) >> 1;

            if (desync != g_can_desync) {
                if (desync) {
                    Serial.println(F("E:CanDesync:E17"));
                    queue_vesc_log("E:CanDesync:E17");
                } else {
                    Serial.println(F("E:CanRestored"));
                    queue_vesc_log("E:CanRestored");
                }
            }

            g_can_desync = desync;
            g_safety.set_can_desync(desync);
            g_safety.set_foc_fault(hw_faults);

            // Change-only ultra-compact terminal stream with displayed E-code (E2)
            static uint8_t s_last_hw_faults = 0;
            if (hw_faults != s_last_hw_faults) {
                s_last_hw_faults = hw_faults;
                if (hw_faults != 0) {
                    Serial.println(F("E:FocFault:E2"));
                    queue_vesc_log("E:FocFault:E2");
                } else if (g_safety.is_fault_recovery_locked()) {
                    Serial.println(F("E:FocCleared:AwaitNeutral:E2"));
                    queue_vesc_log("E:FocCleared:AwaitNeutral:E2");
                }
            }
        } else {
            g_can_desync = false;
            g_safety.set_can_desync(false);
            g_safety.set_foc_fault(0);
        }
    }
}

// =============================================================================
// Non-Blocking Serial Monitor CLI (Zero Dynamic Memory Allocation)
// =============================================================================

static void process_serial_cli() {
    static char s_cli_buf[32];
    static size_t s_cli_len = 0;

    while (Serial.available() > 0) {
        const char c = static_cast<char>(Serial.read());
        if (c == '\r' || c == '\n') {
            if (s_cli_len > 0) {
                s_cli_buf[s_cli_len] = '\0';
                // Trim leading whitespace
                char* cmd = s_cli_buf;
                while (*cmd == ' ' || *cmd == '\t') cmd++;
                // Trim trailing whitespace
                char* end = cmd + strlen(cmd) - 1;
                while (end >= cmd && (*end == ' ' || *end == '\t')) {
                    *end = '\0';
                    end--;
                }
                if (strcasecmp(cmd, "ver") == 0 || strcasecmp(cmd, "version") == 0) {
                    Serial.printf("Esp32:Version:%s:Build:%s:%s:ResetReason:%s:Uptime:%lums\n",
                                  FW_VERSION, BUILD_DATE, BUILD_TIME, s_reset_reason_str, static_cast<unsigned long>(millis()));
                    g_vesc.send_debug_logf("Esp32:Version:%s:Build:%s:%s", FW_VERSION, BUILD_DATE, BUILD_TIME);
                } else if (strcasecmp(cmd, "loss") == 0 || strcasecmp(cmd, "drop") == 0) {
                    s_verbose_packet_loss_log = !s_verbose_packet_loss_log;
                    if (s_verbose_packet_loss_log) {
                        Serial.println(F("E:LossDbg:Verbose:SingleDrop"));
                        queue_vesc_log("E:DropDbg:1");
                    } else {
                        Serial.println(F("E:LossDbg:Normal:StreakOnly"));
                        queue_vesc_log("E:DropDbg:0");
                    }
                } else if (strcasecmp(cmd, "loss 1") == 0 || strcasecmp(cmd, "drop 1") == 0 || strcasecmp(cmd, "loss on") == 0) {
                    s_verbose_packet_loss_log = true;
                    Serial.println(F("E:LossDbg:Verbose:SingleDrop"));
                    queue_vesc_log("E:DropDbg:1");
                } else if (strcasecmp(cmd, "loss 0") == 0 || strcasecmp(cmd, "drop 0") == 0 || strcasecmp(cmd, "loss off") == 0) {
                    s_verbose_packet_loss_log = false;
                    Serial.println(F("E:LossDbg:Normal:StreakOnly"));
                    queue_vesc_log("E:DropDbg:0");
                } else if (strcasecmp(cmd, "diag") == 0 || strcasecmp(cmd, "rate") == 0) {
                    Serial.printf("Diag:Vesc:Pkts:%lu:MaxDt:%lums:Drop1:%lu:Streak:%lu\n",
                                  static_cast<unsigned long>(s_vesc_packet_count),
                                  static_cast<unsigned long>(s_vesc_max_dt_ms),
                                  static_cast<unsigned long>(s_vesc_single_drop_count),
                                  static_cast<unsigned long>(s_vesc_streak_drop_count));
                    Serial.printf("Diag:Disp:Pkts:%lu:MinDt:%lums:MaxDt:%lums:LastDt:%lums:Rej:%lu:Drop1:%lu:Streak:%lu\n",
                                  static_cast<unsigned long>(s_disp_packet_count),
                                  static_cast<unsigned long>(s_disp_min_dt_ms == 0xFFFFFFFF ? 0 : s_disp_min_dt_ms),
                                  static_cast<unsigned long>(s_disp_max_dt_ms),
                                  static_cast<unsigned long>(s_disp_last_dt_ms),
                                  static_cast<unsigned long>(g_display.getRejectedPacketCount()),
                                  static_cast<unsigned long>(s_disp_single_drop_count),
                                  static_cast<unsigned long>(s_disp_streak_drop_count));
                    Serial.printf("Diag:Loop:Rate:%luHz:MaxExec:%luus\n",
                                  static_cast<unsigned long>(s_loop_rate_hz),
                                  static_cast<unsigned long>(s_loop_max_exec_us));
                } else if (strcasecmp(cmd, "diag reset") == 0 || strcasecmp(cmd, "rate reset") == 0) {
                    s_vesc_packet_count = 0;
                    s_vesc_max_dt_ms = 0;
                    s_vesc_single_drop_count = 0;
                    s_vesc_streak_drop_count = 0;
                    s_disp_packet_count = 0;
                    s_disp_min_dt_ms = 0xFFFFFFFF;
                    s_disp_max_dt_ms = 0;
                    s_disp_last_dt_ms = 0;
                    s_disp_single_drop_count = 0;
                    s_disp_streak_drop_count = 0;
                    g_display.resetDiagnosticCounters();
                    s_loop_iter_count = 0;
                    s_loop_rate_hz = 0;
                    s_loop_max_exec_us = 0;
                } else if (strcasecmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) {
                    Serial.println(F("CLI:Commands:ver,loss,diag,rate,diag reset"));
                }
                s_cli_len = 0;
            }
        } else if (s_cli_len < sizeof(s_cli_buf) - 1) {
            s_cli_buf[s_cli_len++] = c;
        } else {
            s_cli_len = 0; // Prevent buffer overrun
        }
    }
}

// =============================================================================
// Main Setup & Core 1 Execution Loop
// =============================================================================

void setup() {
    Serial.begin(HardwareConfig::BaudRate::SerialMonitor);

    // Boot reason check and compact mapping
    const esp_reset_reason_t reason = esp_reset_reason();
    s_reset_reason_str = map_reset_reason(reason);

    // Operational boot banner
    Serial.printf("E:Ver:%s:Build:%s:%s:ResetReason:%s\n",
                  FW_VERSION, BUILD_DATE, BUILD_TIME, s_reset_reason_str);
    if (reason == ESP_RST_BROWNOUT) {
        Serial.println(F("E:ResetReason:Brownout:E2"));
    }

    // Hardware Killswitch & Safety Supervisor initialization (Fail-closed GPIO 2 LOW)
    g_safety.begin();

    // Initialize Display Driver (9600 8N1) and Hardware Accessories
    g_display.begin(HardwareConfig::BaudRate::DisplayUart);

    KukirinG3Pro::PinConfig accPins;
    accPins.leftLed      = HardwareConfig::AccessoryPins::LeftLed;
    accPins.rightLed     = HardwareConfig::AccessoryPins::RightLed;
    accPins.brakeLed     = HardwareConfig::AccessoryPins::BrakeLed;
    accPins.headlightLed = HardwareConfig::AccessoryPins::HeadlightLed;
    accPins.horn         = HardwareConfig::AccessoryPins::Horn;
    accPins.auxLed       = HardwareConfig::AccessoryPins::AuxLed;
    accPins.argb1        = HardwareConfig::AccessoryPins::Argb1;
    accPins.argb2        = HardwareConfig::AccessoryPins::Argb2;
    accPins.argb3        = HardwareConfig::AccessoryPins::Argb3;
    accPins.argb4        = HardwareConfig::AccessoryPins::Argb4;
    accPins.buttonDs     = HardwareConfig::DisplayPins::ButtonDs;
    accPins.dispRx       = HardwareConfig::DisplayPins::Rx;
    accPins.dispTx       = HardwareConfig::DisplayPins::Tx;
    g_display.initGPIO(accPins);
    g_display.setErrorCode(KukirinG3Pro::ErrorCode::E03_MainController); // Assert E-003 until valid VESC telemetry

    // Event-driven button and config callbacks (batched switch dispatch via 10ms TDM slot)
    g_display.onBrake([](bool active) {
        static bool s_last_brake = false;
        if (s_last_brake != active) {
            s_last_brake = active;
            Serial.printf("E:BRK:%d\n", active ? 1 : 0);
        }
    });
    g_display.onLights([](bool on) {
        if (s_sw_lights != on) {
            s_sw_lights = on;
            s_switches_dirty = true;
            Serial.printf("E:LGT:%d\n", on ? 1 : 0);
        }
    });
    g_display.onTurnSignals([](bool left, bool right) {
        const char* s = (left && right) ? "H" : left ? "L" : right ? "R" : "0";
        if (strcmp(s_sw_trn, s) != 0) {
            s_sw_trn = s;
            s_switches_dirty = true;
            Serial.printf("E:TRN:%s\n", s);
        }
    });
    g_display.onHazardLights([](bool active) {
        if (active && strcmp(s_sw_trn, "H") != 0) {
            s_sw_trn = "H";
            s_switches_dirty = true;
            Serial.println(F("E:HAZ:1"));
        }
    });
    g_display.onHorn([](bool active) {
        if (s_sw_horn != active) {
            s_sw_horn = active;
            s_switches_dirty = true;
            Serial.printf("E:HRN:%d\n", active ? 1 : 0);
        }
    });
    g_display.onGear([](uint8_t gear) {
        if (s_sw_gear != gear) {
            s_sw_gear = gear;
            s_switches_dirty = true;
            Serial.printf("E:GER:%u\n", gear);
        }
    });
    g_display.onDual([](bool dual) {
        if (g_dual_motor_mode != dual) {
            g_dual_motor_mode = dual;
            Serial.printf("E:DUL:%d\n", dual ? 1 : 0);
            // Dual mode is continuously streamed at 50 Hz in TXPayloadControl - zero debug logs needed!
        }
    });
    g_display.onPMenu([](const KukirinG3Pro::DecodedSettings& settings) {
        g_pmenu_regen_level = settings.regenLevel;
        g_safety.set_accel_level_pa(settings.accelLevel);
        Serial.printf("E:CFG %uV %.1f\" A%u B%u\n",
                      settings.batteryVolts, settings.wheelInches, settings.accelLevel, settings.regenLevel);
        g_vesc.send_debug_logf("E:CFG %uV %.1f\" A%u B%u",
                               settings.batteryVolts, settings.wheelInches, settings.accelLevel, settings.regenLevel);
    });

    // Initialize VESC UART Bridge (230,400 bps, GPIO 34 RX, GPIO 32 TX)
    g_vesc.begin();
    g_vesc.set_heartbeat_interval_ms(0); // 50 Hz control streaming in loop() handles keepalive
    g_vesc.set_receive_callback(onVESCMessage);

    // Tunnel version string to VESC
    g_vesc.send_debug_logf("E:Ver:%s", FW_VERSION);

    Serial.println(F("E:Boot:Complete:AwaitB2S"));
}

void loop() {
    const uint32_t loop_start_us = micros();
    const uint32_t now = millis();
    s_loop_iter_count++;
    if (now - s_loop_rate_timer_ms >= 1000) {
        s_loop_rate_hz = s_loop_iter_count;
        s_loop_iter_count = 0;
        s_loop_rate_timer_ms = now;
    }

    // Non-blocking Serial monitor CLI (115200 bps)
    process_serial_cli();

    // Service VESC UART bridge parser and evaluate comms health
    g_vesc.update();
    const bool vesc_online = g_vesc.is_connected(VESC_COMMS_TIMEOUT_MS);

    // Monitor VESC UART telemetry link health & packet loss
    const uint32_t last_valid_vesc_ms = g_vesc.get_last_valid_rx_ms();
    const uint32_t vesc_age_ms = (last_valid_vesc_ms > 0) ? (now - last_valid_vesc_ms) : 0;
    static uint8_t s_vesc_drop_stage = 0;

    if (s_vesc_link_established && vesc_online) {
        if (vesc_age_ms < VESCBridge::PowertrainConfig::VESC_DROP_SINGLE_MS) {
            s_vesc_drop_stage = 0;
        } else {
            // Stage 1: Single dropped 20ms frame (silence >= 32ms, logged strictly in verbose mode)
            if (s_verbose_packet_loss_log && s_vesc_drop_stage < 1 && vesc_age_ms >= VESCBridge::PowertrainConfig::VESC_DROP_SINGLE_MS) {
                s_vesc_drop_stage = 1;
                s_vesc_single_drop_count++;
                Serial.println(F("E:VescDrop:Single:E3"));
                queue_vesc_log("E:VescDrop:Single:E3");
            }
            // Stage 2: Consecutive cadence gaps (>= 2 missed 20ms frames, silence >= 65ms, before 500ms timeout)
            if (s_vesc_drop_stage < 2 && vesc_age_ms >= VESCBridge::PowertrainConfig::VESC_DROP_STREAK_MS) {
                s_vesc_drop_stage = 2;
                s_vesc_streak_drop_count++;
                Serial.println(F("E:VescDrop:Streak:E3"));
                queue_vesc_log("E:VescDrop:Streak:E3");
            }
        }
    } else if (!vesc_online) {
        s_vesc_drop_stage = 0;
    }

    // Track VESC link drops (>500ms) and restorations
    static bool s_last_vesc_online = true;
    if (!vesc_online) {
        g_vehicle_speed_mph = 0.0f;
        g_display.setSpeedMph(0.0f);
        if (s_last_vesc_online) {
            Serial.println(F("E:VescLost:E3"));
        }
    } else if (!s_last_vesc_online) {
        Serial.println(F("E:VescRestored:AwaitNeutral:E2"));
        // Tunnel version string to VESC upon connection/restoration
        g_vesc.send_debug_logf("E:Ver:%s", FW_VERSION);
        queue_vesc_log("E:VescRestored:AwaitNeutral:E2");
    }
    s_last_vesc_online = vesc_online;

    // Evaluate fault & update display error code BEFORE g_display.update()
    // so any active fault code is immediately transmitted in the response frame
    static uint16_t s_last_throttle_req = 0;
    const VESCBridge::SafetyFault pre_fault = g_safety.evaluate_fault(vesc_online, g_can_desync, s_last_throttle_req, now);
    const KukirinG3Pro::ErrorCode pre_target_fault = toKukirinErrorCode(pre_fault);
    if (g_display.getErrorCode() != pre_target_fault) {
        g_display.setErrorCode(pre_target_fault);
    }

    // Service Display UART protocol engine (non-blocking every loop iteration)
    const bool display_packet_received = g_display.update();

    // Monitor Display UART EMI link health & packet loss
    const uint32_t last_valid_disp_ms = g_display.getLastValidPacketMs();
    const uint32_t disp_age_ms = (last_valid_disp_ms > 0) ? (now - last_valid_disp_ms) : 0;
    const bool disp_stalled = g_display.isCommsStalled();
    static bool s_last_disp_stalled = false;
    static uint32_t s_last_stall_log_ms = 0;
    static uint32_t s_last_total_rej = 0;
    static uint32_t s_consecutive_rej = 0;
    static uint8_t  s_disp_drop_stage = 0;

    // Track frame rejections (CRC / plausibility failures)
    const uint32_t total_rej = g_display.getRejectedPacketCount();
    if (total_rej > s_last_total_rej) {
        s_consecutive_rej += (total_rej - s_last_total_rej);
        s_last_total_rej = total_rej;
        if (s_verbose_packet_loss_log && s_consecutive_rej == 1) {
            Serial.println(F("E:DispRej:Single:E3"));
            queue_vesc_log("E:DispRej:Single:E3");
        } else if (s_consecutive_rej >= 2) {
            Serial.println(F("E:DispRej:Streak:E3"));
            queue_vesc_log("E:DispRej:Streak:E3");
        }
    }

    if (display_packet_received) {
        s_consecutive_rej = 0;
        s_disp_drop_stage = 0;
        s_disp_packet_count++;
        if (s_disp_last_pkt_time_ms > 0) {
            const uint32_t dt = now - s_disp_last_pkt_time_ms;
            s_disp_last_dt_ms = dt;
            if (dt > s_disp_max_dt_ms) {
                s_disp_max_dt_ms = dt;
            }
            if (dt < s_disp_min_dt_ms) {
                s_disp_min_dt_ms = dt;
            }
        }
        s_disp_last_pkt_time_ms = now;
    }

    if (last_valid_disp_ms > 0) {
        // Track cadence gaps (single dropped frame >= 180ms in verbose mode, streak >= 320ms, before 500ms stall)
        if (!disp_stalled) {
            if (s_verbose_packet_loss_log && s_disp_drop_stage < 1 && disp_age_ms >= VESCBridge::PowertrainConfig::DISPLAY_DROP_SINGLE_MS) {
                s_disp_drop_stage = 1;
                s_disp_single_drop_count++;
                Serial.println(F("E:DispDrop:Single:E3"));
                queue_vesc_log("E:DispDrop:Single:E3");
            }
            if (s_disp_drop_stage < 2 && disp_age_ms >= VESCBridge::PowertrainConfig::DISPLAY_DROP_STREAK_MS) {
                s_disp_drop_stage = 2;
                s_disp_streak_drop_count++;
                Serial.println(F("E:DispDrop:Streak:E3"));
                queue_vesc_log("E:DispDrop:Streak:E3");
            }
        }

        // Track dead-man stall (> 500ms)
        if (disp_stalled) {
            if (!s_last_disp_stalled || (now - s_last_stall_log_ms >= 1000)) {
                s_last_stall_log_ms = now;
                Serial.println(F("E:DispStall:E3"));
                queue_vesc_log("E:DispStall:E3");
            }
        } else if (s_last_disp_stalled) {
            Serial.println(F("E:DispRestored"));
            queue_vesc_log("E:DispRestored");
        }
        s_last_disp_stalled = disp_stalled;
    }

    // Time-Division Multiplexed (TDM) Control & Secondary Dispatcher
    static uint32_t s_last_control_tick_ms = 0;
    static uint32_t s_last_secondary_tick_ms = 0;

    // Control Frame Dispatch: Immediate on Display Packet Arrival OR 20ms Keepalive Fallback
    const bool keepalive_timeout = (now - s_last_control_tick_ms >= PowertrainConfig::VESC_TICK_INTERVAL_MS);

    if (display_packet_received || keepalive_timeout) {
        const float dt_s = (s_last_control_tick_ms > 0 && (now - s_last_control_tick_ms) < 1000)
            ? static_cast<float>(now - s_last_control_tick_ms) / 1000.0f
            : PowertrainConfig::VESC_TICK_INTERVAL_SEC;
        const float clamped_dt = (dt_s < 0.010f) ? 0.010f : ((dt_s > 0.100f) ? 0.100f : dt_s);

        if (display_packet_received) {
            s_last_control_tick_ms = now;
        } else {
            // Maintain phase lock on keepalive fallback, guarding against multi-tick backlog
            s_last_control_tick_ms += PowertrainConfig::VESC_TICK_INTERVAL_MS;
            if (now - s_last_control_tick_ms > 100) {
                s_last_control_tick_ms = now;
            }
        }

        // Sample latest display inputs atomically (getNormalizedThrottle(), getGearLevel(), isBrakeActive())
        const float norm_throttle = g_display.getNormalizedThrottle();
        const uint8_t gear = g_display.getGearLevel();
        const bool brake_active = g_display.isBrakeActive();

        // Compute throttle_req using rate-limited mode scaling (speed governing delegated to VESC per-motor)
        const uint16_t throttle_req = g_safety.compute_mode_scaled_throttle(
            norm_throttle,
            gear,
            0.0f,
            clamped_dt
        );
        s_last_throttle_req = throttle_req;

        // Update Automotive Brake-to-Start (Ready-to-Drive) state machine
        const bool was_armed = g_safety.is_armed();
        if (vesc_online && g_safety.update_b2s(brake_active, throttle_req, now, g_vehicle_speed_mph)) {
            if (!was_armed && g_safety.is_armed()) {
                Serial.println(F("E:Armed"));
                queue_vesc_log("E:Armed");
            }
        }

        // Update display recovery state (remediates display comms stall without dropping GPIO 2 killswitch)
        const bool display_stalled = g_display.isCommsStalled();
        g_safety.update_display_recovery(display_stalled, throttle_req);

        // Evaluate fault & update display error code
        const VESCBridge::SafetyFault fault = g_safety.evaluate_fault(vesc_online, g_can_desync, throttle_req, now);
        const KukirinG3Pro::ErrorCode target_fault = toKukirinErrorCode(fault);
        if (g_display.getErrorCode() != target_fault) {
            g_display.setErrorCode(target_fault);
        }

        // Compute user-governed regenerative braking torque (zero slew lag, 1/5 steps)
        const uint16_t brake_request = VESCBridge::VESCSafetySupervisor::compute_pb_regen(brake_active, g_pmenu_regen_level);

        // Query mode speed limit for VESC local speed governing (0 = unlimited)
        const uint16_t speed_limit_cms = VESCBridge::PowertrainConfig::get_mode_speed_limit_cms(gear);

        const bool was_fault_locked = g_safety.is_fault_recovery_locked();

        // Compute safe control payload
        g_vesc_control_payload = g_safety.compute_safe_control(
            throttle_req,
            brake_request,
            brake_active,
            g_dual_motor_mode,
            g_can_desync,
            vesc_online,
            clamped_dt,
            speed_limit_cms
        );

        // Dispatch primary control frame to VESC immediately (< 50 us latency)
        if (vesc_online) {
            g_vesc.send_control(g_vesc_control_payload);
        }

        if (was_fault_locked && !g_safety.is_fault_recovery_locked()) {
            Serial.println(F("E:FocRestored"));
            queue_vesc_log("E:FocRestored");
        }
    }

    // Slot B: Interleaved 10ms Mid-Tick Secondary Frame (Strict 8-14ms mid-tick window)
    const uint32_t dt_control = now - s_last_control_tick_ms;
    if ((dt_control >= 8) && (dt_control <= 14) && (now - s_last_secondary_tick_ms >= 20)) {
        if (vesc_online) {
            if (s_pending_vesc_log != nullptr) {
                s_last_secondary_tick_ms = now;
                g_vesc.send_debug_log(s_pending_vesc_log);
                s_pending_vesc_log = nullptr;
            } else if (s_switches_dirty) {
                s_last_secondary_tick_ms = now;
                s_switches_dirty = false;
                g_vesc.send_debug_logf("E:SW:G%u:L%d:T%s:H%d", s_sw_gear, s_sw_lights ? 1 : 0, s_sw_trn, s_sw_horn ? 1 : 0);
            }
        }
    }

    // Core 1 RTOS yield to prevent CPU starvation when idle
    if (!display_packet_received) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    const uint32_t loop_exec_us = micros() - loop_start_us;
    if (loop_exec_us > s_loop_max_exec_us) {
        s_loop_max_exec_us = loop_exec_us;
    }
}