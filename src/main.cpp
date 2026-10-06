/**
 * @file main.cpp
 * @brief ESP32 firmware between the Kukirin G3 Pro display and the rear VESC.
 *
 * Each pass of loop():
 * - reads the display's command frames and answers them (KukirinG3ProDisplay),
 * - runs brake-to-start and the lockouts (VESCSafetySupervisor), which also drive the killswitch line,
 * - sends a control frame to the rear VESC when a display frame arrives, and otherwise every 20 ms,
 * - reads VESC telemetry: speed for the display, front VESC status, VESC fault code,
 * - shows faults on the display and logs link problems to the USB console and the VESC Tool terminal.
 * Motor control itself (current commands, regen, speed limit, both motors) runs in vesc/main.lbm.
 *
 * The loop must not block: no delay(), no heap allocation.
 */

#include <Arduino.h>
#include <esp_system.h>
#include "build_version.h"
#include "KukirinDisplay.h"
#include "VESCBridge.h"

namespace PowertrainConfig = VESCBridge::PowertrainConfig;

// Pins and baud rates. GPIO 34-39 are input-only with no pull resistors. GPIO 16 and 17 are used by the
// WROVER module's PSRAM and must not be assigned.

namespace HardwareConfig {
    struct CanBus {
        static constexpr uint8_t CAN_ID_REAR_VESC  = VESCConfig::CAN_ID_REAR_VESC;  /**< Rear VESC (UART to this board). */
        static constexpr uint8_t CAN_ID_FRONT_VESC = VESCConfig::CAN_ID_FRONT_VESC; /**< Front VESC (CAN to the rear VESC). */
    };

    struct BaudRate {
        static constexpr uint32_t SerialMonitor = 115200; /**< USB console. */
        static constexpr uint32_t VescUart      = 230400; /**< Rear VESC; must match VESC-UART-BAUD in vesc/main.lbm. */
        static constexpr uint32_t DisplayUart   = 9600;   /**< Display. */
    };

    struct VescPins {
        static constexpr int8_t Rx         = 34; /**< From the VESC's TX. */
        static constexpr int8_t Tx         = 32; /**< To the VESC's RX. */
        static constexpr int8_t Killswitch = 2;  /**< To the rear VESC's ADC2 input; low = motors off. Low at boot until brake-to-start. */
    };

    struct DisplayPins {
        static constexpr int8_t Rx       = 35; /**< From the display's TX. The display drives 5 V: use the 1.0k/1.8k divider, never a direct wire. */
        static constexpr int8_t Tx       = 33; /**< To the display's RX. */
        static constexpr int8_t ButtonDs = 39; /**< Dual/single button. */
    };

    struct AccessoryPins {
        static constexpr int8_t LeftLed      = 18; /**< Left turn signal MOSFET. */
        static constexpr int8_t RightLed     = 19; /**< Right turn signal MOSFET. */
        static constexpr int8_t BrakeLed     = 21; /**< Brake light MOSFET. */
        static constexpr int8_t HeadlightLed = 22; /**< Head and tail light MOSFET. */
        static constexpr int8_t Horn         = 23; /**< Horn relay. */
        static constexpr int8_t AuxLed       = 4;  /**< Spare, held low. */
        static constexpr int8_t Argb1        = 25; /**< Spare, held low. */
        static constexpr int8_t Argb2        = 26; /**< Spare, held low. */
        static constexpr int8_t Argb3        = 27; /**< Spare, held low. */
        static constexpr int8_t Argb4        = 14; /**< Spare, held low. */
    };
}

static constexpr uint32_t VESC_COMMS_TIMEOUT_MS  = PowertrainConfig::VESC_COMMS_TIMEOUT_MS;
static constexpr uint8_t  DEFAULT_PB_REGEN_LEVEL = 3;

// Telemetry fault_flags: bit 0 = front VESC lost, bits 1-7 = rear VESC fault code
static constexpr uint8_t FAULT_FLAG_CAN_DESYNC   = 0x01;
static constexpr uint8_t FAULT_FLAG_HW_MASK      = ~FAULT_FLAG_CAN_DESYNC;

/**
 * @brief Display code for each fault: E-003 lost VESC link, E-017 lost front VESC, E-002 everything that
 *        waits for the rider (not armed, or a lockout waiting for released throttle).
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

// Reset reason for the boot log

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


static VESCUARTBridge                  g_vesc(&Serial1, HardwareConfig::BaudRate::VescUart, HardwareConfig::VescPins::Rx, HardwareConfig::VescPins::Tx);
static KukirinG3ProDisplay             g_display(&Serial2, HardwareConfig::DisplayPins::Rx, HardwareConfig::DisplayPins::Tx, HardwareConfig::DisplayPins::ButtonDs);
static VESCBridge::VESCSafetySupervisor g_safety(HardwareConfig::VescPins::Killswitch);


static const char*          s_reset_reason_str = "Other";
static bool                 s_vesc_link_established = false;
static bool                 s_verbose_packet_loss_log = false;
static volatile float       g_vehicle_speed_mph = 0.0f;
static volatile bool        g_dual_motor_mode = true;
static volatile bool        g_can_desync = false;
static volatile uint8_t     g_pmenu_regen_level = DEFAULT_PB_REGEN_LEVEL;

// Switch states, sent to the VESC Tool terminal as one log line when one changes
static uint8_t              s_sw_gear = 1;
static bool                 s_sw_lights = false;
static const char*          s_sw_trn = "0";
static bool                 s_sw_horn = false;
static bool                 s_switches_dirty = true; // send once at boot

static TXPayloadControl     g_vesc_control_payload;

// One pending log line for the VESC, sent between control frames (string literals only)
static const char*          s_pending_vesc_log = nullptr;

static inline void queue_vesc_log(const char* msg) {
    s_pending_vesc_log = msg;
}

// Link statistics for the "diag" console command
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

static volatile uint32_t    s_loop_iter_count = 0;
static volatile uint32_t    s_loop_rate_hz = 0;
static volatile uint32_t    s_loop_max_exec_us = 0;
static uint32_t             s_loop_rate_timer_ms = 0;

/**
 * @brief Handles a telemetry frame from the rear VESC (2 bytes speed, optional 3rd byte fault flags).
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

        // Speed of the faster wheel in cm/s, little-endian
        const int16_t speed_raw = static_cast<int16_t>(payload[0] | (static_cast<uint16_t>(payload[1]) << 8));
        g_vehicle_speed_mph = fabsf(PowertrainConfig::cms_to_mph(speed_raw));

        g_display.setSpeedMph(g_vehicle_speed_mph);

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

            // Log fault changes only
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

// USB console commands: ver, loss [0|1], diag, diag reset, help

static void process_serial_cli() {
    static char s_cli_buf[32];
    static size_t s_cli_len = 0;

    while (Serial.available() > 0) {
        const char c = static_cast<char>(Serial.read());
        if (c == '\r' || c == '\n') {
            if (s_cli_len > 0) {
                s_cli_buf[s_cli_len] = '\0';
                char* cmd = s_cli_buf;
                while (*cmd == ' ' || *cmd == '\t') cmd++;
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
            s_cli_len = 0; // line too long: discard it
        }
    }
}


void setup() {
    Serial.begin(HardwareConfig::BaudRate::SerialMonitor);

    const esp_reset_reason_t reason = esp_reset_reason();
    s_reset_reason_str = map_reset_reason(reason);

    Serial.printf("E:Ver:%s:Build:%s:%s:ResetReason:%s\n",
                  FW_VERSION, BUILD_DATE, BUILD_TIME, s_reset_reason_str);
    if (reason == ESP_RST_BROWNOUT) {
        Serial.println(F("E:ResetReason:Brownout:E2"));
    }

    // Killswitch line low before anything else
    g_safety.begin();

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
    g_display.setErrorCode(KukirinG3Pro::ErrorCode::E03_MainController); // until the VESC answers

    // Switch changes are logged; P-menu changes set regen level and throttle rise rate
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
            // Sent to the VESC in every control frame
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

    g_vesc.begin();
    g_vesc.set_heartbeat_interval_ms(0); // control frames every 20 ms keep the link alive
    g_vesc.set_receive_callback(onVESCMessage);

    // The script stores this for its "ver" command
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

    process_serial_cli();

    g_vesc.update();
    const bool vesc_online = g_vesc.is_connected(VESC_COMMS_TIMEOUT_MS);

    // Log telemetry gaps (frames are expected every 20 ms)
    const uint32_t last_valid_vesc_ms = g_vesc.get_last_valid_rx_ms();
    const uint32_t vesc_age_ms = (last_valid_vesc_ms > 0) ? (now - last_valid_vesc_ms) : 0;
    static uint8_t s_vesc_drop_stage = 0;

    if (s_vesc_link_established && vesc_online) {
        if (vesc_age_ms < VESCBridge::PowertrainConfig::VESC_DROP_SINGLE_MS) {
            s_vesc_drop_stage = 0;
        } else {
            // One frame missing (verbose logging only)
            if (s_verbose_packet_loss_log && s_vesc_drop_stage < 1 && vesc_age_ms >= VESCBridge::PowertrainConfig::VESC_DROP_SINGLE_MS) {
                s_vesc_drop_stage = 1;
                s_vesc_single_drop_count++;
                Serial.println(F("E:VescDrop:Single:E3"));
                queue_vesc_log("E:VescDrop:Single:E3");
            }
            // Two or more missing
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

    // Link lost after VESC_COMMS_TIMEOUT_MS: show 0 speed
    static bool s_last_vesc_online = true;
    if (!vesc_online) {
        g_vehicle_speed_mph = 0.0f;
        g_display.setSpeedMph(0.0f);
        if (s_last_vesc_online) {
            Serial.println(F("E:VescLost:E3"));
        }
    } else if (!s_last_vesc_online) {
        Serial.println(F("E:VescRestored:AwaitNeutral:E2"));
        g_vesc.send_debug_logf("E:Ver:%s", FW_VERSION);
        queue_vesc_log("E:VescRestored:AwaitNeutral:E2");
    }
    s_last_vesc_online = vesc_online;

    // Set the error code before g_display.update() so the reply sent in this pass already carries it
    static uint16_t s_last_throttle_req = 0;
    const VESCBridge::SafetyFault pre_fault = g_safety.evaluate_fault(vesc_online, g_can_desync, s_last_throttle_req, now);
    const KukirinG3Pro::ErrorCode pre_target_fault = toKukirinErrorCode(pre_fault);
    if (g_display.getErrorCode() != pre_target_fault) {
        g_display.setErrorCode(pre_target_fault);
    }

    const bool display_packet_received = g_display.update();

    // Log display link problems
    const uint32_t last_valid_disp_ms = g_display.getLastValidPacketMs();
    const uint32_t disp_age_ms = (last_valid_disp_ms > 0) ? (now - last_valid_disp_ms) : 0;
    const bool disp_stalled = g_display.isCommsStalled();
    static bool s_last_disp_stalled = false;
    static uint32_t s_last_stall_log_ms = 0;
    static uint32_t s_last_total_rej = 0;
    static uint32_t s_consecutive_rej = 0;
    static uint8_t  s_disp_drop_stage = 0;

    // Rejected frames (bad checksum or impossible values)
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
        // Gaps: one frame missing (verbose only) or two or more
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

        // Stalled: no valid frame for the display timeout (500 ms)
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

    // Control frame to the VESC
    static uint32_t s_last_control_tick_ms = 0;
    static uint32_t s_last_secondary_tick_ms = 0;

    // Sent right after a display frame (new inputs), and otherwise every 20 ms so the VESC dead-man stays fed
    const bool keepalive_timeout = (now - s_last_control_tick_ms >= PowertrainConfig::VESC_TICK_INTERVAL_MS);

    if (display_packet_received || keepalive_timeout) {
        const float dt_s = (s_last_control_tick_ms > 0 && (now - s_last_control_tick_ms) < 1000)
            ? static_cast<float>(now - s_last_control_tick_ms) / 1000.0f
            : PowertrainConfig::VESC_TICK_INTERVAL_SEC;
        const float clamped_dt = (dt_s < 0.010f) ? 0.010f : ((dt_s > 0.100f) ? 0.100f : dt_s);

        if (display_packet_received) {
            s_last_control_tick_ms = now;
        } else {
            // Keep the 20 ms grid; after a long stall restart it instead of sending a burst
            s_last_control_tick_ms += PowertrainConfig::VESC_TICK_INTERVAL_MS;
            if (now - s_last_control_tick_ms > 100) {
                s_last_control_tick_ms = now;
            }
        }

        const float norm_throttle = g_display.getNormalizedThrottle();
        const uint8_t gear = g_display.getGearLevel();
        const bool brake_active = g_display.isBrakeActive();

        // Mode scaling. Speed 0 is passed on purpose: the speed limit is applied per wheel by the VESC script
        // (sent below as speed_limit_cms), so it keeps working if this link fails.
        const uint16_t throttle_req = g_safety.compute_mode_scaled_throttle(
            norm_throttle,
            gear,
            0.0f,
            clamped_dt
        );
        s_last_throttle_req = throttle_req;

        // Brake-to-start (also drives the killswitch line)
        const bool was_armed = g_safety.is_armed();
        if (vesc_online && g_safety.update_b2s(brake_active, throttle_req, now, g_vehicle_speed_mph)) {
            if (!was_armed && g_safety.is_armed()) {
                Serial.println(F("E:Armed"));
                queue_vesc_log("E:Armed");
            }
        }

        // Display stall lock: zero throttle while display frames are missing; killswitch stays high
        const bool display_stalled = g_display.isCommsStalled();
        g_safety.update_display_recovery(display_stalled, throttle_req);

        const VESCBridge::SafetyFault fault = g_safety.evaluate_fault(vesc_online, g_can_desync, throttle_req, now);
        const KukirinG3Pro::ErrorCode target_fault = toKukirinErrorCode(fault);
        if (g_display.getErrorCode() != target_fault) {
            g_display.setErrorCode(target_fault);
        }

        // Regen: PB level x 2000 counts while the brake is pulled
        const uint16_t brake_request = VESCBridge::VESCSafetySupervisor::compute_pb_regen(brake_active, g_pmenu_regen_level);

        const uint16_t speed_limit_cms = VESCBridge::PowertrainConfig::get_mode_speed_limit_cms(gear);

        const bool was_fault_locked = g_safety.is_fault_recovery_locked();

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

        if (vesc_online) {
            g_vesc.send_control(g_vesc_control_payload);
        }

        if (was_fault_locked && !g_safety.is_fault_recovery_locked()) {
            Serial.println(F("E:FocRestored"));
            queue_vesc_log("E:FocRestored");
        }
    }

    // Log lines go 8-14 ms after a control frame, halfway between two control frames, at most one per 20 ms
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

    // Let other tasks run when there was nothing to do
    if (!display_packet_received) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    const uint32_t loop_exec_us = micros() - loop_start_us;
    if (loop_exec_us > s_loop_max_exec_us) {
        s_loop_max_exec_us = loop_exec_us;
    }
}