#pragma once

#include <Arduino.h>
#include <math.h>
#include "VESCUARTBridge.h"

// Unit conversion macros
#ifndef MS_TO_SEC
#define MS_TO_SEC(ms)        ((ms) * 0.001f)
#endif
#ifndef SEC_TO_MS
#define SEC_TO_MS(sec)       ((sec) * 1000.0f)
#endif
#ifndef MPH_TO_MPS
#define MPH_TO_MPS(mph)      ((mph) * 0.44704f)
#endif
#ifndef MPS_TO_MPH
#define MPS_TO_MPH(mps)      ((mps) * 2.23693629f)
#endif
#ifndef MPS_TO_CMS
#define MPS_TO_CMS(mps)      ((mps) * 100.0f)
#endif
#ifndef CMS_TO_MPS
#define CMS_TO_MPS(cms)      ((cms) * 0.01f)
#endif
#ifndef CMS_TO_MPH
#define CMS_TO_MPH(cms)      ((cms) * 0.0223693629f)
#endif
#ifndef COUNTS_TO_RATIO
#define COUNTS_TO_RATIO(cnt) ((static_cast<float>(cnt)) * 0.0001f)
#endif
#ifndef RATIO_TO_COUNTS
#define RATIO_TO_COUNTS(r)   ((r) * 10000.0f)
#endif

namespace VESCBridge {

/**
 * @enum ArmingState
 * @brief Whether the rider has armed the drivetrain with brake-to-start.
 */
enum class ArmingState : uint8_t {
    BootUnarmed  = 0, /**< @brief After boot or disarm(): killswitch output low, no drive. */
    ReadyToDrive = 1  /**< @brief Armed by brake-to-start: killswitch output high. */
};

/**
 * @enum SafetyFault
 * @brief Condition shown to the rider as a display error code (mapping in src/main.cpp).
 */
enum class SafetyFault : uint8_t {
    None                   = 0, /**< @brief No fault. */
    KillswitchDisarmed     = 1, /**< @brief Not armed, or the brake was released with the throttle open. */
    VescOffline            = 2, /**< @brief No telemetry from the rear VESC for VESC_COMMS_TIMEOUT_MS. */
    CanDesync              = 3, /**< @brief Front VESC lost (the script saw no CAN status for 500 ms). */
    FocFaultLockout        = 4, /**< @brief Rear VESC fault active, or cleared and waiting for neutral throttle. */
    CommsRecoveryLockout   = 5, /**< @brief VESC link came back; waiting for neutral throttle. */
    DisplayStalled         = 6  /**< @brief Display frames stopped, or came back and waiting for neutral throttle. */
};

/**
 * @namespace PowertrainConfig
 * @brief Firmware constants for the drivetrain. Values that vesc/main.lbm also defines are checked by
 *        tools/check_shared_constants.py; edit both files together.
 */
namespace PowertrainConfig {
    // Wheel and motor
    static constexpr float WHEEL_DIAMETER_INCHES        = 10.0f;                                      /**< Nominal wheel diameter. */
    static constexpr uint8_t MOTOR_POLE_PAIRS           = 15;                                         /**< Hub motor pole pairs (30 magnets). */
    static constexpr float METERS_PER_INCH              = 0.0254f;                                    /**< m per inch. */
    static constexpr float PI_CONST                     = 3.14159265358979323846f;                    /**< Pi. */
    static constexpr float WHEEL_DIAMETER_METERS        = WHEEL_DIAMETER_INCHES * METERS_PER_INCH;     /**< 0.254 m. */
    static constexpr float WHEEL_CIRCUMFERENCE_METERS   = WHEEL_DIAMETER_METERS * PI_CONST;           /**< 0.798 m. */

    // Speeds
    static constexpr float MPS_PER_MPH                  = 0.44704f;                                   /**< m/s per mph. */
    static constexpr float SPEED_STATIONARY_MPH         = 0.6f;                                       /**< Below this speed the vehicle counts as stationary (brake-to-start, arming). */
    static constexpr float SPEED_STATIONARY_MPS         = SPEED_STATIONARY_MPH * MPS_PER_MPH;         /**< Same, in m/s. */
    static constexpr float SPEED_STATIONARY_CMS         = SPEED_STATIONARY_MPS * 100.0f;              /**< Same, in cm/s. */
    static constexpr uint32_t SPEED_STATIONARY_ERPM     = 300;                                        /**< Same check on motor ERPM (about 0.6 mph with 15 pole pairs and a 10 in wheel). */
    static constexpr float MODE_1_SPEED_LIMIT_MPH       = 15.0f;                                      /**< Mode 1 speed limit. */
    static constexpr float MODE_2_SPEED_LIMIT_MPH       = 28.0f;                                      /**< Mode 2 speed limit. Mode 3 has none. */
    static constexpr float SPEED_LIMIT_TAPER_WINDOW_MPH = 3.0f;                                       /**< Drive current fades to zero over this window below the mode limit (applied by the VESC script). */
    static constexpr uint16_t MODE_1_SPEED_LIMIT_CMS    = 671;                                        /**< Mode 1 limit sent to the VESC, cm/s. */
    static constexpr uint16_t MODE_2_SPEED_LIMIT_CMS    = 1252;                                       /**< Mode 2 limit sent to the VESC, cm/s. */
    static constexpr uint16_t MODE_3_SPEED_LIMIT_CMS    = 0;                                          /**< 0 = no limit. */
    static constexpr uint16_t SPEED_LIMIT_TAPER_CMS     = 134;                                        /**< Taper window in cm/s. */

    static constexpr uint16_t get_mode_speed_limit_cms(uint8_t gear) {
        if (gear == 1) return MODE_1_SPEED_LIMIT_CMS;
        if (gear == 2) return MODE_2_SPEED_LIMIT_CMS;
        return MODE_3_SPEED_LIMIT_CMS;
    }

    // Throttle scale. The script sends current as a fraction of the motor current limit set in VESC Tool
    // (set-current-rel), so 10000 counts means that limit. MOTOR_MAX_CURRENT_AMPS must match it.
    static constexpr float MOTOR_MAX_CURRENT_AMPS       = 120.0f;                                     /**< Motor current limit configured in VESC Tool, per motor. */
    static constexpr uint16_t THROTTLE_SCALE_MAX_COUNTS = 10000;                                      /**< Full-scale throttle and brake command. */
    static constexpr float CURRENT_COUNTS_PER_AMP       = static_cast<float>(THROTTLE_SCALE_MAX_COUNTS) / MOTOR_MAX_CURRENT_AMPS; /**< 83.3 counts per A. */
    static constexpr float MODE_1_SCALE_RATIO           = 0.50f;                                      /**< Mode 1: 50 % of full current. */
    static constexpr float MODE_2_SCALE_RATIO           = 0.75f;                                      /**< Mode 2: 75 %. */
    static constexpr float MODE_3_SCALE_RATIO           = 1.00f;                                      /**< Mode 3: 100 %. */
    static constexpr uint16_t MODE_1_MAX_COUNTS         = 5000;                                       /**< Mode 1 ceiling: 60 A per motor. */
    static constexpr uint16_t MODE_2_MAX_COUNTS         = 7500;                                       /**< Mode 2 ceiling: 90 A per motor. */
    static constexpr uint16_t MODE_3_MAX_COUNTS         = 10000;                                      /**< Mode 3 ceiling: 120 A per motor. */
    static constexpr uint16_t THROTTLE_NEUTRAL_COUNTS   = 100;                                        /**< Throttle below 1 % counts as released: needed to arm and to clear lockouts; the script freewheels below it. */
    static constexpr uint16_t THROTTLE_DEADBAND_COUNTS  = 50;                                         /**< The command sent to the VESC changes only when it moves by at least this much, so small throttle jitter while holding speed does not reach the motor. */
    static constexpr uint16_t PLAUSIBILITY_CLAMP_MAX     = 1050;                                       /**< Raw throttle above this (full scale is 1000) is treated as a wiring fault and gives zero throttle. */

    // Regen fades in with speed (applied per wheel by the VESC script)
    static constexpr float SPEED_REGEN_MIN_MPH          = 5.0f;                                       /**< No regen below this speed; set from ride testing. */
    static constexpr float SPEED_REGEN_MAX_MPH          = 20.0f;                                      /**< Full regen (the selected PB level) above this speed. */
    static constexpr float SPEED_REGEN_MIN_MPS          = SPEED_REGEN_MIN_MPH * MPS_PER_MPH;          /**< m/s. */
    static constexpr float SPEED_REGEN_MAX_MPS          = SPEED_REGEN_MAX_MPH * MPS_PER_MPH;          /**< m/s. */
    static constexpr float REGEN_LERP_RANGE_MPS         = SPEED_REGEN_MAX_MPS - SPEED_REGEN_MIN_MPS;  /**< m/s. */
    static constexpr float INV_REGEN_RANGE_MPS          = 1.0f / REGEN_LERP_RANGE_MPS;                /**< 1 / range, in s/m. */

    // Timing. All link-loss timeouts are 500 ms, matching the timeout set in VESC Tool on both VESCs.
    static constexpr uint32_t CONTROL_LOOP_RATE_HZ      = 50;                                         /**< Control frames per second to the VESC. */
    static constexpr uint32_t VESC_TICK_RATE_HZ         = 50;                                         /**< Same as CONTROL_LOOP_RATE_HZ. */
    static constexpr uint32_t VESC_TICK_INTERVAL_MS     = 20;                                         /**< Control frame period. */
    static constexpr float VESC_TICK_INTERVAL_SEC       = 0.020f;                                     /**< Same, in s. */
    static constexpr uint32_t DEADMAN_MISSED_TICKS      = 25;                                         /**< VESC_DEADMAN_TIMEOUT_MS in control periods. */
    static constexpr uint32_t VESC_DROP_SINGLE_MS       = 32;                                         /**< Telemetry gap logged as one lost frame (verbose logging only). */
    static constexpr uint32_t VESC_DROP_STREAK_MS       = 65;                                         /**< Telemetry gap logged as two or more lost frames. */
    static constexpr uint32_t DISPLAY_DROP_SINGLE_MS    = 180;                                        /**< Display gap logged as one lost frame (frames arrive about every 125 ms). */
    static constexpr uint32_t DISPLAY_DROP_STREAK_MS    = 320;                                        /**< Display gap logged as two or more lost frames. */
    static constexpr uint32_t VESC_DEADMAN_TIMEOUT_MS   = 500;                                        /**< The script cuts motor current when no control frame arrives for this long (same value in vesc/main.lbm). */
    static constexpr uint32_t VESC_COMMS_TIMEOUT_MS     = 500;                                        /**< The ESP32 treats the VESC as offline after this long without telemetry. */
    static constexpr uint32_t DISPLAY_DEADMAN_TIMEOUT_MS= 500;                                        /**< Display stall timeout (the display library uses its own default of the same value). */
    static constexpr uint32_t FAULT_CYCLE_PERIOD_MS     = 1000;                                       /**< With two faults active, the display alternates between them at this period. */
    static constexpr uint32_t B2S_HOLD_REQUIRED_MS      = 200;                                        /**< Brake hold time needed to arm. */

    // Throttle rise rate (P-menu PA) and regen strength (P-menu PB)
    static constexpr float SLEW_COUNTS_PER_SEC_PER_PA   = 20000.0f;                                   /**< Throttle may rise this fast per PA level (PA 1..5). Release is instant. */
    static constexpr uint8_t DEFAULT_PA_LEVEL           = 3;                                          /**< PA level used until the display reports one: 60,000 counts/s. */
    static constexpr uint8_t PB_REGEN_MAX_LEVEL         = 5;                                          /**< Highest PB level. */
    static constexpr uint16_t REGEN_COUNTS_PER_LEVEL    = 2000;                                       /**< Brake command per PB level. */
    static constexpr float GOVERNOR_RAMP_UP_RATE        = 1.0f;                                       /**< Rise rate of the ESP32 speed governor scale, per s (governor inactive, see compute_speed_governor_scale). */

    // Bounds on the time step used by the throttle rise limiter
    static constexpr float MIN_SLEW_DT_SEC              = 0.001f;                                     /**< s. */
    static constexpr float MAX_SLEW_DT_SEC              = 0.100f;                                     /**< s; a longer gap cannot release a large step at once. */
    static constexpr float THROTTLE_SNAP_EPSILON        = 1.0f;                                       /**< Snap to the target when within 1 count. */

    // Helpers
    inline constexpr float cms_to_mph(int16_t cms) {
        return static_cast<float>(cms) * (0.01f * 2.23693629f);
    }
    inline constexpr int16_t mph_to_cms(float mph) {
        return static_cast<int16_t>(roundf(mph * (0.44704f * 100.0f)));
    }
    inline constexpr float kmh_to_mph(float kmh) {
        return kmh * 0.62137119f;
    }
    inline constexpr float mph_to_kmh(float mph) {
        return mph * 1.609344f;
    }
    inline constexpr bool is_elapsed(uint32_t now, uint32_t start, uint32_t duration) {
        return (now - start) >= duration;
    }
    inline constexpr uint16_t get_mode_max_counts(uint8_t gear) {
        return (gear == 1) ? MODE_1_MAX_COUNTS :
               (gear == 2) ? MODE_2_MAX_COUNTS : MODE_3_MAX_COUNTS;
    }

    // Older names
    static constexpr float MAX_MOTOR_CURRENT_A          = MOTOR_MAX_CURRENT_AMPS;
    static constexpr float THROTTLE_FULL_SCALE          = static_cast<float>(THROTTLE_SCALE_MAX_COUNTS);
    static constexpr float COUNTS_PER_AMP               = CURRENT_COUNTS_PER_AMP;
    static constexpr float CONTROL_LOOP_HZ              = static_cast<float>(CONTROL_LOOP_RATE_HZ);
    static constexpr float TICK_DT_S                    = VESC_TICK_INTERVAL_SEC;
    static constexpr float STATIONARY_SPEED_MPH         = SPEED_STATIONARY_MPH;
    static constexpr uint16_t CURRENT_DEADBAND_COUNTS   = THROTTLE_DEADBAND_COUNTS;
}

/**
 * @struct SlewConfig
 * @brief Throttle rise rate from the P-menu PA setting: PA x 20,000 counts/s, PA clamped to 1..5
 *        (400 to 2,000 counts per 20 ms).
 */
struct SlewConfig {
    static constexpr float get_units_per_sec(uint8_t pa_level) {
        if (pa_level < 1) pa_level = 1;
        if (pa_level > 5) pa_level = 5;
        return static_cast<float>(pa_level) * PowertrainConfig::SLEW_COUNTS_PER_SEC_PER_PA;
    }

    static constexpr float get_max_step_per_tick(uint8_t pa_level) {
        return get_units_per_sec(pa_level) * PowertrainConfig::VESC_TICK_INTERVAL_SEC;
    }
};

/**
 * @class VESCSafetySupervisor
 * @brief Decides what throttle the ESP32 may send to the VESC.
 *
 * - Killswitch output (GPIO 2, wired to the rear VESC's ADC2 input): low at boot and whenever unarmed, so the
 *   script keeps the motors off even if the ESP32 sends throttle. High only after brake-to-start.
 * - Brake-to-start: brake held for B2S_HOLD_REQUIRED_MS with the throttle released and the vehicle stationary.
 * - Brake wins: any brake input zeroes throttle. Releasing the brake with the throttle open keeps throttle at
 *   zero until the throttle is released.
 * - Recovery lockouts: after a VESC fault, a lost VESC link, a stalled display, or the front VESC coming back,
 *   drive stays at zero until the cause is gone and the throttle is released. The killswitch stays high, so the
 *   rider does not have to stop in traffic.
 * - Throttle rise is rate-limited by the PA setting; release is instant.
 * No heap allocation. Called from the main loop only.
 */
class VESCSafetySupervisor {
public:
    static constexpr int8_t DEFAULT_KILLSWITCH_PIN = 2;
    static constexpr uint32_t DEFAULT_B2S_HOLD_MS = PowertrainConfig::B2S_HOLD_REQUIRED_MS;
    static constexpr uint16_t THROTTLE_NEUTRAL_COUNTS = PowertrainConfig::THROTTLE_NEUTRAL_COUNTS;
    static constexpr float DEFAULT_MAX_SLEW_PER_SEC = SlewConfig::get_units_per_sec(PowertrainConfig::DEFAULT_PA_LEVEL);

    explicit VESCSafetySupervisor(int8_t killswitch_pin = DEFAULT_KILLSWITCH_PIN,
                                  uint32_t b2s_hold_ms = DEFAULT_B2S_HOLD_MS,
                                  float max_slew_per_sec = DEFAULT_MAX_SLEW_PER_SEC)
        : _killswitch_pin(killswitch_pin),
          _b2s_hold_ms(b2s_hold_ms),
          _max_slew_per_sec(max_slew_per_sec) {}

    /** @brief Sets the killswitch pin as an output, low, and resets all state. */
    void begin() {
        if (_killswitch_pin >= 0) {
            digitalWrite(_killswitch_pin, LOW);
            pinMode(_killswitch_pin, OUTPUT);
            digitalWrite(_killswitch_pin, LOW);
        }
        _arming_state = ArmingState::BootUnarmed;
        _brake_hold_start_ms = 0;
        _last_brake_state = false;
        _can_recovery_locked = false;
        _last_can_desync = false;
        _ramped_throttle = 0.0f;
        _brake_release_locked = false;
        _last_brake_active = false;
        _fault_recovery_locked = false;
        _comms_recovery_locked = false;
        _display_recovery_locked = false;
        _active_foc_fault = 0;
        _governor_scale = 1.0f;
    }

    /** @brief Drops the killswitch output low and returns to the unarmed state (brake-to-start needed again). */
    void disarm() {
        _arming_state = ArmingState::BootUnarmed;
        if (_killswitch_pin >= 0) {
            digitalWrite(_killswitch_pin, LOW);
        }
        _ramped_throttle = 0.0f;
        _brake_hold_start_ms = 0;
        _brake_release_locked = false;
        _last_brake_active = false;
        _fault_recovery_locked = false;
        _comms_recovery_locked = false;
        _display_recovery_locked = false;
        _active_foc_fault = 0;
        _governor_scale = 1.0f;
    }

    /**
     * @brief Brake-to-start. Arms once the brake has been held for the hold time with the throttle below
     *        THROTTLE_NEUTRAL_COUNTS and the vehicle stationary. Drives the killswitch output every call.
     * @param[in] brake_active Brake lever pulled.
     * @param[in] throttle_req Throttle request, 0..10000.
     * @param[in] now_ms millis().
     * @param[in] speed_mph Vehicle speed.
     * @param[in] erpm Motor ERPM.
     * @return True if armed.
     */
    bool update_b2s(bool brake_active, uint16_t throttle_req, uint32_t now_ms, float speed_mph = 0.0f, float erpm = 0.0f) {
        if (_arming_state == ArmingState::ReadyToDrive) {
            if (_killswitch_pin >= 0) {
                digitalWrite(_killswitch_pin, HIGH);
            }
            return true;
        }

        // Unarmed: keep the VESC killswitch input low
        if (_killswitch_pin >= 0) {
            digitalWrite(_killswitch_pin, LOW);
        }

        const bool stationary = (fabsf(speed_mph) < PowertrainConfig::SPEED_STATIONARY_MPH) &&
                                (fabsf(erpm) < static_cast<float>(PowertrainConfig::SPEED_STATIONARY_ERPM));

        if (brake_active && (throttle_req < PowertrainConfig::THROTTLE_NEUTRAL_COUNTS) && stationary) {
            if (!_last_brake_state) {
                _brake_hold_start_ms = now_ms;
            }
            if ((now_ms - _brake_hold_start_ms) >= _b2s_hold_ms) {
                _arming_state = ArmingState::ReadyToDrive;
                if (_killswitch_pin >= 0) {
                    digitalWrite(_killswitch_pin, HIGH);
                }
                _last_brake_state = brake_active;
                return true;
            }
        } else {
            // Any condition missing: restart the hold timer
            _brake_hold_start_ms = now_ms;
        }

        _last_brake_state = brake_active;
        return false;
    }

    /** @brief True if armed. Reads the killswitch pin back when one is configured. */
    bool is_armed() const {
        if (_killswitch_pin >= 0) {
            return (digitalRead(_killswitch_pin) == HIGH);
        }
        return _arming_state == ArmingState::ReadyToDrive;
    }

    /** @brief Current arming state. */
    ArmingState get_arming_state() const {
        return _arming_state;
    }

    /** @brief Throttle after the rise limiter, 0..10000. */
    float get_ramped_throttle() const {
        return _ramped_throttle;
    }

    /** @brief Front VESC status from telemetry. When it comes back, dual drive waits for released throttle. */
    void set_can_desync(bool desync) {
        if (!desync && _last_can_desync) {
            _can_recovery_locked = true;
        }
        _last_can_desync = desync;
    }

    /** @brief True while dual drive waits for released throttle after the front VESC came back. */
    bool is_can_recovery_locked() const {
        return _can_recovery_locked;
    }

    /** @brief Clears the front VESC recovery lock once the throttle is released. */
    void update_can_recovery(uint16_t throttle_req) {
        if (_can_recovery_locked && throttle_req < PowertrainConfig::THROTTLE_NEUTRAL_COUNTS) {
            _can_recovery_locked = false;
        }
    }

    /**
     * @brief Rear VESC fault code from telemetry (0 = none). A fault zeroes drive until it clears and the
     *        throttle is released. The killswitch output is not touched.
     */
    void set_foc_fault(uint8_t fault_code) {
        _active_foc_fault = fault_code;
        if (fault_code != 0) {
            _fault_recovery_locked = true;
        }
    }

    /** @brief Clears the fault lock once the fault is gone and the throttle is released. */
    void update_fault_recovery(uint16_t throttle_req) {
        if (_fault_recovery_locked && _active_foc_fault == 0 && throttle_req < PowertrainConfig::THROTTLE_NEUTRAL_COUNTS) {
            _fault_recovery_locked = false;
        }
    }

    bool is_fault_recovery_locked() const {
        return _fault_recovery_locked;
    }

    uint8_t get_active_foc_fault() const {
        return _active_foc_fault;
    }

    /** @brief Locks drive while the VESC link is down; clears once it is back and the throttle is released. */
    void update_comms_recovery(bool vesc_online, uint16_t throttle_req) {
        if (!vesc_online) {
            _comms_recovery_locked = true;
        } else if (_comms_recovery_locked && throttle_req < PowertrainConfig::THROTTLE_NEUTRAL_COUNTS) {
            _comms_recovery_locked = false;
        }
    }

    bool is_comms_recovery_locked() const {
        return _comms_recovery_locked;
    }

    /**
     * @brief Locks drive while display frames are missing (the throttle reading would be stale); clears once
     *        frames are back and the throttle is released. The killswitch output stays high.
     */
    void update_display_recovery(bool display_stalled, uint16_t throttle_req) {
        if (display_stalled) {
            _display_recovery_locked = true;
        } else if (_display_recovery_locked && throttle_req < PowertrainConfig::THROTTLE_NEUTRAL_COUNTS) {
            _display_recovery_locked = false;
        }
    }

    bool is_display_recovery_locked() const {
        return _display_recovery_locked;
    }

    /** @brief Sets the throttle rise rate from the P-menu PA level (1..5). */
    void set_accel_level_pa(uint8_t pa_level) {
        _max_slew_per_sec = SlewConfig::get_units_per_sec(pa_level);
    }

    float get_max_slew_per_sec() const {
        return _max_slew_per_sec;
    }

    /**
     * @brief ESP32-side speed governor: 1 below the taper window, smoothstep to 0 at the limit.
     * @note Inactive in the firmware: main.cpp passes speed 0, so this returns 1. The speed limit is applied by
     *       the VESC script per wheel instead, so it still works if the ESP32 link fails.
     */
    static inline float compute_speed_governor_scale(float current_speed_mph, float speed_limit_mph) {
        if (speed_limit_mph <= 0.0f) return 1.0f;
        if (current_speed_mph >= speed_limit_mph) return 0.0f;
        const float taper_start = speed_limit_mph - PowertrainConfig::SPEED_LIMIT_TAPER_WINDOW_MPH;
        if (current_speed_mph <= taper_start) return 1.0f;

        float t = (speed_limit_mph - current_speed_mph) / PowertrainConfig::SPEED_LIMIT_TAPER_WINDOW_MPH;
        if (t < 0.0f) {
            t = 0.0f;
        } else if (t > 1.0f) {
            t = 1.0f;
        }

        // Smoothstep: t^2 (3 - 2t)
        return t * t * (3.0f - 2.0f * t);
    }

    /** @brief Governor scale follows a drop at once and rises at GOVERNOR_RAMP_UP_RATE per second. */
    float update_governor_scale(float target_scale, float dt_s) {
        if (target_scale < 0.0f) {
            target_scale = 0.0f;
        } else if (target_scale > 1.0f) {
            target_scale = 1.0f;
        }

        if (target_scale < _governor_scale) {
            _governor_scale = target_scale;
        } else if (target_scale > _governor_scale) {
            if (dt_s > 0.0f && dt_s <= 0.5f) {
                const float max_ramp = PowertrainConfig::GOVERNOR_RAMP_UP_RATE * dt_s;
                if ((target_scale - _governor_scale) > max_ramp) {
                    _governor_scale += max_ramp;
                } else {
                    _governor_scale = target_scale;
                }
            } else {
                _governor_scale = target_scale;
            }
        }
        return _governor_scale;
    }

    float update_governor_scale(float current_speed_mph, float speed_limit_mph, float dt_s) {
        const float target = compute_speed_governor_scale(current_speed_mph, speed_limit_mph);
        return update_governor_scale(target, dt_s);
    }

    float get_governor_scale() const {
        return _governor_scale;
    }

    void set_governor_scale(float scale) {
        if (scale < 0.0f) scale = 0.0f;
        else if (scale > 1.0f) scale = 1.0f;
        _governor_scale = scale;
    }

    /**
     * @brief Throttle request for the selected mode: normalized throttle x mode ratio x 10000, with full
     *        throttle mapped exactly to the mode ceiling.
     * @param[in] normalized Throttle 0..1 from the display library.
     * @param[in] gear Mode 1, 2 or 3.
     * @param[in] current_speed_mph Speed for the ESP32 governor (the firmware passes 0, see compute_speed_governor_scale).
     * @param[in] dt_s Time since the previous call.
     * @return Throttle request, 0..10000.
     */
    uint16_t compute_mode_scaled_throttle(float normalized, uint8_t gear, float current_speed_mph, float dt_s) {
        if (normalized <= 0.0f) return 0;

        float mode_scale = 1.0f;
        float speed_limit = 0.0f;
        uint16_t max_counts = PowertrainConfig::MODE_3_MAX_COUNTS;

        if (gear == 1) {
            mode_scale = PowertrainConfig::MODE_1_SCALE_RATIO;
            speed_limit = PowertrainConfig::MODE_1_SPEED_LIMIT_MPH;
            max_counts = PowertrainConfig::MODE_1_MAX_COUNTS;
        } else if (gear == 2) {
            mode_scale = PowertrainConfig::MODE_2_SCALE_RATIO;
            speed_limit = PowertrainConfig::MODE_2_SPEED_LIMIT_MPH;
            max_counts = PowertrainConfig::MODE_2_MAX_COUNTS;
        } else {
            mode_scale = PowertrainConfig::MODE_3_SCALE_RATIO;
            speed_limit = 0.0f;
            max_counts = PowertrainConfig::MODE_3_MAX_COUNTS;
        }

        const float target_scale = compute_speed_governor_scale(current_speed_mph, speed_limit);
        const float speed_scale = update_governor_scale(target_scale, dt_s);
        if (speed_scale <= 0.0f) return 0;

        if (normalized >= 1.0f) {
            return static_cast<uint16_t>(roundf(static_cast<float>(max_counts) * speed_scale));
        }

        float target = normalized * mode_scale * static_cast<float>(PowertrainConfig::THROTTLE_SCALE_MAX_COUNTS) * speed_scale;
        if (target > static_cast<float>(PowertrainConfig::THROTTLE_SCALE_MAX_COUNTS)) {
            target = static_cast<float>(PowertrainConfig::THROTTLE_SCALE_MAX_COUNTS);
        }
        return static_cast<uint16_t>(roundf(target));
    }

    /** @brief Raw throttle (0..1000) to 0..1, with 0 at or below deadband and 1 at or above ceiling. Not used by the firmware, which normalizes in the display library. */
    static inline float normalize_raw_adc(uint16_t raw_adc, uint16_t deadband = 15, uint16_t ceiling = 980) {
        if (raw_adc > PowertrainConfig::PLAUSIBILITY_CLAMP_MAX) return 0.0f;
        if (raw_adc <= deadband) return 0.0f;
        if (raw_adc >= ceiling) return 1.0f;
        const float range = static_cast<float>(ceiling - deadband);
        return (range > 0.0f) ? (static_cast<float>(raw_adc - deadband) / range) : 0.0f;
    }

    uint16_t compute_mode_scaled_throttle(uint16_t raw_adc, uint8_t gear, float current_speed_mph, float dt_s) {
        if (raw_adc > PowertrainConfig::PLAUSIBILITY_CLAMP_MAX) return 0;
        return compute_mode_scaled_throttle(normalize_raw_adc(raw_adc), gear, current_speed_mph, dt_s);
    }

    /** @brief Same as the four-argument version, without the governor's rise limit. */
    static inline uint16_t compute_mode_scaled_throttle(float normalized, uint8_t gear, float current_speed_mph = 0.0f) {
        if (normalized <= 0.0f) return 0;

        float mode_scale = 1.0f;
        float speed_limit = 0.0f;
        uint16_t max_counts = PowertrainConfig::MODE_3_MAX_COUNTS;

        if (gear == 1) {
            mode_scale = PowertrainConfig::MODE_1_SCALE_RATIO;
            speed_limit = PowertrainConfig::MODE_1_SPEED_LIMIT_MPH;
            max_counts = PowertrainConfig::MODE_1_MAX_COUNTS;
        } else if (gear == 2) {
            mode_scale = PowertrainConfig::MODE_2_SCALE_RATIO;
            speed_limit = PowertrainConfig::MODE_2_SPEED_LIMIT_MPH;
            max_counts = PowertrainConfig::MODE_2_MAX_COUNTS;
        } else {
            mode_scale = PowertrainConfig::MODE_3_SCALE_RATIO;
            speed_limit = 0.0f;
            max_counts = PowertrainConfig::MODE_3_MAX_COUNTS;
        }

        const float speed_scale = compute_speed_governor_scale(current_speed_mph, speed_limit);
        if (speed_scale <= 0.0f) return 0;

        if (normalized >= 1.0f) {
            return static_cast<uint16_t>(roundf(static_cast<float>(max_counts) * speed_scale));
        }

        float target = normalized * mode_scale * static_cast<float>(PowertrainConfig::THROTTLE_SCALE_MAX_COUNTS) * speed_scale;
        if (target > static_cast<float>(PowertrainConfig::THROTTLE_SCALE_MAX_COUNTS)) {
            target = static_cast<float>(PowertrainConfig::THROTTLE_SCALE_MAX_COUNTS);
        }
        return static_cast<uint16_t>(roundf(target));
    }

    /** @brief Same, from a raw throttle reading. */
    static inline uint16_t compute_mode_scaled_throttle(uint16_t raw_adc, uint8_t gear, float current_speed_mph = 0.0f) {
        if (raw_adc > PowertrainConfig::PLAUSIBILITY_CLAMP_MAX) return 0;
        return compute_mode_scaled_throttle(normalize_raw_adc(raw_adc), gear, current_speed_mph);
    }

    /** @brief Brake command while the brake is pulled: PB level x 2000 counts (PB 0 = no regen). */
    static inline uint16_t compute_pb_regen(bool brake_active, uint8_t pb_level) {
        if (!brake_active || pb_level == 0) return 0;
        if (pb_level > PowertrainConfig::PB_REGEN_MAX_LEVEL) pb_level = PowertrainConfig::PB_REGEN_MAX_LEVEL;
        return static_cast<uint16_t>(pb_level * PowertrainConfig::REGEN_COUNTS_PER_LEVEL);
    }

    /** @brief True while throttle is held at zero because the brake was released with the throttle open. */
    bool is_brake_release_locked() const {
        return _brake_release_locked;
    }

    /**
     * @brief Builds the control frame payload from the rider inputs and the current lockouts.
     * @param[in] raw_throttle Throttle request, 0..10000 (mode scaling already applied).
     * @param[in] brake_request Regen request, 0..10000.
     * @param[in] physical_brake_active Brake lever pulled.
     * @param[in] dual_mode Rider selected dual drive.
     * @param[in] can_desync Front VESC lost.
     * @param[in] vesc_online VESC telemetry current.
     * @param[in] dt_s Time since the previous call, for the rise limiter.
     * @param[in] speed_limit_cms Mode speed limit for the script, 0 = none.
     */
    TXPayloadControl compute_safe_control(uint16_t raw_throttle,
                                          uint16_t brake_request,
                                          bool physical_brake_active,
                                          bool dual_mode,
                                          bool can_desync,
                                          bool vesc_online,
                                          float dt_s,
                                          uint16_t speed_limit_cms = 0) {
        TXPayloadControl out;
        out.brake = brake_request;
        out.dual_mode = (dual_mode && !can_desync && !_can_recovery_locked) ? 1 : 0;
        out.speed_limit_cms = speed_limit_cms;

        // Brake wins over throttle
        const bool brake_active = physical_brake_active || (brake_request > 0);
        if (brake_active) {
            _last_brake_active = true;
            _ramped_throttle = 0.0f;
            _last_dispatched_throttle = 0;
            out.throttle = 0;
            return out;
        } else {
            // Brake released with the throttle open: hold throttle at zero until it is released
            if (_last_brake_active) {
                _last_brake_active = false;
                if (raw_throttle >= PowertrainConfig::THROTTLE_NEUTRAL_COUNTS) {
                    _brake_release_locked = true;
                }
            }
        }

        if (_brake_release_locked) {
            if (raw_throttle < PowertrainConfig::THROTTLE_NEUTRAL_COUNTS) {
                _brake_release_locked = false;
            } else {
                _ramped_throttle = 0.0f;
                _last_dispatched_throttle = 0;
                out.throttle = 0;
                return out;
            }
        }

        update_can_recovery(raw_throttle);

        update_fault_recovery(raw_throttle);

        update_comms_recovery(vesc_online, raw_throttle);

        // Drive only when armed, the VESC is online, no lockout is active and the throttle is above neutral
        float target_throttle = 0.0f;
        if (is_armed() && vesc_online && !_fault_recovery_locked && !_comms_recovery_locked && !_display_recovery_locked && raw_throttle >= PowertrainConfig::THROTTLE_NEUTRAL_COUNTS) {
            // With the front VESC lost the rear keeps the full mode ceiling: one working motor is better than
            // none, and the rider limits power with the throttle and the mode
            target_throttle = static_cast<float>(raw_throttle);
        }

        // Rise limited by the PA rate; release is immediate
        const float effective_dt = (dt_s < PowertrainConfig::MIN_SLEW_DT_SEC) ? PowertrainConfig::MIN_SLEW_DT_SEC :
                                   ((dt_s > PowertrainConfig::MAX_SLEW_DT_SEC) ? PowertrainConfig::MAX_SLEW_DT_SEC : dt_s);
        const float max_step = _max_slew_per_sec * effective_dt;
        if (target_throttle > _ramped_throttle) {
            _ramped_throttle += (target_throttle - _ramped_throttle > max_step) ? max_step : (target_throttle - _ramped_throttle);
        } else {
            _ramped_throttle = target_throttle;
        }

        // Snap to the target when within 1 count
        if (fabsf(target_throttle - _ramped_throttle) < PowertrainConfig::THROTTLE_SNAP_EPSILON) {
            _ramped_throttle = target_throttle;
        }

        uint16_t target_int = static_cast<uint16_t>(lroundf(_ramped_throttle));

        // Within 1 count of a mode ceiling: use the ceiling exactly, so the jitter hold below never stops short of it
        if (target_int >= PowertrainConfig::MODE_3_MAX_COUNTS - 1) {
            target_int = PowertrainConfig::MODE_3_MAX_COUNTS;
        } else if (target_int >= PowertrainConfig::MODE_2_MAX_COUNTS - 1 && target_int <= PowertrainConfig::MODE_2_MAX_COUNTS) {
            target_int = PowertrainConfig::MODE_2_MAX_COUNTS;
        } else if (target_int >= PowertrainConfig::MODE_1_MAX_COUNTS - 1 && target_int <= PowertrainConfig::MODE_1_MAX_COUNTS) {
            target_int = PowertrainConfig::MODE_1_MAX_COUNTS;
        }

        // Jitter hold: only send a new value when it moved by THROTTLE_DEADBAND_COUNTS, reaches zero, or hits a
        // mode ceiling
        if (target_int == 0) {
            _last_dispatched_throttle = 0;
            out.throttle = 0;
        } else if (abs(static_cast<int32_t>(target_int) - static_cast<int32_t>(_last_dispatched_throttle)) >= PowertrainConfig::THROTTLE_DEADBAND_COUNTS ||
                   target_int >= PowertrainConfig::MODE_3_MAX_COUNTS ||
                   target_int == PowertrainConfig::MODE_1_MAX_COUNTS ||
                   target_int == PowertrainConfig::MODE_2_MAX_COUNTS) {
            _last_dispatched_throttle = target_int;
            out.throttle = target_int;
        } else {
            out.throttle = _last_dispatched_throttle;
        }

        return out;
    }

    /** @brief Same, also updating the display stall lock. */
    TXPayloadControl compute_safe_control(uint16_t raw_throttle,
                                          uint16_t brake_request,
                                          bool physical_brake_active,
                                          bool dual_mode,
                                          bool can_desync,
                                          bool vesc_online,
                                          float dt_s,
                                          bool display_stalled,
                                          uint16_t speed_limit_cms = 0) {
        update_display_recovery(display_stalled, raw_throttle);
        return compute_safe_control(raw_throttle, brake_request, physical_brake_active, dual_mode, can_desync, vesc_online, dt_s, speed_limit_cms);
    }

    /**
     * @brief Fault to show on the display. A lost VESC link overrides everything. Otherwise the first two active
     *        conditions, in the order below, alternate every FAULT_CYCLE_PERIOD_MS (now_ms = 0 shows the first).
     */
    SafetyFault evaluate_fault(bool vesc_online, bool can_desync, uint16_t throttle_req = 0, uint32_t now_ms = 0) const {
        (void)throttle_req;

        if (!vesc_online) {
            return SafetyFault::VescOffline;
        }

        SafetyFault primary = SafetyFault::None;
        SafetyFault secondary = SafetyFault::None;
        if (can_desync) {
            if (primary == SafetyFault::None) primary = SafetyFault::CanDesync;
            else if (secondary == SafetyFault::None) secondary = SafetyFault::CanDesync;
        }
        if (_fault_recovery_locked || _active_foc_fault != 0) {
            if (primary == SafetyFault::None) primary = SafetyFault::FocFaultLockout;
            else if (secondary == SafetyFault::None) secondary = SafetyFault::FocFaultLockout;
        }
        if (_comms_recovery_locked) {
            if (primary == SafetyFault::None) primary = SafetyFault::CommsRecoveryLockout;
            else if (secondary == SafetyFault::None) secondary = SafetyFault::CommsRecoveryLockout;
        }
        if (_display_recovery_locked) {
            if (primary == SafetyFault::None) primary = SafetyFault::DisplayStalled;
            else if (secondary == SafetyFault::None) secondary = SafetyFault::DisplayStalled;
        }
        // Read the killswitch pin itself, so the display shows what the VESC sees
        const bool hw_armed = (_killswitch_pin >= 0) ? (digitalRead(_killswitch_pin) == HIGH) : is_armed();
        if (!hw_armed || _brake_release_locked) {
            if (primary == SafetyFault::None) primary = SafetyFault::KillswitchDisarmed;
            else if (secondary == SafetyFault::None) secondary = SafetyFault::KillswitchDisarmed;
        }

        if (primary == SafetyFault::None) return SafetyFault::None;
        if (secondary == SafetyFault::None || now_ms == 0) return primary;

        const uint32_t phase = (now_ms / PowertrainConfig::FAULT_CYCLE_PERIOD_MS) % 2;
        return (phase == 0) ? primary : secondary;
    }

private:
    int8_t _killswitch_pin;
    uint32_t _b2s_hold_ms;
    float _max_slew_per_sec;

    volatile ArmingState _arming_state = ArmingState::BootUnarmed;
    uint32_t _brake_hold_start_ms = 0;
    bool _last_brake_state = false;
    volatile bool _can_recovery_locked = false;
    volatile bool _last_can_desync = false;
    float _ramped_throttle = 0.0f;
    volatile bool _brake_release_locked = false;
    volatile bool _last_brake_active = false;
    volatile bool _fault_recovery_locked = false;
    volatile bool _comms_recovery_locked = false;
    volatile bool _display_recovery_locked = false;
    volatile uint8_t _active_foc_fault = 0;
    uint16_t _last_dispatched_throttle = 0;
    float _governor_scale = 1.0f;
};

} // namespace VESCBridge
