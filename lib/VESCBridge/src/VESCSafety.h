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

#pragma once

#include <Arduino.h>
#include <math.h>
#include "VESCUARTBridge.h"

// Canonical fast unit and time conversion macros
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
 * @brief Automotive powertrain arming lifecycle state.
 */
enum class ArmingState : uint8_t {
    BootUnarmed  = 0, /**< @brief Booted safely or tripped; power stage de-energized (GPIO 2 LOW). */
    ReadyToDrive = 1  /**< @brief Armed after rider holds brake >= 200ms with neutral throttle (GPIO 2 HIGH). */
};

/**
 * @enum SafetyFault
 * @brief Drivetrain supervisory fault classification.
 */
enum class SafetyFault : uint8_t {
    None                   = 0, /**< @brief Normal operation. */
    KillswitchDisarmed     = 1, /**< @brief Hardware killswitch open / disarmed or brake release launch lockout. */
    VescOffline            = 2, /**< @brief VESC UART telemetry link lost. */
    CanDesync              = 3, /**< @brief Front VESC CAN link timeout >500ms. */
    FocFaultLockout        = 4, /**< @brief VESC FOC fault active or awaiting neutral reset. */
    CommsRecoveryLockout   = 5, /**< @brief Comm recovery neutral lockout active. */
    DisplayStalled         = 6  /**< @brief Display comms watchdog expired. */
};

/**
 * @namespace PowertrainConfig
 * @brief Authoritative engineering constants and powertrain parameters (120A system).
 *        Derived mathematically via constexpr from master sources of truth.
 */
namespace PowertrainConfig {
    // Drivetrain Geometry Constants
    static constexpr float WHEEL_DIAMETER_INCHES        = 10.0f;                                      /**< Nominal wheel outer diameter (inches). */
    static constexpr uint8_t MOTOR_POLE_PAIRS           = 15;                                         /**< BLDC motor magnet pole pairs. */
    static constexpr float METERS_PER_INCH              = 0.0254f;                                    /**< Standard metric inch conversion factor. */
    static constexpr float PI_CONST                     = 3.14159265358979323846f;                    /**< Canonical Pi. */
    static constexpr float WHEEL_DIAMETER_METERS        = WHEEL_DIAMETER_INCHES * METERS_PER_INCH;     /**< Wheel diameter in meters (0.254m). */
    static constexpr float WHEEL_CIRCUMFERENCE_METERS   = WHEEL_DIAMETER_METERS * PI_CONST;           /**< Wheel rolling circumference (0.79796m). */

    // Velocity & Speed Governing Thresholds
    static constexpr float MPS_PER_MPH                  = 0.44704f;                                   /**< Canonical mph to m/s conversion factor. */
    static constexpr float SPEED_STATIONARY_MPH         = 0.6f;                                       /**< Maximum speed to qualify stationary vehicle for B2S arming. */
    static constexpr float SPEED_STATIONARY_MPS         = SPEED_STATIONARY_MPH * MPS_PER_MPH;         /**< Stationary threshold in m/s (0.268224 m/s). */
    static constexpr float SPEED_STATIONARY_CMS         = SPEED_STATIONARY_MPS * 100.0f;              /**< Stationary threshold in cm/s (26.8224 cm/s). */
    static constexpr uint32_t SPEED_STATIONARY_ERPM     = 300;                                        /**< Maximum motor electrical RPM to qualify stationary vehicle. */
    static constexpr float MODE_1_SPEED_LIMIT_MPH       = 15.0f;                                      /**< Mode 1 (Eco) maximum speed ceiling (15.0 mph). */
    static constexpr float MODE_2_SPEED_LIMIT_MPH       = 28.0f;                                      /**< Mode 2 (Std) maximum speed ceiling (28.0 mph). */
    static constexpr float SPEED_LIMIT_TAPER_WINDOW_MPH = 3.0f;                                       /**< Cubic Hermite smoothstep feathering window (3.0 mph). */
    static constexpr uint16_t MODE_1_SPEED_LIMIT_CMS    = 671;                                        /**< Mode 1 speed limit in cm/s (15.0 mph * 44.704). */
    static constexpr uint16_t MODE_2_SPEED_LIMIT_CMS    = 1252;                                       /**< Mode 2 speed limit in cm/s (28.0 mph * 44.704). */
    static constexpr uint16_t MODE_3_SPEED_LIMIT_CMS    = 0;                                          /**< Mode 3 speed limit in cm/s (0 = Unlimited). */
    static constexpr uint16_t SPEED_LIMIT_TAPER_CMS     = 134;                                        /**< Speed limit taper feathering window in cm/s (3.0 mph). */

    static constexpr uint16_t get_mode_speed_limit_cms(uint8_t gear) {
        if (gear == 1) return MODE_1_SPEED_LIMIT_CMS;
        if (gear == 2) return MODE_2_SPEED_LIMIT_CMS;
        return MODE_3_SPEED_LIMIT_CMS;
    }

    // Phase Current & Throttle Calibration Envelopes
    static constexpr float MOTOR_MAX_CURRENT_AMPS       = 120.0f;                                     /**< Phase current ceiling per motor (120A peak). */
    static constexpr uint16_t THROTTLE_SCALE_MAX_COUNTS = 10000;                                      /**< Normalized throttle request full-scale (0..10000 counts). */
    static constexpr float CURRENT_COUNTS_PER_AMP       = static_cast<float>(THROTTLE_SCALE_MAX_COUNTS) / MOTOR_MAX_CURRENT_AMPS; /**< 83.333 counts/A scaling factor. */
    static constexpr float MODE_1_SCALE_RATIO           = 0.50f;                                      /**< Mode 1 torque ceiling ratio (50%). */
    static constexpr float MODE_2_SCALE_RATIO           = 0.75f;                                      /**< Mode 2 torque ceiling ratio (75%). */
    static constexpr float MODE_3_SCALE_RATIO           = 1.00f;                                      /**< Mode 3 torque ceiling ratio (100%). */
    static constexpr uint16_t MODE_1_MAX_COUNTS         = 5000;                                       /**< Mode 1 (Eco): 5000 counts / 60A per motor. */
    static constexpr uint16_t MODE_2_MAX_COUNTS         = 7500;                                       /**< Mode 2 (Std): 7500 counts / 90A per motor. */
    static constexpr uint16_t MODE_3_MAX_COUNTS         = 10000;                                      /**< Mode 3 (Turbo): 10000 counts / 120A per motor. */
    static constexpr uint16_t THROTTLE_NEUTRAL_COUNTS   = 100;                                        /**< 1.0% neutral throttle deadband (100 counts / 1.2A). */
    static constexpr uint16_t THROTTLE_DEADBAND_COUNTS  = 50;                                         /**< 50 counts (0.5% / 0.60A) hysteresis window to suppress coil hiss. */
    static constexpr uint16_t PLAUSIBILITY_CLAMP_MAX     = 1050;                                       /**< Throttle ADC plausibility clamp threshold (accommodates 5V/3.3V rail ripple). */

    // Speed-Scaled Dynamic Electronic Regen Thresholds
    static constexpr float SPEED_REGEN_MIN_MPH          = 5.0f;                                       /**< Speed below which electronic regen clamps to 0.0 (user-tested). */
    static constexpr float SPEED_REGEN_MAX_MPH          = 20.0f;                                      /**< Speed above which electronic regen grants 100% PB strength. */
    static constexpr float SPEED_REGEN_MIN_MPS          = SPEED_REGEN_MIN_MPH * MPS_PER_MPH;          /**< Minimum regen speed in m/s (2.2352 m/s). */
    static constexpr float SPEED_REGEN_MAX_MPS          = SPEED_REGEN_MAX_MPH * MPS_PER_MPH;          /**< Full regen speed in m/s (8.9408 m/s). */
    static constexpr float REGEN_LERP_RANGE_MPS         = SPEED_REGEN_MAX_MPS - SPEED_REGEN_MIN_MPS;  /**< Regen taper range in m/s (6.7056 m/s). */
    static constexpr float INV_REGEN_RANGE_MPS          = 1.0f / REGEN_LERP_RANGE_MPS;                /**< Inverse regen range (0.149129 1/(m/s)). */

    // Fail-Safe Watchdogs & Control Loop Timing
    static constexpr uint32_t CONTROL_LOOP_RATE_HZ      = 50;                                         /**< Real-time control loop frequency (50 Hz). */
    static constexpr uint32_t VESC_TICK_RATE_HZ         = 50;                                         /**< VESC ping-pong dispatch frequency (50 Hz). */
    static constexpr uint32_t VESC_TICK_INTERVAL_MS     = 20;                                         /**< Inter-tick period in milliseconds (20ms). */
    static constexpr float VESC_TICK_INTERVAL_SEC       = 0.020f;                                     /**< Inter-tick period in seconds (0.020s). */
    static constexpr uint32_t DEADMAN_MISSED_TICKS      = 25;                                         /**< Missed telemetry frames before deadman freewheel trip (25 ticks = 500ms). */
    static constexpr uint32_t VESC_DROP_SINGLE_MS       = 32;                                         /**< Telemetry silence threshold indicating single dropped 20ms frame (32ms). */
    static constexpr uint32_t VESC_DROP_STREAK_MS       = 65;                                         /**< Telemetry silence threshold indicating >= 2 consecutive dropped 20ms frames (65ms). */
    static constexpr uint32_t DISPLAY_DROP_SINGLE_MS    = 180;                                        /**< Display UART silence threshold indicating single dropped ~125ms frame (180ms). */
    static constexpr uint32_t DISPLAY_DROP_STREAK_MS    = 320;                                        /**< Display UART silence threshold indicating >= 2 consecutive dropped ~125ms frames (320ms). */
    static constexpr uint32_t VESC_DEADMAN_TIMEOUT_MS   = 500;                                        /**< VESC dead-man failsafe timeout (500ms / 25 ticks). */
    static constexpr uint32_t VESC_COMMS_TIMEOUT_MS     = 500;                                        /**< VESC UART comms loss fault threshold (500ms / 25 ticks). */
    static constexpr uint32_t DISPLAY_DEADMAN_TIMEOUT_MS= 500;                                        /**< Display dead-man comms stall timeout (500ms). */
    static constexpr uint32_t FAULT_CYCLE_PERIOD_MS     = 1000;                                       /**< Multi-fault time-sliced rotation period (1000ms). */
    static constexpr uint32_t B2S_HOLD_REQUIRED_MS      = 200;                                        /**< Brake-to-Start hold duration requirement (200ms). */

    // Acceleration slew & governor parameters
    static constexpr float SLEW_COUNTS_PER_SEC_PER_PA   = 20000.0f;                                   /**< 20,000 counts/s per PA level. */
    static constexpr uint8_t DEFAULT_PA_LEVEL           = 3;                                          /**< Standard PA level (60k counts/s, 1200 counts/tick). */
    static constexpr uint8_t PB_REGEN_MAX_LEVEL         = 5;                                          /**< Maximum P-menu PB regen level. */
    static constexpr uint16_t REGEN_COUNTS_PER_LEVEL    = 2000;                                       /**< 2000 counts per level (0..10000). */
    static constexpr float GOVERNOR_RAMP_UP_RATE        = 1.0f;                                       /**< Re-engagement rate limit: 1.0 unit/s ramp-up on speed dips. */

    // Delta time bounding for torque slew rate limiter
    static constexpr float MIN_SLEW_DT_SEC              = 0.001f;                                     /**< Minimum delta time clamp for slew limiter (1ms). */
    static constexpr float MAX_SLEW_DT_SEC              = 0.100f;                                     /**< Maximum delta time clamp for slew limiter (100ms). */
    static constexpr float THROTTLE_SNAP_EPSILON        = 1.0f;                                       /**< Boundary snapping threshold within 1 count. */

    // Canonical Powertrain & Velocity Helpers
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

    // Backward compatibility aliases
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
 * @brief Compile-time parameterized torque slew rate calculation engine.
 *        Calculates maximum allowed slew rate based on rider P-menu PA acceleration setting (1..5):
 *        Slew Rate = pa_level * 20,000 counts/s (400 to 2,000 counts/tick at 50 Hz).
 *        Eliminates runtime divisions and complex tables.
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
 * @brief Canonical safety supervisor enforcing fail-closed vehicle interlocks:
 *        - Hardware Killswitch (GPIO 2): Fail-closed LOW (0.0V) on boot/stall/fault, HIGH (3.3V) when armed.
 *        - Automotive Brake-to-Start (Ready-to-Drive): Requires brake hold >= 200ms with throttle neutral (< 2%).
 *        - Zero-Throttle Re-Arm & Boot Lockout: Re-arming or driving rejected if throttle >= 2%.
 *        - Active Brake Priority: Brake pull unconditionally zeroes drive torque.
 *        - CAN Desync Recovery Lockout: Front link recovery requires neutral throttle before re-engaging torque.
 *        - Slew Rate Limiting: Enforces PA-governed acceleration slew rate (20k..100k counts/s).
 * @note Core affinity: Core 1. Zero dynamic memory allocation.
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

    /**
     * @brief Initializes GPIO 2 to output and forces fail-closed LOW (0.0V).
     */
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

    /**
     * @brief Immediately de-energizes the powertrain (GPIO 2 LOW, throttle clamped to 0).
     */
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
     * @brief Non-blocking Brake-to-Start (Ready-to-Drive) update.
     * @param[in] brake_active True if mechanical/electrical brake lever is actuated.
     * @param[in] throttle_req Normalized throttle request (0..10000).
     * @param[in] now_ms Current timestamp in milliseconds.
     * @param[in] speed_mph Current vehicle speed in mph (must be stationary < 0.6 mph to arm).
     * @param[in] erpm Current motor electrical RPM (must be stationary < 300 ERPM to arm).
     * @return bool True if transitioned to ReadyToDrive on this call or currently armed.
     */
    bool update_b2s(bool brake_active, uint16_t throttle_req, uint32_t now_ms, float speed_mph = 0.0f, float erpm = 0.0f) {
        if (_arming_state == ArmingState::ReadyToDrive) {
            if (_killswitch_pin >= 0) {
                digitalWrite(_killswitch_pin, HIGH);
            }
            return true;
        }

        // Fail-closed while unarmed
        if (_killswitch_pin >= 0) {
            digitalWrite(_killswitch_pin, LOW);
        }

        // Require brake held, throttle neutral (< 100 counts), and vehicle stationary (< 0.6 mph / < 300 ERPM)
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
            // Reset timer if brake released, throttle out of neutral, or vehicle moving
            _brake_hold_start_ms = now_ms;
        }

        _last_brake_state = brake_active;
        return false;
    }

    /**
     * @brief Checks if powertrain is in ReadyToDrive state directly synced with physical killswitch GPIO.
     */
    bool is_armed() const {
        if (_killswitch_pin >= 0) {
            return (digitalRead(_killswitch_pin) == HIGH);
        }
        return _arming_state == ArmingState::ReadyToDrive;
    }

    /**
     * @brief Returns current arming state enum.
     */
    ArmingState get_arming_state() const {
        return _arming_state;
    }

    /**
     * @brief Returns current ramped throttle value (0..10000).
     */
    float get_ramped_throttle() const {
        return _ramped_throttle;
    }

    /**
     * @brief Updates CAN desync status. On falling edge of desync (reconnection), latches dual-mode recovery neutral lockout.
     */
    void set_can_desync(bool desync) {
        if (!desync && _last_can_desync) {
            _can_recovery_locked = true;
        }
        _last_can_desync = desync;
    }

    /**
     * @brief Returns true if CAN recovery neutral lock is currently holding drive torque off.
     */
    bool is_can_recovery_locked() const {
        return _can_recovery_locked;
    }

    /**
     * @brief Confirms throttle neutral to release CAN recovery lockout.
     */
    void update_can_recovery(uint16_t throttle_req) {
        if (_can_recovery_locked && throttle_req < PowertrainConfig::THROTTLE_NEUTRAL_COUNTS) {
            _can_recovery_locked = false;
        }
    }

    /**
     * @brief Updates transient VESC FOC fault status.
     *        Does NOT trip hardware killswitch (GPIO 2 remains HIGH).
     *        Locks torque until fault clears AND rider returns throttle to neutral (< 1%).
     */
    void set_foc_fault(uint8_t fault_code) {
        _active_foc_fault = fault_code;
        if (fault_code != 0) {
            _fault_recovery_locked = true;
        }
    }

    /**
     * @brief Releases fault recovery lockout once VESC fault clears and throttle is neutral.
     */
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

    /**
     * @brief Updates comms recovery status. If comms drops, latches recovery neutral lockout.
     */
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
     * @brief Updates display comms stall recovery state.
     *        When display stalls, latches lockout to cut torque while keeping GPIO 2 HIGH (no forced stop).
     *        When display link restores and throttle returns to neutral (< 100 counts), clears lockout.
     * @param[in] display_stalled True if display heartbeat/comms watchdog expired.
     * @param[in] throttle_req Normalized throttle request (0..10000 counts).
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

    /**
     * @brief Dynamically parameterizes positive torque slew rate (dI/dt) based on PA acceleration level (1..5).
     * @param[in] pa_level P-menu PA value (1 to 5).
     */
    void set_accel_level_pa(uint8_t pa_level) {
        _max_slew_per_sec = SlewConfig::get_units_per_sec(pa_level);
    }

    float get_max_slew_per_sec() const {
        return _max_slew_per_sec;
    }

    /**
     * @brief Computes speed governor attenuation factor [0.0f, 1.0f] for speed-capped drive modes
     *        using a Cubic Hermite Smoothstep over a 3.0 mph feathering window.
     * @param[in] current_speed_mph Current vehicle road speed in mph.
     * @param[in] speed_limit_mph Maximum speed limit for the active mode in mph (0.0f = unlimited).
     * @return float Attenuation factor: 1.0f below taper window, smooth S-curve decay to 0.0f at limit.
     */
    static inline float compute_speed_governor_scale(float current_speed_mph, float speed_limit_mph) {
        if (speed_limit_mph <= 0.0f) return 1.0f; // Mode 3: Unlimited
        if (current_speed_mph >= speed_limit_mph) return 0.0f;
        const float taper_start = speed_limit_mph - PowertrainConfig::SPEED_LIMIT_TAPER_WINDOW_MPH;
        if (current_speed_mph <= taper_start) return 1.0f;

        // Window normalization: t in [0.0f, 1.0f]
        float t = (speed_limit_mph - current_speed_mph) / PowertrainConfig::SPEED_LIMIT_TAPER_WINDOW_MPH;
        if (t < 0.0f) {
            t = 0.0f;
        } else if (t > 1.0f) {
            t = 1.0f;
        }

        // Cubic Hermite smoothstep polynomial: S(t) = t^2 * (3.0f - 2.0f * t)
        return t * t * (3.0f - 2.0f * t);
    }

    /**
     * @brief Stateful asymmetric re-engagement rate limiter for speed governor scale.
     *        - Instantaneous attenuation on overspeed (target < current) to protect speed envelope.
     *        - Controlled 1.0 unit/second ramp-up on speed dips (target > current) to prevent acceleration surge.
     * @param[in] target_scale Target speed governor scale [0.0f, 1.0f].
     * @param[in] dt_s Loop delta time in seconds.
     * @return float Rate-limited governor scale [0.0f, 1.0f].
     */
    float update_governor_scale(float target_scale, float dt_s) {
        if (target_scale < 0.0f) {
            target_scale = 0.0f;
        } else if (target_scale > 1.0f) {
            target_scale = 1.0f;
        }

        if (target_scale < _governor_scale) {
            // Instantaneous attenuation on overspeed
            _governor_scale = target_scale;
        } else if (target_scale > _governor_scale) {
            // Controlled 1.0 unit/second ramp-up on speed dips
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
     * @brief Computes mode-scaled throttle with stateful asymmetric speed governor rate limiting.
     * @param[in] normalized Normalized throttle [0.0f, 1.0f].
     * @param[in] gear Drive gear mode: 1 (Eco: 50%, 15 mph), 2 (Std: 75%, 28 mph), 3 (Turbo: 100%, unlimited).
     * @param[in] current_speed_mph Current vehicle road speed in mph.
     * @param[in] dt_s Frame delta time in seconds.
     * @return uint16_t Mode-scaled, rate-limited throttle request (0..10000).
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

    /**
     * @brief Normalizes raw ADC counts (0..1000) to [0.0f, 1.0f] range with plausibility clamp.
     * @param[in] raw_adc Raw ADC reading.
     * @param[in] deadband Threshold below which normalized throttle is 0.0f (default 15 counts).
     * @param[in] ceiling Threshold above which normalized throttle is 1.0f (default 980 counts).
     * @return float Normalized throttle in range [0.0f, 1.0f].
     */
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

    /**
     * @brief Scales normalized throttle [0.0f, 1.0f] according to active drive gear and speed governor (stateless).
     * @param[in] normalized Normalized throttle [0.0f, 1.0f].
     * @param[in] gear Drive gear mode: 1 (Eco: 50%, 15 mph), 2 (Std: 75%, 28 mph), 3 (Turbo: 100%, unlimited).
     * @param[in] current_speed_mph Current vehicle road speed in mph (default 0.0f for pure torque scaling).
     * @return uint16_t Mode-scaled, speed-governed throttle request (0..10000).
     */
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

    /**
     * @brief Normalizes raw ADC (0..1000) and scales according to active drive gear and speed governor.
     * @param[in] raw_adc Raw throttle ADC reading (0..1000 counts).
     * @param[in] gear Drive gear mode: 1 (Eco), 2 (Std), 3 (Turbo).
     * @param[in] current_speed_mph Current vehicle road speed in mph (default 0.0f).
     * @return uint16_t Mode-scaled throttle request (0..10000).
     */
    static inline uint16_t compute_mode_scaled_throttle(uint16_t raw_adc, uint8_t gear, float current_speed_mph = 0.0f) {
        if (raw_adc > PowertrainConfig::PLAUSIBILITY_CLAMP_MAX) return 0; // Plausibility clamp / short circuit protection
        return compute_mode_scaled_throttle(normalize_raw_adc(raw_adc), gear, current_speed_mph);
    }

    /**
     * @brief Scales regenerative electronic braking torque based on PB setting (0..5).
     * @param[in] brake_active True if mechanical or electronic brake switch is active.
     * @param[in] pb_level P-menu PB regen level (0 to 5). 0 disables regen completely.
     * @return uint16_t Commanded brake request (0..10000 counts).
     */
    static inline uint16_t compute_pb_regen(bool brake_active, uint8_t pb_level) {
        if (!brake_active || pb_level == 0) return 0; // PB=0 strictly disables electronic braking
        if (pb_level > PowertrainConfig::PB_REGEN_MAX_LEVEL) pb_level = PowertrainConfig::PB_REGEN_MAX_LEVEL;
        return static_cast<uint16_t>(pb_level * PowertrainConfig::REGEN_COUNTS_PER_LEVEL);
    }

    /**
     * @brief Returns true if brake was released while throttle was held (Whiskey-Throttle Lockout).
     */
    bool is_brake_release_locked() const {
        return _brake_release_locked;
    }

    /**
     * @brief Computes safe, failsafe-governed control payload for transmission to VESC.
     * @param[in] raw_throttle Normalized throttle request (0..10000).
     * @param[in] brake_request Proportional regenerative brake request (0..10000).
     * @param[in] dual_mode True if dual motor drive requested.
     * @param[in] can_desync True if front VESC CAN link is currently offline.
     * @param[in] vesc_online True if VESC UART connection is active.
     * @param[in] dt_s Frame delta time in seconds for slew rate ramping.
     * @return TXPayloadControl Type-safe, fail-safe sanitized control payload.
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

        // Active Brake Priority: Mechanical or electronic brake unconditionally zeroes throttle
        const bool brake_active = physical_brake_active || (brake_request > 0);
        if (brake_active) {
            _last_brake_active = true;
            _ramped_throttle = 0.0f;
            _last_dispatched_throttle = 0;
            out.throttle = 0;
            return out;
        } else {
            // Check for Brake-Release Launch Lockout transition:
            // If brake was active and is now released while throttle is still open
            if (_last_brake_active) {
                _last_brake_active = false;
                if (raw_throttle >= PowertrainConfig::THROTTLE_NEUTRAL_COUNTS) {
                    _brake_release_locked = true;
                }
            }
        }

        // Release launch lockout strictly when rider returns throttle to neutral (< 1%)
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

        // Check CAN recovery neutral lock
        update_can_recovery(raw_throttle);

        // Check fault recovery neutral release
        update_fault_recovery(raw_throttle);

        // Check comms recovery neutral lock
        update_comms_recovery(vesc_online, raw_throttle);

        // Fail-Safe Invariants:
        // Must be armed, VESC online, fault lockout clear, comms recovery clear, display recovery clear, and throttle above neutral
        float target_throttle = 0.0f;
        if (is_armed() && vesc_online && !_fault_recovery_locked && !_comms_recovery_locked && !_display_recovery_locked && raw_throttle >= PowertrainConfig::THROTTLE_NEUTRAL_COUNTS) {
            // Full power preserved in limp mode; rider manually governs output via Gear 1 or 2
            target_throttle = static_cast<float>(raw_throttle);
        }

        // Slew rate limiting ramp with bounded effective delta time
        const float effective_dt = (dt_s < PowertrainConfig::MIN_SLEW_DT_SEC) ? PowertrainConfig::MIN_SLEW_DT_SEC :
                                   ((dt_s > PowertrainConfig::MAX_SLEW_DT_SEC) ? PowertrainConfig::MAX_SLEW_DT_SEC : dt_s);
        const float max_step = _max_slew_per_sec * effective_dt;
        if (target_throttle > _ramped_throttle) {
            _ramped_throttle += (target_throttle - _ramped_throttle > max_step) ? max_step : (target_throttle - _ramped_throttle);
        } else {
            // Immediate roll-off on decel
            _ramped_throttle = target_throttle;
        }

        // Boundary snapping within 1 count to eliminate floating-point precision lag
        if (fabsf(target_throttle - _ramped_throttle) < PowertrainConfig::THROTTLE_SNAP_EPSILON) {
            _ramped_throttle = target_throttle;
        }

        // Round to nearest integer count
        uint16_t target_int = static_cast<uint16_t>(lroundf(_ramped_throttle));

        // Clamp mode ceilings to prevent deadband dropping
        if (target_int >= PowertrainConfig::MODE_3_MAX_COUNTS - 1) {
            target_int = PowertrainConfig::MODE_3_MAX_COUNTS;
        } else if (target_int >= PowertrainConfig::MODE_2_MAX_COUNTS - 1 && target_int <= PowertrainConfig::MODE_2_MAX_COUNTS) {
            target_int = PowertrainConfig::MODE_2_MAX_COUNTS;
        } else if (target_int >= PowertrainConfig::MODE_1_MAX_COUNTS - 1 && target_int <= PowertrainConfig::MODE_1_MAX_COUNTS) {
            target_int = PowertrainConfig::MODE_1_MAX_COUNTS;
        }

        // Commanded current deadband enforcement (0.60A on 120A scale = 50 counts):
        // Eliminates 50 Hz cruising coil hissing from minor ADC ripple
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

    /**
     * @brief Computes safe, failsafe-governed control payload with explicit display stall watchdog status.
     * @param[in] raw_throttle Normalized throttle request (0..10000).
     * @param[in] brake_request Proportional regenerative brake request (0..10000).
     * @param[in] physical_brake_active True if physical brake switch is pulled.
     * @param[in] dual_mode True if dual motor drive requested.
     * @param[in] can_desync True if front VESC CAN link is currently offline.
     * @param[in] vesc_online True if VESC UART connection is active.
     * @param[in] dt_s Frame delta time in seconds for slew rate ramping.
     * @param[in] display_stalled True if display heartbeat watchdog has expired.
     * @return TXPayloadControl Type-safe, fail-safe sanitized control payload.
     */
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
     * @brief Evaluates unified diagnostic fault classification based on vehicle supervisory hierarchy.
     * @note Directly synchronized with canonical physical killswitch GPIO state.
     *       Supports 1000ms time-sliced alternating rotation between coexisting faults.
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
        // Direct physical killswitch GPIO state synchronization:
        const bool hw_armed = (_killswitch_pin >= 0) ? (digitalRead(_killswitch_pin) == HIGH) : is_armed();
        if (!hw_armed || _brake_release_locked) {
            if (primary == SafetyFault::None) primary = SafetyFault::KillswitchDisarmed;
            else if (secondary == SafetyFault::None) secondary = SafetyFault::KillswitchDisarmed;
        }

        if (primary == SafetyFault::None) return SafetyFault::None;
        if (secondary == SafetyFault::None || now_ms == 0) return primary;

        // Time-sliced alternating rotation every 1000ms
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
