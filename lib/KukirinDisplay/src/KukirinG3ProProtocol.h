#pragma once

#include <Arduino.h>

/**
 * @namespace KukirinG3Pro
 * @brief Serial protocol of the Kukirin G3 Pro display (TFM13-FEIMI-1), 9600 8N1.
 *
 * The display is the master: it sends a 20-byte command frame (TXPacket) about 8 times per second, and the
 * controller must answer each one with a status frame (RXPacket). Field meanings were found by sending
 * controlled values and reading the LCD; see docs/protocol/.
 *
 * Wiring: the display's TX line is 5 V. ESP32 inputs are not 5 V tolerant, so it reaches GPIO 35 through a
 * divider (1.0 kOhm / 1.8 kOhm) with a small filter capacitor.
 */
namespace KukirinG3Pro {

    // Framing

    /** @brief First byte of a display command frame. */
    constexpr uint8_t TX_START = 0x01;

    /** @brief Length byte of a display command frame (20). */
    constexpr uint8_t TX_LEN = 0x14;

    /** @brief First byte of a controller status frame. */
    constexpr uint8_t RX_START = 0x02;

    /** @brief Length byte of a controller status frame (14). */
    constexpr uint8_t RX_LEN = 0x0E;

    // Default ESP32 pins for this build (see docs/hardware/)
    namespace Pins {
        constexpr int8_t LeftLed      = 18; /**< @brief Left turn signal MOSFET. */
        constexpr int8_t RightLed     = 19; /**< @brief Right turn signal MOSFET. */
        constexpr int8_t BrakeLed     = 21; /**< @brief Brake light MOSFET. */
        constexpr int8_t HeadlightLed = 22; /**< @brief Headlight and taillight MOSFET. */
        constexpr int8_t Horn         = 23; /**< @brief 12 V horn relay. */
        constexpr int8_t AuxLed       = 4;  /**< @brief Spare lighting output, held low. */
        constexpr int8_t Argb1        = 25; /**< @brief Spare LED strip output, held low. */
        constexpr int8_t Argb2        = 26; /**< @brief Spare LED strip output, held low. */
        constexpr int8_t Argb3        = 27; /**< @brief Spare LED strip output, held low. */
        constexpr int8_t Argb4        = 14; /**< @brief Spare LED strip output, held low. */
        constexpr int8_t ButtonDs     = 39; /**< @brief Handlebar dual/single button (input-only pin). */
        constexpr int8_t DispRx       = 35; /**< @brief UART2 RX from the display, through the 5 V divider (input-only pin). */
        constexpr int8_t DispTx       = 33; /**< @brief UART2 TX to the display. */
    }

    /** @brief Pin assignment; defaults to Pins. Set an unused pin to -1. */
    struct PinConfig {
        int8_t leftLed      = Pins::LeftLed;
        int8_t rightLed     = Pins::RightLed;
        int8_t brakeLed     = Pins::BrakeLed;
        int8_t headlightLed = Pins::HeadlightLed;
        int8_t horn         = Pins::Horn;
        int8_t auxLed       = Pins::AuxLed;
        int8_t argb1        = Pins::Argb1;
        int8_t argb2        = Pins::Argb2;
        int8_t argb3        = Pins::Argb3;
        int8_t argb4        = Pins::Argb4;
        int8_t buttonDs     = Pins::ButtonDs;
        int8_t dispRx       = Pins::DispRx;
        int8_t dispTx       = Pins::DispTx;
    };


    /**
     * @enum ErrorCode
     * @brief Error codes the display shows (names as in the G3 Pro user manual) and the status frame bit that
     *        makes it show each one, found in the register sweeps.
     */
    enum class ErrorCode : uint8_t {
        None                       = 0,   /**< @brief No error. */
        E01_RearMotorHall          = 1,   /**< @brief E-001 rear motor hall sensor: byte 3 = 0x40. */
        E02_Diagnostic2            = 2,   /**< @brief E-002: byte 3 = 0x20. Used by this firmware for "not armed" and recovery lockouts. */
        E03_MainController         = 3,   /**< @brief E-003 main controller: byte 3 = 0x10. Used for a lost VESC link. */
        E05_VoltageFault           = 5,   /**< @brief E-005 voltage: byte 3 = 0x08. */
        E06_ReceiverTimeoutCrc     = 6,   /**< @brief E-006 receive error: shown after about 2.5 s without valid replies, or forced by inverting the checksum. */
        E07_SendingError           = 7,   /**< @brief E-007 sending error: byte 4 bit 0x10. */
        E09_ControllerOverTemp     = 9,   /**< @brief E-009 controller over-temperature: byte 4 bit 0x01. */
        E11_FrontMotorHall         = 11,  /**< @brief E-011 front motor hall sensor: byte 5 = 0x40. */
        E13_SecondaryController    = 13,  /**< @brief E-013 secondary controller: byte 5 = 0x10. */
        E15_Diagnostic15           = 15,  /**< @brief E-015: byte 5 = 0x08. */
        E17_SubControllerRecv      = 17,  /**< @brief E-017 sub controller receiving: byte 6 bit 0x10. Used for a lost front VESC. */
        E31_LatchingE00            = 31   /**< @brief Byte 5 = 0x20: the display shows E-031, then E-00, and stays latched until power-cycled. */
    };

    /** @brief Mode selected on the display, command frame byte 0x04. */
    enum class DriveMode : uint8_t {
        Eco   = 0x05, /**< @brief Mode 1. */
        Std   = 0x0A, /**< @brief Mode 2. */
        Turbo = 0x0F  /**< @brief Mode 3. */
    };

    /** @brief DriveMode to mode number 1..3. */
    inline constexpr uint8_t driveModeToGear(DriveMode mode) {
        switch (mode) {
            case DriveMode::Turbo: return 3;
            case DriveMode::Std:   return 2;
            case DriveMode::Eco:
            default:               return 1;
        }
    }

    /** @brief Mode number 1..3 to DriveMode. */
    inline constexpr DriveMode gearToDriveMode(uint8_t gear) {
        switch (gear) {
            case 3:  return DriveMode::Turbo;
            case 2:  return DriveMode::Std;
            case 1:
            default: return DriveMode::Eco;
        }
    }

    /** @brief Handlebar switch bits in command frame byte 0x12. */
    enum class IndicatorFlag : uint8_t {
        None        = 0x00,
        LeftTurn    = 0x08, /**< @brief Left turn signal switch. */
        RightTurn   = 0x10, /**< @brief Right turn signal switch. */
        BrakeActive = 0x20, /**< @brief Brake lever. */
        HornActive  = 0x80  /**< @brief Horn button. */
    };

    /** @brief Function bits in command frame byte 0x05. */
    enum class FunctionFlag : uint8_t {
        None      = 0x00,
        LightsOn  = 0x20, /**< @brief Lights switched on. */
        KickStart = 0x40  /**< @brief P06 kick-start setting on. */
    };

    /** @brief Status frame byte 0x04. */
    enum class SystemStatus : uint8_t {
        Running       = 0xC0, /**< @brief Normal. */
        Braking       = 0xE0, /**< @brief Shows the brake icon. */
        DualDrive     = 0x08, /**< @brief Shows dual drive. */
        FaultSending  = 0x10, /**< @brief Shows E-007. */
        FaultOverTemp = 0x01  /**< @brief Shows E-009. */
    };

    /** @brief Status frame byte 0x02 (and the keepalive value used in byte 0x03). */
    enum class StatusType : uint8_t {
        OdoLifetime   = 0x01, /**< @brief Normal status, total odometer. */
        TripOdometer  = 0x02, /**< @brief Trip odometer. */
        KeepAlive     = 0x80  /**< @brief Keepalive flag, sent every 50 frames (see isKeepaliveFrame). */
    };

    /** @brief Bit flags for reporting which handlebar switches changed. */
    enum class SwitchMask : uint8_t {
        None          = 0x00,
        Brake         = 0x01,
        Lights        = 0x02,
        LeftTurn      = 0x04,
        RightTurn     = 0x08,
        HazardLights  = 0x10,
        Horn          = 0x20,
        Dual          = 0x40,
        All           = 0x7F,
        Uninitialized = 0xFF
    };

    /** @brief speedRaw value sent when stopped; the display shows 0 at this value and above. */
    constexpr uint16_t SPEED_RAW_STATIONARY = 3500;

    /** @brief Drop a partial frame after this much silence. */
    constexpr uint32_t FRAMING_TIMEOUT_MS         = 50;

    /** @brief Most bytes read per update() call. */
    constexpr uint16_t MAX_SERIAL_READ_ITERATIONS = 128;

    /** @brief Speed profile word seen with P07 at 100 %. */
    constexpr uint16_t PROFILE_OPEN               = 0x0C64;

    /** @brief Raw throttle above this (full scale 1000) is treated as a wiring fault. */
    constexpr uint16_t PLAUSIBILITY_CLAMP_MAX     = 1050;

    // Configuration

    /**
     * @struct ThrottleConfig
     * @brief Thresholds for the raw throttle value (0..1000) sent by the display.
     */
    struct ThrottleConfig {
        uint16_t idleThreshold        = 15;    /**< @brief At or below this, normalized throttle is 0. */
        uint16_t maxThreshold         = 980;   /**< @brief At or above this, normalized throttle is 1. */
        uint16_t deadbandMin          = 20;    /**< @brief Unused. */
        uint16_t deadbandEngage       = 25;    /**< @brief Idle switch: throttle starts counting above this... */
        uint16_t deadbandDisengage    = 15;    /**< @brief ...and drops back to 0 below this. */
        uint16_t ceilingMax           = 980;   /**< @brief Unused. */
        uint16_t ceilingEngage        = 975;   /**< @brief Full-throttle switch: snaps to full scale at or above this... */
        uint16_t ceilingRelease       = 965;   /**< @brief ...and releases below this. */
        uint16_t rawMax               = 1000;  /**< @brief Full scale of the raw value. */
        uint16_t rawAdcClampMax       = PLAUSIBILITY_CLAMP_MAX; /**< @brief Raw values above this are clamped to full scale by updateThrottle(). */
        uint16_t maxDeltaPerTick      = 350;   /**< @brief Unused (VESCSafetySupervisor limits the rise). */
        uint16_t maxDecelDeltaPerTick = 500;   /**< @brief Unused. */
    };

    /**
     * @struct BridgeTimingConfig
     * @brief Timing and filtering settings. Frame counts refer to display frames, about 125 ms apart.
     */
    struct BridgeTimingConfig {
        uint32_t framingTimeoutMs     = 50;   /**< @brief Drop a partial frame after this much silence. */
        uint32_t deadmanTimeoutMs     = 500;  /**< @brief The display link counts as stalled after this long without a valid frame. */
        uint32_t buttonDebounceMs     = 20;   /**< @brief Button debounce time. */
        uint32_t blinkPeriodMs        = 400;  /**< @brief Turn signal half-period (400 ms on, 400 ms off). */
        uint32_t brakeStrobePeriodMs  = 250;  /**< @brief Brake light flash half-period while braking. */
        uint32_t hazardWindowMs       = 800;  /**< @brief Hazards turn on when the same turn switch is pushed twice within this time... */
        uint32_t hazardMinTapGapMs    = 100;  /**< @brief ...but not faster than this, which rejects switch bounce. */
        uint32_t hornMaxContinuousMs  = 3000; /**< @brief Horn turns off after this long held, to protect the relay and horn. */
        uint32_t hornRearmQuietMs     = 100;  /**< @brief After a cutoff, the button must be released this long before the horn works again. */
        uint32_t faultCyclePeriodMs   = 1000; /**< @brief Unused (the firmware uses PowertrainConfig::FAULT_CYCLE_PERIOD_MS). */
        float    walkingSpeedLimitMph = 3.0f; /**< @brief P-menu changes made above this speed wait until the vehicle slows below it. */
        uint8_t  driveModeQuorum      = 2;    /**< @brief Frames a new mode must persist before it is used. */
        uint8_t  functionQuorum       = 2;    /**< @brief Frames a lights or kick-start change must persist. */
        uint8_t  pmenuQuorum          = 3;    /**< @brief Frames a P-menu change must persist. */
        uint8_t  brakeReleaseQuorum   = 1;    /**< @brief Frames the brake must read released before it counts as released (1 = next frame). */
        uint8_t  turnSignalQuorum     = 2;    /**< @brief Frames a turn switch change must persist. */
    };

    /** @brief Older name. */
    using InputFilterConfig = BridgeTimingConfig;

    // Frames

    /**
     * @struct TXPacket
     * @brief Command frame from the display, 20 bytes.
     */
    struct __attribute__((packed)) TXPacket {
        uint8_t startMarker;      /**< @brief 0x01. Offset 0x00. */
        uint8_t packetLength;     /**< @brief 0x14. Offset 0x01. */
        uint8_t commandType;      /**< @brief Always 0x01 in captures. Offset 0x02. */
        uint8_t subCommand;       /**< @brief Meaning unknown. Offset 0x03. */
        uint8_t driveMode;        /**< @brief Mode: 0x05, 0x0A or 0x0F. Offset 0x04. */
        uint8_t functionBitmask;  /**< @brief FunctionFlag bits. Offset 0x05. */
        uint8_t poleCount_L;      /**< @brief P04 magnet count, low byte. Offset 0x06. */
        uint8_t poleCount_H;      /**< @brief P04, high byte. Offset 0x07. */
        uint8_t wheelCirc_L;      /**< @brief P03 wheel diameter in 0.1 in, low byte. Offset 0x08. */
        uint8_t wheelCirc_H;      /**< @brief P03, high byte. Offset 0x09. */
        uint8_t regenAccelByte;   /**< @brief High nibble PB regen level 0-5, low nibble PA acceleration level 1-5. Offset 0x0A. */
        uint8_t reserved_0x0B;    /**< @brief Meaning unknown. Offset 0x0B. */
        uint8_t speedProfile_L;   /**< @brief P07 speed limit percent. Offset 0x0C. */
        uint8_t speedProfile_H;   /**< @brief Speed profile high byte. Offset 0x0D. */
        uint8_t batteryConfig_H;  /**< @brief P02 low-voltage cutoff in 0.1 V, high byte (big-endian). Offset 0x0E. */
        uint8_t batteryConfig_L;  /**< @brief P02, low byte. Offset 0x0F. */
        uint8_t throttle_H;       /**< @brief Raw throttle 0..1000, high byte (big-endian). Offset 0x10. */
        uint8_t throttle_L;       /**< @brief Raw throttle, low byte. Offset 0x11. */
        uint8_t indicator_L;      /**< @brief IndicatorFlag bits. Offset 0x12. */
        uint8_t checksum;         /**< @brief XOR of bytes 0..18. Offset 0x13. */
    };

    /**
     * @struct RXPacket
     * @brief Status frame sent to the display in reply to each command frame (16 bytes on the wire,
     *        although the length byte says 14).
     */
    struct __attribute__((packed)) RXPacket {
        uint8_t startMarker;       /**< @brief 0x02. Offset 0x00. */
        uint8_t packetLength;      /**< @brief 0x0E. Offset 0x01. */
        uint8_t statusType;        /**< @brief StatusType. Offset 0x02. */
        uint8_t statusFlag;        /**< @brief 0x00 normal, 0x80 keepalive, or an error code bit. Offset 0x03. */
        uint8_t systemStatus;      /**< @brief SystemStatus bits. Offset 0x04. */
        uint8_t speedField[3];     /**< @brief Byte 5: error code bits; byte 6: dual drive and E-017 bits; byte 7 unused. Offset 0x05. */
        uint8_t speedRaw_H;        /**< @brief speedRaw high byte (see the speed functions below). Offset 0x08. */
        uint8_t speedRaw_L;        /**< @brief speedRaw low byte. Offset 0x09. */
        uint8_t currentField[2];   /**< @brief Thought to be motor current (not confirmed); this firmware sends 0. Offset 0x0A. */
        uint8_t calculatedStatus;  /**< @brief XOR of bytes 0..11. Offset 0x0C. */
        uint8_t unknown_0x0D;      /**< @brief 0x00. Offset 0x0D. */
        uint8_t echo_H;            /**< @brief Repeats the start byte, 0x02. Offset 0x0E. */
        uint8_t echo_L;            /**< @brief Repeats the length byte, 0x0E. Offset 0x0F. */
    };

    /** @brief P-menu settings decoded from a command frame. */
    struct DecodedSettings {
        uint8_t regenLevel = 3;               /**< @brief PB regen level 0..5. */
        uint8_t accelLevel = 3;               /**< @brief PA acceleration level 1..5. */
        uint8_t batteryVolts = 52;            /**< @brief Nominal pack voltage guessed from the P02 cutoff. */
        float cutoffVolts = 46.0f;            /**< @brief P02 low-voltage cutoff, V. */
        float wheelInches = 10.0f;            /**< @brief P03 wheel diameter, in. */
        uint16_t poleCount = 30;              /**< @brief P04 magnet count (30 = 15 pole pairs). Does not affect the speed shown. */
        bool kickStart = false;               /**< @brief P06 kick-start. */
        uint8_t speedLimitPercent = 100;      /**< @brief P07 speed limit percent. */
        uint16_t speedProfile = PROFILE_OPEN; /**< @brief Bytes 0x0C-0x0D as a word. */
        uint8_t checksum = 0x00;              /**< @brief Checksum byte of the frame. */
    };

    // Checksums and validation

    /** @brief XOR of command frame bytes 0..18. */
    inline uint8_t calculateTXChecksum(const TXPacket& tx) {
        const uint8_t* raw = reinterpret_cast<const uint8_t*>(&tx);
        uint8_t cs = 0;
        for (size_t i = 0; i < 19; i++) {
            cs ^= raw[i];
        }
        return cs;
    }

    /** @brief XOR of status frame bytes 0..11. */
    inline uint8_t calculateRXChecksum(const RXPacket& rx) {
        const uint8_t* raw = reinterpret_cast<const uint8_t*>(&rx);
        uint8_t checksum = 0;
        for (size_t i = 0; i < 12; i++) {
            checksum ^= raw[i];
        }
        return checksum;
    }

    /** @brief True if the buffer is long enough and starts with 0x01 0x14. */
    inline bool isValidTXPacket(const uint8_t* buffer, size_t length) {
        if (buffer == nullptr || length < sizeof(TXPacket)) return false;
        return (buffer[0] == TX_START && buffer[1] == TX_LEN);
    }

    /** @brief True if the checksum byte matches. */
    inline bool verifyTXChecksum(const TXPacket& tx) {
        return calculateTXChecksum(tx) == tx.checksum;
    }

    /**
     * @brief Range checks on a frame that already passed the checksum. A frame with a valid checksum but
     *        impossible values (unknown mode, both turn signals on, out-of-range throttle or settings) is
     *        rejected instead of acted on.
     */
    inline bool isPlausibleTXPacket(const TXPacket& tx) {
        if (tx.startMarker != TX_START || tx.packetLength != TX_LEN) return false;
        if (tx.commandType != 0x01) return false;

        if (tx.driveMode != static_cast<uint8_t>(DriveMode::Eco) &&
            tx.driveMode != static_cast<uint8_t>(DriveMode::Std) &&
            tx.driveMode != static_cast<uint8_t>(DriveMode::Turbo)) {
            return false;
        }

        // P04 magnets 1..100
        if (tx.poleCount_L < 1 || tx.poleCount_L > 100) return false;

        // P03 5.0..20.0 in
        if (tx.wheelCirc_L < 50 || tx.wheelCirc_L > 200) return false;

        const uint8_t regenLvl = static_cast<uint8_t>((tx.regenAccelByte >> 4) & 0x0F);
        const uint8_t accelLvl = static_cast<uint8_t>(tx.regenAccelByte & 0x0F);
        if (regenLvl > 5) return false;
        if (accelLvl < 1 || accelLvl > 5) return false;

        // P07 percent
        if (tx.speedProfile_L > 100) return false;

        // P02 cutoff 24.0..80.0 V
        const uint16_t battCutoff = static_cast<uint16_t>((static_cast<uint16_t>(tx.batteryConfig_H) << 8) | tx.batteryConfig_L);
        if (battCutoff < 240 || battCutoff > 800) return false;

        const uint16_t rawThrot = static_cast<uint16_t>((static_cast<uint16_t>(tx.throttle_H) << 8) | tx.throttle_L);
        if (rawThrot > 1050) return false;

        // The turn switch is a rocker: left and right cannot both be on
        if ((tx.indicator_L & static_cast<uint8_t>(IndicatorFlag::LeftTurn)) &&
            (tx.indicator_L & static_cast<uint8_t>(IndicatorFlag::RightTurn))) {
            return false;
        }

        return true;
    }

    /** @brief Start bytes, length and checksum of a received command frame. */
    inline bool verifyTXPacket(const uint8_t* buffer, size_t length) {
        if (!isValidTXPacket(buffer, length)) return false;
        return verifyTXChecksum(*reinterpret_cast<const TXPacket*>(buffer));
    }

    /**
     * @brief After a bad frame, finds the next 0x01 0x14 (or a trailing 0x01) after the first byte and moves it
     *        to the start of the buffer, so a real frame that began inside the bad one is not lost.
     * @return Bytes left in the buffer; 0 if no start was found.
     */
    inline size_t resyncTXBuffer(uint8_t* buf, size_t currentLen) {
        if (buf == nullptr || currentLen <= 1) return 0;
        size_t nextStart = 0;
        for (size_t i = 1; i < currentLen; i++) {
            if (buf[i] == TX_START) {
                if (i + 1 < currentLen) {
                    if (buf[i + 1] == TX_LEN) {
                        nextStart = i;
                        break;
                    }
                } else {
                    nextStart = i;
                    break;
                }
            }
        }
        if (nextStart > 0) {
            const size_t rem = currentLen - nextStart;
            memmove(buf, &buf[nextStart], rem);
            return rem;
        }
        return 0;
    }

    /**
     * @struct InputFilter
     * @brief Filters the inputs read from command frames (one call per frame, about 125 ms apart).
     *
     * - Throttle: idle and full-throttle switches with hysteresis; brake forces 0.
     * - Brake: on at the first frame that shows it; off after brakeReleaseQuorum frames (1 = next frame).
     * - Turn signals, mode and function bits: a change must repeat for N frames before it is used.
     * - P-menu: a change must repeat for 3 frames; a change made while moving faster than walking speed is held
     *   until the vehicle slows down with the throttle released, so settings never change under load.
     * - Hazards: two pushes of the same turn switch 100-800 ms apart.
     */
    struct InputFilter {
        // Throttle state
        uint16_t filteredThrottle = 0;
        bool     throttleInitialized = false;
        bool     throttleEngaged = false;
        bool     throttleFullEngaged = false;

        // Brake state
        bool     filteredBrake = false;
        uint8_t  brakeReleaseCounter = 0;
        uint8_t  brakeReleaseQuorum = 1;
        uint32_t deadmanTimeoutMs = 500;

        // Turn signal quorum state
        bool    rawLeftTurnPrev = false;
        bool    rawRightTurnPrev = false;
        uint8_t leftQuorumCount = 0;
        uint8_t rightQuorumCount = 0;
        bool    filteredLeft = false;
        bool    filteredRight = false;

        // Hazard state
        bool     hazardActive = false;
        bool     rawLeftHazardPrev = false;
        bool     rawRightHazardPrev = false;
        uint32_t lastLeftRisingMs = 0;
        uint32_t lastRightRisingMs = 0;

        // Mode filter state
        DriveMode rawDriveModePrev = DriveMode::Eco;
        DriveMode filteredDriveMode = DriveMode::Eco;
        uint8_t   driveModeQuorumCount = 0;

        // Function bits filter state
        uint8_t rawFunctionPrev = static_cast<uint8_t>(FunctionFlag::None);
        uint8_t filteredFunction = static_cast<uint8_t>(FunctionFlag::None);
        uint8_t functionQuorumCount = 0;

        // P-menu filter and held-change state
        DecodedSettings pendingSettings;
        DecodedSettings stableSettings;
        DecodedSettings stagedSettings;
        uint8_t         pmenuQuorumCount = 0;
        bool            pmenuInitialized = false;
        bool            pmenuDesyncPending = false;

        /**
         * @brief Throttle with idle and full-throttle hysteresis. Returns 0 while the brake is on. Values above
         *        rawAdcClampMax are treated as full scale here; the frame plausibility check rejects them earlier.
         * @return Filtered raw throttle, 0..1000.
         */
        uint16_t updateThrottle(uint16_t rawAdc, bool brakeActive = false, const ThrottleConfig& cfg = ThrottleConfig()) {
            if (brakeActive) {
                filteredThrottle = 0;
                throttleEngaged = false;
                throttleFullEngaged = false;
                return 0;
            }
            if (!throttleInitialized) {
                filteredThrottle = (rawAdc <= cfg.rawMax) ? rawAdc : 0;
                throttleEngaged = (filteredThrottle > cfg.deadbandEngage);
                throttleFullEngaged = (filteredThrottle >= cfg.ceilingEngage);
                if (!throttleEngaged) filteredThrottle = 0;
                if (throttleFullEngaged) filteredThrottle = cfg.rawMax;
                throttleInitialized = true;
                return filteredThrottle;
            }
            if (rawAdc > cfg.rawAdcClampMax) rawAdc = cfg.rawMax;

            // No rate limit here: VESCSafetySupervisor limits how fast the command rises
            filteredThrottle = rawAdc;

            // Idle switch with hysteresis
            if (throttleEngaged) {
                if (filteredThrottle < cfg.deadbandDisengage) {
                    throttleEngaged = false;
                    filteredThrottle = 0;
                }
            } else {
                if (filteredThrottle > cfg.deadbandEngage) {
                    throttleEngaged = true;
                } else {
                    filteredThrottle = 0;
                }
            }

            // Full-throttle switch with hysteresis
            if (throttleFullEngaged) {
                if (filteredThrottle < cfg.ceilingRelease) {
                    throttleFullEngaged = false;
                } else {
                    filteredThrottle = cfg.rawMax;
                }
            } else {
                if (filteredThrottle >= cfg.ceilingEngage) {
                    throttleFullEngaged = true;
                    filteredThrottle = cfg.rawMax;
                }
            }

            return filteredThrottle;
        }

        /** @brief Brake on immediately; off after quorumFrames frames without it. */
        bool updateBrake(bool rawBrake, uint8_t quorumFrames = 1) {
            if (rawBrake) {
                filteredBrake = true;
                brakeReleaseCounter = 0;
            } else {
                if (filteredBrake) {
                    brakeReleaseCounter++;
                    if (brakeReleaseCounter >= quorumFrames) {
                        filteredBrake = false;
                        brakeReleaseCounter = 0;
                    }
                }
            }
            return filteredBrake;
        }

        /** @brief Each turn switch change must repeat for quorumFrames frames. */
        void updateTurnSignals(bool rawLeft, bool rawRight, bool& outLeft, bool& outRight, uint8_t quorumFrames = 2) {
            if (rawLeft == rawLeftTurnPrev) {
                if (leftQuorumCount < quorumFrames) {
                    leftQuorumCount++;
                    if (leftQuorumCount >= quorumFrames) {
                        filteredLeft = rawLeft;
                    }
                }
            } else {
                rawLeftTurnPrev = rawLeft;
                leftQuorumCount = 1;
            }

            if (rawRight == rawRightTurnPrev) {
                if (rightQuorumCount < quorumFrames) {
                    rightQuorumCount++;
                    if (rightQuorumCount >= quorumFrames) {
                        filteredRight = rawRight;
                    }
                }
            } else {
                rawRightTurnPrev = rawRight;
                rightQuorumCount = 1;
            }

            outLeft = filteredLeft;
            outRight = filteredRight;
        }

        /** @brief A mode change must repeat for quorumFrames frames. */
        DriveMode updateDriveMode(DriveMode rawMode, uint8_t quorumFrames = 2) {
            if (rawMode == rawDriveModePrev) {
                if (driveModeQuorumCount < quorumFrames) {
                    driveModeQuorumCount++;
                    if (driveModeQuorumCount >= quorumFrames) {
                        filteredDriveMode = rawMode;
                    }
                }
            } else {
                rawDriveModePrev = rawMode;
                driveModeQuorumCount = 1;
            }
            return filteredDriveMode;
        }

        /** @brief A function bit change must repeat for quorumFrames frames. */
        uint8_t updateFunctions(uint8_t rawFunc, uint8_t quorumFrames = 2) {
            if (rawFunc == rawFunctionPrev) {
                if (functionQuorumCount < quorumFrames) {
                    functionQuorumCount++;
                    if (functionQuorumCount >= quorumFrames) {
                        filteredFunction = rawFunc;
                    }
                }
            } else {
                rawFunctionPrev = rawFunc;
                functionQuorumCount = 1;
            }
            return filteredFunction;
        }

        /** @brief Hazards on after two pushes of the same turn switch minGapMs..maxGapMs apart. */
        bool updateHazard(bool rawLeft, bool rawRight, uint32_t nowMs, uint32_t minGapMs = 100, uint32_t maxGapMs = 800) {
            const bool leftRising = (!rawLeftHazardPrev && rawLeft);
            const bool rightRising = (!rawRightHazardPrev && rawRight);
            rawLeftHazardPrev = rawLeft;
            rawRightHazardPrev = rawRight;

            if (leftRising) {
                lastRightRisingMs = 0;
                if (lastLeftRisingMs > 0) {
                    const uint32_t gap = nowMs - lastLeftRisingMs;
                    if (gap >= minGapMs && gap <= maxGapMs) {
                        hazardActive = true;
                        lastLeftRisingMs = 0;
                    } else {
                        lastLeftRisingMs = nowMs;
                    }
                } else {
                    lastLeftRisingMs = nowMs;
                }
            }
            if (rightRising) {
                lastLeftRisingMs = 0;
                if (lastRightRisingMs > 0) {
                    const uint32_t gap = nowMs - lastRightRisingMs;
                    if (gap >= minGapMs && gap <= maxGapMs) {
                        hazardActive = true;
                        lastRightRisingMs = 0;
                    } else {
                        lastRightRisingMs = nowMs;
                    }
                } else {
                    lastRightRisingMs = nowMs;
                }
            }
            return hazardActive;
        }

        void resetHazard() {
            hazardActive = false;
            filteredLeft = false;
            filteredRight = false;
            leftQuorumCount = 0;
            rightQuorumCount = 0;
            rawLeftTurnPrev = false;
            rawRightTurnPrev = false;
        }

        /** @brief Turn signals and hazards together; centering the switch turns both off. */
        void updateTurnAndHazard(bool rawLeft, bool rawRight, uint32_t nowMs,
                                 bool& outLeft, bool& outRight, bool& outHazard,
                                 uint8_t quorumFrames = 2,
                                 uint32_t minGapMs = 100, uint32_t maxGapMs = 800) {
            const bool hazard = updateHazard(rawLeft, rawRight, nowMs, minGapMs, maxGapMs);

            if (!rawLeft && !rawRight) {
                resetHazard();
                outLeft = false;
                outRight = false;
                outHazard = false;
                return;
            }

            updateTurnSignals(rawLeft, rawRight, outLeft, outRight, quorumFrames);
            outHazard = hazard;
        }

        /** @brief Discards half-counted changes after a bad frame or a stall. */
        void resetPending() {
            leftQuorumCount = 0;
            rightQuorumCount = 0;
            rawLeftTurnPrev = filteredLeft;
            rawRightTurnPrev = filteredRight;
            rawLeftHazardPrev = false;
            rawRightHazardPrev = false;
            // Brake and throttle state are kept
            pmenuQuorumCount = 0;
            driveModeQuorumCount = 0;
            functionQuorumCount = 0;
            rawDriveModePrev = filteredDriveMode;
            rawFunctionPrev = filteredFunction;
            if (pmenuInitialized && !pmenuDesyncPending) {
                pendingSettings = stableSettings;
            }
        }

        /** @brief Sets the brake release frame count and stall timeout. */
        void resetTiming(uint8_t brakeReleaseQuorum = 1, uint32_t deadmanTimeoutMs = 500) {
            this->brakeReleaseQuorum = brakeReleaseQuorum;
            this->deadmanTimeoutMs = deadmanTimeoutMs;
            brakeReleaseCounter = 0;
        }

        /**
         * @brief P-menu filter. A change must repeat for quorumFrames frames. If the vehicle is faster than
         *        walkingSpeedLimitMph, the change is held and applied once it slows down with the throttle released.
         * @param[in] rawSettings Settings decoded from this frame.
         * @param[out] outSettings Settings in effect.
         * @param[in] currentSpeedMph Vehicle speed.
         * @param[in] throttleNeutral Throttle released.
         * @return True when the settings in effect changed (the first call always returns true).
         */
        bool updatePMenu(const DecodedSettings& rawSettings, DecodedSettings& outSettings,
                         float currentSpeedMph = 0.0f, bool throttleNeutral = true,
                         float walkingSpeedLimitMph = 3.0f, uint8_t quorumFrames = 3) {
            if (!pmenuInitialized) {
                stableSettings = rawSettings;
                pendingSettings = rawSettings;
                stagedSettings = rawSettings;
                pmenuQuorumCount = quorumFrames;
                pmenuInitialized = true;
                pmenuDesyncPending = false;
                outSettings = stableSettings;
                return true;
            }

            // A held change is applied once slow with the throttle released
            if (pmenuDesyncPending && currentSpeedMph <= walkingSpeedLimitMph && throttleNeutral) {
                stableSettings = stagedSettings;
                pmenuDesyncPending = false;
                outSettings = stableSettings;
                return true;
            }

            const bool matchesPending = (
                rawSettings.wheelInches == pendingSettings.wheelInches &&
                rawSettings.batteryVolts == pendingSettings.batteryVolts &&
                rawSettings.cutoffVolts == pendingSettings.cutoffVolts &&
                rawSettings.poleCount == pendingSettings.poleCount &&
                rawSettings.kickStart == pendingSettings.kickStart &&
                rawSettings.regenLevel == pendingSettings.regenLevel &&
                rawSettings.accelLevel == pendingSettings.accelLevel &&
                rawSettings.speedLimitPercent == pendingSettings.speedLimitPercent &&
                rawSettings.speedProfile == pendingSettings.speedProfile
            );

            if (matchesPending) {
                if (pmenuQuorumCount < quorumFrames) {
                    pmenuQuorumCount++;
                    if (pmenuQuorumCount >= quorumFrames) {
                        const bool changedFromStable = (
                            pendingSettings.wheelInches != stableSettings.wheelInches ||
                            pendingSettings.batteryVolts != stableSettings.batteryVolts ||
                            pendingSettings.cutoffVolts != stableSettings.cutoffVolts ||
                            pendingSettings.poleCount != stableSettings.poleCount ||
                            pendingSettings.kickStart != stableSettings.kickStart ||
                            pendingSettings.regenLevel != stableSettings.regenLevel ||
                            pendingSettings.accelLevel != stableSettings.accelLevel ||
                            pendingSettings.speedLimitPercent != stableSettings.speedLimitPercent ||
                            pendingSettings.speedProfile != stableSettings.speedProfile
                        );
                        if (pmenuDesyncPending && !changedFromStable) {
                            // Changed back while moving: drop the held change
                            pmenuDesyncPending = false;
                            stagedSettings = stableSettings;
                        } else if (changedFromStable) {
                            if (currentSpeedMph > walkingSpeedLimitMph) {
                                // Moving: hold the change, keep the current settings
                                stagedSettings = pendingSettings;
                                pmenuDesyncPending = true;
                                outSettings = stableSettings;
                                return false;
                            } else {
                                stableSettings = pendingSettings;
                                stagedSettings = pendingSettings;
                                pmenuDesyncPending = false;
                                outSettings = stableSettings;
                                return true;
                            }
                        }
                    }
                }
            } else {
                pendingSettings = rawSettings;
                pmenuQuorumCount = 1;
            }

            outSettings = stableSettings;
            return false;
        }
    };

    /**
     * @struct AccessoryEngine
     * @brief Lights, turn signals, brake light and horn outputs.
     *
     * With the lights on, the turn and brake lamps stay lit and flash by turning off; with the lights off they
     * flash by turning on. A flash starts immediately on the first press. The horn cuts off after
     * hornMaxContinuousMs held. The handlebar button toggles dual drive.
     */
    struct AccessoryEngine {
        // Output states
        bool leftLed = false;
        bool rightLed = false;
        bool brakeLed = false;
        bool headlightLed = false;
        bool hornRelay = false;

        // Flash state
        bool blinkState = false;
        bool lastTurnActive = false;
        uint32_t lastBlinkMs = 0;

        bool brakeStrobeState = false;
        bool lastBrakeActive = false;
        uint32_t lastBrakeStrobeMs = 0;

        // Horn state
        bool hornArmed = true;
        bool hornActiveState = false;
        bool hornCutoffTriggered = false;
        bool lastHornRaw = false;
        uint32_t hornStartMs = 0;
        uint32_t hornReleaseMs = 0;

        // Dual/single button state
        bool btnLastRaw = false;
        bool btnStableState = false;
        uint32_t btnLastChangeMs = 0;
        bool dualMode = true;

        /** @brief Computes the output states for this moment; applyHardwarePins() writes them. */
        void update(uint32_t nowMs, bool lights, bool brake, bool left, bool right,
                    bool hazard, bool horn, const BridgeTimingConfig& timing = BridgeTimingConfig{}) {
            headlightLed = lights;

            // Brake light flashes while braking
            if (!lastBrakeActive && brake) {
                brakeStrobeState = !lights; // first flash state is the opposite of the resting state
                lastBrakeStrobeMs = nowMs;
            }
            lastBrakeActive = brake;

            if (brake && (nowMs - lastBrakeStrobeMs >= timing.brakeStrobePeriodMs)) {
                lastBrakeStrobeMs = nowMs;
                brakeStrobeState = !brakeStrobeState;
            }
            brakeLed = brake ? brakeStrobeState : lights;

            // Turn signals and hazards
            const bool leftActive = hazard || left;
            const bool rightActive = hazard || right;
            const bool anyTurnActive = leftActive || rightActive;

            if (!lastTurnActive && anyTurnActive) {
                blinkState = !lights;
                lastBlinkMs = nowMs;
            }
            lastTurnActive = anyTurnActive;

            if (anyTurnActive && (nowMs - lastBlinkMs >= timing.blinkPeriodMs)) {
                lastBlinkMs = nowMs;
                blinkState = !blinkState;
            }

            leftLed  = leftActive  ? blinkState : lights;
            rightLed = rightActive ? blinkState : lights;

            // Horn: cut off after hornMaxContinuousMs; works again after a release of hornRearmQuietMs
            if (lastHornRaw && !horn) {
                hornReleaseMs = nowMs;
            }
            lastHornRaw = horn;

            if (horn) {
                if (hornArmed) {
                    if (!hornActiveState) {
                        hornActiveState = true;
                        hornStartMs = nowMs;
                    } else if (nowMs - hornStartMs >= timing.hornMaxContinuousMs) {
                        hornArmed = false;
                        hornCutoffTriggered = true;
                    }
                }
            } else {
                hornActiveState = false;
                if (!hornArmed && (nowMs - hornReleaseMs >= timing.hornRearmQuietMs)) {
                    hornArmed = true;
                    hornCutoffTriggered = false;
                }
            }
            hornRelay = horn && hornArmed;
        }

        /** @brief Debounced button; each press toggles dualMode. @return True on a press. */
        bool processButton(uint32_t nowMs, bool rawPressed, uint32_t debounceMs = 20) {
            bool pressedEdge = false;
            if (rawPressed != btnLastRaw) {
                btnLastRaw = rawPressed;
                btnLastChangeMs = nowMs;
            }
            if ((nowMs - btnLastChangeMs) >= debounceMs) {
                if (rawPressed != btnStableState) {
                    btnStableState = rawPressed;
                    if (btnStableState) {
                        dualMode = !dualMode;
                        pressedEdge = true;
                    }
                }
            }
            return pressedEdge;
        }

        /** @brief Writes the output states to the pins. */
        void applyHardwarePins(const PinConfig& pins = PinConfig{}) const {
            if (pins.headlightLed >= 0) digitalWrite(pins.headlightLed, headlightLed ? HIGH : LOW);
            if (pins.brakeLed >= 0)     digitalWrite(pins.brakeLed,     brakeLed ? HIGH : LOW);
            if (pins.leftLed >= 0)      digitalWrite(pins.leftLed,      leftLed ? HIGH : LOW);
            if (pins.rightLed >= 0)     digitalWrite(pins.rightLed,     rightLed ? HIGH : LOW);
            if (pins.horn >= 0)         digitalWrite(pins.horn,         hornRelay ? HIGH : LOW);
        }

        /** @brief Sets each output low before making it an output, so nothing turns on briefly at boot. */
        static void initGPIO(const PinConfig& pins = PinConfig{}) {
            if (pins.leftLed >= 0)      { digitalWrite(pins.leftLed, LOW);      pinMode(pins.leftLed, OUTPUT); }
            if (pins.rightLed >= 0)     { digitalWrite(pins.rightLed, LOW);     pinMode(pins.rightLed, OUTPUT); }
            if (pins.brakeLed >= 0)     { digitalWrite(pins.brakeLed, LOW);     pinMode(pins.brakeLed, OUTPUT); }
            if (pins.headlightLed >= 0) { digitalWrite(pins.headlightLed, LOW); pinMode(pins.headlightLed, OUTPUT); }
            if (pins.horn >= 0)         { digitalWrite(pins.horn, LOW);          pinMode(pins.horn, OUTPUT); }

            // Unused outputs held low
            if (pins.auxLed >= 0)       { digitalWrite(pins.auxLed, LOW);       pinMode(pins.auxLed, OUTPUT); }
            if (pins.argb1 >= 0)        { digitalWrite(pins.argb1, LOW);        pinMode(pins.argb1, OUTPUT); }
            if (pins.argb2 >= 0)        { digitalWrite(pins.argb2, LOW);        pinMode(pins.argb2, OUTPUT); }
            if (pins.argb3 >= 0)        { digitalWrite(pins.argb3, LOW);        pinMode(pins.argb3, OUTPUT); }
            if (pins.argb4 >= 0)        { digitalWrite(pins.argb4, LOW);        pinMode(pins.argb4, OUTPUT); }

            if (pins.buttonDs >= 0)     { pinMode(pins.buttonDs, INPUT); }
        }
    };

    // Field access

    /** @brief Raw throttle, 0..1000. */
    inline uint16_t extractRawThrottle(const TXPacket& tx) {
        return static_cast<uint16_t>((static_cast<uint16_t>(tx.throttle_H) << 8) | tx.throttle_L);
    }

    /** @brief Raw throttle to 0..1 between idleThreshold and maxThreshold. */
    inline float normalizeThrottleAdc(uint16_t rawAdc, const ThrottleConfig& cfg = ThrottleConfig()) {
        if (rawAdc <= cfg.idleThreshold) return 0.0f;
        if (rawAdc >= cfg.maxThreshold) return 1.0f;
        return static_cast<float>(rawAdc - cfg.idleThreshold) / static_cast<float>(cfg.maxThreshold - cfg.idleThreshold);
    }

    /** @brief Same as normalizeThrottleAdc(). */
    inline float normalizeThrottle(uint16_t rawAdc, const ThrottleConfig& cfg = ThrottleConfig{}) {
        return normalizeThrottleAdc(rawAdc, cfg);
    }

    /** @brief Brake lever bit. */
    inline bool isBrakeActive(const TXPacket& tx) {
        return (tx.indicator_L & static_cast<uint8_t>(IndicatorFlag::BrakeActive)) != 0;
    }

    /** @brief Lights bit. */
    inline bool isLightsActive(const TXPacket& tx) {
        return (tx.functionBitmask & static_cast<uint8_t>(FunctionFlag::LightsOn)) != 0;
    }

    /** @brief Left turn bit. */
    inline bool isLeftTurnActive(const TXPacket& tx) {
        return (tx.indicator_L & static_cast<uint8_t>(IndicatorFlag::LeftTurn)) != 0;
    }

    /** @brief Right turn bit. */
    inline bool isRightTurnActive(const TXPacket& tx) {
        return (tx.indicator_L & static_cast<uint8_t>(IndicatorFlag::RightTurn)) != 0;
    }

    /** @brief Horn bit. */
    inline bool isHornActive(const TXPacket& tx) {
        return (tx.indicator_L & static_cast<uint8_t>(IndicatorFlag::HornActive)) != 0;
    }

    /** @brief Mode; unknown values read as mode 1. */
    inline DriveMode extractDriveMode(const TXPacket& tx) {
        if (tx.driveMode == static_cast<uint8_t>(DriveMode::Turbo)) return DriveMode::Turbo;
        if (tx.driveMode == static_cast<uint8_t>(DriveMode::Std))   return DriveMode::Std;
        return DriveMode::Eco;
    }

    /** @brief Mode number 1..3. */
    inline uint8_t extractGearLevel(const TXPacket& tx) {
        return driveModeToGear(extractDriveMode(tx));
    }

    // P-menu

    /** @brief P-menu settings from a command frame. The pack voltage is inferred from the P02 cutoff. */
    inline DecodedSettings decodeTx(const TXPacket& tx) {
        DecodedSettings s;
        s.regenLevel = static_cast<uint8_t>((tx.regenAccelByte >> 4) & 0x0F);
        s.accelLevel = static_cast<uint8_t>(tx.regenAccelByte & 0x0F);
        s.kickStart  = (tx.functionBitmask & static_cast<uint8_t>(FunctionFlag::KickStart)) != 0;
        s.poleCount  = static_cast<uint16_t>(tx.poleCount_L | (static_cast<uint16_t>(tx.poleCount_H) << 8));

        const uint8_t p03_raw = tx.wheelCirc_L;
        s.wheelInches = (p03_raw >= 50 && p03_raw <= 200) ? (static_cast<float>(p03_raw) / 10.0f) : 10.0f;

        const uint16_t batt_cfg = static_cast<uint16_t>((static_cast<uint16_t>(tx.batteryConfig_H) << 8) | tx.batteryConfig_L);
        s.cutoffVolts = (batt_cfg > 0) ? (static_cast<float>(batt_cfg) / 10.0f) : 46.0f;
        
        if (batt_cfg == 0)       s.batteryVolts = 52;
        else if (batt_cfg < 370) s.batteryVolts = 36;
        else if (batt_cfg < 440) s.batteryVolts = 48;
        else if (batt_cfg < 500) s.batteryVolts = 52;
        else if (batt_cfg < 600) s.batteryVolts = 60;
        else                     s.batteryVolts = 72;

        s.speedLimitPercent = tx.speedProfile_L;
        s.speedProfile = static_cast<uint16_t>(tx.speedProfile_L | (static_cast<uint16_t>(tx.speedProfile_H) << 8));
        s.checksum = tx.checksum;

        return s;
    }

    // Speed
    //
    // The status frame carries speedRaw, not a speed. The display treats it as the time for one wheel turn in
    // ms and computes the speed it shows with integer math in two steps:
    //   speed_tenths = floor(287.275 * P03 / speedRaw)    0.1 km/h, with P03 = wheel diameter in 0.1 in
    //                                                      and 287.275 = pi * 0.0254 * 3600
    //   km/h shown   = floor(speed_tenths / 10)
    //   mph shown    = floor(speed_tenths * 6214 / 100000)
    // This model was derived from how a small MCU would do the math, then confirmed against the LCD by stepping
    // speedRaw through every value from 1 to 2299 (km/h) and 1 to 1352 (mph) with P03 = 80 and reading the
    // display by eye: no mismatches. The display does not average: it shows each new value directly.

    /** @brief km/h to mph factor. */
    constexpr float KMH_TO_MPH = 0.62137119f;

    /** @brief mph to km/h factor. */
    constexpr float MPH_TO_KMH = 1.609344f;

    /** @brief km/h to mph. */
    inline float kmh_to_mph(float kmh) {
        return kmh * KMH_TO_MPH;
    }

    /** @brief mph to km/h. */
    inline float mph_to_kmh(float mph) {
        return mph * MPH_TO_KMH;
    }

    /** @brief The mph value the display shows for speedRaw (model above). 0 for 0 or >= 3500. */
    inline uint16_t decodeSpeedRawMph(uint16_t speedRaw, float wheelInches = 10.0f) {
        if (speedRaw == 0 || speedRaw >= 3500) return 0;
        const float diameter_tenths = (wheelInches > 0.0f) ? (wheelInches * 10.0f) : 100.0f;
        const uint32_t num = static_cast<uint32_t>(287.275f * diameter_tenths);
        const uint32_t speed_tenths = num / static_cast<uint32_t>(speedRaw);
        const uint16_t mph = static_cast<uint16_t>((speed_tenths * 6214) / 100000);
        return (mph > 999) ? 999 : mph;
    }

    /** @brief The km/h value the display shows for speedRaw (model above). 0 for 0 or >= 3500. */
    inline uint16_t decodeSpeedRawKmh(uint16_t speedRaw, float wheelInches = 10.0f) {
        if (speedRaw == 0 || speedRaw >= 3500) return 0;
        const float diameter_tenths = (wheelInches > 0.0f) ? (wheelInches * 10.0f) : 100.0f;
        const uint32_t num = static_cast<uint32_t>(287.275f * diameter_tenths);
        const uint32_t speed_tenths = num / static_cast<uint32_t>(speedRaw);
        const uint16_t kmh = static_cast<uint16_t>(speed_tenths / 10);
        return (kmh > 999) ? 999 : kmh;
    }

    /**
     * @brief speedRaw for a speed in mph: 17.8512 * P03 / mph, rounded (17.8512 = 287.275 * 0.06214, the
     *        inverse of the model). Below 0.2 mph returns 3500 (shows 0).
     */
    inline uint16_t encodeSpeedRawMph(float speedMph, float wheelInches = 10.0f) {
        if (isnan(speedMph) || speedMph < 0.2f) return 3500;
        const float diameter_tenths = (wheelInches > 0.0f) ? (wheelInches * 10.0f) : 100.0f;
        float calculated = (17.8512f * diameter_tenths) / speedMph;
        if (calculated < 1.0f) calculated = 1.0f;
        if (calculated > 3500.0f || isnan(calculated)) calculated = 3500.0f;
        return static_cast<uint16_t>(lroundf(calculated));
    }

    /** @brief speedRaw for a speed in km/h: 28.7275 * P03 / km/h, rounded. poleCount is ignored (P04 does not affect the speed shown). */
    inline uint16_t encodeSpeedRaw(float speedKmh, uint16_t poleCount = 30, float wheelInches = 10.0f) {
        (void)poleCount;
        if (isnan(speedKmh) || speedKmh < 0.3f) return 3500;
        const float diameter_tenths = (wheelInches > 0.0f) ? (wheelInches * 10.0f) : 100.0f;
        float calculated = (28.7275f * diameter_tenths) / speedKmh;
        if (calculated < 1.0f) calculated = 1.0f;
        if (calculated > 3500.0f || isnan(calculated)) calculated = 3500.0f;
        return static_cast<uint16_t>(lroundf(calculated));
    }

    // Status frame

    /** @brief Sets the status frame bit that makes the display show `code` (see ErrorCode). */
    inline void applyErrorCode(RXPacket& pkt, ErrorCode code) {
        switch (code) {
            case ErrorCode::E01_RearMotorHall:          pkt.statusFlag = 0x40; break;      // Byte 3 Bit 6
            case ErrorCode::E02_Diagnostic2:            pkt.statusFlag = 0x20; break;      // Byte 3 Bit 5
            case ErrorCode::E03_MainController:         pkt.statusFlag = 0x10; break;      // Byte 3 Bit 4
            case ErrorCode::E05_VoltageFault:           pkt.statusFlag = 0x08; break;      // Byte 3 Bit 3
            case ErrorCode::E06_ReceiverTimeoutCrc:     pkt.calculatedStatus ^= 0xFF; break;// Checksum corruption
            case ErrorCode::E07_SendingError:           pkt.systemStatus |= 0x10; break;   // Byte 4 Bit 4
            case ErrorCode::E09_ControllerOverTemp:     pkt.systemStatus |= 0x01; break;   // Byte 4 Bit 0
            case ErrorCode::E11_FrontMotorHall:         pkt.speedField[0] = 0x40; break;   // Byte 5 Bit 6
            case ErrorCode::E13_SecondaryController:    pkt.speedField[0] = 0x10; break;   // Byte 5 Bit 4
            case ErrorCode::E15_Diagnostic15:           pkt.speedField[0] = 0x08; break;   // Byte 5 Bit 3
            case ErrorCode::E17_SubControllerRecv:      pkt.speedField[1] |= 0x10; break;  // Byte 6 Bit 4
            case ErrorCode::E31_LatchingE00:            pkt.speedField[0] = 0x20; break;   // Byte 5 Bit 5
            default: break;
        }
    }

    /**
     * @brief True for the replies that carry the keepalive flag: the 2nd frame after start-up, then every 50th
     *        (about every 6 s at 8 frames per second).
     */
    inline constexpr bool isKeepaliveFrame(uint32_t packetCount) {
        return (packetCount % 50 == 2);
    }

    /** @brief Builds a status frame. While a fault is shown, the speed is sent as stopped. */
    inline void formatResponsePacket(RXPacket& pkt, uint16_t raw_speed, bool dual_motor = true,
                                     uint8_t fault_mask = 0, bool trip_mode = false, bool is_keepalive = false) {
        memset(&pkt, 0, sizeof(pkt));
        pkt.startMarker = RX_START;
        pkt.packetLength = RX_LEN;
        pkt.statusType = trip_mode ? static_cast<uint8_t>(StatusType::TripOdometer) : static_cast<uint8_t>(StatusType::OdoLifetime);
        pkt.echo_H = RX_START;
        pkt.echo_L = RX_LEN;

        uint16_t effective_speed = (fault_mask > 0) ? SPEED_RAW_STATIONARY : raw_speed;
        pkt.speedRaw_H = static_cast<uint8_t>((effective_speed >> 8) & 0xFF);
        pkt.speedRaw_L = static_cast<uint8_t>(effective_speed & 0xFF);

        pkt.systemStatus = static_cast<uint8_t>(SystemStatus::Running);
        if (dual_motor) {
            pkt.systemStatus |= static_cast<uint8_t>(SystemStatus::DualDrive);
            pkt.speedField[1] |= static_cast<uint8_t>(SystemStatus::DualDrive);
        }

        if (is_keepalive) {
            pkt.statusFlag = static_cast<uint8_t>(StatusType::KeepAlive);
        } else if (fault_mask > 0) {
            pkt.statusFlag = static_cast<uint8_t>(StatusType::OdoLifetime) & 0x00;
            pkt.systemStatus |= (fault_mask & 0x3F);
        } else {
            pkt.statusFlag = static_cast<uint8_t>(StatusType::OdoLifetime) & 0x00;
        }

        pkt.calculatedStatus = calculateRXChecksum(pkt);
    }

} // namespace KukirinG3Pro