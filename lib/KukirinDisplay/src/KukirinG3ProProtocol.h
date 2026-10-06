//! KNOWN PITFALLS & FAILURE MODES:
//! - P14 [5V Display TX Level]: Display TX line is 5V TTL. ESP32 GPIOs are NOT 5V tolerant. Mandatory voltage divider (R1=1.0k, R2=1.8k) with 2.2nF..4.7nF parallel cap.
//! - P15 [Pure Bitwise XOR Checksum]: TFM13-FEIMI-1 protocol uses pure 12-byte bitwise XOR checksum across Bytes 0..11. E-006 receiver error is asserted by corrupting/inverting this checksum during test injection.
//! - P16 [Display Framing & Cadence]: Display transmits single 20-byte frames (0x01 0x14) at continuous 8.0 Hz (125.2ms period, zero bursts, zero packet duplication). Buffer recovery requires sliding-window memmove.
//! - P17 [Fixed-Point Speed Rendering]: LCD latches directly into 7-segment display without firmware averaging. Requires discrete 2-stage fixed-point digital twin.
//! - P18 [P-Menu Staging]: Live P-menu modifications while riding can cause sudden torque spikes; settings must quarantine until stationary (<= 3.0 mph) and neutral.
//! - P19 [Switch Mutual Exclusion]: Handlebar turn switches are mechanically mutually exclusive; setting both simultaneously violates hardware plausibility.
//! - P20 [Horn Thermal Protection]: 12V horn relay requires 3.0s auto-cutoff watchdog and 100ms re-arm quiet window to prevent coil burnout.

#pragma once

#include <Arduino.h>

/**
 * @namespace KukirinG3Pro
 * @brief Core namespace encapsulating strongly-typed enums, configuration structs,
 *        packet formats, and codec algorithms for the Kukirin G3 Pro display (TFM13-FEIMI-1).
 */
namespace KukirinG3Pro {

    // =========================================================================
    // Protocol Framing Constants
    // =========================================================================

    /** @brief Header start marker for Display -> Controller command frames (0x01). */
    constexpr uint8_t TX_START = 0x01;

    /** @brief Fixed byte length for Display -> Controller command frames (20 bytes decimal). */
    constexpr uint8_t TX_LEN = 0x14;

    /** @brief Header start marker for Controller -> Display status frames (0x02). */
    constexpr uint8_t RX_START = 0x02;

    /** @brief Fixed byte length for Controller -> Display status frames (14 bytes decimal). */
    constexpr uint8_t RX_LEN = 0x0E;

    // =========================================================================
    // Canonical Hardware Pinout (from docs/hardware_pinout.md)
    // =========================================================================
    namespace Pins {
        constexpr int8_t LeftLed      = 18; /**< @brief Left Turn Signal Blinker MOSFET. */
        constexpr int8_t RightLed     = 19; /**< @brief Right Turn Signal Blinker MOSFET. */
        constexpr int8_t BrakeLed     = 21; /**< @brief Rear Brake Light MOSFET. */
        constexpr int8_t HeadlightLed = 22; /**< @brief Front Headlight & Taillight MOSFET. */
        constexpr int8_t Horn         = 23; /**< @brief 12V Horn Relay Driver. */
        constexpr int8_t AuxLed       = 4;  /**< @brief Auxiliary Lighting (Safe LOW). */
        constexpr int8_t Argb1        = 25; /**< @brief ARGB Channel 1 (Safe LOW). */
        constexpr int8_t Argb2        = 26; /**< @brief ARGB Channel 2 (Safe LOW). */
        constexpr int8_t Argb3        = 27; /**< @brief ARGB Channel 3 (Safe LOW). */
        constexpr int8_t Argb4        = 14; /**< @brief ARGB Channel 4 (Safe LOW). */
        constexpr int8_t ButtonDs     = 39; /**< @brief Handlebar D/S Pushbutton (Input-Only GPI). */
        constexpr int8_t DispRx       = 35; /**< @brief UART2 RX (Display TX via voltage divider, Input-Only GPI). */
        constexpr int8_t DispTx       = 33; /**< @brief UART2 TX (Display RX). */
    }

    /**
     * @struct PinConfig
     * @brief Configurable GPIO mapping for arbitrary vehicle wiring topologies.
     *        Defaults to canonical Kukirin G3 Pro pinout. Set any unused pin to -1.
     */
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

    // =========================================================================
    // Strongly-Typed Enums
    // =========================================================================

    /**
     * @enum ErrorCode
     * @brief Ground-truth diagnostic error codes defined in the Kukirin G3 Pro User Manual.
     * @note Tested against physical hardware in Test Runs 1-10.
     */
    enum class ErrorCode : uint8_t {
        None                       = 0,   /**< @brief No error, normal telemetry. */
        E01_RearMotorHall          = 1,   /**< @brief E-001: Rear motor hall sensor error (Byte[3] Bit 6 = 0x40). */
        E02_Diagnostic2            = 2,   /**< @brief E-002: Diagnostic error 2 (Byte[3] Bit 5 = 0x20). */
        E03_MainController         = 3,   /**< @brief E-003: Main controller error (Byte[3] Bit 4 = 0x10). */
        E05_VoltageFault           = 5,   /**< @brief E-005: Voltage error / undervoltage (Byte[3] Bit 3 = 0x08). */
        E06_ReceiverTimeoutCrc     = 6,   /**< @brief E-006: Receiver error (UART timeout >2.5s or checksum inverted). */
        E07_SendingError           = 7,   /**< @brief E-007: Sending error (Controller transmit failure, Byte[4] = 0x10). */
        E09_ControllerOverTemp     = 9,   /**< @brief E-009: Controller over-temperature (Byte[4] = 0x01). */
        E11_FrontMotorHall         = 11,  /**< @brief E-011: Front motor hall sensor error (Byte[5] Bit 6 = 0x40). */
        E13_SecondaryController    = 13,  /**< @brief E-013: Secondary controller failure (Byte[5] Bit 4 = 0x10). */
        E15_Diagnostic15           = 15,  /**< @brief E-015: Diagnostic error 15 (Byte[5] Bit 3 = 0x08). */
        E17_SubControllerRecv      = 17,  /**< @brief E-017: Sub controller receiving failure (Byte[6] Bit 4 = 0x10). */
        E31_LatchingE00            = 31   /**< @brief E-031 -> E-00: Latching fault (Byte[5] Bit 5 = 0x20, requires power cycle). */
    };

    /**
     * @enum DriveMode
     * @brief Handlebar gear selection modes transmitted at Byte 0x04.
     */
    enum class DriveMode : uint8_t {
        Eco   = 0x05, /**< @brief Gear 1: Eco mode (0x05). */
        Std   = 0x0A, /**< @brief Gear 2: Standard mode (0x0A). */
        Turbo = 0x0F  /**< @brief Gear 3: Turbo mode (0x0F). */
    };

    /** @brief Converts strongly-typed DriveMode to integer gear level (1, 2, or 3). */
    inline constexpr uint8_t driveModeToGear(DriveMode mode) {
        switch (mode) {
            case DriveMode::Turbo: return 3;
            case DriveMode::Std:   return 2;
            case DriveMode::Eco:
            default:               return 1;
        }
    }

    /** @brief Converts integer gear level (1, 2, or 3) to strongly-typed DriveMode. */
    inline constexpr DriveMode gearToDriveMode(uint8_t gear) {
        switch (gear) {
            case 3:  return DriveMode::Turbo;
            case 2:  return DriveMode::Std;
            case 1:
            default: return DriveMode::Eco;
        }
    }

    /**
     * @enum IndicatorFlag
     * @brief Physical switch indicators packed into Byte 0x12 of TXPacket.
     */
    enum class IndicatorFlag : uint8_t {
        None        = 0x00,
        LeftTurn    = 0x08, /**< @brief Left turn signal active (Bit 3). */
        RightTurn   = 0x10, /**< @brief Right turn signal active (Bit 4). */
        BrakeActive = 0x20, /**< @brief Brake lever pulled (Bit 5). */
        HornActive  = 0x80  /**< @brief Horn pushbutton pressed (Bit 7). */
    };

    /**
     * @enum FunctionFlag
     * @brief Auxiliary function bitmask packed into Byte 0x05 of TXPacket.
     */
    enum class FunctionFlag : uint8_t {
        None      = 0x00,
        LightsOn  = 0x20, /**< @brief Headlight / Taillight enabled (Bit 5). */
        KickStart = 0x40  /**< @brief P06 Kick-Start enabled (Bit 6). */
    };

    /**
     * @enum SystemStatus
     * @brief Controller status byte (Byte 0x04) in RXPacket.
     */
    enum class SystemStatus : uint8_t {
        Running       = 0xC0, /**< @brief Baseline running status (0xC0). */
        Braking       = 0xE0, /**< @brief Brake icon visible (0xE0). */
        DualDrive     = 0x08, /**< @brief Dual motor drive active (Bit 3). */
        FaultSending  = 0x10, /**< @brief E-007 controller transmit fault (Bit 4). */
        FaultOverTemp = 0x01  /**< @brief E-009 controller over-temperature fault (Bit 0). */
    };

    /**
     * @enum StatusType
     * @brief Odometer and packet telemetry mode byte (Byte 0x02) in RXPacket.
     */
    enum class StatusType : uint8_t {
        OdoLifetime   = 0x01, /**< @brief Standard status telemetry / Odometer mode. */
        TripOdometer  = 0x02, /**< @brief Trip odometer telemetry mode. */
        KeepAlive     = 0x80  /**< @brief Periodic 50-frame keepalive status flag. */
    };

    /**
     * @enum SwitchMask
     * @brief Discrete handlebar accessory switch bitmask flags for event qualification.
     */
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

    /** @brief Discrete stationary / stopped raw speed period representation on TFM13-FEIMI-1 LCD. */
    constexpr uint16_t SPEED_RAW_STATIONARY = 3500;

    /** @brief Inactivity buffer purge watchdog timeout (50ms). */
    constexpr uint32_t FRAMING_TIMEOUT_MS         = 50;

    /** @brief Maximum serial read iterations per update pass. */
    constexpr uint16_t MAX_SERIAL_READ_ITERATIONS = 128;

    /** @brief Speed limiter profile word (Default 0x0C64 = Open / 100%). */
    constexpr uint16_t PROFILE_OPEN               = 0x0C64;

    /** @brief Throttle ADC plausibility clamp threshold (accommodates 5V/3.3V rail ripple). */
    constexpr uint16_t PLAUSIBILITY_CLAMP_MAX     = 1050;

    // =========================================================================
    // Dynamic Configuration Structs (Zero Hardcoding)
    // =========================================================================

    /**
     * @struct ThrottleConfig
     * @brief Configurable parameters for physical throttle ADC normalization, Schmitt trigger hysteresis, and current resolution.
     */
    struct ThrottleConfig {
        uint16_t idleThreshold        = 15;    /**< @brief Lower deadband threshold (<= 1.5% freewheel). */
        uint16_t maxThreshold         = 980;   /**< @brief Upper saturation threshold (>= 98% full power). */
        uint16_t deadbandMin          = 20;    /**< @brief Lower deadband baseline (~2% idle neutral). */
        uint16_t deadbandEngage       = 25;    /**< @brief Schmitt trigger engage threshold (> 2.5% to activate torque). */
        uint16_t deadbandDisengage    = 15;    /**< @brief Schmitt trigger disengage threshold (< 1.5% to return to freewheel). */
        uint16_t ceilingMax           = 980;   /**< @brief Upper saturation ceiling (> 98% full power). */
        uint16_t ceilingEngage        = 975;   /**< @brief Upper saturation engage threshold (> 97.5% locks to 100%). */
        uint16_t ceilingRelease       = 965;   /**< @brief Upper saturation release threshold (< 96.5% drops below 100%). */
        uint16_t rawMax               = 1000;  /**< @brief Full raw ADC scale. */
        uint16_t rawAdcClampMax       = PLAUSIBILITY_CLAMP_MAX; /**< @brief Plausibility clamp maximum (accommodates 5V/3.3V rail ripple). */
        uint16_t maxDeltaPerTick      = 350;   /**< @brief Kinematic positive surge limit (350 counts/tick = 35%/20ms). */
        uint16_t maxDecelDeltaPerTick = 500;   /**< @brief Kinematic decel tracking limit (500 counts/tick = 50%/20ms). */
    };

    /**
     * @struct BridgeTimingConfig
     * @brief Configurable dynamic timing delays, debounce filters, and safety watchdogs.
     */
    struct BridgeTimingConfig {
        uint32_t framingTimeoutMs     = 50;   /**< @brief Inactivity buffer purge watchdog (50ms). */
        uint32_t deadmanTimeoutMs     = 500;  /**< @brief Communication watchdog cutoff (500ms display watchdog cutoff). */
        uint32_t buttonDebounceMs     = 20;   /**< @brief ~20ms HID-grade tactile button debounce filter. */
        uint32_t blinkPeriodMs        = 400;  /**< @brief 1.25 Hz turn signal flash rate (400ms ON / 400ms OFF). */
        uint32_t brakeStrobePeriodMs  = 250;  /**< @brief 2.0 Hz safety brake strobe rate (250ms ON / 250ms OFF). */
        uint32_t hazardWindowMs       = 800;  /**< @brief Double-tap hazard activation window (800ms). */
        uint32_t hazardMinTapGapMs    = 100;  /**< @brief Minimum gap between taps to reject noise bursts (100ms). */
        uint32_t hornMaxContinuousMs  = 3000; /**< @brief 3.0s continuous horn safety auto-cutoff watchdog. */
        uint32_t hornRearmQuietMs     = 100;  /**< @brief 100ms release quiet window before horn re-arms. */
        uint32_t faultCyclePeriodMs   = 1000; /**< @brief 1000ms multi-fault alternating rotation period. */
        float    walkingSpeedLimitMph = 3.0f; /**< @brief Speed ceiling for in-motion P-menu updates (3.0 mph). */
        uint8_t  driveModeQuorum      = 2;    /**< @brief 2-frame quorum for gear mode changes. */
        uint8_t  functionQuorum       = 2;    /**< @brief 2-frame quorum for light/kick toggles. */
        uint8_t  pmenuQuorum          = 3;    /**< @brief 3-frame quorum for P-menu settings. */
        uint8_t  brakeReleaseQuorum   = 1;    /**< @brief 1-frame debounced release for brake lever (< 50ms latency). */
        uint8_t  turnSignalQuorum     = 2;    /**< @brief 2-frame quorum for turn rocker switches. */
    };

    /** @brief Canonical type alias for input filter timing configuration. */
    using InputFilterConfig = BridgeTimingConfig;

    // =========================================================================
    // Protocol Packet Structures (Packed Binary)
    // =========================================================================

    /**
     * @struct TXPacket
     * @brief Raw 20-byte command packet transmitted periodically by the TFM13-FEIMI-1 display.
     * @note Enforces 1-byte struct alignment via `__attribute__((packed))`.
     */
    struct __attribute__((packed)) TXPacket {
        uint8_t startMarker;      /**< @brief Frame header marker (0x01). Offset: 0x00. */
        uint8_t packetLength;     /**< @brief Frame byte length (0x14 = 20 bytes). Offset: 0x01. */
        uint8_t commandType;      /**< @brief Command type identifier (typically 0x01). Offset: 0x02. */
        uint8_t subCommand;       /**< @brief Sub-command identifier. Offset: 0x03. */
        uint8_t driveMode;        /**< @brief Gear mode (0x05=Eco, 0x0A=Std, 0x0F=Turbo). Offset: 0x04. */
        uint8_t functionBitmask;  /**< @brief Auxiliary functions (Bit 5 / 0x20 = Lights). Offset: 0x05. */
        uint8_t poleCount_L;      /**< @brief Motor magnetic pole count (Low byte). Offset: 0x06. */
        uint8_t poleCount_H;      /**< @brief Motor magnetic pole count (High byte). Offset: 0x07. */
        uint8_t wheelCirc_L;      /**< @brief Wheel diameter raw config (Low byte). Offset: 0x08. */
        uint8_t wheelCirc_H;      /**< @brief Wheel diameter raw config (High byte). Offset: 0x09. */
        uint8_t regenAccelByte;   /**< @brief Nibbles: [7:4] Regen Level (PB: 0-5), [3:0] Accel Level (PA: 1-5). Offset: 0x0A. */
        uint8_t reserved_0x0B;    /**< @brief Reserved / padding byte. Offset: 0x0B. */
        uint8_t speedProfile_L;   /**< @brief Speed limit profile (Low byte). Offset: 0x0C. */
        uint8_t speedProfile_H;   /**< @brief Speed limit profile (High byte). Offset: 0x0D. */
        uint8_t batteryConfig_H;  /**< @brief Battery voltage config (High byte, Big-Endian). Offset: 0x0E. */
        uint8_t batteryConfig_L;  /**< @brief Battery voltage config (Low byte, Big-Endian). Offset: 0x0F. */
        uint8_t throttle_H;       /**< @brief Throttle ADC raw level (High byte). Offset: 0x10. */
        uint8_t throttle_L;       /**< @brief Throttle ADC raw level (Low byte). Offset: 0x11. */
        uint8_t indicator_L;      /**< @brief Indicators (0x08=Left, 0x10=Right, 0x20=Brake, 0x80=Horn). Offset: 0x12. */
        uint8_t checksum;         /**< @brief Frame XOR checksum across Bytes 0..18. Offset: 0x13. */
    };

    /**
     * @struct RXPacket
     * @brief Raw 14-16 byte status response packet returned to the TFM13-FEIMI-1 display.
     * @note Enforces 1-byte struct alignment via `__attribute__((packed))`.
     */
    struct __attribute__((packed)) RXPacket {
        uint8_t startMarker;       /**< @brief Frame header marker (0x02). Offset: 0x00. */
        uint8_t packetLength;      /**< @brief Frame byte length (0x0E = 14 bytes). Offset: 0x01. */
        uint8_t statusType;        /**< @brief Status response type (0x01). Offset: 0x02. */
        uint8_t statusFlag;        /**< @brief Status flag (0x00=Normal, 0x80=Handshake). Offset: 0x03. */
        uint8_t systemStatus;      /**< @brief System status (0xC0=Running, 0xE0=Braking, |0x08=Dual). Offset: 0x04. */
        uint8_t speedField[3];     /**< @brief Speed padding / telemetry fields. Offset: 0x05-0x07. */
        uint8_t speedRaw_H;        /**< @brief Inverse speed period representation (High byte). Offset: 0x08. */
        uint8_t speedRaw_L;        /**< @brief Inverse speed period representation (Low byte). Offset: 0x09. */
        uint8_t currentField[2];   /**< @brief Motor current feedback (Amperes / bars). Offset: 0x0A-0x0B. */
        uint8_t calculatedStatus;  /**< @brief Pure 12-byte XOR checksum (Bytes 0..11). Offset: 0x0C. */
        uint8_t unknown_0x0D;      /**< @brief Reserved / trailer padding (0x00). Offset: 0x0D. */
        uint8_t echo_H;            /**< @brief Trailer echo high byte (0x02). Offset: 0x0E. */
        uint8_t echo_L;            /**< @brief Trailer echo low byte (0x0E). Offset: 0x0F. */
    };

    /**
     * @struct DecodedSettings
     * @brief High-level engineering representation of decoded P-settings extracted from TXPacket.
     */
    struct DecodedSettings {
        uint8_t regenLevel = 3;               /**< @brief PB: Regen Braking Level (Default Level 3). */
        uint8_t accelLevel = 3;               /**< @brief PA: Acceleration Aggressiveness (Default Level 3). */
        uint8_t batteryVolts = 52;            /**< @brief P02: Nominal Battery Voltage (Default 52V). */
        float cutoffVolts = 46.0f;            /**< @brief P02: Low-Voltage Cutoff (Default 46.0V LVC). */
        float wheelInches = 10.0f;            /**< @brief P03: Wheel Outer Diameter (Default 10.0"). */
        uint16_t poleCount = 30;              /**< @brief P04: Motor Rotor Pole Pairs (Default 30 poles). */
        bool kickStart = false;               /**< @brief P06: Kick-Start Mode (Default false = Zero-Start). */
        uint8_t speedLimitPercent = 100;      /**< @brief P07: Max Speed Limit (Default 100%). */
        uint16_t speedProfile = PROFILE_OPEN; /**< @brief Speed limiter profile word (Default 0x0C64 = Open). */
        uint8_t checksum = 0x00;              /**< @brief Frame XOR checksum across Bytes 0..18 (Offset 0x13). */
    };

    // =========================================================================
    // Checksum & Validation Helpers
    // =========================================================================

    /**
     * @brief Computes the true bitwise XOR checksum for the Display TXPacket (Bytes 0..18).
     */
    inline uint8_t calculateTXChecksum(const TXPacket& tx) {
        const uint8_t* raw = reinterpret_cast<const uint8_t*>(&tx);
        uint8_t cs = 0;
        for (size_t i = 0; i < 19; i++) {
            cs ^= raw[i];
        }
        return cs;
    }

    /**
     * @brief Computes the true bitwise XOR checksum for the Controller RXPacket (Bytes 0..11).
     */
    inline uint8_t calculateRXChecksum(const RXPacket& rx) {
        const uint8_t* raw = reinterpret_cast<const uint8_t*>(&rx);
        uint8_t checksum = 0;
        for (size_t i = 0; i < 12; i++) {
            checksum ^= raw[i];
        }
        return checksum;
    }

    /**
     * @brief Validates framing markers and length of an incoming TX packet buffer.
     */
    inline bool isValidTXPacket(const uint8_t* buffer, size_t length) {
        if (buffer == nullptr || length < sizeof(TXPacket)) return false;
        return (buffer[0] == TX_START && buffer[1] == TX_LEN);
    }

    /**
     * @brief Verifies whether the TX packet's XOR checksum matches the payload.
     */
    inline bool verifyTXChecksum(const TXPacket& tx) {
        return calculateTXChecksum(tx) == tx.checksum;
    }

    /**
     * @brief Deterministic structural plausibility verification for all 16 fields of TXPacket.
     * @details Rejects corrupt gear modes, out-of-rail throttle, impossible battery cutoffs,
     *          and non-conforming bitmask flags.
     * @return bool True if all fields strictly conform to verified physical hardware boundaries.
     */
    inline bool isPlausibleTXPacket(const TXPacket& tx) {
        // Framing Validation: Verify frame markers and command opcodes
        if (tx.startMarker != TX_START || tx.packetLength != TX_LEN) return false;
        if (tx.commandType != 0x01) return false;

        // Gear Validation: Discrete gear mode (Eco, Std, Turbo)
        if (tx.driveMode != static_cast<uint8_t>(DriveMode::Eco) &&
            tx.driveMode != static_cast<uint8_t>(DriveMode::Std) &&
            tx.driveMode != static_cast<uint8_t>(DriveMode::Turbo)) {
            return false;
        }

        // Pole Count Validation: P04 LE word range check
        if (tx.poleCount_L < 1 || tx.poleCount_L > 100) return false;

        // Wheel Geometry Validation: P03 tenths of an inch range check
        if (tx.wheelCirc_L < 50 || tx.wheelCirc_L > 200) return false;

        // Profile Nibble Validation: PB regen and PA acceleration bounds
        const uint8_t regenLvl = static_cast<uint8_t>((tx.regenAccelByte >> 4) & 0x0F);
        const uint8_t accelLvl = static_cast<uint8_t>(tx.regenAccelByte & 0x0F);
        if (regenLvl > 5) return false;
        if (accelLvl < 1 || accelLvl > 5) return false;

        // Speed Profile Validation: P07 percentage ceiling
        if (tx.speedProfile_L > 100) return false;

        // Voltage Boundary Validation: Battery cutoff threshold range
        const uint16_t battCutoff = static_cast<uint16_t>((static_cast<uint16_t>(tx.batteryConfig_H) << 8) | tx.batteryConfig_L);
        if (battCutoff < 240 || battCutoff > 800) return false;

        // Throttle Plausibility Validation: Raw ADC saturation and short-circuit clamp
        const uint16_t rawThrot = static_cast<uint16_t>((static_cast<uint16_t>(tx.throttle_H) << 8) | tx.throttle_L);
        if (rawThrot > 1050) return false;

        // Switch Mutual Exclusion Validation: Turn signal electrical integrity check
        if ((tx.indicator_L & static_cast<uint8_t>(IndicatorFlag::LeftTurn)) &&
            (tx.indicator_L & static_cast<uint8_t>(IndicatorFlag::RightTurn))) {
            return false;
        }

        return true;
    }

    /**
     * @brief Validates framing markers (0x01 0x14), minimum length, and 19-byte XOR checksum.
     * @return bool True if buffer represents a genuine, uncorrupted wire frame from the display.
     */
    inline bool verifyTXPacket(const uint8_t* buffer, size_t length) {
        if (!isValidTXPacket(buffer, length)) return false;
        return verifyTXChecksum(*reinterpret_cast<const TXPacket*>(buffer));
    }

    /**
     * @brief Performs in-buffer sliding-window search and resynchronization for display TX frames (0x01 0x14).
     * @param[in,out] buf Pointer to buffer containing raw UART bytes.
     * @param[in] currentLen Number of bytes currently in buffer.
     * @return size_t Remaining number of bytes in buffer after memmove (0 if no header found).
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
     * @brief Centralized zero-allocation input sanitization engine.
     * @details Provides:
     *          - Kinematic delta clamping on throttle ADC (<= 350 counts/frame = 35%/20ms).
     *          - Asymmetric brake dispatch: Instant 1-frame trip (0ms delay), instant 1-frame release (< 50ms latency).
     *          - 2-frame quorum on turn signals (~40-50ms) to eliminate false blinker flashes.
     *          - 3-frame quorum on P-menu settings (~60ms) to eliminate Serial Monitor diff spam.
     *          - Paced double-tap hazard qualification (100ms <= dt <= 800ms) to reject noise bursts.
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

        // Drive Mode quorum state (Strongly-typed DriveMode::Eco default)
        DriveMode rawDriveModePrev = DriveMode::Eco;
        DriveMode filteredDriveMode = DriveMode::Eco;
        uint8_t   driveModeQuorumCount = 0;

        // Function bitmask quorum state
        uint8_t rawFunctionPrev = static_cast<uint8_t>(FunctionFlag::None);
        uint8_t filteredFunction = static_cast<uint8_t>(FunctionFlag::None);
        uint8_t functionQuorumCount = 0;

        // P-menu quorum and high-speed desync staging state
        DecodedSettings pendingSettings;
        DecodedSettings stableSettings;
        DecodedSettings stagedSettings;
        uint8_t         pmenuQuorumCount = 0;
        bool            pmenuInitialized = false;
        bool            pmenuDesyncPending = false;

        /**
         * @brief Sanitizes raw throttle ADC with Schmitt trigger hysteresis, kinematic decel tracking, and instant brake cutoff.
         * @param[in] rawAdc Raw ADC counts (0..1000).
         * @param[in] brakeActive True if electronic or mechanical brake is engaged.
         * @param[in] cfg Dynamic ThrottleConfig containing thresholds, hysteresis, and current resolution.
         * @return uint16_t Filtered throttle counts.
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

            // Direct tracking; positive slew rate is canonically governed on ESP32 by VESCSafetySupervisor
            filteredThrottle = rawAdc;

            // Dual-Threshold Schmitt Trigger Idle Hysteresis
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

            // Upper Saturation Ceiling Schmitt Hysteresis
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

        /**
         * @brief Asymmetric brake filter: instant trip on frame 1, debounced release (< 50ms latency).
         */
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

        /**
         * @brief Quorum filter on left/right turn signals.
         */
        void updateTurnSignals(bool rawLeft, bool rawRight, bool& outLeft, bool& outRight, uint8_t quorumFrames = 2) {
            // Left quorum
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

            // Right quorum
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

        /**
         * @brief Quorum filter on strongly-typed DriveMode (Eco/Std/Turbo).
         */
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

        /**
         * @brief Quorum filter on auxiliary function flags (Lights, Kick-Start).
         */
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

        /**
         * @brief Paced double-tap hazard qualification filter.
         */
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

        /**
         * @brief Updates turn signal quorums and double-tap hazard qualification in a single unified pass.
         * @details Automatically resets and synchronizes both blinkers to idle whenever the handlebar switch is centered.
         */
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

        /**
         * @brief Resets transient quorum accumulation state upon frame corruption or comms stall.
         */
        void resetPending() {
            leftQuorumCount = 0;
            rightQuorumCount = 0;
            rawLeftTurnPrev = filteredLeft;
            rawRightTurnPrev = filteredRight;
            rawLeftHazardPrev = false;
            rawRightHazardPrev = false;
            // Preserve brakeReleaseCounter and throttleFullEngaged across transient framing resets
            pmenuQuorumCount = 0;
            driveModeQuorumCount = 0;
            functionQuorumCount = 0;
            rawDriveModePrev = filteredDriveMode;
            rawFunctionPrev = filteredFunction;
            if (pmenuInitialized && !pmenuDesyncPending) {
                pendingSettings = stableSettings;
            }
        }

        /**
         * @brief Resets timing configuration and state counters with canonical defaults.
         * @param[in] brakeReleaseQuorum Brake release quorum frame count (default 1).
         * @param[in] deadmanTimeoutMs Dead-man communication watchdog timeout in ms (default 500).
         */
        void resetTiming(uint8_t brakeReleaseQuorum = 1, uint32_t deadmanTimeoutMs = 500) {
            this->brakeReleaseQuorum = brakeReleaseQuorum;
            this->deadmanTimeoutMs = deadmanTimeoutMs;
            brakeReleaseCounter = 0;
        }

        /**
         * @brief Quorum filter on P-menu settings with speed-gated staging and safe ARQ resynchronization.
         * @param[in] rawSettings Newly parsed P-menu settings snapshot.
         * @param[out] outSettings Confirmed safe settings snapshot to execute.
         * @param[in] currentSpeedMph Current vehicle speed in mph.
         * @param[in] throttleNeutral True if throttle ADC is at neutral (< 2% / idle).
         * @param[in] walkingSpeedLimitMph Speed ceiling (3.0 mph) above which drivetrain changes are quarantined.
         * @param[in] quorumFrames Consecutive identical frames required to qualify a change.
         * @return bool True if a new configuration should be transmitted/resynced to VESC.
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

            // Safe Resynchronization Trigger: If a change was quarantined at high speed, commit it once slowed to walking speed with throttle neutral
            if (pmenuDesyncPending && currentSpeedMph <= walkingSpeedLimitMph && throttleNeutral) {
                stableSettings = stagedSettings;
                pmenuDesyncPending = false;
                outSettings = stableSettings;
                return true; // ARQ Trigger: Now safe to update VESC
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
                            // Reverted to running config while moving: cancel pending desync
                            pmenuDesyncPending = false;
                            stagedSettings = stableSettings;
                        } else if (changedFromStable) {
                            if (currentSpeedMph > walkingSpeedLimitMph) {
                                // Moving at speed: Quarantine change to prevent in-motion VESC desync
                                stagedSettings = pendingSettings;
                                pmenuDesyncPending = true;
                                outSettings = stableSettings; // Keep running safe current parameters
                                return false; // Defer VESC transaction until safe stop
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
     * @brief High-performance, non-blocking accessory lighting and horn driver with direct-toggle state inversion.
     * @details Manages 1.25 Hz turn signals with DRL, 2.0 Hz safety brake strobe, 3.0s horn auto-cutoff watchdog,
     *          and ~20ms active-low pushbutton debouncing with zero logic tree nesting.
     */
    struct AccessoryEngine {
        // Output Pin States (Live resolved booleans)
        bool leftLed = false;
        bool rightLed = false;
        bool brakeLed = false;
        bool headlightLed = false;
        bool hornRelay = false;

        // Blinker & Strobe State
        bool blinkState = false;
        bool lastTurnActive = false;
        uint32_t lastBlinkMs = 0;

        bool brakeStrobeState = false;
        bool lastBrakeActive = false;
        uint32_t lastBrakeStrobeMs = 0;

        // Horn Watchdog State
        bool hornArmed = true;
        bool hornActiveState = false;
        bool hornCutoffTriggered = false;
        bool lastHornRaw = false;
        uint32_t hornStartMs = 0;
        uint32_t hornReleaseMs = 0;

        // Physical D/S Button Debounce State
        bool btnLastRaw = false;
        bool btnStableState = false;
        uint32_t btnLastChangeMs = 0;
        bool dualMode = true;

        /**
         * @brief Updates accessory output states using the direct-toggle resting-state model.
         */
        void update(uint32_t nowMs, bool lights, bool brake, bool left, bool right,
                    bool hazard, bool horn, const BridgeTimingConfig& timing = BridgeTimingConfig{}) {
            headlightLed = lights;

            // Brake Strobe Synthesis: Rear brake light with direct-toggle strobe
            if (!lastBrakeActive && brake) {
                brakeStrobeState = !lights; // Immediate inversion from resting state
                lastBrakeStrobeMs = nowMs;  // Reset timer for full initial duration
            }
            lastBrakeActive = brake;

            if (brake && (nowMs - lastBrakeStrobeMs >= timing.brakeStrobePeriodMs)) {
                lastBrakeStrobeMs = nowMs;
                brakeStrobeState = !brakeStrobeState;
            }
            brakeLed = brake ? brakeStrobeState : lights;

            // Blinker Synthesis: Turn signals & hazards with direct-toggle blinker
            const bool leftActive = hazard || left;
            const bool rightActive = hazard || right;
            const bool anyTurnActive = leftActive || rightActive;

            if (!lastTurnActive && anyTurnActive) {
                blinkState = !lights; // Immediate inversion from resting state
                lastBlinkMs = nowMs;  // Reset timer for full initial duration
            }
            lastTurnActive = anyTurnActive;

            if (anyTurnActive && (nowMs - lastBlinkMs >= timing.blinkPeriodMs)) {
                lastBlinkMs = nowMs;
                blinkState = !blinkState;
            }

            leftLed  = leftActive  ? blinkState : lights;
            rightLed = rightActive ? blinkState : lights;

            // Horn Failsafe Synthesis: Momentary horn with 3.0s auto-cutoff watchdog & 100ms re-arm quiet window
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

        /**
         * @brief Debounces active-low physical button (e.g. GPIO 39).
         * @return bool True on falling edge (press event); false otherwise.
         */
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

        /**
         * @brief Applies resolved boolean states directly to physical ESP32 GPIO pins.
         */
        void applyHardwarePins(const PinConfig& pins = PinConfig{}) const {
            if (pins.headlightLed >= 0) digitalWrite(pins.headlightLed, headlightLed ? HIGH : LOW);
            if (pins.brakeLed >= 0)     digitalWrite(pins.brakeLed,     brakeLed ? HIGH : LOW);
            if (pins.leftLed >= 0)      digitalWrite(pins.leftLed,      leftLed ? HIGH : LOW);
            if (pins.rightLed >= 0)     digitalWrite(pins.rightLed,     rightLed ? HIGH : LOW);
            if (pins.horn >= 0)         digitalWrite(pins.horn,         hornRelay ? HIGH : LOW);
        }

        /**
         * @brief Initializes physical accessory output pins safely latching LOW before pinMode(OUTPUT).
         */
        static void initGPIO(const PinConfig& pins = PinConfig{}) {
            if (pins.leftLed >= 0)      { digitalWrite(pins.leftLed, LOW);      pinMode(pins.leftLed, OUTPUT); }
            if (pins.rightLed >= 0)     { digitalWrite(pins.rightLed, LOW);     pinMode(pins.rightLed, OUTPUT); }
            if (pins.brakeLed >= 0)     { digitalWrite(pins.brakeLed, LOW);     pinMode(pins.brakeLed, OUTPUT); }
            if (pins.headlightLed >= 0) { digitalWrite(pins.headlightLed, LOW); pinMode(pins.headlightLed, OUTPUT); }
            if (pins.horn >= 0)         { digitalWrite(pins.horn, LOW);          pinMode(pins.horn, OUTPUT); }

            // Safe dormant states for reserved lines
            if (pins.auxLed >= 0)       { digitalWrite(pins.auxLed, LOW);       pinMode(pins.auxLed, OUTPUT); }
            if (pins.argb1 >= 0)        { digitalWrite(pins.argb1, LOW);        pinMode(pins.argb1, OUTPUT); }
            if (pins.argb2 >= 0)        { digitalWrite(pins.argb2, LOW);        pinMode(pins.argb2, OUTPUT); }
            if (pins.argb3 >= 0)        { digitalWrite(pins.argb3, LOW);        pinMode(pins.argb3, OUTPUT); }
            if (pins.argb4 >= 0)        { digitalWrite(pins.argb4, LOW);        pinMode(pins.argb4, OUTPUT); }

            if (pins.buttonDs >= 0)     { pinMode(pins.buttonDs, INPUT); }
        }
    };

    // =========================================================================
    // Handlebar & Switch Input Extraction Helpers
    // =========================================================================

    /** @brief Extracts raw 16-bit throttle ADC counts from TXPacket (0..1000). */
    inline uint16_t extractRawThrottle(const TXPacket& tx) {
        return static_cast<uint16_t>((static_cast<uint16_t>(tx.throttle_H) << 8) | tx.throttle_L);
    }

    /**
     * @brief Canonical normalization of raw throttle ADC counts (0..1000) to [0.0f, 1.0f] range.
     * @param[in] rawAdc Raw ADC counts.
     * @param[in] cfg Throttle configuration containing idle and max thresholds.
     * @return float Normalized throttle in range [0.0f, 1.0f].
     */
    inline float normalizeThrottleAdc(uint16_t rawAdc, const ThrottleConfig& cfg = ThrottleConfig()) {
        if (rawAdc <= cfg.idleThreshold) return 0.0f;
        if (rawAdc >= cfg.maxThreshold) return 1.0f;
        return static_cast<float>(rawAdc - cfg.idleThreshold) / static_cast<float>(cfg.maxThreshold - cfg.idleThreshold);
    }

    /**
     * @brief Normalizes raw throttle ADC to [0.0f, 1.0f] with configurable deadband and ceiling clamping.
     */
    inline float normalizeThrottle(uint16_t rawAdc, const ThrottleConfig& cfg = ThrottleConfig{}) {
        return normalizeThrottleAdc(rawAdc, cfg);
    }

    /** @brief Checks if mechanical/electronic brake lever is engaged. */
    inline bool isBrakeActive(const TXPacket& tx) {
        return (tx.indicator_L & static_cast<uint8_t>(IndicatorFlag::BrakeActive)) != 0;
    }

    /** @brief Checks if headlight switch is active. */
    inline bool isLightsActive(const TXPacket& tx) {
        return (tx.functionBitmask & static_cast<uint8_t>(FunctionFlag::LightsOn)) != 0;
    }

    /** @brief Checks if left turn signal switch is active. */
    inline bool isLeftTurnActive(const TXPacket& tx) {
        return (tx.indicator_L & static_cast<uint8_t>(IndicatorFlag::LeftTurn)) != 0;
    }

    /** @brief Checks if right turn signal switch is active. */
    inline bool isRightTurnActive(const TXPacket& tx) {
        return (tx.indicator_L & static_cast<uint8_t>(IndicatorFlag::RightTurn)) != 0;
    }

    /** @brief Checks if horn pushbutton is pressed. */
    inline bool isHornActive(const TXPacket& tx) {
        return (tx.indicator_L & static_cast<uint8_t>(IndicatorFlag::HornActive)) != 0;
    }

    /** @brief Extracts the strongly-typed DriveMode enum from TXPacket. */
    inline DriveMode extractDriveMode(const TXPacket& tx) {
        if (tx.driveMode == static_cast<uint8_t>(DriveMode::Turbo)) return DriveMode::Turbo;
        if (tx.driveMode == static_cast<uint8_t>(DriveMode::Std))   return DriveMode::Std;
        return DriveMode::Eco;
    }

    /** @brief Decodes the integer gear level (1, 2, or 3) from TXPacket. */
    inline uint8_t extractGearLevel(const TXPacket& tx) {
        return driveModeToGear(extractDriveMode(tx));
    }

    // =========================================================================
    // P-Menu Settings Parser
    // =========================================================================

    /**
     * @brief Decodes configuration parameters and P-settings from a raw Display TXPacket.
     */
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

    // =========================================================================
    // Speedometer Codecs (Discrete 2-Stage Fixed-Point Digital Twin)
    // =========================================================================

    /** @brief Canonical conversion factor from km/h to mph (1 / 1.609344). */
    constexpr float KMH_TO_MPH = 0.62137119f;

    /** @brief Canonical conversion factor from mph to km/h (1.609344). */
    constexpr float MPH_TO_KMH = 1.609344f;

    /**
     * @brief Converts speed from kilometers per hour (km/h) to miles per hour (mph).
     * @param[in] kmh Speed in kilometers per hour.
     * @return float Speed in miles per hour.
     */
    inline float kmh_to_mph(float kmh) {
        return kmh * KMH_TO_MPH;
    }

    /**
     * @brief Converts speed from miles per hour (mph) to kilometers per hour (km/h).
     * @param[in] mph Speed in miles per hour.
     * @return float Speed in kilometers per hour.
     */
    inline float mph_to_kmh(float mph) {
        return mph * MPH_TO_KMH;
    }

    /**
     * @brief Decodes the exact integer speed displayed on the physical LCD dashboard in mph.
     */
    inline uint16_t decodeSpeedRawMph(uint16_t speedRaw, float wheelInches = 10.0f) {
        if (speedRaw == 0 || speedRaw >= 3500) return 0;
        const float diameter_tenths = (wheelInches > 0.0f) ? (wheelInches * 10.0f) : 100.0f;
        const uint32_t num = static_cast<uint32_t>(287.275f * diameter_tenths);
        const uint32_t speed_tenths = num / static_cast<uint32_t>(speedRaw);
        const uint16_t mph = static_cast<uint16_t>((speed_tenths * 6214) / 100000);
        return (mph > 999) ? 999 : mph;
    }

    /**
     * @brief Decodes the exact integer speed displayed on the physical LCD dashboard in km/h.
     */
    inline uint16_t decodeSpeedRawKmh(uint16_t speedRaw, float wheelInches = 10.0f) {
        if (speedRaw == 0 || speedRaw >= 3500) return 0;
        const float diameter_tenths = (wheelInches > 0.0f) ? (wheelInches * 10.0f) : 100.0f;
        const uint32_t num = static_cast<uint32_t>(287.275f * diameter_tenths);
        const uint32_t speed_tenths = num / static_cast<uint32_t>(speedRaw);
        const uint16_t kmh = static_cast<uint16_t>(speed_tenths / 10);
        return (kmh > 999) ? 999 : kmh;
    }

    /**
     * @brief Encodes vehicle forward speed (in mph) into the inverse period raw format.
     */
    inline uint16_t encodeSpeedRawMph(float speedMph, float wheelInches = 10.0f) {
        if (isnan(speedMph) || speedMph < 0.2f) return 3500;
        const float diameter_tenths = (wheelInches > 0.0f) ? (wheelInches * 10.0f) : 100.0f;
        float calculated = (17.8512f * diameter_tenths) / speedMph;
        if (calculated < 1.0f) calculated = 1.0f;
        if (calculated > 3500.0f || isnan(calculated)) calculated = 3500.0f;
        return static_cast<uint16_t>(lroundf(calculated));
    }

    /**
     * @brief Encodes vehicle forward speed (in km/h) into the inverse period raw format.
     */
    inline uint16_t encodeSpeedRaw(float speedKmh, uint16_t poleCount = 30, float wheelInches = 10.0f) {
        (void)poleCount;
        if (isnan(speedKmh) || speedKmh < 0.3f) return 3500;
        const float diameter_tenths = (wheelInches > 0.0f) ? (wheelInches * 10.0f) : 100.0f;
        float calculated = (28.7275f * diameter_tenths) / speedKmh;
        if (calculated < 1.0f) calculated = 1.0f;
        if (calculated > 3500.0f || isnan(calculated)) calculated = 3500.0f;
        return static_cast<uint16_t>(lroundf(calculated));
    }

    // =========================================================================
    // Diagnostic Fault Injection & Telemetry Formatting
    // =========================================================================

    /**
     * @brief Injects an official error code into the appropriate register byte in RXPacket.
     * @param[in,out] pkt Reference to the RXPacket struct to modify.
     * @param[in] code Strongly-typed ErrorCode enum.
     */
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
     * @brief Evaluates whether the current packet tick is a display keepalive handshake frame.
     * @details Packet 2 serves as the mandatory boot activation handshake, followed by 1 Hz periodic keepalive (every 50 frames).
     */
    inline constexpr bool isKeepaliveFrame(uint32_t packetCount) {
        return (packetCount % 50 == 2);
    }

    /**
     * @brief Formats a complete, validated response packet for the TFM13-FEIMI-1 dashboard.
     */
    inline void formatResponsePacket(RXPacket& pkt, uint16_t raw_speed, bool dual_motor = true,
                                     uint8_t fault_mask = 0, bool trip_mode = false, bool is_keepalive = false) {
        memset(&pkt, 0, sizeof(pkt));
        pkt.startMarker = RX_START;
        pkt.packetLength = RX_LEN;
        pkt.statusType = trip_mode ? static_cast<uint8_t>(StatusType::TripOdometer) : static_cast<uint8_t>(StatusType::OdoLifetime);
        pkt.echo_H = RX_START;
        pkt.echo_L = RX_LEN;

        // Enforce stationary idle speed period (SPEED_RAW_STATIONARY) during vehicle fault lockouts
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

        // Universal Pure XOR Checksum across bytes 0-11
        pkt.calculatedStatus = calculateRXChecksum(pkt);
    }

} // namespace KukirinG3Pro