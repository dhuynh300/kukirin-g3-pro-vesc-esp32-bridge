//! KNOWN PITFALLS & FAILURE MODES:
//! - P14 [5V Display TX Level]: Display TX line is 5V TTL. ESP32 GPIOs are NOT 5V tolerant. Mandatory voltage divider (R1=1.0k, R2=1.8k) with 2.2nF..4.7nF parallel cap.
//! - P15 [Pure Bitwise XOR Checksum]: TFM13-FEIMI-1 protocol uses pure 12-byte bitwise XOR checksum across Bytes 0..11. E-006 receiver error is asserted by corrupting/inverting this checksum during test injection.
//! - P16 [Display Framing & Cadence]: Display transmits single 20-byte frames (0x01 0x14) at continuous 8.0 Hz (125.2ms period, zero bursts, zero packet duplication). Buffer recovery requires sliding-window memmove.
//! - P17 [Fixed-Point Speed Rendering]: LCD latches directly into 7-segment display without firmware averaging. Requires discrete 2-stage fixed-point digital twin.
//! - P18 [P-Menu Staging]: Live P-menu modifications while riding can cause sudden torque spikes; settings must quarantine until stationary (<= 3.0 mph) and neutral.
//! - P21 [Synchronous Response Mandate]: Controller response must transmit synchronously within the frame transaction window to satisfy display timeout (<2.5s).
//! - P22 [Brake Release Quorum]: Asymmetric debounce (1-frame instant trip, debounced release) guarantees zero lag on emergency braking.

#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <atomic>
#include "KukirinG3ProProtocol.h"

/**
 * @struct KukirinInputs
 * @brief High-level snapshot of all physical handlebar controls, switches, and P-menu settings.
 */
struct KukirinInputs {
    uint16_t rawThrottle = 0;              /**< @brief Raw ADC throttle counts (0..1000). */
    float    normalizedThrottle = 0.0f;   /**< @brief Normalized throttle [0.0f..1.0f] with deadband and ceiling. */
    bool     brakeActive = false;          /**< @brief True if mechanical or electronic brake lever is engaged. */
    bool     lightsActive = false;         /**< @brief True if headlight/taillight switch is ON. */
    bool     leftTurn = false;             /**< @brief True if left turn signal rocker switch is ON. */
    bool     rightTurn = false;            /**< @brief True if right turn signal rocker switch is ON. */
    bool     hazardLightsActive = false;   /**< @brief True if double-tap 4-way hazard lights mode is active. */
    bool     hornActive = false;           /**< @brief True if horn pushbutton is currently pressed. */
    uint8_t  gear = 1;                     /**< @brief Drive gear: 1 (Eco), 2 (Std), 3 (Turbo). */
    KukirinG3Pro::DriveMode driveMode = KukirinG3Pro::DriveMode::Eco;
    KukirinG3Pro::DecodedSettings settings;/**< @brief Parsed P-menu configuration parameters (P01..PB). */
    bool     pmenuUpdated = false;         /**< @brief True if P-menu settings were committed/updated this frame. */
    float    frameDt = 0.1f;               /**< @brief Inter-frame arrival delta time in seconds. */
    uint32_t packetCount = 0;              /**< @brief Cumulative frame counter. */
};

/**
 * @struct KukirinResponse
 * @brief Telemetry payload returned synchronously to the TFM13-FEIMI-1 dashboard.
 */
struct KukirinResponse {
    float speedMph = 0.0f;                    /**< @brief Vehicle forward speed in miles per hour. */
    bool  dualMode = true;                    /**< @brief True to display Dual-Motor icon on screen. */
    KukirinG3Pro::ErrorCode errorCode = KukirinG3Pro::ErrorCode::None; /**< @brief Diagnostic fault code. */
    bool  tripMode = false;                   /**< @brief True = Trip Odometer, False = Lifetime Odometer (ODO). */
};

/**
 * @class KukirinG3ProDisplay
 * @brief Production C++ driver library for the Kukirin G3 Pro TFM13-FEIMI-1 display protocol.
 * @details Features:
 *          - Dual Operation: Pattern B (Atomic Transaction Callback) & Pattern A/C (State Polling & Setters)
 *          - Thread-Safe / Cross-Core Safe (Core 1 real-time UART execution, Core 0 async setters/getters)
 *          - Zero dynamic memory allocation in main loop paths
 *          - Lock-free std::atomic fast-path getters for real-time motor control loops
 *          - Bounded FreeRTOS mutexes (pdMS_TO_TICKS(5)) to eliminate priority inversion and core stalls
 *          - Optional integrated accessory GPIO engine (Left/Right turn blinkers, DRLs, brake strobe, horn watchdog)
 *          - Frame-synchronous 0ms response latency
 */
class KukirinG3ProDisplay {
public:
    // Canonical Hardware Pin Aliases (backed by KukirinG3Pro::Pins)
    static constexpr int8_t PIN_LEFT_LED      = KukirinG3Pro::Pins::LeftLed;
    static constexpr int8_t PIN_RIGHT_LED     = KukirinG3Pro::Pins::RightLed;
    static constexpr int8_t PIN_BRAKE_LED     = KukirinG3Pro::Pins::BrakeLed;
    static constexpr int8_t PIN_HEADLIGHT_LED = KukirinG3Pro::Pins::HeadlightLed;
    static constexpr int8_t PIN_HORN          = KukirinG3Pro::Pins::Horn;
    static constexpr int8_t PIN_AUX_LED       = KukirinG3Pro::Pins::AuxLed;
    static constexpr int8_t PIN_ARGB_1        = KukirinG3Pro::Pins::Argb1;
    static constexpr int8_t PIN_ARGB_2        = KukirinG3Pro::Pins::Argb2;
    static constexpr int8_t PIN_ARGB_3        = KukirinG3Pro::Pins::Argb3;
    static constexpr int8_t PIN_ARGB_4        = KukirinG3Pro::Pins::Argb4;
    static constexpr int8_t PIN_BUTTON_DS     = KukirinG3Pro::Pins::ButtonDs;

    // Function Signatures (Zero Dynamic Allocation)
    typedef void (*TransactionCallback)(const KukirinInputs& in, KukirinResponse& out);
    typedef void (*ThrottleCallback)(uint16_t raw, float normalized);
    typedef void (*BrakeCallback)(bool active);
    typedef void (*LightsCallback)(bool on);
    typedef void (*TurnSignalCallback)(bool left, bool right);
    typedef void (*HazardLightsCallback)(bool active);
    typedef void (*HornCallback)(bool active);
    typedef void (*GearCallback)(uint8_t gear);
    typedef void (*DriveModeCallback)(KukirinG3Pro::DriveMode mode);
    typedef void (*DualCallback)(bool dual);
    typedef void (*PMenuCallback)(const KukirinG3Pro::DecodedSettings& settings);

    using TXPacket = KukirinG3Pro::TXPacket;
    using RXPacket = KukirinG3Pro::RXPacket;

    /**
     * @brief Constructs a new display driver instance.
     * @param[in] displaySerial Pointer to HardwareSerial connected to display (e.g. &Serial2).
     * @param[in] rxPin ESP32 GPIO pin for UART RX (Display TX via voltage divider).
     * @param[in] txPin ESP32 GPIO pin for UART TX (Display RX).
     * @param[in] buttonDsPin ESP32 GPIO pin for handlebar D/S switch (default GPIO 39, -1 to disable).
     */
    KukirinG3ProDisplay(HardwareSerial* displaySerial,
                        int8_t rxPin = KukirinG3Pro::Pins::DispRx,
                        int8_t txPin = KukirinG3Pro::Pins::DispTx,
                        int8_t buttonDsPin = KukirinG3Pro::Pins::ButtonDs)
        : _port(displaySerial), _rxPin(rxPin), _txPin(txPin), _buttonDsPin(buttonDsPin)
    {
        _mutex = xSemaphoreCreateMutex();
        memset(&_rxPacket, 0, sizeof(_rxPacket));
        _rxPacket.startMarker = KukirinG3Pro::RX_START;
        _rxPacket.packetLength = KukirinG3Pro::RX_LEN;
        _rxPacket.statusType = static_cast<uint8_t>(KukirinG3Pro::StatusType::OdoLifetime);
        _rxPacket.systemStatus = static_cast<uint8_t>(KukirinG3Pro::SystemStatus::Running);
        _rxPacket.speedRaw_H = static_cast<uint8_t>((KukirinG3Pro::SPEED_RAW_STATIONARY >> 8) & 0xFF);
        _rxPacket.speedRaw_L = static_cast<uint8_t>(KukirinG3Pro::SPEED_RAW_STATIONARY & 0xFF);
        _rxPacket.echo_H = KukirinG3Pro::RX_START;
        _rxPacket.echo_L = KukirinG3Pro::RX_LEN;
    }

    ~KukirinG3ProDisplay() {
        if (_mutex != nullptr) {
            vSemaphoreDelete(_mutex);
            _mutex = nullptr;
        }
    }

    /**
     * @brief Initializes UART serial port at 9600 8N1.
     */
    void begin(uint32_t baudRate = 9600) {
        if (_port != nullptr) {
            _port->begin(baudRate, SERIAL_8N1, _rxPin, _txPin);
        }
    }

    /**
     * @brief Initializes physical GPIO pins for lighting and accessories safely via AccessoryEngine.
     * @param[in] pins Configurable pin mapping (defaults to canonical Kukirin G3 Pro pinout).
     */
    void initGPIO(const KukirinG3Pro::PinConfig& pins = KukirinG3Pro::PinConfig{}) {
        _pins = pins;
        _buttonDsPin = pins.buttonDs;
        KukirinG3Pro::AccessoryEngine::initGPIO(_pins);
        _gpioInitialized = true;
    }

    // =========================================================================
    // Dynamic Configuration Setters & Getters
    // =========================================================================
    void setTimingConfig(const KukirinG3Pro::BridgeTimingConfig& cfg) {
        if (_mutex && xSemaphoreTake(_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            _timing = cfg;
            xSemaphoreGive(_mutex);
        }
    }

    void setThrottleConfig(const KukirinG3Pro::ThrottleConfig& cfg) {
        if (_mutex && xSemaphoreTake(_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            _throtCfg = cfg;
            xSemaphoreGive(_mutex);
        }
    }

    // =========================================================================
    // Pattern B: Atomic Transaction Callback Hook
    // =========================================================================
    /**
     * @brief Registers the atomic request-response transaction hook.
     * @param[in] cb Callback invoked immediately on valid frame receipt.
     */
    void onTransaction(TransactionCallback cb) { _onTransaction = cb; }

    // Optional Granular Callbacks
    void onThrottle(ThrottleCallback cb)       { _onThrottle = cb; }
    void onBrake(BrakeCallback cb)             { _onBrake = cb; }
    void onLights(LightsCallback cb)           { _onLights = cb; }
    void onTurnSignals(TurnSignalCallback cb)  { _onTurnSignals = cb; }
    void onHazardLights(HazardLightsCallback cb){ _onHazardLights = cb; }
    void onHorn(HornCallback cb)               { _onHorn = cb; }
    void onGear(GearCallback cb)               { _onGear = cb; }
    void onDriveMode(DriveModeCallback cb)     { _onDriveMode = cb; }
    void onDual(DualCallback cb)               { _onDual = cb; }
    void onPMenu(PMenuCallback cb)             { _onPMenu = cb; }

    // =========================================================================
    // Pattern A/C: Thread-Safe State Setters (Cross-Core Safe)
    // =========================================================================
    void setSpeedMph(float mph) {
        const float cleanMph = (mph >= 0.0f) ? mph : 0.0f;
        _atomicSpeedMph.store(cleanMph, std::memory_order_relaxed);
        if (_mutex && xSemaphoreTake(_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            _cachedResponse.speedMph = cleanMph;
            xSemaphoreGive(_mutex);
        }
    }

    void setSpeedKmh(float kmh) {
        setSpeedMph((kmh >= 0.0f) ? KukirinG3Pro::kmh_to_mph(kmh) : 0.0f);
    }

    void setDualMode(bool dual) {
        _dualModeState = dual;
        _accessoryEngine.dualMode = dual;
        _atomicDualMode.store(dual, std::memory_order_relaxed);
        if (_mutex && xSemaphoreTake(_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            _cachedResponse.dualMode = dual;
            xSemaphoreGive(_mutex);
        }
        if (dual) {
            _prevSwitchMask |= static_cast<uint8_t>(KukirinG3Pro::SwitchMask::Dual);
        } else {
            _prevSwitchMask &= ~static_cast<uint8_t>(KukirinG3Pro::SwitchMask::Dual);
        }
    }

    inline bool isDualMode() const {
        return _atomicDualMode.load(std::memory_order_relaxed);
    }

    void setErrorCode(KukirinG3Pro::ErrorCode code) {
        _atomicErrorCode.store(static_cast<uint8_t>(code), std::memory_order_relaxed); // Atomic fast-path store
        if (_mutex && xSemaphoreTake(_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            _cachedResponse.errorCode = code;
            xSemaphoreGive(_mutex);
        }
    }

    void setErrorCode(uint8_t code) {
        setErrorCode(static_cast<KukirinG3Pro::ErrorCode>(code));
    }

    /** @brief Atomic fast-path read of current active error code (0 = None). */
    inline uint8_t getErrorCodeRaw() const {
        return _atomicErrorCode.load(std::memory_order_relaxed);
    }

    /** @brief Strongly-typed atomic fast-path read of current active error code. */
    inline KukirinG3Pro::ErrorCode getErrorCode() const {
        return static_cast<KukirinG3Pro::ErrorCode>(_atomicErrorCode.load(std::memory_order_relaxed));
    }

    void setTripMode(bool trip) {
        _atomicTripMode.store(trip, std::memory_order_relaxed); // Atomic fast-path store
        if (_mutex && xSemaphoreTake(_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            _cachedResponse.tripMode = trip;
            xSemaphoreGive(_mutex);
        }
    }

    // =========================================================================
    // Lock-Free Fast-Path Getters for Core 1 Motor Control Loops
    // =========================================================================
    inline float getNormalizedThrottle() const {
        return _atomicNormalizedThrottle.load(std::memory_order_relaxed);
    }

    inline uint16_t getRawThrottle() const {
        return _atomicRawThrottle.load(std::memory_order_relaxed);
    }

    inline float getSpeedMph() const {
        return _atomicSpeedMph.load(std::memory_order_relaxed);
    }

    inline float getSpeedKmh() const {
        return KukirinG3Pro::mph_to_kmh(getSpeedMph());
    }

    inline bool isBrakeActive() const {
        return _atomicBrakeActive.load(std::memory_order_relaxed);
    }

    inline uint8_t getGearLevel() const {
        return _atomicGearLevel.load(std::memory_order_relaxed);
    }

    inline KukirinG3Pro::DriveMode getDriveMode() const {
        return KukirinG3Pro::gearToDriveMode(_atomicGearLevel.load(std::memory_order_relaxed));
    }

    inline bool isLightsActive() const {
        return _atomicLightsActive.load(std::memory_order_relaxed);
    }

    inline bool isLeftTurnActive() const {
        return _atomicLeftTurnActive.load(std::memory_order_relaxed);
    }

    inline bool isRightTurnActive() const {
        return _atomicRightTurnActive.load(std::memory_order_relaxed);
    }

    inline bool isHazardLightsActive() const {
        return _atomicHazardActive.load(std::memory_order_relaxed);
    }

    inline bool isHornActive() const {
        return _atomicHornActive.load(std::memory_order_relaxed);
    }

    inline uint32_t getPacketCount() const {
        return _packetCounter;
    }

    inline uint32_t getRejectedPacketCount() const {
        return _rejectedPacketCount;
    }

    inline uint32_t getResyncShiftCount() const {
        return _resyncShiftCount;
    }

    inline uint32_t getLastValidPacketMs() const {
        return _atomicLastValidPacketMs.load(std::memory_order_relaxed);
    }

    inline void resetDiagnosticCounters() {
        _rejectedPacketCount = 0;
        _resyncShiftCount = 0;
    }

    inline bool isCommsStalled() const {
        const uint32_t lastValid = _atomicLastValidPacketMs.load(std::memory_order_relaxed);
        return (lastValid == 0 || (millis() - lastValid > _timing.deadmanTimeoutMs));
    }

    /**
     * @brief Thread-safe atomic copy of the full handlebar input structure.
     */
    bool getInputs(KukirinInputs& out) const {
        if (_mutex && xSemaphoreTake(_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            out = _cachedInputs;
            xSemaphoreGive(_mutex);
            return true;
        }
        return false;
    }

    /**
     * @brief Performs sliding-window search and resynchronization on ingress buffer.
     * @return bool True if a valid frame header (0x01 0x14) was located and shifted to buffer index 0.
     */
    bool resyncTXBuffer() {
        const size_t rem = KukirinG3Pro::resyncTXBuffer(_txBuf, sizeof(KukirinG3Pro::TXPacket));
        if (rem > 0) {
            _resyncShiftCount++;
            _txIdx = rem;
            return true;
        }
        _txIdx = 0;
        return false;
    }

    // =========================================================================
    // Real-Time Execution Engine (Core 1, Non-Blocking)
    // =========================================================================
    /**
     * @brief Updates UART streaming, evaluates frame watchdogs, executes callbacks, and transmits responses.
     * @return bool True if a new complete display packet was received and answered this cycle.
     */
    bool update() {
        if (_port == nullptr) return false;

        const uint32_t nowMs = millis();

        // Handlebar Pushbutton Processing: Sample physical D/S switch
        if (_gpioInitialized) {
            _processPhysicalButton(nowMs);
            _updateAccessories(nowMs);
        }

        // Framing Watchdog: Purge buffer after silence interval
        if (_txIdx > 0 && (nowMs - _lastRxByteTime >= _timing.framingTimeoutMs)) {
            _txIdx = 0;
            _inputFilter.resetPending();
        }

        // UART Ingestion: Drain serial ring buffer non-blocking
        uint16_t iterations = 0;
        while (_port->available() && iterations < KukirinG3Pro::MAX_SERIAL_READ_ITERATIONS) {
            iterations++;
            const uint8_t b = static_cast<uint8_t>(_port->read());
            _lastRxByteTime = nowMs;
            _atomicLastRxByteMs.store(nowMs, std::memory_order_relaxed);

            if (_txIdx == 0) {
                if (b == KukirinG3Pro::TX_START) {
                    _txBuf[_txIdx++] = b;
                }
            } else if (_txIdx == 1) {
                if (b == KukirinG3Pro::TX_LEN) {
                    _txBuf[_txIdx++] = b;
                } else if (b == KukirinG3Pro::TX_START) {
                    _txBuf[0] = b;
                } else {
                    _txIdx = 0;
                }
            } else {
                _txBuf[_txIdx++] = b;
                if (_txIdx >= sizeof(KukirinG3Pro::TXPacket)) {
                    const bool validChecksum = KukirinG3Pro::verifyTXPacket(_txBuf, sizeof(KukirinG3Pro::TXPacket));
                    if (validChecksum) {
                        KukirinG3Pro::TXPacket command;
                        memcpy(&command, _txBuf, sizeof(KukirinG3Pro::TXPacket));
                        _txIdx = 0;

                        const float frameDt = (_lastPacketMs > 0) ? (static_cast<float>(nowMs - _lastPacketMs) / 1000.0f) : 0.1f;

                        // Ingress Validation & Parsing: Unpack frame into local stack inputs if plausible
                        KukirinInputs localInputs;
                        const bool plausible = KukirinG3Pro::isPlausibleTXPacket(command);
                        if (plausible) {
                            _packetCounter++;
                            _atomicLastValidPacketMs.store(nowMs, std::memory_order_relaxed);
                            _lastPacketMs = nowMs;
                            _unpackInputs(command, localInputs, frameDt);
                        } else {
                            _rejectedPacketCount++;
                            // Enforce safe stationary neutral defaults on plausibility fault
                            localInputs.frameDt = frameDt;
                            localInputs.packetCount = _packetCounter;
                        }

                        // Response Synthesis: Formulate outgoing telemetry frame (Pattern B callback or cached setters)
                        KukirinResponse localResponse;
                        if (_mutex && xSemaphoreTake(_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
                            localResponse = _cachedResponse;
                            xSemaphoreGive(_mutex);
                        } else {
                            // Atomic fallback under mutex contention preserves active error codes and state
                            localResponse.speedMph = _atomicSpeedMph.load(std::memory_order_relaxed);
                            localResponse.dualMode = _atomicDualMode.load(std::memory_order_relaxed);
                            localResponse.errorCode = static_cast<KukirinG3Pro::ErrorCode>(_atomicErrorCode.load(std::memory_order_relaxed));
                            localResponse.tripMode = _atomicTripMode.load(std::memory_order_relaxed);
                        }

                        // Execute Pattern B atomic transaction callback BEFORE publishing inputs
                        // Ensures downstream listeners (onThrottle, etc.) observe fresh transaction state
                        TransactionCallback txCb = _onTransaction;
                        if (txCb != nullptr) {
                            txCb(localInputs, localResponse);
                            _atomicSpeedMph.store(localResponse.speedMph, std::memory_order_relaxed);
                            _atomicDualMode.store(localResponse.dualMode, std::memory_order_relaxed);
                            _atomicErrorCode.store(static_cast<uint8_t>(localResponse.errorCode), std::memory_order_relaxed);
                            _atomicTripMode.store(localResponse.tripMode, std::memory_order_relaxed);
                        }

                        if (plausible) {
                            _publishInputs(localInputs);
                        }

                        // Synchronous Wire Dispatch: Transmit binary response frame (0ms phase lag, satisfies display watchdog)
                        _sendResponse(localResponse, localInputs);

                        return true;
                    } else {
                        _rejectedPacketCount++;
                        if (_txIdx >= sizeof(KukirinG3Pro::TXPacket) && !resyncTXBuffer()) {
                            _txIdx = 0;
                        }
                    }
                }
            }
        }
        return false;
    }

private:
    HardwareSerial* _port = nullptr;
    int8_t _rxPin = KukirinG3Pro::Pins::DispRx;
    int8_t _txPin = KukirinG3Pro::Pins::DispTx;
    int8_t _buttonDsPin = KukirinG3Pro::Pins::ButtonDs;
    KukirinG3Pro::PinConfig _pins;
    bool   _gpioInitialized = false;

    SemaphoreHandle_t _mutex = nullptr;
    KukirinInputs     _cachedInputs;
    KukirinResponse   _cachedResponse;

    // Lock-free atomic fast-path variables
    std::atomic<float>    _atomicNormalizedThrottle{0.0f};
    std::atomic<uint16_t> _atomicRawThrottle{0};
    std::atomic<float>    _atomicSpeedMph{0.0f};
    std::atomic<bool>     _atomicBrakeActive{false};
    std::atomic<uint8_t>  _atomicGearLevel{1};
    std::atomic<bool>     _atomicLightsActive{false};
    std::atomic<bool>     _atomicLeftTurnActive{false};
    std::atomic<bool>     _atomicRightTurnActive{false};
    std::atomic<bool>     _atomicHazardActive{false};
    std::atomic<bool>     _atomicHornActive{false};
    std::atomic<uint32_t> _atomicLastRxByteMs{0};
    std::atomic<uint32_t> _atomicLastValidPacketMs{0};
    std::atomic<bool>     _atomicDualMode{true};
    std::atomic<uint8_t>  _atomicErrorCode{static_cast<uint8_t>(KukirinG3Pro::ErrorCode::None)};
    std::atomic<bool>     _atomicTripMode{false};

    // Centralized library-level input filtering & quorum engine
    KukirinG3Pro::InputFilter _inputFilter;

    // Protocol state
    KukirinG3Pro::RXPacket _rxPacket;
    uint8_t  _txBuf[sizeof(KukirinG3Pro::TXPacket)];
    size_t   _txIdx = 0;
    uint32_t _lastRxByteTime = 0;
    uint32_t _lastPacketMs = 0;
    uint32_t _packetCounter = 0;
    uint32_t _rejectedPacketCount = 0;
    uint32_t _resyncShiftCount = 0;

    // Configurable Dynamic Parameters
    KukirinG3Pro::BridgeTimingConfig _timing;
    KukirinG3Pro::ThrottleConfig     _throtCfg;

    // Canonical Accessory State & Driving Engine
    KukirinG3Pro::AccessoryEngine _accessoryEngine;
    bool     _dualModeState = true;
    static constexpr uint8_t UNINITIALIZED_GEAR = 0xFF;
    static constexpr uint8_t UNINITIALIZED_SWITCH_MASK = static_cast<uint8_t>(KukirinG3Pro::SwitchMask::Uninitialized);
    uint8_t  _prevSwitchMask = UNINITIALIZED_SWITCH_MASK; // Forces initial baseline dispatch on first frame
    uint16_t _prevRawThrottle = 0xFFFF;
    float    _prevNormalizedThrottle = -1.0f;
    uint8_t  _prevGear = UNINITIALIZED_GEAR;

    // Callbacks
    TransactionCallback  _onTransaction = nullptr;
    ThrottleCallback     _onThrottle = nullptr;
    BrakeCallback        _onBrake = nullptr;
    LightsCallback       _onLights = nullptr;
    TurnSignalCallback   _onTurnSignals = nullptr;
    HazardLightsCallback _onHazardLights = nullptr;
    HornCallback         _onHorn = nullptr;
    GearCallback         _onGear = nullptr;
    DriveModeCallback    _onDriveMode = nullptr;
    DualCallback         _onDual = nullptr;
    PMenuCallback        _onPMenu = nullptr;

    void _unpackInputs(const KukirinG3Pro::TXPacket& tx, KukirinInputs& in, float frameDt) {
        const uint32_t nowMs = millis();

        // Asymmetric Brake Dispatch: Instant 1-frame trip, 2-frame debounced release
        in.brakeActive = _inputFilter.updateBrake(KukirinG3Pro::isBrakeActive(tx), _timing.brakeReleaseQuorum);

        // Throttle Rate Limiting: Asymmetric positive slew governing with instant brake cutoff
        const uint16_t rawThrot = KukirinG3Pro::extractRawThrottle(tx);
        in.rawThrottle = _inputFilter.updateThrottle(rawThrot, in.brakeActive, _throtCfg);
        in.normalizedThrottle = KukirinG3Pro::normalizeThrottle(in.rawThrottle, _throtCfg);

        // Drive Mode & Function Quorums
        const KukirinG3Pro::DriveMode rawMode = KukirinG3Pro::extractDriveMode(tx);
        const KukirinG3Pro::DriveMode filteredMode = _inputFilter.updateDriveMode(rawMode, _timing.driveModeQuorum);
        in.driveMode = filteredMode;
        in.gear = KukirinG3Pro::driveModeToGear(filteredMode);

        const uint8_t rawFunc = tx.functionBitmask;
        const uint8_t filteredFunc = _inputFilter.updateFunctions(rawFunc, _timing.functionQuorum);
        in.lightsActive = (filteredFunc & static_cast<uint8_t>(KukirinG3Pro::FunctionFlag::LightsOn)) != 0;
        in.hornActive = KukirinG3Pro::isHornActive(tx);

        // Configuration Staging: Speed-gated P-menu staging with standstill ARQ resynchronization
        const KukirinG3Pro::DecodedSettings decoded = KukirinG3Pro::decodeTx(tx);
        const float curSpeed = _atomicSpeedMph.load(std::memory_order_relaxed);
        const bool throttleNeutral = (in.rawThrottle < _throtCfg.deadbandEngage);
        in.pmenuUpdated = _inputFilter.updatePMenu(decoded, in.settings, curSpeed, throttleNeutral, _timing.walkingSpeedLimitMph, _timing.pmenuQuorum);

        in.frameDt = frameDt;
        in.packetCount = _packetCounter;

        // Lighting Qualification: 2-frame turn signal quorum and paced hazard qualification
        const bool rawLeft = KukirinG3Pro::isLeftTurnActive(tx);
        const bool rawRight = KukirinG3Pro::isRightTurnActive(tx);
        _inputFilter.updateTurnAndHazard(rawLeft, rawRight, nowMs, in.leftTurn, in.rightTurn, in.hazardLightsActive,
                                         _timing.turnSignalQuorum, _timing.hazardMinTapGapMs, _timing.hazardWindowMs);
    }

    void _publishInputs(const KukirinInputs& in) {
        // Atomic scalar publishing
        _atomicNormalizedThrottle.store(in.normalizedThrottle, std::memory_order_relaxed);
        _atomicRawThrottle.store(in.rawThrottle, std::memory_order_relaxed);
        _atomicBrakeActive.store(in.brakeActive, std::memory_order_relaxed);
        _atomicGearLevel.store(in.gear, std::memory_order_relaxed);
        _atomicLightsActive.store(in.lightsActive, std::memory_order_relaxed);
        _atomicLeftTurnActive.store(in.leftTurn, std::memory_order_relaxed);
        _atomicRightTurnActive.store(in.rightTurn, std::memory_order_relaxed);
        _atomicHazardActive.store(in.hazardLightsActive, std::memory_order_relaxed);
        _atomicHornActive.store(in.hornActive, std::memory_order_relaxed);

        // Safe full struct copy under bounded mutex
        if (_mutex && xSemaphoreTake(_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            _cachedInputs = in;
            xSemaphoreGive(_mutex);
        }

        // Discrete switches bitmask: [0]=Brake, [1]=Lights, [2]=LeftTurn, [3]=RightTurn, [4]=Hazard, [5]=Horn, [6]=Dual
        const uint8_t current_mask = (in.brakeActive ? static_cast<uint8_t>(KukirinG3Pro::SwitchMask::Brake) : 0) |
                                     (in.lightsActive ? static_cast<uint8_t>(KukirinG3Pro::SwitchMask::Lights) : 0) |
                                     (in.leftTurn ? static_cast<uint8_t>(KukirinG3Pro::SwitchMask::LeftTurn) : 0) |
                                     (in.rightTurn ? static_cast<uint8_t>(KukirinG3Pro::SwitchMask::RightTurn) : 0) |
                                     (in.hazardLightsActive ? static_cast<uint8_t>(KukirinG3Pro::SwitchMask::HazardLights) : 0) |
                                     (in.hornActive ? static_cast<uint8_t>(KukirinG3Pro::SwitchMask::Horn) : 0) |
                                     (_dualModeState ? static_cast<uint8_t>(KukirinG3Pro::SwitchMask::Dual) : 0);

        const uint8_t changed = current_mask ^ _prevSwitchMask;
        if (changed != 0) {
            _prevSwitchMask = current_mask;
            if ((changed & static_cast<uint8_t>(KukirinG3Pro::SwitchMask::Brake)) && _onBrake)               _onBrake(in.brakeActive);
            if ((changed & static_cast<uint8_t>(KukirinG3Pro::SwitchMask::Lights)) && _onLights)             _onLights(in.lightsActive);
            if ((changed & (static_cast<uint8_t>(KukirinG3Pro::SwitchMask::LeftTurn) |
                            static_cast<uint8_t>(KukirinG3Pro::SwitchMask::RightTurn))) && _onTurnSignals)   _onTurnSignals(in.leftTurn, in.rightTurn);
            if ((changed & static_cast<uint8_t>(KukirinG3Pro::SwitchMask::HazardLights)) && _onHazardLights) _onHazardLights(in.hazardLightsActive);
            if ((changed & static_cast<uint8_t>(KukirinG3Pro::SwitchMask::Horn)) && _onHorn)                 _onHorn(in.hornActive);
            if ((changed & static_cast<uint8_t>(KukirinG3Pro::SwitchMask::Dual)) && _onDual)                 _onDual(_dualModeState);
        }

        // Gear / Drive Mode edge detection
        if (in.gear != _prevGear) {
            _prevGear = in.gear;
            if (_onGear) _onGear(in.gear);
            if (_onDriveMode) _onDriveMode(in.driveMode);
        }

        // Throttle change edge / deadband detection (>= 1% or boundary 0% / 100%)
        const bool throt_changed = (_prevRawThrottle == 0xFFFF) ||
                                   (fabsf(in.normalizedThrottle - _prevNormalizedThrottle) >= 0.01f) ||
                                   (in.normalizedThrottle == 0.0f && _prevNormalizedThrottle > 0.0f) ||
                                   (in.normalizedThrottle == 1.0f && _prevNormalizedThrottle < 1.0f);
        if (throt_changed) {
            _prevRawThrottle = in.rawThrottle;
            _prevNormalizedThrottle = in.normalizedThrottle;
            if (_onThrottle) _onThrottle(in.rawThrottle, in.normalizedThrottle);
        }

        if (_onPMenu && in.pmenuUpdated) _onPMenu(in.settings);
    }

    void _sendResponse(const KukirinResponse& resp, const KukirinInputs& in) {
        // Calculate discrete speedRaw
        uint16_t speedRaw;
        if (resp.errorCode == KukirinG3Pro::ErrorCode::E03_MainController) {
            speedRaw = KukirinG3Pro::SPEED_RAW_STATIONARY;
        } else {
            speedRaw = KukirinG3Pro::encodeSpeedRawMph(resp.speedMph, in.settings.wheelInches);
        }

        // Format baseline packed response
        const bool is_keepalive = (resp.errorCode == KukirinG3Pro::ErrorCode::None) && KukirinG3Pro::isKeepaliveFrame(_packetCounter);
        KukirinG3Pro::formatResponsePacket(_rxPacket, speedRaw, resp.dualMode, 0, resp.tripMode, is_keepalive);

        // Apply brake warning icon
        if (in.brakeActive) {
            _rxPacket.systemStatus |= static_cast<uint8_t>(KukirinG3Pro::SystemStatus::Braking);
        }

        // Inject diagnostic error code
        if (resp.errorCode != KukirinG3Pro::ErrorCode::None) {
            KukirinG3Pro::applyErrorCode(_rxPacket, resp.errorCode);
        }

        // 12-byte Bitwise XOR Checksum
        _rxPacket.calculatedStatus = KukirinG3Pro::calculateRXChecksum(_rxPacket);

        if (resp.errorCode == KukirinG3Pro::ErrorCode::E06_ReceiverTimeoutCrc) {
            _rxPacket.calculatedStatus ^= 0xFF;
        }

        _port->write(reinterpret_cast<const uint8_t*>(&_rxPacket), sizeof(KukirinG3Pro::RXPacket));
    }

    void _processPhysicalButton(uint32_t nowMs) {
        if (_buttonDsPin < 0) return;
        const bool raw_pressed = (digitalRead(_buttonDsPin) == LOW);
        if (_accessoryEngine.processButton(nowMs, raw_pressed, _timing.buttonDebounceMs)) {
            const bool dualState = _accessoryEngine.dualMode;
            _dualModeState = dualState;
            _atomicDualMode.store(dualState, std::memory_order_relaxed);
            if (_mutex && xSemaphoreTake(_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
                _cachedResponse.dualMode = dualState;
                xSemaphoreGive(_mutex);
            }
            if (dualState) {
                _prevSwitchMask |= static_cast<uint8_t>(KukirinG3Pro::SwitchMask::Dual);
            } else {
                _prevSwitchMask &= ~static_cast<uint8_t>(KukirinG3Pro::SwitchMask::Dual);
            }
            if (_onDual) _onDual(dualState);
        }
    }

    void _updateAccessories(uint32_t nowMs) {
        const bool lights = _atomicLightsActive.load(std::memory_order_relaxed);
        const bool brake  = _atomicBrakeActive.load(std::memory_order_relaxed);
        const bool left   = _atomicLeftTurnActive.load(std::memory_order_relaxed);
        const bool right  = _atomicRightTurnActive.load(std::memory_order_relaxed);
        const bool hazard = _atomicHazardActive.load(std::memory_order_relaxed);
        const bool horn   = _atomicHornActive.load(std::memory_order_relaxed);

        _accessoryEngine.update(nowMs, lights, brake, left, right, hazard, horn, _timing);
        _accessoryEngine.applyHardwarePins(_pins);
    }
};
