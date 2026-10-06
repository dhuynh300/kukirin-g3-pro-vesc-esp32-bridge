#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <atomic>
#include "KukirinG3ProProtocol.h"

/**
 * @struct KukirinInputs
 * @brief Filtered rider inputs from one display frame.
 */
struct KukirinInputs {
    uint16_t rawThrottle = 0;              /**< @brief Throttle after the idle and full-throttle switches, 0..1000. */
    float    normalizedThrottle = 0.0f;   /**< @brief Throttle 0..1. */
    bool     brakeActive = false;          /**< @brief Brake lever pulled. */
    bool     lightsActive = false;         /**< @brief Lights on. */
    bool     leftTurn = false;             /**< @brief Left turn signal on. */
    bool     rightTurn = false;            /**< @brief Right turn signal on. */
    bool     hazardLightsActive = false;   /**< @brief Hazards on (turn switch pushed twice). */
    bool     hornActive = false;           /**< @brief Horn button pressed. */
    uint8_t  gear = 1;                     /**< @brief Mode 1..3. */
    KukirinG3Pro::DriveMode driveMode = KukirinG3Pro::DriveMode::Eco;
    KukirinG3Pro::DecodedSettings settings;/**< @brief P-menu settings in effect. */
    bool     pmenuUpdated = false;         /**< @brief The settings in effect changed with this frame. */
    float    frameDt = 0.1f;               /**< @brief Time since the previous valid frame, s. */
    uint32_t packetCount = 0;              /**< @brief Valid frames received so far. */
};

/**
 * @struct KukirinResponse
 * @brief What the next status frame shows on the display.
 */
struct KukirinResponse {
    float speedMph = 0.0f;                    /**< @brief Speed to show, mph. */
    bool  dualMode = true;                    /**< @brief Show dual drive. */
    KukirinG3Pro::ErrorCode errorCode = KukirinG3Pro::ErrorCode::None; /**< @brief Error code to show. */
    bool  tripMode = false;                   /**< @brief Trip odometer instead of total. */
};

/**
 * @class KukirinG3ProDisplay
 * @brief Plays the controller's role towards the G3 Pro display.
 *
 * update() reads command frames and answers each valid one immediately with a status frame; the display shows
 * E-006 if replies stop for about 2.5 s. Inputs can be used in two ways: a transaction callback called once per
 * frame (it can set the reply), or getters and setters. Getters read std::atomic copies, so they are safe to
 * call from the other core; the full structs are guarded by a mutex with a 2-5 ms wait limit.
 * Optional: lights, turn signals, brake light and horn outputs (initGPIO()). No heap allocation after construction.
 */
class KukirinG3ProDisplay {
public:
    // Default pins (KukirinG3Pro::Pins)
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

    // Callback types (plain function pointers)
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
     * @param[in] displaySerial UART connected to the display, e.g. &Serial2.
     * @param[in] rxPin RX GPIO; the display's 5 V TX line must go through a divider.
     * @param[in] txPin TX GPIO.
     * @param[in] buttonDsPin Dual/single button input, -1 if none.
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

    /** @brief Opens the UART, 8N1. */
    void begin(uint32_t baudRate = 9600) {
        if (_port != nullptr) {
            _port->begin(baudRate, SERIAL_8N1, _rxPin, _txPin);
        }
    }

    /** @brief Sets up the light, horn and button pins and enables the accessory outputs in update(). */
    void initGPIO(const KukirinG3Pro::PinConfig& pins = KukirinG3Pro::PinConfig{}) {
        _pins = pins;
        _buttonDsPin = pins.buttonDs;
        KukirinG3Pro::AccessoryEngine::initGPIO(_pins);
        _gpioInitialized = true;
    }

    // Configuration
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

    // Callbacks

    /** @brief Called once per valid frame, before the reply is sent; it can change the reply. */
    void onTransaction(TransactionCallback cb) { _onTransaction = cb; }

    // Called when the value changes
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

    // Reply content (safe from either core)
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
        _atomicErrorCode.store(static_cast<uint8_t>(code), std::memory_order_relaxed);
        if (_mutex && xSemaphoreTake(_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            _cachedResponse.errorCode = code;
            xSemaphoreGive(_mutex);
        }
    }

    void setErrorCode(uint8_t code) {
        setErrorCode(static_cast<KukirinG3Pro::ErrorCode>(code));
    }

    /** @brief Error code being shown, as a number (0 = none). */
    inline uint8_t getErrorCodeRaw() const {
        return _atomicErrorCode.load(std::memory_order_relaxed);
    }

    /** @brief Error code being shown. */
    inline KukirinG3Pro::ErrorCode getErrorCode() const {
        return static_cast<KukirinG3Pro::ErrorCode>(_atomicErrorCode.load(std::memory_order_relaxed));
    }

    void setTripMode(bool trip) {
        _atomicTripMode.store(trip, std::memory_order_relaxed);
        if (_mutex && xSemaphoreTake(_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            _cachedResponse.tripMode = trip;
            xSemaphoreGive(_mutex);
        }
    }

    // Latest inputs (std::atomic, safe from either core)
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

    /** @brief Copies the latest full input struct. False if the mutex was busy for 5 ms. */
    bool getInputs(KukirinInputs& out) const {
        if (_mutex && xSemaphoreTake(_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            out = _cachedInputs;
            xSemaphoreGive(_mutex);
            return true;
        }
        return false;
    }

    /** @brief After a bad frame, moves the next frame start in the buffer to the front. True if one was found. */
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

    /**
     * @brief Reads available bytes, answers a complete frame, and updates the accessory outputs. Does not block.
     * @return True if a frame was received and answered in this call.
     */
    bool update() {
        if (_port == nullptr) return false;

        const uint32_t nowMs = millis();

        if (_gpioInitialized) {
            _processPhysicalButton(nowMs);
            _updateAccessories(nowMs);
        }

        // Drop a partial frame after framingTimeoutMs of silence
        if (_txIdx > 0 && (nowMs - _lastRxByteTime >= _timing.framingTimeoutMs)) {
            _txIdx = 0;
            _inputFilter.resetPending();
        }

        // Read up to MAX_SERIAL_READ_ITERATIONS bytes
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

                        // A frame with a good checksum but impossible values is answered, but its inputs are not used
                        KukirinInputs localInputs;
                        const bool plausible = KukirinG3Pro::isPlausibleTXPacket(command);
                        if (plausible) {
                            _packetCounter++;
                            _atomicLastValidPacketMs.store(nowMs, std::memory_order_relaxed);
                            _lastPacketMs = nowMs;
                            _unpackInputs(command, localInputs, frameDt);
                        } else {
                            _rejectedPacketCount++;
                            localInputs.frameDt = frameDt;
                            localInputs.packetCount = _packetCounter;
                        }

                        // Reply content from the setters...
                        KukirinResponse localResponse;
                        if (_mutex && xSemaphoreTake(_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
                            localResponse = _cachedResponse;
                            xSemaphoreGive(_mutex);
                        } else {
                            // Mutex busy: use the atomic copies
                            localResponse.speedMph = _atomicSpeedMph.load(std::memory_order_relaxed);
                            localResponse.dualMode = _atomicDualMode.load(std::memory_order_relaxed);
                            localResponse.errorCode = static_cast<KukirinG3Pro::ErrorCode>(_atomicErrorCode.load(std::memory_order_relaxed));
                            localResponse.tripMode = _atomicTripMode.load(std::memory_order_relaxed);
                        }

                        // ...or from the transaction callback, which runs before the change callbacks below
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

                        // Reply right away; the display expects one reply per frame
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

    // Copies readable from either core
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

    KukirinG3Pro::BridgeTimingConfig _timing;
    KukirinG3Pro::ThrottleConfig     _throtCfg;

    KukirinG3Pro::AccessoryEngine _accessoryEngine;
    bool     _dualModeState = true;
    static constexpr uint8_t UNINITIALIZED_GEAR = 0xFF;
    static constexpr uint8_t UNINITIALIZED_SWITCH_MASK = static_cast<uint8_t>(KukirinG3Pro::SwitchMask::Uninitialized);
    uint8_t  _prevSwitchMask = UNINITIALIZED_SWITCH_MASK; // so the first frame reports every switch
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

        // Brake first: it zeroes the throttle below
        in.brakeActive = _inputFilter.updateBrake(KukirinG3Pro::isBrakeActive(tx), _timing.brakeReleaseQuorum);

        const uint16_t rawThrot = KukirinG3Pro::extractRawThrottle(tx);
        in.rawThrottle = _inputFilter.updateThrottle(rawThrot, in.brakeActive, _throtCfg);
        in.normalizedThrottle = KukirinG3Pro::normalizeThrottle(in.rawThrottle, _throtCfg);

        const KukirinG3Pro::DriveMode rawMode = KukirinG3Pro::extractDriveMode(tx);
        const KukirinG3Pro::DriveMode filteredMode = _inputFilter.updateDriveMode(rawMode, _timing.driveModeQuorum);
        in.driveMode = filteredMode;
        in.gear = KukirinG3Pro::driveModeToGear(filteredMode);

        const uint8_t rawFunc = tx.functionBitmask;
        const uint8_t filteredFunc = _inputFilter.updateFunctions(rawFunc, _timing.functionQuorum);
        in.lightsActive = (filteredFunc & static_cast<uint8_t>(KukirinG3Pro::FunctionFlag::LightsOn)) != 0;
        in.hornActive = KukirinG3Pro::isHornActive(tx);

        // P-menu: changes made while moving wait until the vehicle slows down
        const KukirinG3Pro::DecodedSettings decoded = KukirinG3Pro::decodeTx(tx);
        const float curSpeed = _atomicSpeedMph.load(std::memory_order_relaxed);
        const bool throttleNeutral = (in.rawThrottle < _throtCfg.deadbandEngage);
        in.pmenuUpdated = _inputFilter.updatePMenu(decoded, in.settings, curSpeed, throttleNeutral, _timing.walkingSpeedLimitMph, _timing.pmenuQuorum);

        in.frameDt = frameDt;
        in.packetCount = _packetCounter;

        const bool rawLeft = KukirinG3Pro::isLeftTurnActive(tx);
        const bool rawRight = KukirinG3Pro::isRightTurnActive(tx);
        _inputFilter.updateTurnAndHazard(rawLeft, rawRight, nowMs, in.leftTurn, in.rightTurn, in.hazardLightsActive,
                                         _timing.turnSignalQuorum, _timing.hazardMinTapGapMs, _timing.hazardWindowMs);
    }

    void _publishInputs(const KukirinInputs& in) {
        _atomicNormalizedThrottle.store(in.normalizedThrottle, std::memory_order_relaxed);
        _atomicRawThrottle.store(in.rawThrottle, std::memory_order_relaxed);
        _atomicBrakeActive.store(in.brakeActive, std::memory_order_relaxed);
        _atomicGearLevel.store(in.gear, std::memory_order_relaxed);
        _atomicLightsActive.store(in.lightsActive, std::memory_order_relaxed);
        _atomicLeftTurnActive.store(in.leftTurn, std::memory_order_relaxed);
        _atomicRightTurnActive.store(in.rightTurn, std::memory_order_relaxed);
        _atomicHazardActive.store(in.hazardLightsActive, std::memory_order_relaxed);
        _atomicHornActive.store(in.hornActive, std::memory_order_relaxed);

        if (_mutex && xSemaphoreTake(_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            _cachedInputs = in;
            xSemaphoreGive(_mutex);
        }

        // Call the change callbacks for switches that changed
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

        if (in.gear != _prevGear) {
            _prevGear = in.gear;
            if (_onGear) _onGear(in.gear);
            if (_onDriveMode) _onDriveMode(in.driveMode);
        }

        // Throttle callback on a change of 1 % or more, or on reaching 0 or full
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
        // Speed shows 0 while the VESC link is lost (E-003)
        uint16_t speedRaw;
        if (resp.errorCode == KukirinG3Pro::ErrorCode::E03_MainController) {
            speedRaw = KukirinG3Pro::SPEED_RAW_STATIONARY;
        } else {
            speedRaw = KukirinG3Pro::encodeSpeedRawMph(resp.speedMph, in.settings.wheelInches);
        }

        const bool is_keepalive = (resp.errorCode == KukirinG3Pro::ErrorCode::None) && KukirinG3Pro::isKeepaliveFrame(_packetCounter);
        KukirinG3Pro::formatResponsePacket(_rxPacket, speedRaw, resp.dualMode, 0, resp.tripMode, is_keepalive);

        if (in.brakeActive) {
            _rxPacket.systemStatus |= static_cast<uint8_t>(KukirinG3Pro::SystemStatus::Braking);
        }

        if (resp.errorCode != KukirinG3Pro::ErrorCode::None) {
            KukirinG3Pro::applyErrorCode(_rxPacket, resp.errorCode);
        }

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
