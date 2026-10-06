//! KNOWN PITFALLS & FAILURE MODES:
//! - P01 [VESC UART Baud Rate]: Must strictly operate at 230,400 bps. 115,200 bps causes framing corruption and link loss.
//! - P10 [Dead-man Watchdog]: 5 missed frames (100ms) trips failsafe: Ticks 1-4 Zero-Order Hold (ZOH), Tick 5 cuts torque to 0.0A freewheel.
//! - P11 [Multi-Packet Pacing]: Secondary debug/telemetry packets must not be dispatched back-to-back on same tick (enforce >= 2ms wire gap).
//! - P12 [Packet Framing & CRC]: Frames use 0xAA 0x55 header with CCITT CRC16. Sliding-window resync via memmove required on CRC error.
//! - P13 [Zero Heap Allocation]: Control path must never invoke malloc/new/free in real-time loops.

#pragma once

#include <Arduino.h>

/**
 * @namespace VESCConfig
 * @brief Authoritative configuration constants for VESC powertrain and CAN bus topology.
 */
namespace VESCConfig {
    static constexpr uint8_t CAN_ID_REAR_VESC  = 26;  /**< @brief Master Rear VESC controller connected via UART1. */
    static constexpr uint8_t CAN_ID_FRONT_VESC = 123; /**< @brief Slave Front VESC controller connected via CAN bus. */
}

/**
 * @struct TXPayloadControl
 * @brief Binary control payload transmitted to the VESC LispBM motor control thread.
 * @details Transmits normalized throttle, regenerative brake commands, dual-motor flags, and speed limit in cm/s.
 * @note Enforces 1-byte struct alignment via `__attribute__((packed))`. Total size: 7 bytes.
 */
struct __attribute__((packed)) TXPayloadControl {
    uint16_t throttle;        /**< @brief Normalized throttle request (0 to 10000 corresponding to 0.0% to 100.0%). Offset: 0-1. */
    uint16_t brake;           /**< @brief Normalized regenerative brake request (0 to 10000 corresponding to 0.0% to 100.0%). Offset: 2-3. */
    uint8_t  dual_mode;       /**< @brief Dual motor drive mode flag (1 = Dual Drive, 0 = Single Motor). Offset: 4. */
    uint16_t speed_limit_cms; /**< @brief Speed ceiling in cm/s (0 = Unlimited / Mode 3). Offset: 5-6. */
};

/**
 * @struct RXTelemetryPayload
 * @brief Binary telemetry payload received from the VESC LispBM telemetry thread.
 * @details Contains motor speed feedback in cm/s (m/s * 100).
 * @note Enforces 1-byte struct alignment via `__attribute__((packed))`. Total size: 3 bytes.
 */
struct __attribute__((packed)) RXTelemetryPayload {
    int16_t speed;       /**< @brief Vehicle speed in cm/s (m/s * 100). Offset: 0-1. Valid range: -327.68 to +327.67 m/s (~733 mph). */
    uint8_t fault_flags; /**< @brief Diagnostic fault flags (Bit 0: Front VESC CAN desync > 0.1s). Offset: 2. */
};

/**
 * @struct TXPayloadTestCommand
 * @brief Test harness command payload dispatched to VESC LispBM for automated closed-loop testing.
 * @note Enforces 1-byte struct alignment via `__attribute__((packed))`. Total size: 5 bytes.
 */
struct __attribute__((packed)) TXPayloadTestCommand {
    uint8_t test_id;         /**< @brief Active test case ID (1..20). Offset: 0. */
    uint8_t sim_flags;       /**< @brief Simulation flags (B0: Force CAN desync, B1: Synth speed, B2: Force PLC decay). Offset: 1. */
    int16_t sim_speed_cm_s;  /**< @brief Synthetic speed in cm/s (0.01 m/s). Offset: 2-3. */
    uint8_t reserved;        /**< @brief Plausibility padding (0x00). Offset: 4. */
};

/**
 * @struct RXPayloadTestFeedback
 * @brief Evaluation feedback payload returned by VESC LispBM to ESP32 for closed-loop assertion.
 * @note Enforces 1-byte struct alignment via `__attribute__((packed))`. Total size: 8 bytes.
 */
struct __attribute__((packed)) RXPayloadTestFeedback {
    uint8_t ack_test_id;         /**< @brief Echoed test case ID. Offset: 0. */
    uint8_t eval_flags;          /**< @brief Safety interlock bitmask (B0: Brake priority, B1: Unarmed lock, B2: Clamp, B3: Deadman, B4: CAN desync, B5: Re-arm lock, B6: CRC err, B7: Frame err). Offset: 1. */
    int16_t sim_rear_current;    /**< @brief Commanded/simulated Rear phase current in 0.1A units (e.g. 225 = 22.5A). Offset: 2-3. */
    int16_t sim_front_current;   /**< @brief Commanded/simulated Front phase current in 0.1A units. Offset: 4-5. */
    int16_t measured_speed;      /**< @brief Computed vehicle speed in cm/s. Offset: 6-7. */
};

/**
 * @enum MessageType
 * @brief Binary protocol message type identifiers for ESP32 <-> VESC UART communication.
 */
enum MessageType : uint8_t {
    MSG_TX_HEARTBEAT  = 0x00, /**< @brief Keep-alive heartbeat message (Length 0, transmitted at 10/50 Hz). */
    MSG_TX_CONTROL    = 0x01, /**< @brief Real-time throttle and brake control command payload. */
    MSG_TX_AUDIO      = 0x02, /**< @brief Quantized 8-bit PCM audio sample stream for FOC motor sound. */
    MSG_RX_TELEMETRY  = 0x03, /**< @brief Live vehicle telemetry feedback from VESC. */
    MSG_TEST_COMMAND  = 0x04, /**< @brief Closed-loop automated test command payload. */
    MSG_TEST_FEEDBACK = 0x05, /**< @brief Closed-loop evaluation feedback payload from VESC LispBM. */
    MSG_TX_DEBUG_LOG  = 0x06  /**< @brief ASCII diagnostic string tunneled to VESC Tool BLE terminal. */
};

/**
 * @class VESCUARTBridge
 * @brief Handles framing, CRC16 validation, heartbeats, and zero-copy dispatch for ESP32 <-> VESC communication.
 * @details Operates over hardware serial (230,400 bps standard) with CRC16-CCITT packet integrity checks.
 *          Enforces automatic 100ms dead-man heartbeat transmission and 50ms RX framing timeout watchdog.
 * @note Core affinity: Core 1 (real-time vehicle loop). Non-blocking, zero dynamic memory allocation.
 */
class VESCUARTBridge {
public:
    /**
     * @brief Zero-allocation message dispatcher callback function pointer type.
     * @param[in] type Decoded MessageType enum value.
     * @param[in] payload Pointer to the verified payload byte array.
     * @param[in] len Length in bytes of the payload.
     */
    typedef void (*MessageCallback)(MessageType type, const uint8_t* payload, uint8_t len);

    /**
     * @brief Constructs a new VESC UART Bridge instance and initializes the hardware serial interface.
     * @param[in] serial_port Pointer to HardwareSerial peripheral (e.g. &Serial1).
     * @param[in] baud_rate Serial baud rate (defaults to 230,400 bps standard).
     * @param[in] rx_pin ESP32 GPIO pin for UART RX. Valid range: 0-39 (default GPIO 34).
     * @param[in] tx_pin ESP32 GPIO pin for UART TX. Valid range: 0-33 (default GPIO 32).
     */
    VESCUARTBridge(HardwareSerial* serial_port, uint32_t baud_rate = 230400, int8_t rx_pin = 34, int8_t tx_pin = 32);
    
    /**
     * @brief Initializes or reconfigures the hardware serial port (safe to call in setup()).
     * @param[in] baud_rate Serial baud rate (defaults to constructor value if 0).
     * @param[in] rx_pin ESP32 GPIO pin for UART RX (defaults to constructor value if -1).
     * @param[in] tx_pin ESP32 GPIO pin for UART TX (defaults to constructor value if -1).
     */
    void begin(uint32_t baud_rate = 0, int8_t rx_pin = -1, int8_t tx_pin = -1);

    /**
     * @brief Destructor that cleans up FreeRTOS synchronization primitives.
     * @note Core affinity: Core 1 / Core 0.
     */
    virtual ~VESCUARTBridge();
    
    /**
     * @brief Real-time update loop handling RX stream processing, framing watchdogs, and keep-alive heartbeats.
     * @note Must be called continuously in loop() on Core 1. Non-blocking, bounded iteration loop.
     */
    void update(); 

    /**
     * @brief Transmits a structured binary message with framing and CRC16-CCITT to the VESC.
     * @param[in] type The MessageType identifier.
     * @param[in] payload Pointer to the binary payload data buffer (may be nullptr if len == 0).
     * @param[in] len Length in bytes of the payload. Valid range: 0 to 255.
     * @return bool True if successfully formatted and written to the UART TX FIFO; false on mutex timeout or invalid arguments.
     * @note Thread-safe across Core 0 and Core 1 via bounded mutex. Zero-copy transmission directly to hardware TX buffer.
     * @see MessageType, calculate_crc16
     */
    bool send_message(MessageType type, const uint8_t* payload, uint8_t len);

    /**
     * @brief Transmits a structured control payload (MSG_TX_CONTROL) to the VESC.
     * @param[in] payload Reference to TXPayloadControl containing throttle, brake, and drive mode.
     * @return bool True if successfully formatted and written to the UART TX FIFO; false on error.
     */
    inline bool send_control(const TXPayloadControl& payload) {
        return send_message(MSG_TX_CONTROL, reinterpret_cast<const uint8_t*>(&payload), sizeof(TXPayloadControl));
    }

    /**
     * @brief Transmits an ASCII debug log string (MSG_TX_DEBUG_LOG) tunneled to VESC Tool mobile BLE terminal.
     * @param[in] str Null-terminated ASCII text string. Clamped to 120 bytes max.
     * @return bool True if successfully formatted and dispatched; false on error or null pointer.
     * @note Non-blocking, zero dynamic heap allocation.
     */
    inline bool send_debug_log(const char* str) {
        if (!str) return false;
        size_t len = strlen(str);
        if (len == 0) return true;
        if (len > 120) len = 120;
        // Wire pacing enforced centrally in send_message (>= 2ms gap for all non-heartbeat frames)
        return send_message(MSG_TX_DEBUG_LOG, reinterpret_cast<const uint8_t*>(str), static_cast<uint8_t>(len));
    }

    /**
     * @brief Formats and transmits a printf-style diagnostic string tunneled to VESC Tool mobile BLE terminal.
     * @param[in] fmt Printf format string.
     * @param[in] ... Variable format arguments.
     * @return bool True if successfully formatted and dispatched; false on error or null pointer.
     * @note Stack-allocated 128-byte buffer. Strictly zero heap allocation. Clamped to 120 bytes max.
     */
    inline bool send_debug_logf(const char* fmt, ...) {
        if (!fmt) return false;
        char buf[128];
        va_list args;
        va_start(args, fmt);
        int len = vsnprintf(buf, sizeof(buf), fmt, args);
        va_end(args);
        if (len <= 0) return false;
        if (len > 120) len = 120;
        // Wire pacing enforced centrally in send_message (>= 2ms gap for all non-heartbeat frames)
        return send_message(MSG_TX_DEBUG_LOG, reinterpret_cast<const uint8_t*>(buf), static_cast<uint8_t>(len));
    }

    /**
     * @brief Registers the callback invoked upon successful decoding and CRC validation of an incoming frame.
     * @param[in] cb Static function pointer of type MessageCallback. Pass nullptr to disable.
     * @note Zero heap allocation. Core affinity: Core 1.
     */
    void set_receive_callback(MessageCallback cb);

    /**
     * @brief Configures the keepalive heartbeat transmission interval in milliseconds.
     * @param[in] interval_ms Period in ms between keepalive heartbeats (default 20ms for 50Hz streaming).
     *                        Set to 0 to disable automatic keepalives.
     * @note Production standard is 20ms (50Hz).
     */
    void set_heartbeat_interval_ms(uint32_t interval_ms) { _heartbeat_interval_ms = interval_ms; }

    /**
     * @brief Gets the configured keepalive heartbeat transmission interval in milliseconds.
     * @return uint32_t Heartbeat period in ms.
     */
    uint32_t get_heartbeat_interval_ms() const { return _heartbeat_interval_ms; }

    /**
     * @brief Checks if valid telemetry packets have been received from VESC within the specified timeout.
     * @param[in] timeout_ms Maximum allowable silence threshold in ms (default 100ms = 5 frames at 50Hz).
     * @return bool True if connection is alive and healthy; false if timed out or not yet received.
     */
    bool is_connected(uint32_t timeout_ms = 100) const;

    /**
     * @brief Returns the timestamp in milliseconds of the last successfully decoded, CRC-verified packet.
     * @return uint32_t Millis timestamp of last valid frame.
     */
    uint32_t get_last_valid_rx_ms() const;

private:
    /**
     * @brief Transmits a zero-length keep-alive heartbeat frame (MSG_TX_HEARTBEAT).
     * @note Core affinity: Core 1. Zero dynamic memory allocation.
     * @see MSG_TX_HEARTBEAT, send_message
     */
    void send_heartbeat();

    /**
     * @brief Feed a single incoming serial byte into the RX framing state machine.
     * @param[in] incoming_byte Raw byte from UART RX FIFO. Valid range: 0x00 to 0xFF.
     * @note Core affinity: Core 1. Zero dynamic memory allocation.
     */
    void process_incoming_byte(uint8_t incoming_byte);

    /**
     * @brief Computes single-pass CRC16-CCITT (Polynomial 0x1021) with initial value 0x0000.
     * @param[in] data Pointer to input data buffer.
     * @param[in] len Number of bytes to process.
     * @return uint16_t Computed CRC16 checksum.
     * @note Thread-safe, reentrant, zero allocation.
     */
    uint16_t calculate_crc16(const uint8_t *data, uint16_t len);

    /**
     * @brief Computes running CRC16-CCITT across discontinuous memory buffers.
     * @param[in] data Pointer to input data chunk.
     * @param[in] len Number of bytes in this chunk.
     * @param[in] current_crc Running CRC accumulator from previous segments.
     * @return uint16_t Updated running CRC16 checksum.
     * @note Thread-safe, reentrant, zero allocation.
     */
    uint16_t calculate_crc16(const uint8_t *data, uint16_t len, uint16_t current_crc);

    HardwareSerial* _serial = nullptr;                                  /**< @brief Hardware UART stream pointer. */
    MessageCallback _on_message = nullptr;                              /**< @brief Registered RX callback. */
    SemaphoreHandle_t _tx_mutex = nullptr;                              /**< @brief Mutex protecting concurrent UART packet transmission across cores. */
    uint32_t _baud_rate = 230400;                                       /**< @brief Configured UART baud rate. */
    int8_t _rx_pin = -1;                                                /**< @brief Configured UART RX pin. */
    int8_t _tx_pin = -1;                                                /**< @brief Configured UART TX pin. */
    
    // Configuration & Protocol Constants
    uint32_t _heartbeat_interval_ms = 20;                               /**< @brief Keepalive heartbeat interval (default 20ms = 50Hz standard; 0 = disabled). */
    static const uint32_t FRAMING_TIMEOUT_MS = 50;                     /**< @brief Purge partial RX packets after 50ms idle. */
    static const uint16_t MAX_PAYLOAD_SIZE = 256;                      /**< @brief Upper size limit for payload buffers. */
    static const uint8_t START_BYTE_1 = 0xAA;                          /**< @brief Protocol header sync byte 1. */
    static const uint8_t START_BYTE_2 = 0x55;                          /**< @brief Protocol header sync byte 2. */

    // State Variables
    uint32_t _last_tx_time = 0;                                         /**< @brief Timestamp of most recent TX packet (ms). */
    uint32_t _last_tx_micros = 0;                                       /**< @brief High-resolution timestamp of most recent TX packet (us). */
    bool _last_tx_was_secondary = false;                                /**< @brief True if previous TX frame was a secondary packet requiring >= 2ms pacing. */
    uint32_t _last_rx_byte_time = 0;                                    /**< @brief Timestamp of most recent RX byte received. */
    uint32_t _last_valid_rx_time = 0;                                   /**< @brief Timestamp of most recent verified CRC-passing packet. */
    
    /** @brief RX parsing state machine states. */
    enum RXState { 
        WAIT_START_1, 
        WAIT_START_2, 
        EXPECT_TYPE, 
        EXPECT_LEN, 
        EXPECT_PAYLOAD, 
        EXPECT_CRC1, 
        EXPECT_CRC2 
    };
    RXState _rx_state = WAIT_START_1;                                   /**< @brief Active parsing state. */
    
    MessageType _rx_type = MSG_TX_HEARTBEAT;                            /**< @brief Decoded packet message type. */
    uint8_t _rx_len = 0;                                                /**< @brief Decoded payload length. */
    uint8_t _rx_buffer[MAX_PAYLOAD_SIZE];                               /**< @brief Static RX payload buffer (zero dynamic allocation). */
    uint8_t _rx_index = 0;                                              /**< @brief Current payload index. */
    uint8_t _rx_crc_high = 0;                                           /**< @brief Received CRC high byte. */
};

#include "VESCSafety.h"