#pragma once

#include <Arduino.h>

/**
 * @namespace VESCConfig
 * @brief CAN IDs of the two VESCs. vesc/main.lbm defines the same values (checked by tools/check_shared_constants.py).
 */
namespace VESCConfig {
    static constexpr uint8_t CAN_ID_REAR_VESC  = 26;  /**< @brief Rear VESC: runs the LispBM script, connected to the ESP32 by UART. */
    static constexpr uint8_t CAN_ID_FRONT_VESC = 123; /**< @brief Front VESC: commanded by the rear VESC over CAN. */
}

/**
 * @struct TXPayloadControl
 * @brief Control frame payload, ESP32 to rear VESC (7 bytes, packed, little-endian).
 */
struct __attribute__((packed)) TXPayloadControl {
    uint16_t throttle;        /**< @brief Throttle, 0..10000 counts = 0..100 % of the VESC current limit. Offset 0. */
    uint16_t brake;           /**< @brief Regen brake, 0..10000 counts. Offset 2. */
    uint8_t  dual_mode;       /**< @brief 1 = drive both motors, 0 = rear only. Offset 4. */
    uint16_t speed_limit_cms; /**< @brief Speed limit in cm/s, applied per wheel by the VESC script; 0 = no limit. Offset 5. */
};

/**
 * @struct RXTelemetryPayload
 * @brief Telemetry payload, rear VESC to ESP32, sent once per script loop (3 bytes, packed, little-endian).
 */
struct __attribute__((packed)) RXTelemetryPayload {
    int16_t speed;       /**< @brief Speed of the faster wheel in cm/s. Offset 0. */
    uint8_t fault_flags; /**< @brief Bit 0: front VESC lost (no CAN status for 500 ms). Bits 1-7: rear VESC fault code. Offset 2. */
};

/**
 * @struct TXPayloadTestCommand
 * @brief Command payload of the bench test harness (5 bytes, packed). Not sent by the firmware.
 */
struct __attribute__((packed)) TXPayloadTestCommand {
    uint8_t test_id;         /**< @brief Test case ID. Offset 0. */
    uint8_t sim_flags;       /**< @brief Bit 0: simulate front VESC loss, bit 1: use sim_speed_cm_s, bit 2: simulate dropped control frames. Offset 1. */
    int16_t sim_speed_cm_s;  /**< @brief Simulated speed in cm/s. Offset 2. */
    uint8_t reserved;        /**< @brief Always 0. Offset 4. */
};

/**
 * @struct RXPayloadTestFeedback
 * @brief Result payload of the bench test harness (8 bytes, packed). Not sent by the production script.
 */
struct __attribute__((packed)) RXPayloadTestFeedback {
    uint8_t ack_test_id;         /**< @brief Test case ID being answered. Offset 0. */
    uint8_t eval_flags;          /**< @brief Bits: 0 brake priority, 1 unarmed lock, 2 clamp, 3 dead-man, 4 front lost, 5 re-arm lock, 6 CRC error, 7 frame error. Offset 1. */
    int16_t sim_rear_current;    /**< @brief Rear motor current command in 0.1 A. Offset 2. */
    int16_t sim_front_current;   /**< @brief Front motor current command in 0.1 A. Offset 4. */
    int16_t measured_speed;      /**< @brief Vehicle speed in cm/s. Offset 6. */
};

/**
 * @enum MessageType
 * @brief Message types on the ESP32-VESC link. vesc/main.lbm defines the same values.
 */
enum MessageType : uint8_t {
    MSG_TX_HEARTBEAT  = 0x00, /**< @brief Keepalive, no payload. */
    MSG_TX_CONTROL    = 0x01, /**< @brief TXPayloadControl. */
    MSG_TX_AUDIO      = 0x02, /**< @brief 8-bit PCM audio for motor-coil sound (planned; the script does not handle it yet). */
    MSG_RX_TELEMETRY  = 0x03, /**< @brief RXTelemetryPayload. */
    MSG_TEST_COMMAND  = 0x04, /**< @brief TXPayloadTestCommand (bench test harness). */
    MSG_TEST_FEEDBACK = 0x05, /**< @brief RXPayloadTestFeedback (bench test harness). */
    MSG_TX_DEBUG_LOG  = 0x06  /**< @brief ASCII log line, printed by the script to the VESC Tool terminal. */
};

/**
 * @class VESCUARTBridge
 * @brief Framed serial link to the rear VESC.
 *
 * Frame: 0xAA 0x55, type, length, payload, CRC16-CCITT (polynomial 0x1021, initial value 0, over type, length
 * and payload), CRC sent high byte first. The baud rate must match `VESC-UART-BAUD` in vesc/main.lbm.
 * The receiver drops a partial frame after 50 ms of silence and, after a CRC error, rescans the bytes it
 * already has for the next 0xAA 0x55 instead of discarding them. No heap allocation.
 */
class VESCUARTBridge {
public:
    /**
     * @brief Called for every received frame that passes the CRC check.
     * @param[in] type Message type.
     * @param[in] payload Payload bytes; valid only during the call.
     * @param[in] len Payload length in bytes.
     */
    typedef void (*MessageCallback)(MessageType type, const uint8_t* payload, uint8_t len);

    /**
     * @brief Stores the port settings; call begin() to open the port.
     * @param[in] serial_port Hardware UART, e.g. &Serial1.
     * @param[in] baud_rate Baud rate.
     * @param[in] rx_pin RX GPIO (input-only pins 34-39 are fine).
     * @param[in] tx_pin TX GPIO (must be output-capable, 0-33).
     */
    VESCUARTBridge(HardwareSerial* serial_port, uint32_t baud_rate = 230400, int8_t rx_pin = 34, int8_t tx_pin = 32);

    /**
     * @brief Opens (or reopens) the serial port with a 512-byte receive buffer.
     * @param[in] baud_rate 0 keeps the constructor value.
     * @param[in] rx_pin -1 keeps the constructor value.
     * @param[in] tx_pin -1 keeps the constructor value.
     */
    void begin(uint32_t baud_rate = 0, int8_t rx_pin = -1, int8_t tx_pin = -1);

    /** @brief Deletes the transmit mutex. */
    virtual ~VESCUARTBridge();

    /**
     * @brief Reads up to 512 received bytes, drops a stale partial frame, and sends a keepalive when due.
     *        Call from loop(); does not block.
     */
    void update();

    /**
     * @brief Frames and writes one message to the UART.
     * @param[in] type Message type.
     * @param[in] payload Payload bytes (may be nullptr when len is 0).
     * @param[in] len Payload length, 0..255.
     * @return False if the transmit mutex was not free within 10 ms or the arguments are invalid.
     * @note Safe to call from both cores. Log and audio frames, and any frame right after one, wait until 2 ms
     *       have passed since the previous frame; this wait is a busy loop of up to 2 ms.
     */
    bool send_message(MessageType type, const uint8_t* payload, uint8_t len);

    /** @brief Sends a control frame. Returns false on mutex timeout. */
    inline bool send_control(const TXPayloadControl& payload) {
        return send_message(MSG_TX_CONTROL, reinterpret_cast<const uint8_t*>(&payload), sizeof(TXPayloadControl));
    }

    /**
     * @brief Sends a log line for the VESC Tool terminal. Longer strings are cut to 120 bytes, the script's
     *        payload limit. The VESC's serial receive queue holds 128 bytes, so a long line can crowd out a
     *        control frame (docs/pitfalls.md).
     */
    inline bool send_debug_log(const char* str) {
        if (!str) return false;
        size_t len = strlen(str);
        if (len == 0) return true;
        if (len > 120) len = 120;
        return send_message(MSG_TX_DEBUG_LOG, reinterpret_cast<const uint8_t*>(str), static_cast<uint8_t>(len));
    }

    /** @brief printf-style send_debug_log(), formatted into a 128-byte stack buffer and cut to 120 bytes. */
    inline bool send_debug_logf(const char* fmt, ...) {
        if (!fmt) return false;
        char buf[128];
        va_list args;
        va_start(args, fmt);
        int len = vsnprintf(buf, sizeof(buf), fmt, args);
        va_end(args);
        if (len <= 0) return false;
        if (len > 120) len = 120;
        return send_message(MSG_TX_DEBUG_LOG, reinterpret_cast<const uint8_t*>(buf), static_cast<uint8_t>(len));
    }

    /** @brief Sets the receive callback; nullptr disables it. */
    void set_receive_callback(MessageCallback cb);

    /**
     * @brief Sends a keepalive frame whenever nothing was sent for interval_ms (default 20 ms; 0 disables).
     *        The firmware disables it because its own control frames, at least every 20 ms, keep the link alive.
     */
    void set_heartbeat_interval_ms(uint32_t interval_ms) { _heartbeat_interval_ms = interval_ms; }

    /** @brief Keepalive interval in ms (0 = disabled). */
    uint32_t get_heartbeat_interval_ms() const { return _heartbeat_interval_ms; }

    /**
     * @brief True if a valid frame arrived within the last timeout_ms. The firmware passes
     *        PowertrainConfig::VESC_COMMS_TIMEOUT_MS (500 ms); the 100 ms default is not used.
     */
    bool is_connected(uint32_t timeout_ms = 100) const;

    /** @brief millis() when the last valid frame arrived; 0 if none yet. */
    uint32_t get_last_valid_rx_ms() const;

private:
    /** @brief Sends an empty MSG_TX_HEARTBEAT frame. */
    void send_heartbeat();

    /** @brief Advances the receive state machine by one byte. */
    void process_incoming_byte(uint8_t incoming_byte);

    /** @brief CRC16-CCITT (polynomial 0x1021, initial value 0) of one buffer. */
    uint16_t calculate_crc16(const uint8_t *data, uint16_t len);

    /** @brief Continues a CRC16-CCITT over another buffer, starting from current_crc. */
    uint16_t calculate_crc16(const uint8_t *data, uint16_t len, uint16_t current_crc);

    HardwareSerial* _serial = nullptr;                                  /**< @brief UART in use. */
    MessageCallback _on_message = nullptr;                              /**< @brief Receive callback. */
    SemaphoreHandle_t _tx_mutex = nullptr;                              /**< @brief Keeps frames sent from two cores from interleaving. */
    uint32_t _baud_rate = 230400;                                       /**< @brief Baud rate. */
    int8_t _rx_pin = -1;                                                /**< @brief RX GPIO. */
    int8_t _tx_pin = -1;                                                /**< @brief TX GPIO. */

    uint32_t _heartbeat_interval_ms = 20;                               /**< @brief Keepalive interval in ms; 0 = disabled. */
    static const uint32_t FRAMING_TIMEOUT_MS = 50;                     /**< @brief Drop a partial frame after this much silence. */
    static const uint16_t MAX_PAYLOAD_SIZE = 256;                      /**< @brief Receive buffer size. */
    static const uint8_t START_BYTE_1 = 0xAA;                          /**< @brief First frame start byte. */
    static const uint8_t START_BYTE_2 = 0x55;                          /**< @brief Second frame start byte. */

    uint32_t _last_tx_time = 0;                                         /**< @brief millis() of the last frame sent. */
    uint32_t _last_tx_micros = 0;                                       /**< @brief micros() of the last frame sent, for the 2 ms gap. */
    bool _last_tx_was_secondary = false;                                /**< @brief True if the last frame sent was a log or audio frame. */
    uint32_t _last_rx_byte_time = 0;                                    /**< @brief millis() of the last byte received. */
    uint32_t _last_valid_rx_time = 0;                                   /**< @brief millis() of the last frame that passed the CRC check. */

    /** @brief Receive state machine states. */
    enum RXState {
        WAIT_START_1,
        WAIT_START_2,
        EXPECT_TYPE,
        EXPECT_LEN,
        EXPECT_PAYLOAD,
        EXPECT_CRC1,
        EXPECT_CRC2
    };
    RXState _rx_state = WAIT_START_1;                                   /**< @brief Current receive state. */

    MessageType _rx_type = MSG_TX_HEARTBEAT;                            /**< @brief Type of the frame being received. */
    uint8_t _rx_len = 0;                                                /**< @brief Length of the frame being received. */
    uint8_t _rx_buffer[MAX_PAYLOAD_SIZE];                               /**< @brief Payload of the frame being received. */
    uint8_t _rx_index = 0;                                              /**< @brief Bytes of payload received so far. */
    uint8_t _rx_crc_high = 0;                                           /**< @brief CRC high byte, waiting for the low byte. */
};

#include "VESCSafety.h"
