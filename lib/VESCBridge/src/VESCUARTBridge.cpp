/**
 * @file VESCUARTBridge.cpp
 * @brief Implementation of the robust serial communication bridge between ESP32 and VESC motor controller.
 * @details Implements CRC16-CCITT packet validation, framing synchronization, keep-alive heartbeat generation,
 *          and a 50ms RX framing timeout watchdog.
 * @date 2026-08-25
 */

#include "VESCUARTBridge.h"

// #define VESC_DBG

/**
 * @brief Constructs a new VESCUARTBridge instance and initializes the hardware serial interface.
 * @param[in] serial_port Pointer to HardwareSerial peripheral (e.g. &Serial1).
 * @param[in] baud_rate Serial baud rate (defaults to 230,400 bps standard).
 * @param[in] rx_pin ESP32 GPIO pin for UART RX. Valid range: 0-39.
 * @param[in] tx_pin ESP32 GPIO pin for UART TX. Valid range: 0-33.
 */
VESCUARTBridge::VESCUARTBridge(HardwareSerial* serial_port, uint32_t baud_rate, int8_t rx_pin, int8_t tx_pin)
    : _serial(serial_port), _baud_rate(baud_rate), _rx_pin(rx_pin), _tx_pin(tx_pin) {
    _tx_mutex = xSemaphoreCreateMutex();
}

/**
 * @brief Initializes or reconfigures the hardware serial port (safe to call in setup()).
 * @param[in] baud_rate Serial baud rate (defaults to constructor value if 0).
 * @param[in] rx_pin ESP32 GPIO pin for UART RX (defaults to constructor value if -1).
 * @param[in] tx_pin ESP32 GPIO pin for UART TX (defaults to constructor value if -1).
 */
void VESCUARTBridge::begin(uint32_t baud_rate, int8_t rx_pin, int8_t tx_pin) {
    if (baud_rate != 0) _baud_rate = baud_rate;
    if (rx_pin != -1) _rx_pin = rx_pin;
    if (tx_pin != -1) _tx_pin = tx_pin;

    if (_serial != nullptr) {
        _serial->end();
        _serial->setRxBufferSize(512);
        _serial->begin(_baud_rate, SERIAL_8N1, _rx_pin, _tx_pin);
        Serial.printf("Esp32:VescUart:Configured:Baud:%u:Rx:%d:Tx:%d\n", _baud_rate, _rx_pin, _tx_pin);
    } else {
        Serial.println(F("Esp32:VescUart:ErrNullPort:E7"));
    }
}

/**
 * @brief Destructor that safely cleans up FreeRTOS synchronization mutex.
 * @note Core affinity: Core 1 / Core 0.
 */
VESCUARTBridge::~VESCUARTBridge() {
    if (_tx_mutex != nullptr) {
        vSemaphoreDelete(_tx_mutex);
        _tx_mutex = nullptr;
    }
}

/**
 * @brief Registers the callback invoked upon successful decoding and CRC validation of an incoming frame.
 * @param[in] cb Static function pointer of type MessageCallback. Pass nullptr to clear.
 */
void VESCUARTBridge::set_receive_callback(MessageCallback cb) {
    _on_message = cb;

    #ifdef VESC_DBG
        if (cb != nullptr) {
            Serial.println(F("Esp32:Vesc:RxCallbackRegistered"));
        } else {
            Serial.println(F("Esp32:Vesc:RxCallbackCleared"));
        }
    #endif
}

/**
 * @brief Real-time update loop handling RX stream processing, framing watchdogs, and keep-alive heartbeats.
 * @note Enforces 50ms framing watchdog on stalled partial RX frames. Capped at 512 bytes per update.
 * @note Core affinity: Core 1. Zero dynamic memory allocation. Non-blocking.
 */
void VESCUARTBridge::update() {
    // Prevent hardware panic if _serial is null
    if (_serial == nullptr) {
        #ifdef VESC_DBG
            static bool null_warned = false;
            if (!null_warned) {
                Serial.println(F("Esp32:Vesc:ErrNullPortOnUpdate:E7"));
                null_warned = true; // Prevent spamming the console
            }
        #endif

        return;
    }

    // Framing Watchdog: Purge partial frame if RX stalled for > 50ms
    if (_rx_state != WAIT_START_1 && (millis() - _last_rx_byte_time >= FRAMING_TIMEOUT_MS)) {
        _rx_state = WAIT_START_1;
        _rx_index = 0;
    }

    // Process incoming bytes, but cap the limit to prevent Watchdog Starvation from EMI floods
    uint16_t bytes_processed = 0;
    while (_serial->available() && bytes_processed < 512) {
        process_incoming_byte(static_cast<uint8_t>(_serial->read()));
        bytes_processed++;
    }

    // Enforce the Keep-Alive Heartbeat
    // Note: millis() rollover is safely handled by the subtraction logic
    if (_heartbeat_interval_ms > 0 && (millis() - _last_tx_time >= _heartbeat_interval_ms)) {
        send_heartbeat();
    }
}

/**
 * @brief Transmits a structured binary message with framing and CRC16-CCITT to the VESC.
 * @param[in] type The MessageType identifier.
 * @param[in] payload Pointer to the binary payload data buffer (may be nullptr if len == 0).
 * @param[in] len Length in bytes of the payload. Valid range: 0 to 255.
 * @return bool True if successfully formatted and written to the UART TX FIFO; false on mutex timeout or invalid arguments.
 * @note Thread-safe across Core 0 and Core 1 via bounded mutex. Zero-copy transmission directly to hardware TX buffer.
 * @see MessageType, calculate_crc16
 */
bool VESCUARTBridge::send_message(MessageType type, const uint8_t* payload, uint8_t len) {
    if (_serial == nullptr) return false;

    // Fail-fast on malformed payload parameters.
    // If payload length is non-zero, payload pointer must be valid.
    if (len > 0 && payload == nullptr) {
        #ifdef VESC_DBG
            Serial.printf("Esp32:VescTx:ErrNullPayload:Type:0x%02X:E7\n", static_cast<uint8_t>(type));
        #endif

        return false;
    }

    // Acquire TX mutex with bounded timeout (10ms) to ensure atomic packet transmission across cores
    if (_tx_mutex != nullptr) {
        if (xSemaphoreTake(_tx_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
            return false; // Mutex contention timeout
        }
    }

    // Multi-packet wire pacing: enforce >= 2ms inter-frame gap strictly for secondary frames
    // (MSG_TX_DEBUG_LOG, MSG_TX_AUDIO) and any frame following a secondary frame.
    // Primary control frames (MSG_TX_CONTROL) stream with sub-50us Core 1 real-time phase lag during healthy operation.
    const bool is_secondary = (type != MSG_TX_CONTROL && type != MSG_TX_HEARTBEAT);
    if (is_secondary || _last_tx_was_secondary) {
        while (micros() - _last_tx_micros < 2000) {
            // Spin-wait for remaining fraction of 2ms quiet interval
        }
    }

    // Calculate CRC over TYPE, LENGTH, and PAYLOAD
    uint16_t crc = 0x0000; 
    crc = calculate_crc16(reinterpret_cast<const uint8_t*>(&type), 1, crc); 
    crc = calculate_crc16(reinterpret_cast<const uint8_t*>(&len), 1, crc);
    if (len > 0) {
        crc = calculate_crc16(payload, len, crc);
    }

    // Write framing and payload directly to the hardware TX FIFO (Zero-Copy)
    _serial->write(START_BYTE_1);
    _serial->write(START_BYTE_2);
    _serial->write(type);
    _serial->write(len);
    
    if (len > 0) {
        _serial->write(payload, len);
    }
    
    _serial->write(static_cast<uint8_t>(crc >> 8));   // CRC High Byte
    _serial->write(static_cast<uint8_t>(crc & 0xFF)); // CRC Low Byte

    // Reset heartbeat timer and high-resolution wire pacing timestamp
    _last_tx_time = millis();
    _last_tx_micros = micros();
    _last_tx_was_secondary = is_secondary;

    if (_tx_mutex != nullptr) {
        xSemaphoreGive(_tx_mutex);
    }
    return true;
}

/**
 * @brief Transmits a zero-length keep-alive heartbeat frame (MSG_TX_HEARTBEAT).
 * @note Core affinity: Core 1. Zero dynamic memory allocation. Thread-safe via TX mutex.
 * @see MSG_TX_HEARTBEAT, send_message
 */
void VESCUARTBridge::send_heartbeat() {
    send_message(MSG_TX_HEARTBEAT, nullptr, 0);
}

/**
 * @brief Feed a single incoming serial byte into the RX framing state machine.
 * @param[in] incoming_byte Raw byte from UART RX FIFO. Valid range: 0x00 to 0xFF.
 * @note Core affinity: Core 1. Zero dynamic memory allocation.
 */
void VESCUARTBridge::process_incoming_byte(uint8_t incoming_byte) {
    _last_rx_byte_time = millis();

    switch (_rx_state) {
        case WAIT_START_1:
            if (incoming_byte == START_BYTE_1) {
                _rx_state = WAIT_START_2;
            }
            break;

        case WAIT_START_2:
            if (incoming_byte == START_BYTE_2) {
                _rx_state = EXPECT_TYPE;
            } else if (incoming_byte == START_BYTE_1) {
                // Handle duplicated start bytes (e.g., 0xAA 0xAA 0x55)
                _rx_state = WAIT_START_2; 
            } else {
                _rx_state = WAIT_START_1; // False alarm
            }
            break;

        case EXPECT_TYPE:
            if (incoming_byte > MSG_TX_DEBUG_LOG) {
                #ifdef VESC_DBG
                    Serial.println(F("Esp32:VescRx:ErrUnknownTypeDrop:E7"));
                #endif
                _rx_state = (incoming_byte == START_BYTE_1) ? WAIT_START_2 : WAIT_START_1;
            } else {
                _rx_type = static_cast<MessageType>(incoming_byte);
                _rx_state = EXPECT_LEN;
            }
            break;

        case EXPECT_LEN:
            _rx_len = incoming_byte;
            // Strict type-length plausibility verification
            if (_rx_type == MSG_TX_HEARTBEAT && _rx_len != 0) {
                _rx_state = (incoming_byte == START_BYTE_1) ? WAIT_START_2 : WAIT_START_1;
            } else if (_rx_type == MSG_TX_CONTROL && _rx_len != sizeof(TXPayloadControl)) {
                _rx_state = (incoming_byte == START_BYTE_1) ? WAIT_START_2 : WAIT_START_1;
            } else if (_rx_type == MSG_RX_TELEMETRY && (_rx_len < 2 || _rx_len > sizeof(RXTelemetryPayload))) {
                _rx_state = (incoming_byte == START_BYTE_1) ? WAIT_START_2 : WAIT_START_1;
            } else if (_rx_type == MSG_TEST_COMMAND && _rx_len != sizeof(TXPayloadTestCommand)) {
                _rx_state = (incoming_byte == START_BYTE_1) ? WAIT_START_2 : WAIT_START_1;
            } else if (_rx_type == MSG_TEST_FEEDBACK && _rx_len != sizeof(RXPayloadTestFeedback)) {
                _rx_state = (incoming_byte == START_BYTE_1) ? WAIT_START_2 : WAIT_START_1;
            } else if (_rx_len > MAX_PAYLOAD_SIZE) {
                #ifdef VESC_DBG
                    Serial.println(F("Esp32:VescRx:ErrOverflowDrop:E7"));
                #endif
                // Buffer overflow protection: drop packet and reset
                _rx_state = (incoming_byte == START_BYTE_1) ? WAIT_START_2 : WAIT_START_1;
            } else if (_rx_len == 0) {
                // Empty payload (like a heartbeat), skip straight to CRC
                _rx_state = EXPECT_CRC1;
            } else {
                _rx_index = 0;
                _rx_state = EXPECT_PAYLOAD;
            }
            break;

        case EXPECT_PAYLOAD:
            _rx_buffer[_rx_index++] = incoming_byte;
            if (_rx_index >= _rx_len) {
                _rx_state = EXPECT_CRC1;
            }
            break;

        case EXPECT_CRC1:
            _rx_crc_high = incoming_byte;
            _rx_state = EXPECT_CRC2;
            break;

        case EXPECT_CRC2:
            {
                const uint16_t received_crc = static_cast<uint16_t>((static_cast<uint16_t>(_rx_crc_high) << 8) | incoming_byte);
                
                // Extract the raw byte from the enum to guarantee memory safety during CRC calculation
                const uint8_t raw_type_byte = static_cast<uint8_t>(_rx_type);
                
                uint16_t calculated_crc = 0x0000;
                calculated_crc = calculate_crc16(&raw_type_byte, 1, calculated_crc);
                calculated_crc = calculate_crc16(&_rx_len, 1, calculated_crc);
                
                if (_rx_len > 0) {
                    calculated_crc = calculate_crc16(_rx_buffer, _rx_len, calculated_crc);
                }

                // If valid, dispatch to the application layer
                if (calculated_crc == received_crc) {
                    _last_valid_rx_time = millis();
                    #ifdef VESC_DBG
                        Serial.printf("Esp32:VescRx:Valid:Type:0x%02X:Len:%u\n", raw_type_byte, _rx_len);
                        if (_on_message == nullptr) {
                            Serial.println(F("Esp32:VescRx:WarnDroppedNoCallback"));
                        }
                    #endif
                    
                    if (_on_message != nullptr) {
                        _on_message(_rx_type, _rx_buffer, _rx_len);
                    }
                    _rx_state = WAIT_START_1;
                } else {
                    static uint32_t last_crc_err_time = 0;
                    if (millis() - last_crc_err_time >= 1000) {
                        last_crc_err_time = millis();
                        Serial.printf("Comm:CrcErr:Calc:0x%04X:Recv:0x%04X:Type:0x%02X:Len:%u:E3\n",
                                      calculated_crc, received_crc, raw_type_byte, _rx_len);
                    }

                    // Sliding-window resynchronization: search buffered frame bytes for next 0xAA 0x55
                    int16_t next_start_idx = -1;
                    if (raw_type_byte == START_BYTE_1 && _rx_len == START_BYTE_2) {
                        next_start_idx = 0;
                    }
                    if (next_start_idx == -1 && _rx_len == START_BYTE_1 && _rx_len > 0 && _rx_buffer[0] == START_BYTE_2) {
                        next_start_idx = 1;
                    }
                    if (next_start_idx == -1) {
                        for (uint16_t i = 0; i + 1 < _rx_len; ++i) {
                            if (_rx_buffer[i] == START_BYTE_1 && _rx_buffer[i + 1] == START_BYTE_2) {
                                next_start_idx = static_cast<int16_t>(2 + i);
                                break;
                            }
                        }
                    }
                    if (next_start_idx == -1 && _rx_len > 0 && _rx_buffer[_rx_len - 1] == START_BYTE_1 && _rx_crc_high == START_BYTE_2) {
                        next_start_idx = static_cast<int16_t>(2 + _rx_len - 1);
                    }
                    if (next_start_idx == -1 && _rx_crc_high == START_BYTE_1 && incoming_byte == START_BYTE_2) {
                        next_start_idx = static_cast<int16_t>(2 + _rx_len);
                    }

                    if (next_start_idx != -1) {
                        // Replay bytes starting from next start marker
                        uint8_t replay_buf[MAX_PAYLOAD_SIZE + 4];
                        uint16_t total_bytes = 0;
                        replay_buf[total_bytes++] = raw_type_byte;
                        replay_buf[total_bytes++] = _rx_len;
                        for (uint16_t i = 0; i < _rx_len; ++i) {
                            replay_buf[total_bytes++] = _rx_buffer[i];
                        }
                        replay_buf[total_bytes++] = _rx_crc_high;
                        replay_buf[total_bytes++] = incoming_byte;

                        _rx_state = WAIT_START_1;
                        _rx_index = 0;
                        for (uint16_t i = static_cast<uint16_t>(next_start_idx); i < total_bytes; ++i) {
                            process_incoming_byte(replay_buf[i]);
                        }
                    } else {
                        _rx_state = (incoming_byte == START_BYTE_1) ? WAIT_START_2 : WAIT_START_1;
                    }
                }
            }
            break;
    }
}

/**
 * @brief Standard CRC16-CCITT (Polynomial 0x1021) across segmented buffers.
 * @param[in] data Pointer to input data chunk.
 * @param[in] len Number of bytes in this chunk.
 * @param[in] current_crc Running CRC accumulator from previous segments.
 * @return uint16_t Updated running CRC16 checksum.
 */
uint16_t VESCUARTBridge::calculate_crc16(const uint8_t *data, uint16_t len, uint16_t current_crc) {
    if (data == nullptr) {
        return current_crc; 
    }

    uint16_t crc = current_crc;
    for (uint16_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(static_cast<uint16_t>(data[i]) << 8);
        for (uint8_t j = 0; j < 8; ++j) {
            if (crc & 0x8000) {
                crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
            } else {
                crc = static_cast<uint16_t>(crc << 1);
            }
        }
    }
    return crc;
}

/**
 * @brief Wrapper for single-pass CRC16-CCITT calculation.
 * @param[in] data Pointer to input data buffer.
 * @param[in] len Number of bytes to process.
 * @return uint16_t Computed CRC16 checksum.
 */
uint16_t VESCUARTBridge::calculate_crc16(const uint8_t *data, uint16_t len) {
    return calculate_crc16(data, len, 0x0000);
}

/**
 * @brief Checks if valid telemetry packets have been received from VESC within the specified timeout.
 * @param[in] timeout_ms Maximum allowable silence threshold in ms (default 100ms = 5 frames at 50Hz).
 * @return bool True if connection is alive and healthy; false if timed out or not yet received.
 */
bool VESCUARTBridge::is_connected(uint32_t timeout_ms) const {
    if (_last_valid_rx_time == 0) return false;
    return (millis() - _last_valid_rx_time) <= timeout_ms;
}

/**
 * @brief Returns the timestamp in milliseconds of the last successfully decoded, CRC-verified packet.
 * @return uint32_t Millis timestamp of last valid frame.
 */
uint32_t VESCUARTBridge::get_last_valid_rx_ms() const {
    return _last_valid_rx_time;
}