/**
 * @file VESCUARTBridge.cpp
 * @brief Framed serial link to the rear VESC. See VESCUARTBridge.h for the frame format.
 */

#include "VESCUARTBridge.h"

// #define VESC_DBG

VESCUARTBridge::VESCUARTBridge(HardwareSerial* serial_port, uint32_t baud_rate, int8_t rx_pin, int8_t tx_pin)
    : _serial(serial_port), _baud_rate(baud_rate), _rx_pin(rx_pin), _tx_pin(tx_pin) {
    _tx_mutex = xSemaphoreCreateMutex();
}

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

VESCUARTBridge::~VESCUARTBridge() {
    if (_tx_mutex != nullptr) {
        vSemaphoreDelete(_tx_mutex);
        _tx_mutex = nullptr;
    }
}

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

void VESCUARTBridge::update() {
    if (_serial == nullptr) {
        #ifdef VESC_DBG
            static bool null_warned = false;
            if (!null_warned) {
                Serial.println(F("Esp32:Vesc:ErrNullPortOnUpdate:E7"));
                null_warned = true;
            }
        #endif

        return;
    }

    // Drop a partial frame after FRAMING_TIMEOUT_MS of silence
    if (_rx_state != WAIT_START_1 && (millis() - _last_rx_byte_time >= FRAMING_TIMEOUT_MS)) {
        _rx_state = WAIT_START_1;
        _rx_index = 0;
    }

    // Read at most 512 bytes per call so a burst of noise cannot stall the loop
    uint16_t bytes_processed = 0;
    while (_serial->available() && bytes_processed < 512) {
        process_incoming_byte(static_cast<uint8_t>(_serial->read()));
        bytes_processed++;
    }

    // Keepalive when nothing was sent for _heartbeat_interval_ms (unsigned subtraction handles millis() rollover)
    if (_heartbeat_interval_ms > 0 && (millis() - _last_tx_time >= _heartbeat_interval_ms)) {
        send_heartbeat();
    }
}

bool VESCUARTBridge::send_message(MessageType type, const uint8_t* payload, uint8_t len) {
    if (_serial == nullptr) return false;

    if (len > 0 && payload == nullptr) {
        #ifdef VESC_DBG
            Serial.printf("Esp32:VescTx:ErrNullPayload:Type:0x%02X:E7\n", static_cast<uint8_t>(type));
        #endif

        return false;
    }

    // Wait at most 10 ms for the other core to finish its frame
    if (_tx_mutex != nullptr) {
        if (xSemaphoreTake(_tx_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
            return false;
        }
    }

    // Keep at least 2 ms between a log or audio frame and the frame before and after it.
    // Control and keepalive frames that follow a control frame are not delayed.
    const bool is_secondary = (type != MSG_TX_CONTROL && type != MSG_TX_HEARTBEAT);
    if (is_secondary || _last_tx_was_secondary) {
        while (micros() - _last_tx_micros < 2000) {
            // Busy wait, at most 2 ms
        }
    }

    // CRC over type, length and payload
    uint16_t crc = 0x0000; 
    crc = calculate_crc16(reinterpret_cast<const uint8_t*>(&type), 1, crc); 
    crc = calculate_crc16(reinterpret_cast<const uint8_t*>(&len), 1, crc);
    if (len > 0) {
        crc = calculate_crc16(payload, len, crc);
    }

    _serial->write(START_BYTE_1);
    _serial->write(START_BYTE_2);
    _serial->write(type);
    _serial->write(len);
    
    if (len > 0) {
        _serial->write(payload, len);
    }
    
    _serial->write(static_cast<uint8_t>(crc >> 8));
    _serial->write(static_cast<uint8_t>(crc & 0xFF));

    _last_tx_time = millis();
    _last_tx_micros = micros();
    _last_tx_was_secondary = is_secondary;

    if (_tx_mutex != nullptr) {
        xSemaphoreGive(_tx_mutex);
    }
    return true;
}

void VESCUARTBridge::send_heartbeat() {
    send_message(MSG_TX_HEARTBEAT, nullptr, 0);
}

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
                // 0xAA 0xAA 0x55: the second 0xAA may be the real start
                _rx_state = WAIT_START_2; 
            } else {
                _rx_state = WAIT_START_1;
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
            // Each known type has a fixed or bounded length; anything else is not a real frame
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
                _rx_state = (incoming_byte == START_BYTE_1) ? WAIT_START_2 : WAIT_START_1;
            } else if (_rx_len == 0) {
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
                
                const uint8_t raw_type_byte = static_cast<uint8_t>(_rx_type);
                
                uint16_t calculated_crc = 0x0000;
                calculated_crc = calculate_crc16(&raw_type_byte, 1, calculated_crc);
                calculated_crc = calculate_crc16(&_rx_len, 1, calculated_crc);
                
                if (_rx_len > 0) {
                    calculated_crc = calculate_crc16(_rx_buffer, _rx_len, calculated_crc);
                }

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

                    // The bytes after a false start may hold the next real frame: find 0xAA 0x55 in what was
                    // received and feed everything from there back through the state machine
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

uint16_t VESCUARTBridge::calculate_crc16(const uint8_t *data, uint16_t len) {
    return calculate_crc16(data, len, 0x0000);
}

bool VESCUARTBridge::is_connected(uint32_t timeout_ms) const {
    if (_last_valid_rx_time == 0) return false;
    return (millis() - _last_valid_rx_time) <= timeout_ms;
}

uint32_t VESCUARTBridge::get_last_valid_rx_ms() const {
    return _last_valid_rx_time;
}