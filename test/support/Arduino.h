// Minimal Arduino and FreeRTOS stand-ins for running the libraries' logic on a PC (pio test -e native).
// Only what the libraries use is provided. Time, GPIO and serial ports are controlled by the tests through
// the fake:: functions below. Not used by the firmware build.
#pragma once

#include <atomic>
#include <cmath>
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <string>
#include <vector>

#define HIGH 1
#define LOW 0
#define INPUT 0x01
#define OUTPUT 0x03
#define SERIAL_8N1 0x800001c
#define F(s) (s)

namespace fake {
    inline uint32_t& now_us() { static uint32_t t = 0; return t; }
    // Microseconds added on every micros() call, so code that busy-waits on micros() can finish
    inline uint32_t& auto_advance_us() { static uint32_t step = 0; return step; }
    inline void set_ms(uint32_t ms) { now_us() = ms * 1000u; }
    inline void advance_ms(uint32_t ms) { now_us() += ms * 1000u; }

    inline int* pin_levels() { static int levels[64] = {0}; return levels; }
    inline int* pin_modes() { static int modes[64] = {0}; return modes; }
    inline void reset_pins() {
        for (int i = 0; i < 64; i++) { pin_levels()[i] = 0; pin_modes()[i] = 0; }
    }
}

inline uint32_t millis() { return fake::now_us() / 1000u; }
inline uint32_t micros() { fake::now_us() += fake::auto_advance_us(); return fake::now_us(); }
inline void pinMode(int pin, int mode) { if (pin >= 0 && pin < 64) fake::pin_modes()[pin] = mode; }
inline void digitalWrite(int pin, int level) { if (pin >= 0 && pin < 64) fake::pin_levels()[pin] = level ? HIGH : LOW; }
inline int digitalRead(int pin) { return (pin >= 0 && pin < 64) ? fake::pin_levels()[pin] : LOW; }

/** Serial port with a receive queue the test fills and a transmit log the test reads. */
class HardwareSerial {
public:
    std::deque<uint8_t> rx;
    std::vector<uint8_t> tx;
    std::string text;

    void begin(unsigned long, uint32_t = SERIAL_8N1, int8_t = -1, int8_t = -1) {}
    void end() {}
    void setRxBufferSize(size_t) {}
    int available() { return static_cast<int>(rx.size()); }
    int read() {
        if (rx.empty()) return -1;
        const uint8_t b = rx.front();
        rx.pop_front();
        return b;
    }
    size_t write(uint8_t b) { tx.push_back(b); return 1; }
    size_t write(const uint8_t* buf, size_t len) { tx.insert(tx.end(), buf, buf + len); return len; }
    size_t print(const char* s) { text += s; return std::strlen(s); }
    size_t println(const char* s = "") { text += s; text += "\n"; return std::strlen(s) + 1; }
    size_t printf(const char* fmt, ...) {
        char buf[256];
        va_list args;
        va_start(args, fmt);
        const int n = vsnprintf(buf, sizeof(buf), fmt, args);
        va_end(args);
        text += buf;
        return n > 0 ? static_cast<size_t>(n) : 0;
    }
    void feed(const std::vector<uint8_t>& bytes) { rx.insert(rx.end(), bytes.begin(), bytes.end()); }
    void clear() { rx.clear(); tx.clear(); text.clear(); }
};

inline HardwareSerial Serial;
inline HardwareSerial Serial1;
inline HardwareSerial Serial2;

// FreeRTOS: single-threaded tests, so the mutex always succeeds
typedef void* SemaphoreHandle_t;
typedef uint32_t TickType_t;
#define pdTRUE 1
#define pdFALSE 0
#define portMAX_DELAY 0xFFFFFFFFu
#define pdMS_TO_TICKS(ms) (static_cast<TickType_t>(ms))
inline SemaphoreHandle_t xSemaphoreCreateMutex() { static int token; return &token; }
inline int xSemaphoreTake(SemaphoreHandle_t, TickType_t) { return pdTRUE; }
inline int xSemaphoreGive(SemaphoreHandle_t) { return pdTRUE; }
inline void vSemaphoreDelete(SemaphoreHandle_t) {}
inline void vTaskDelay(TickType_t) {}
