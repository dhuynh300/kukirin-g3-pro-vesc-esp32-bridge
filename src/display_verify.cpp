// Display verification firmware (build: pio run -e display-verify). Lab tool, not the bridge firmware.
//
// Answers the display like a controller, but every field of the status frame is set from the USB serial
// console, so tools/display_verify.py can step through speeds, error bits and checksums while the LCD is
// read by eye. The VESC link is not started and the killswitch output (GPIO 2) is held low, so a
// connected VESC keeps its motors off.
//
// Commands (one per line, 115200 baud); every command answers "OK <command>" or "ERR <reason>":
//   ping                 check the link
//   speedraw N           send speedRaw N (stops alternation)
//   alt A B N            alternate speedRaw A and B, switching every N display frames
//   dual 0|1             dual drive indicator
//   set B V              force status byte B (0-15) to V after the frame is built; checksum recomputed
//   unset B / clear      remove one override / reset everything to a normal idle frame
//   badsum 0|1           invert the checksum (the display should show E-006)
//   frame                print the last valid command frame: FRAME <40 hex digits>
//   log 0|1              print every command frame: F,<ms>,<hex>
//   stat [reset]         frame count, bad frames and spacing: STAT ...
#ifdef KUKIRIN_DISPLAY_VERIFY

#include <Arduino.h>
#include "KukirinDisplay.h"

using namespace KukirinG3Pro;

static const int8_t KILLSWITCH_PIN = 2;
static const int8_t DISPLAY_RX_PIN = 35;
static const int8_t DISPLAY_TX_PIN = 33;

static uint16_t s_speed_raw = SPEED_RAW_STATIONARY;
static bool s_dual = true;
static int16_t s_override[sizeof(RXPacket)];
static bool s_invert_checksum = false;
static bool s_log_frames = false;

static bool s_alt_active = false;
static uint16_t s_alt_a = 0, s_alt_b = 0;
static uint32_t s_alt_every = 1;

static uint8_t s_rx[sizeof(TXPacket)];
static size_t s_rx_len = 0;
static uint32_t s_last_byte_ms = 0;
static uint8_t s_last_frame[sizeof(TXPacket)];
static bool s_have_frame = false;

static uint32_t s_frames = 0, s_bad = 0;
static uint32_t s_last_frame_us = 0, s_dt_min_us = 0xFFFFFFFF, s_dt_max_us = 0;
static uint64_t s_dt_sum_us = 0;
static uint32_t s_dt_count = 0;

static void reset_state() {
    s_speed_raw = SPEED_RAW_STATIONARY;
    s_dual = true;
    for (auto& o : s_override) o = -1;
    s_invert_checksum = false;
    s_alt_active = false;
}

static void reset_stats() {
    s_frames = s_bad = 0;
    s_dt_min_us = 0xFFFFFFFF;
    s_dt_max_us = 0;
    s_dt_sum_us = 0;
    s_dt_count = 0;
}

static void print_hex(const uint8_t* b, size_t n) {
    static const char hex[] = "0123456789ABCDEF";
    for (size_t i = 0; i < n; i++) {
        Serial.write(hex[b[i] >> 4]);
        Serial.write(hex[b[i] & 0x0F]);
    }
}

static void send_reply() {
    uint16_t speed = s_speed_raw;
    if (s_alt_active) speed = ((s_frames / s_alt_every) % 2 == 0) ? s_alt_a : s_alt_b;

    RXPacket pkt;
    formatResponsePacket(pkt, speed, s_dual);
    uint8_t* b = reinterpret_cast<uint8_t*>(&pkt);
    for (size_t i = 0; i < sizeof(RXPacket); i++) {
        if (s_override[i] >= 0) b[i] = static_cast<uint8_t>(s_override[i]);
    }
    if (s_override[12] < 0) pkt.calculatedStatus = calculateRXChecksum(pkt);
    if (s_invert_checksum) pkt.calculatedStatus ^= 0xFF;
    Serial2.write(b, sizeof(RXPacket));
}

static void on_frame() {
    const uint32_t now_us = micros();
    if (s_last_frame_us != 0) {
        const uint32_t dt = now_us - s_last_frame_us;
        if (dt < s_dt_min_us) s_dt_min_us = dt;
        if (dt > s_dt_max_us) s_dt_max_us = dt;
        s_dt_sum_us += dt;
        s_dt_count++;
    }
    s_last_frame_us = now_us;
    s_frames++;
    memcpy(s_last_frame, s_rx, sizeof(s_last_frame));
    s_have_frame = true;
    send_reply();
    if (s_log_frames) {
        Serial.printf("F,%lu,", static_cast<unsigned long>(millis()));
        print_hex(s_rx, sizeof(s_rx));
        Serial.write('\n');
    }
}

static void read_display() {
    if (s_rx_len > 0 && millis() - s_last_byte_ms > FRAMING_TIMEOUT_MS) s_rx_len = 0;
    while (Serial2.available()) {
        const uint8_t b = static_cast<uint8_t>(Serial2.read());
        s_last_byte_ms = millis();
        if (s_rx_len == 0 && b != TX_START) continue;
        if (s_rx_len == 1 && b != TX_LEN) {
            s_rx_len = (b == TX_START) ? 1 : 0;
            continue;
        }
        s_rx[s_rx_len++] = b;
        if (s_rx_len == sizeof(TXPacket)) {
            if (verifyTXPacket(s_rx, s_rx_len)) {
                on_frame();
                s_rx_len = 0;
            } else {
                s_bad++;
                s_rx_len = resyncTXBuffer(s_rx, s_rx_len);
            }
        }
    }
}

static bool parse_uint(const char* s, long lo, long hi, long& out) {
    char* end = nullptr;
    const long v = strtol(s, &end, 0);
    if (end == s || v < lo || v > hi) return false;
    out = v;
    return true;
}

static void handle(char* line) {
    char* cmd = strtok(line, " ");
    char* a1 = strtok(nullptr, " ");
    char* a2 = strtok(nullptr, " ");
    char* a3 = strtok(nullptr, " ");
    if (!cmd) return;
    long v1 = 0, v2 = 0, v3 = 0;

    if (!strcmp(cmd, "ping")) {
        Serial.println("OK ping");
    } else if (!strcmp(cmd, "speedraw") && a1 && parse_uint(a1, 0, 65535, v1)) {
        s_alt_active = false;
        s_speed_raw = static_cast<uint16_t>(v1);
        Serial.printf("OK speedraw %ld\n", v1);
    } else if (!strcmp(cmd, "alt") && a1 && a2 && a3 && parse_uint(a1, 0, 65535, v1) &&
               parse_uint(a2, 0, 65535, v2) && parse_uint(a3, 1, 1000, v3)) {
        s_alt_a = static_cast<uint16_t>(v1);
        s_alt_b = static_cast<uint16_t>(v2);
        s_alt_every = static_cast<uint32_t>(v3);
        s_alt_active = true;
        Serial.printf("OK alt %ld %ld %ld\n", v1, v2, v3);
    } else if (!strcmp(cmd, "dual") && a1 && parse_uint(a1, 0, 1, v1)) {
        s_dual = v1 != 0;
        Serial.printf("OK dual %ld\n", v1);
    } else if (!strcmp(cmd, "set") && a1 && a2 && parse_uint(a1, 0, sizeof(RXPacket) - 1, v1) && parse_uint(a2, 0, 255, v2)) {
        s_override[v1] = static_cast<int16_t>(v2);
        Serial.printf("OK set %ld %ld\n", v1, v2);
    } else if (!strcmp(cmd, "unset") && a1 && parse_uint(a1, 0, sizeof(RXPacket) - 1, v1)) {
        s_override[v1] = -1;
        Serial.printf("OK unset %ld\n", v1);
    } else if (!strcmp(cmd, "clear")) {
        reset_state();
        Serial.println("OK clear");
    } else if (!strcmp(cmd, "badsum") && a1 && parse_uint(a1, 0, 1, v1)) {
        s_invert_checksum = v1 != 0;
        Serial.printf("OK badsum %ld\n", v1);
    } else if (!strcmp(cmd, "log") && a1 && parse_uint(a1, 0, 1, v1)) {
        s_log_frames = v1 != 0;
        Serial.printf("OK log %ld\n", v1);
    } else if (!strcmp(cmd, "frame")) {
        if (!s_have_frame) {
            Serial.println("ERR no frame yet");
        } else {
            Serial.print("FRAME ");
            print_hex(s_last_frame, sizeof(s_last_frame));
            Serial.write('\n');
        }
    } else if (!strcmp(cmd, "stat")) {
        if (a1 && !strcmp(a1, "reset")) {
            reset_stats();
            Serial.println("OK stat reset");
        } else {
            const unsigned long mean = s_dt_count ? static_cast<unsigned long>(s_dt_sum_us / s_dt_count) : 0;
            Serial.printf("STAT frames=%lu bad=%lu dt_min_us=%lu dt_max_us=%lu dt_mean_us=%lu\n",
                          static_cast<unsigned long>(s_frames), static_cast<unsigned long>(s_bad),
                          static_cast<unsigned long>(s_dt_count ? s_dt_min_us : 0),
                          static_cast<unsigned long>(s_dt_max_us), mean);
        }
    } else {
        Serial.printf("ERR unknown or bad arguments: %s\n", cmd);
    }
}

static void read_console() {
    static char buf[64];
    static size_t len = 0;
    while (Serial.available()) {
        const char c = static_cast<char>(Serial.read());
        if (c == '\r' || c == '\n') {
            if (len > 0) {
                buf[len] = '\0';
                handle(buf);
                len = 0;
            }
        } else if (len < sizeof(buf) - 1) {
            buf[len++] = c;
        } else {
            len = 0;
        }
    }
}

void setup() {
    pinMode(KILLSWITCH_PIN, OUTPUT);
    digitalWrite(KILLSWITCH_PIN, LOW);
    Serial.begin(115200);
    Serial2.begin(9600, SERIAL_8N1, DISPLAY_RX_PIN, DISPLAY_TX_PIN);
    reset_state();
    reset_stats();
    Serial.println("VERIFY ready (killswitch low, VESC link off)");
}

void loop() {
    read_console();
    read_display();
    vTaskDelay(pdMS_TO_TICKS(1));
}

#endif  // KUKIRIN_DISPLAY_VERIFY
