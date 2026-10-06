// ESP32-VESC serial link (VESCUARTBridge): frame format, CRC, receive, resync, timeouts and log pacing.
#include <unity.h>
#include "VESCBridge.h"

// Independent CRC16-CCITT (polynomial 0x1021, initial value 0), table-driven, to check the bridge's
// bitwise implementation against.
static uint16_t ref_crc16(const std::vector<uint8_t>& data) {
    static uint16_t table[256];
    static bool ready = false;
    if (!ready) {
        for (int i = 0; i < 256; i++) {
            uint16_t c = static_cast<uint16_t>(i << 8);
            for (int b = 0; b < 8; b++) c = (c & 0x8000) ? static_cast<uint16_t>((c << 1) ^ 0x1021) : static_cast<uint16_t>(c << 1);
            table[i] = c;
        }
        ready = true;
    }
    uint16_t crc = 0;
    for (uint8_t byte : data) crc = static_cast<uint16_t>((crc << 8) ^ table[((crc >> 8) ^ byte) & 0xFF]);
    return crc;
}

// A complete frame as the other side would send it
static std::vector<uint8_t> frame(uint8_t type, const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> body = {type, static_cast<uint8_t>(payload.size())};
    body.insert(body.end(), payload.begin(), payload.end());
    const uint16_t crc = ref_crc16(body);
    std::vector<uint8_t> out = {0xAA, 0x55};
    out.insert(out.end(), body.begin(), body.end());
    out.push_back(static_cast<uint8_t>(crc >> 8));
    out.push_back(static_cast<uint8_t>(crc & 0xFF));
    return out;
}

static int g_rx_count = 0;
static std::vector<uint8_t> g_rx_payload;
static MessageType g_rx_type;
static void on_message(MessageType type, const uint8_t* payload, uint8_t len) {
    g_rx_count++;
    g_rx_type = type;
    g_rx_payload.assign(payload, payload + len);
}

void setUp() {
    fake::set_ms(1000);
    fake::auto_advance_us() = 0;
    Serial1.clear();
    g_rx_count = 0;
    g_rx_payload.clear();
}
void tearDown() {}

static const std::vector<uint8_t> kTelemetry = {0x9F, 0x02, 0x01};  // 671 cm/s, front VESC lost

void test_reference_crc_matches_standard_check_value() {
    // CRC-16/XMODEM check value for "123456789"
    TEST_ASSERT_EQUAL_HEX16(0x31C3, ref_crc16({'1', '2', '3', '4', '5', '6', '7', '8', '9'}));
}

void test_heartbeat_frame_bytes() {
    VESCUARTBridge b(&Serial1);
    b.begin();
    TEST_ASSERT_TRUE(b.send_message(MSG_TX_HEARTBEAT, nullptr, 0));
    const std::vector<uint8_t> expected = {0xAA, 0x55, 0x00, 0x00, 0x00, 0x00};
    TEST_ASSERT_EQUAL_UINT32(expected.size(), Serial1.tx.size());
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected.data(), Serial1.tx.data(), expected.size());
}

void test_control_frame_bytes_match_reference() {
    VESCUARTBridge b(&Serial1);
    b.begin();
    TXPayloadControl c;
    c.throttle = 1000;
    c.brake = 0;
    c.dual_mode = 1;
    c.speed_limit_cms = 671;
    TEST_ASSERT_TRUE(b.send_control(c));
    const std::vector<uint8_t> expected = frame(MSG_TX_CONTROL, {0xE8, 0x03, 0x00, 0x00, 0x01, 0x9F, 0x02});
    TEST_ASSERT_EQUAL_UINT32(expected.size(), Serial1.tx.size());
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected.data(), Serial1.tx.data(), expected.size());
}

void test_valid_telemetry_reaches_callback() {
    VESCUARTBridge b(&Serial1);
    b.begin();
    b.set_heartbeat_interval_ms(0);
    b.set_receive_callback(on_message);
    Serial1.feed(frame(MSG_RX_TELEMETRY, kTelemetry));
    b.update();
    TEST_ASSERT_EQUAL(1, g_rx_count);
    TEST_ASSERT_EQUAL(MSG_RX_TELEMETRY, g_rx_type);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(kTelemetry.data(), g_rx_payload.data(), kTelemetry.size());
    TEST_ASSERT_EQUAL_UINT32(1000, b.get_last_valid_rx_ms());
}

void test_bad_crc_is_dropped() {
    VESCUARTBridge b(&Serial1);
    b.begin();
    b.set_heartbeat_interval_ms(0);
    b.set_receive_callback(on_message);
    std::vector<uint8_t> bad = frame(MSG_RX_TELEMETRY, kTelemetry);
    bad[4] ^= 0x01;
    Serial1.feed(bad);
    b.update();
    TEST_ASSERT_EQUAL(0, g_rx_count);
    TEST_ASSERT_FALSE(b.is_connected(500));
}

void test_frame_starting_inside_a_broken_one_is_recovered() {
    VESCUARTBridge b(&Serial1);
    b.begin();
    b.set_heartbeat_interval_ms(0);
    b.set_receive_callback(on_message);
    // A frame cut off after its header, immediately followed by a complete frame
    std::vector<uint8_t> stream = {0xAA, 0x55, MSG_RX_TELEMETRY, 0x03};
    const std::vector<uint8_t> good = frame(MSG_RX_TELEMETRY, kTelemetry);
    stream.insert(stream.end(), good.begin(), good.end());
    Serial1.feed(stream);
    b.update();
    TEST_ASSERT_EQUAL(1, g_rx_count);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(kTelemetry.data(), g_rx_payload.data(), kTelemetry.size());
}

void test_doubled_start_byte_is_tolerated() {
    VESCUARTBridge b(&Serial1);
    b.begin();
    b.set_heartbeat_interval_ms(0);
    b.set_receive_callback(on_message);
    std::vector<uint8_t> stream = {0xAA};
    const std::vector<uint8_t> good = frame(MSG_RX_TELEMETRY, kTelemetry);
    stream.insert(stream.end(), good.begin(), good.end());
    Serial1.feed(stream);
    b.update();
    TEST_ASSERT_EQUAL(1, g_rx_count);
}

void test_wrong_length_for_type_is_not_a_frame() {
    VESCUARTBridge b(&Serial1);
    b.begin();
    b.set_heartbeat_interval_ms(0);
    b.set_receive_callback(on_message);
    Serial1.feed(frame(MSG_TX_HEARTBEAT, {0x01}));  // heartbeat must be empty
    b.update();
    TEST_ASSERT_EQUAL(0, g_rx_count);
}

void test_partial_frame_dropped_after_silence() {
    VESCUARTBridge b(&Serial1);
    b.begin();
    b.set_heartbeat_interval_ms(0);
    b.set_receive_callback(on_message);
    Serial1.feed({0xAA, 0x55, MSG_RX_TELEMETRY, 0x03, 0x10});
    b.update();
    fake::advance_ms(60);
    b.update();  // more than 50 ms of silence: the partial frame is dropped
    Serial1.feed(frame(MSG_RX_TELEMETRY, kTelemetry));
    b.update();
    TEST_ASSERT_EQUAL(1, g_rx_count);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(kTelemetry.data(), g_rx_payload.data(), kTelemetry.size());
}

void test_link_counts_as_lost_after_timeout() {
    VESCUARTBridge b(&Serial1);
    b.begin();
    b.set_heartbeat_interval_ms(0);
    b.set_receive_callback(on_message);
    TEST_ASSERT_FALSE(b.is_connected(500));  // nothing received yet
    Serial1.feed(frame(MSG_RX_TELEMETRY, kTelemetry));
    b.update();
    fake::advance_ms(500);
    TEST_ASSERT_TRUE(b.is_connected(500));
    fake::advance_ms(1);
    TEST_ASSERT_FALSE(b.is_connected(500));
}

void test_keepalive_sent_when_idle() {
    VESCUARTBridge b(&Serial1);
    b.begin();
    b.set_heartbeat_interval_ms(20);
    fake::advance_ms(20);
    b.update();
    TEST_ASSERT_EQUAL_UINT32(6, Serial1.tx.size());
    b.set_heartbeat_interval_ms(0);
    fake::advance_ms(100);
    b.update();
    TEST_ASSERT_EQUAL_UINT32(6, Serial1.tx.size());  // disabled, as in the firmware
}

void test_log_line_is_cut_to_120_bytes() {
    VESCUARTBridge b(&Serial1);
    b.begin();
    fake::auto_advance_us() = 10;
    std::string longline(200, 'x');
    b.send_debug_log(longline.c_str());
    TEST_ASSERT_EQUAL_UINT8(120, Serial1.tx[3]);
    TEST_ASSERT_EQUAL_UINT32(2 + 2 + 120 + 2, Serial1.tx.size());
}

void test_log_frame_waits_2ms_after_previous_frame() {
    VESCUARTBridge b(&Serial1);
    b.begin();
    fake::auto_advance_us() = 10;
    TXPayloadControl c = {};
    b.send_control(c);
    const uint32_t before = micros();
    b.send_debug_log("E:Test");
    TEST_ASSERT_TRUE(micros() - before >= 2000 - 20);
    // A control frame right after a log frame also waits
    const uint32_t before_ctrl = micros();
    b.send_control(c);
    TEST_ASSERT_TRUE(micros() - before_ctrl >= 2000 - 20);
    // Control after control does not wait
    const uint32_t before_second = micros();
    b.send_control(c);
    TEST_ASSERT_TRUE(micros() - before_second < 200);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_reference_crc_matches_standard_check_value);
    RUN_TEST(test_heartbeat_frame_bytes);
    RUN_TEST(test_control_frame_bytes_match_reference);
    RUN_TEST(test_valid_telemetry_reaches_callback);
    RUN_TEST(test_bad_crc_is_dropped);
    RUN_TEST(test_frame_starting_inside_a_broken_one_is_recovered);
    RUN_TEST(test_doubled_start_byte_is_tolerated);
    RUN_TEST(test_wrong_length_for_type_is_not_a_frame);
    RUN_TEST(test_partial_frame_dropped_after_silence);
    RUN_TEST(test_link_counts_as_lost_after_timeout);
    RUN_TEST(test_keepalive_sent_when_idle);
    RUN_TEST(test_log_line_is_cut_to_120_bytes);
    RUN_TEST(test_log_frame_waits_2ms_after_previous_frame);
    return UNITY_END();
}
