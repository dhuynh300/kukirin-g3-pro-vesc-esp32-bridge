// KukirinG3ProDisplay end to end: command frame bytes in, status frame bytes out, inputs published,
// stall detection.
#include <unity.h>
#include "KukirinDisplay.h"

using namespace KukirinG3Pro;

static TXPacket base_frame() {
    TXPacket p;
    memset(&p, 0, sizeof(p));
    p.startMarker = TX_START;
    p.packetLength = TX_LEN;
    p.commandType = 0x01;
    p.driveMode = static_cast<uint8_t>(DriveMode::Eco);
    p.poleCount_L = 30;
    p.wheelCirc_L = 100;
    p.regenAccelByte = 0x33;
    p.speedProfile_L = 100;
    p.speedProfile_H = 0x0C;
    p.batteryConfig_H = 434 >> 8;
    p.batteryConfig_L = 434 & 0xFF;
    p.checksum = calculateTXChecksum(p);
    return p;
}

static std::vector<uint8_t> bytes(TXPacket p, bool fix_checksum = true) {
    if (fix_checksum) p.checksum = calculateTXChecksum(p);
    const uint8_t* raw = reinterpret_cast<const uint8_t*>(&p);
    return std::vector<uint8_t>(raw, raw + sizeof(p));
}

static uint16_t reply_speed_raw() {
    return static_cast<uint16_t>((Serial2.tx[8] << 8) | Serial2.tx[9]);
}

static uint8_t xor_0_11(const std::vector<uint8_t>& v) {
    uint8_t x = 0;
    for (int i = 0; i < 12; i++) x ^= v[i];
    return x;
}

void setUp() { fake::reset_pins(); fake::set_ms(1000); Serial2.clear(); }
void tearDown() {}

void test_each_valid_frame_gets_one_status_frame() {
    KukirinG3ProDisplay d(&Serial2);
    d.begin();
    Serial2.feed(bytes(base_frame()));
    TEST_ASSERT_TRUE(d.update());
    TEST_ASSERT_EQUAL_UINT32(16, Serial2.tx.size());
    TEST_ASSERT_EQUAL_HEX8(RX_START, Serial2.tx[0]);
    TEST_ASSERT_EQUAL_HEX8(RX_LEN, Serial2.tx[1]);
    TEST_ASSERT_EQUAL_HEX8(xor_0_11(Serial2.tx), Serial2.tx[12]);
    TEST_ASSERT_EQUAL_UINT16(SPEED_RAW_STATIONARY, reply_speed_raw());
    TEST_ASSERT_FALSE(d.update());  // nothing new
    TEST_ASSERT_EQUAL_UINT32(16, Serial2.tx.size());
}

void test_reply_carries_the_set_speed() {
    KukirinG3ProDisplay d(&Serial2);
    d.begin();
    d.setSpeedMph(20.0f);
    Serial2.feed(bytes(base_frame()));
    d.update();
    TEST_ASSERT_EQUAL_UINT16(encodeSpeedRawMph(20.0f, 10.0f), reply_speed_raw());
}

void test_transaction_callback_sets_the_reply() {
    KukirinG3ProDisplay d(&Serial2);
    d.begin();
    d.onTransaction([](const KukirinInputs&, KukirinResponse& out) { out.speedMph = 12.0f; });
    Serial2.feed(bytes(base_frame()));
    d.update();
    TEST_ASSERT_EQUAL_UINT16(encodeSpeedRawMph(12.0f, 10.0f), reply_speed_raw());
}

void test_lost_vesc_shows_e003_and_zero_speed() {
    KukirinG3ProDisplay d(&Serial2);
    d.begin();
    d.setSpeedMph(20.0f);
    d.setErrorCode(ErrorCode::E03_MainController);
    Serial2.feed(bytes(base_frame()));
    d.update();
    TEST_ASSERT_EQUAL_UINT16(SPEED_RAW_STATIONARY, reply_speed_raw());
    TEST_ASSERT_TRUE((Serial2.tx[3] & 0x10) != 0);
}

void test_e006_inverts_the_checksum() {
    KukirinG3ProDisplay d(&Serial2);
    d.begin();
    d.setErrorCode(ErrorCode::E06_ReceiverTimeoutCrc);
    Serial2.feed(bytes(base_frame()));
    d.update();
    TEST_ASSERT_EQUAL_HEX8(static_cast<uint8_t>(xor_0_11(Serial2.tx) ^ 0xFF), Serial2.tx[12]);
}

void test_bad_checksum_gets_no_reply() {
    KukirinG3ProDisplay d(&Serial2);
    d.begin();
    TXPacket p = base_frame();
    p.checksum ^= 0x01;
    Serial2.feed(bytes(p, false));
    TEST_ASSERT_FALSE(d.update());
    TEST_ASSERT_EQUAL_UINT32(0, Serial2.tx.size());
    TEST_ASSERT_EQUAL_UINT32(1, d.getRejectedPacketCount());
    Serial2.feed(bytes(base_frame()));
    TEST_ASSERT_TRUE(d.update());
}

void test_frame_split_across_reads() {
    KukirinG3ProDisplay d(&Serial2);
    d.begin();
    const std::vector<uint8_t> f = bytes(base_frame());
    Serial2.feed(std::vector<uint8_t>(f.begin(), f.begin() + 7));
    TEST_ASSERT_FALSE(d.update());
    fake::advance_ms(5);
    Serial2.feed(std::vector<uint8_t>(f.begin() + 7, f.end()));
    TEST_ASSERT_TRUE(d.update());
}

void test_partial_frame_dropped_after_silence() {
    KukirinG3ProDisplay d(&Serial2);
    d.begin();
    const std::vector<uint8_t> f = bytes(base_frame());
    Serial2.feed(std::vector<uint8_t>(f.begin(), f.begin() + 7));
    d.update();
    fake::advance_ms(60);
    d.update();
    Serial2.feed(f);
    TEST_ASSERT_TRUE(d.update());
}

void test_inputs_are_published() {
    KukirinG3ProDisplay d(&Serial2);
    d.begin();
    TXPacket p = base_frame();
    p.throttle_H = 500 >> 8;
    p.throttle_L = 500 & 0xFF;
    Serial2.feed(bytes(p));
    d.update();
    TEST_ASSERT_EQUAL_UINT16(500, d.getRawThrottle());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 485.0f / 965.0f, d.getNormalizedThrottle());
    TEST_ASSERT_FALSE(d.isBrakeActive());

    TXPacket b = base_frame();
    b.indicator_L = static_cast<uint8_t>(IndicatorFlag::BrakeActive);
    b.throttle_L = 0;
    Serial2.clear();
    Serial2.feed(bytes(b));
    d.update();
    TEST_ASSERT_TRUE(d.isBrakeActive());
    TEST_ASSERT_EQUAL_HEX8(0xE0, Serial2.tx[4] & 0xE0);  // brake icon
}

void test_implausible_frame_is_answered_but_not_used() {
    KukirinG3ProDisplay d(&Serial2);
    d.begin();
    TXPacket p = base_frame();
    p.throttle_H = 500 >> 8;
    p.throttle_L = 500 & 0xFF;
    Serial2.feed(bytes(p));
    d.update();
    TXPacket bad = base_frame();
    bad.throttle_H = 900 >> 8;
    bad.throttle_L = 900 & 0xFF;
    bad.indicator_L = static_cast<uint8_t>(IndicatorFlag::LeftTurn) | static_cast<uint8_t>(IndicatorFlag::RightTurn);
    Serial2.clear();
    Serial2.feed(bytes(bad));
    TEST_ASSERT_TRUE(d.update());
    TEST_ASSERT_EQUAL_UINT32(16, Serial2.tx.size());
    TEST_ASSERT_EQUAL_UINT16(500, d.getRawThrottle());  // previous value kept
}

void test_stall_detected_after_500_ms() {
    KukirinG3ProDisplay d(&Serial2);
    d.begin();
    TEST_ASSERT_TRUE(d.isCommsStalled());  // no frame yet
    Serial2.feed(bytes(base_frame()));
    d.update();
    TEST_ASSERT_FALSE(d.isCommsStalled());
    fake::advance_ms(500);
    TEST_ASSERT_FALSE(d.isCommsStalled());
    fake::advance_ms(1);
    TEST_ASSERT_TRUE(d.isCommsStalled());
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_each_valid_frame_gets_one_status_frame);
    RUN_TEST(test_reply_carries_the_set_speed);
    RUN_TEST(test_transaction_callback_sets_the_reply);
    RUN_TEST(test_lost_vesc_shows_e003_and_zero_speed);
    RUN_TEST(test_e006_inverts_the_checksum);
    RUN_TEST(test_bad_checksum_gets_no_reply);
    RUN_TEST(test_frame_split_across_reads);
    RUN_TEST(test_partial_frame_dropped_after_silence);
    RUN_TEST(test_inputs_are_published);
    RUN_TEST(test_implausible_frame_is_answered_but_not_used);
    RUN_TEST(test_stall_detected_after_500_ms);
    return UNITY_END();
}
