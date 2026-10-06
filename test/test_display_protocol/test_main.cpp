// Display protocol (KukirinG3ProProtocol.h): frame checks, P-menu decoding, status frame, error code bits,
// resync after a bad frame, input filters and accessory outputs.
#include <unity.h>
#include "KukirinDisplay.h"

using namespace KukirinG3Pro;

void setUp() { fake::reset_pins(); fake::set_ms(0); }
void tearDown() {}

// A command frame as the display sends it: mode 1, lights off, P03 = 10.0 in, P04 = 30 magnets,
// PB 3 / PA 3, P07 100 %, P02 cutoff 43.4 V, throttle released.
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

static TXPacket with_checksum(TXPacket p) { p.checksum = calculateTXChecksum(p); return p; }

// --- Frame checks ---

void test_checksum_is_xor_of_bytes_0_to_18() {
    TXPacket p = base_frame();
    const uint8_t* raw = reinterpret_cast<const uint8_t*>(&p);
    uint8_t x = 0;
    for (int i = 0; i < 19; i++) x ^= raw[i];
    TEST_ASSERT_EQUAL_HEX8(x, p.checksum);
    TEST_ASSERT_TRUE(verifyTXPacket(raw, sizeof(p)));
}

void test_corrupt_or_misframed_frames_are_rejected() {
    TXPacket p = base_frame();
    p.throttle_L ^= 0x01;  // one bit flipped, checksum not updated
    TEST_ASSERT_FALSE(verifyTXPacket(reinterpret_cast<const uint8_t*>(&p), sizeof(p)));
    TXPacket q = base_frame();
    q.startMarker = 0x02;
    TEST_ASSERT_FALSE(verifyTXPacket(reinterpret_cast<const uint8_t*>(&q), sizeof(q)));
    TEST_ASSERT_FALSE(verifyTXPacket(reinterpret_cast<const uint8_t*>(&q), sizeof(q) - 1));
}

void test_impossible_values_are_rejected_even_with_good_checksum() {
    TEST_ASSERT_TRUE(isPlausibleTXPacket(base_frame()));

    TXPacket both_turns = base_frame();
    both_turns.indicator_L = static_cast<uint8_t>(IndicatorFlag::LeftTurn) | static_cast<uint8_t>(IndicatorFlag::RightTurn);
    TEST_ASSERT_FALSE(isPlausibleTXPacket(with_checksum(both_turns)));

    TXPacket bad_mode = base_frame();
    bad_mode.driveMode = 0x07;
    TEST_ASSERT_FALSE(isPlausibleTXPacket(with_checksum(bad_mode)));

    TXPacket throttle_short = base_frame();
    throttle_short.throttle_H = 1051 >> 8;
    throttle_short.throttle_L = 1051 & 0xFF;
    TEST_ASSERT_FALSE(isPlausibleTXPacket(with_checksum(throttle_short)));

    TXPacket pa_zero = base_frame();
    pa_zero.regenAccelByte = 0x30;
    TEST_ASSERT_FALSE(isPlausibleTXPacket(with_checksum(pa_zero)));

    TXPacket pb_six = base_frame();
    pb_six.regenAccelByte = 0x63;
    TEST_ASSERT_FALSE(isPlausibleTXPacket(with_checksum(pb_six)));
}

void test_p_menu_decoding() {
    TXPacket p = base_frame();
    p.regenAccelByte = 0x25;  // PB 2, PA 5
    const DecodedSettings s = decodeTx(p);
    TEST_ASSERT_EQUAL_UINT8(2, s.regenLevel);
    TEST_ASSERT_EQUAL_UINT8(5, s.accelLevel);
    TEST_ASSERT_EQUAL_FLOAT(10.0f, s.wheelInches);
    TEST_ASSERT_EQUAL_FLOAT(43.4f, s.cutoffVolts);
    TEST_ASSERT_EQUAL_UINT16(30, s.poleCount);
    TEST_ASSERT_EQUAL_UINT8(100, s.speedLimitPercent);
    TEST_ASSERT_EQUAL_UINT8(1, extractGearLevel(p));
}

void test_resync_finds_a_frame_that_started_inside_a_bad_one() {
    uint8_t buf[20] = {0x01, 0x14, 0xAA, 0xBB, 0xCC, 0x01, 0x14, 0x01, 0x00};
    const size_t rem = resyncTXBuffer(buf, sizeof(buf));
    TEST_ASSERT_EQUAL_UINT32(15, rem);
    TEST_ASSERT_EQUAL_HEX8(0x01, buf[0]);
    TEST_ASSERT_EQUAL_HEX8(0x14, buf[1]);

    uint8_t none[20] = {0x01, 0x14, 0x55};
    TEST_ASSERT_EQUAL_UINT32(0, resyncTXBuffer(none, sizeof(none)));

    uint8_t trailing[20] = {0x01, 0x14};
    trailing[19] = 0x01;  // a start byte at the very end may begin the next frame
    TEST_ASSERT_EQUAL_UINT32(1, resyncTXBuffer(trailing, sizeof(trailing)));
}

// --- Status frame ---

void test_status_frame_layout_and_checksum() {
    RXPacket pkt;
    formatResponsePacket(pkt, 0x0123, true);
    TEST_ASSERT_EQUAL_HEX8(RX_START, pkt.startMarker);
    TEST_ASSERT_EQUAL_HEX8(RX_LEN, pkt.packetLength);
    TEST_ASSERT_EQUAL_HEX8(RX_START, pkt.echo_H);
    TEST_ASSERT_EQUAL_HEX8(RX_LEN, pkt.echo_L);
    TEST_ASSERT_EQUAL_HEX8(0x01, pkt.speedRaw_H);  // big-endian
    TEST_ASSERT_EQUAL_HEX8(0x23, pkt.speedRaw_L);
    TEST_ASSERT_EQUAL_HEX8(0xC0 | 0x08, pkt.systemStatus);  // running, dual drive
    TEST_ASSERT_EQUAL_HEX8(calculateRXChecksum(pkt), pkt.calculatedStatus);
    TEST_ASSERT_EQUAL_UINT32(16, sizeof(RXPacket));
}

void test_status_frame_sends_stopped_while_faulted() {
    RXPacket pkt;
    formatResponsePacket(pkt, 100, false, 0x02);
    const uint16_t raw = static_cast<uint16_t>((pkt.speedRaw_H << 8) | pkt.speedRaw_L);
    TEST_ASSERT_EQUAL_UINT16(SPEED_RAW_STATIONARY, raw);
}

void test_keepalive_flag_on_frame_2_then_every_50() {
    TEST_ASSERT_TRUE(isKeepaliveFrame(2));
    TEST_ASSERT_TRUE(isKeepaliveFrame(52));
    TEST_ASSERT_FALSE(isKeepaliveFrame(3));
    TEST_ASSERT_FALSE(isKeepaliveFrame(50));
}

void test_error_code_bits() {
    struct Case { ErrorCode code; int byte; uint8_t bit; };
    const Case cases[] = {
        {ErrorCode::E01_RearMotorHall, 3, 0x40}, {ErrorCode::E02_Diagnostic2, 3, 0x20},
        {ErrorCode::E03_MainController, 3, 0x10}, {ErrorCode::E05_VoltageFault, 3, 0x08},
        {ErrorCode::E07_SendingError, 4, 0x10}, {ErrorCode::E09_ControllerOverTemp, 4, 0x01},
        {ErrorCode::E11_FrontMotorHall, 5, 0x40}, {ErrorCode::E13_SecondaryController, 5, 0x10},
        {ErrorCode::E15_Diagnostic15, 5, 0x08}, {ErrorCode::E17_SubControllerRecv, 6, 0x10},
        {ErrorCode::E31_LatchingE00, 5, 0x20},
    };
    for (const Case& c : cases) {
        RXPacket pkt;
        formatResponsePacket(pkt, SPEED_RAW_STATIONARY, false);
        applyErrorCode(pkt, c.code);
        const uint8_t* raw = reinterpret_cast<const uint8_t*>(&pkt);
        TEST_ASSERT_TRUE_MESSAGE((raw[c.byte] & c.bit) == c.bit, "error bit not set");
    }
    RXPacket pkt;
    formatResponsePacket(pkt, SPEED_RAW_STATIONARY, false);
    const uint8_t good = pkt.calculatedStatus;
    applyErrorCode(pkt, ErrorCode::E06_ReceiverTimeoutCrc);
    TEST_ASSERT_EQUAL_HEX8(static_cast<uint8_t>(good ^ 0xFF), pkt.calculatedStatus);
}

// --- Input filters (one call per display frame) ---

void test_throttle_idle_switch_hysteresis() {
    InputFilter f;
    const ThrottleConfig cfg;
    TEST_ASSERT_EQUAL_UINT16(0, f.updateThrottle(20, false, cfg));   // first frame, below engage
    TEST_ASSERT_EQUAL_UINT16(0, f.updateThrottle(25, false, cfg));   // not above 25
    TEST_ASSERT_EQUAL_UINT16(26, f.updateThrottle(26, false, cfg));  // engaged
    TEST_ASSERT_EQUAL_UINT16(15, f.updateThrottle(15, false, cfg));  // stays engaged down to 15
    TEST_ASSERT_EQUAL_UINT16(0, f.updateThrottle(14, false, cfg));   // drops out below 15
    TEST_ASSERT_EQUAL_UINT16(0, f.updateThrottle(20, false, cfg));   // must exceed 25 again
}

void test_throttle_full_switch_and_brake() {
    InputFilter f;
    const ThrottleConfig cfg;
    f.updateThrottle(100, false, cfg);
    TEST_ASSERT_EQUAL_UINT16(1000, f.updateThrottle(975, false, cfg));
    TEST_ASSERT_EQUAL_UINT16(1000, f.updateThrottle(966, false, cfg));
    TEST_ASSERT_EQUAL_UINT16(964, f.updateThrottle(964, false, cfg));
    TEST_ASSERT_EQUAL_UINT16(0, f.updateThrottle(964, true, cfg));   // brake forces 0
}

void test_brake_on_at_once_off_after_quorum() {
    InputFilter f;
    TEST_ASSERT_TRUE(f.updateBrake(true, 1));
    TEST_ASSERT_FALSE(f.updateBrake(false, 1));
    TEST_ASSERT_TRUE(f.updateBrake(true, 2));
    TEST_ASSERT_TRUE(f.updateBrake(false, 2));
    TEST_ASSERT_FALSE(f.updateBrake(false, 2));
}

void test_turn_signal_needs_two_frames() {
    InputFilter f;
    bool l = false, r = false;
    f.updateTurnSignals(true, false, l, r, 2);
    TEST_ASSERT_FALSE(l);
    f.updateTurnSignals(true, false, l, r, 2);
    TEST_ASSERT_TRUE(l);
}

void test_hazards_from_two_pushes_100_to_800_ms_apart() {
    InputFilter ok;
    ok.updateHazard(true, false, 1000);
    ok.updateHazard(false, false, 1100);
    TEST_ASSERT_TRUE(ok.updateHazard(true, false, 1400));

    InputFilter too_fast;
    too_fast.updateHazard(true, false, 1000);
    too_fast.updateHazard(false, false, 1020);
    TEST_ASSERT_FALSE(too_fast.updateHazard(true, false, 1050));

    InputFilter too_slow;
    too_slow.updateHazard(true, false, 1000);
    too_slow.updateHazard(false, false, 1100);
    TEST_ASSERT_FALSE(too_slow.updateHazard(true, false, 1900));
}

void test_p_menu_change_waits_until_slow_with_throttle_released() {
    InputFilter f;
    DecodedSettings out;
    DecodedSettings a = decodeTx(base_frame());
    TEST_ASSERT_TRUE(f.updatePMenu(a, out));  // first frame sets the settings

    DecodedSettings b = a;
    b.accelLevel = 5;
    // Changed while riding at 10 mph: held for three frames and not applied
    for (int i = 0; i < 4; i++) {
        TEST_ASSERT_FALSE(f.updatePMenu(b, out, 10.0f, true));
        TEST_ASSERT_EQUAL_UINT8(3, out.accelLevel);
    }
    // Still moving fast, or slow but throttle open: still held
    TEST_ASSERT_FALSE(f.updatePMenu(b, out, 2.0f, false));
    // Slow with throttle released: applied
    TEST_ASSERT_TRUE(f.updatePMenu(b, out, 2.0f, true));
    TEST_ASSERT_EQUAL_UINT8(5, out.accelLevel);
}

void test_p_menu_change_while_stopped_applies_after_three_frames() {
    InputFilter f;
    DecodedSettings out;
    DecodedSettings a = decodeTx(base_frame());
    f.updatePMenu(a, out);
    DecodedSettings b = a;
    b.regenLevel = 5;
    TEST_ASSERT_FALSE(f.updatePMenu(b, out));
    TEST_ASSERT_FALSE(f.updatePMenu(b, out));
    TEST_ASSERT_TRUE(f.updatePMenu(b, out));
    TEST_ASSERT_EQUAL_UINT8(5, out.regenLevel);
}

// --- Accessory outputs ---

void test_brake_light_flashes_while_braking() {
    AccessoryEngine e;
    const BridgeTimingConfig t;
    e.update(0, false, true, false, false, false, false, t);
    TEST_ASSERT_TRUE(e.brakeLed);                       // lights off: first flash is on
    e.update(t.brakeStrobePeriodMs, false, true, false, false, false, false, t);
    TEST_ASSERT_FALSE(e.brakeLed);
    e.update(t.brakeStrobePeriodMs + 10, false, false, false, false, false, false, t);
    TEST_ASSERT_FALSE(e.brakeLed);                      // released: back to the lights state
}

void test_turn_signal_flashes_by_turning_off_when_lights_are_on() {
    AccessoryEngine e;
    const BridgeTimingConfig t;
    e.update(0, true, false, true, false, false, false, t);
    TEST_ASSERT_FALSE(e.leftLed);
    TEST_ASSERT_TRUE(e.rightLed);
    e.update(t.blinkPeriodMs, true, false, true, false, false, false, t);
    TEST_ASSERT_TRUE(e.leftLed);
}

void test_horn_cuts_off_and_rearms_after_release() {
    AccessoryEngine e;
    const BridgeTimingConfig t;
    e.update(0, false, false, false, false, false, true, t);
    TEST_ASSERT_TRUE(e.hornRelay);
    e.update(t.hornMaxContinuousMs, false, false, false, false, false, true, t);
    TEST_ASSERT_FALSE(e.hornRelay);
    e.update(t.hornMaxContinuousMs + 50, false, false, false, false, false, false, t);
    e.update(t.hornMaxContinuousMs + 60, false, false, false, false, false, true, t);
    TEST_ASSERT_FALSE(e.hornRelay);                     // released for less than the quiet time
    e.update(t.hornMaxContinuousMs + 70, false, false, false, false, false, false, t);
    e.update(t.hornMaxContinuousMs + 70 + t.hornRearmQuietMs, false, false, false, false, false, false, t);
    e.update(t.hornMaxContinuousMs + 80 + t.hornRearmQuietMs, false, false, false, false, false, true, t);
    TEST_ASSERT_TRUE(e.hornRelay);
}

void test_outputs_start_low() {
    for (int i = 0; i < 64; i++) fake::pin_levels()[i] = HIGH;
    AccessoryEngine::initGPIO();
    const PinConfig pins;
    const int8_t outs[] = {pins.leftLed, pins.rightLed, pins.brakeLed, pins.headlightLed, pins.horn,
                           pins.auxLed, pins.argb1, pins.argb2, pins.argb3, pins.argb4};
    for (int8_t pin : outs) {
        TEST_ASSERT_EQUAL(LOW, fake::pin_levels()[pin]);
        TEST_ASSERT_EQUAL(OUTPUT, fake::pin_modes()[pin]);
    }
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_checksum_is_xor_of_bytes_0_to_18);
    RUN_TEST(test_corrupt_or_misframed_frames_are_rejected);
    RUN_TEST(test_impossible_values_are_rejected_even_with_good_checksum);
    RUN_TEST(test_p_menu_decoding);
    RUN_TEST(test_resync_finds_a_frame_that_started_inside_a_bad_one);
    RUN_TEST(test_status_frame_layout_and_checksum);
    RUN_TEST(test_status_frame_sends_stopped_while_faulted);
    RUN_TEST(test_keepalive_flag_on_frame_2_then_every_50);
    RUN_TEST(test_error_code_bits);
    RUN_TEST(test_throttle_idle_switch_hysteresis);
    RUN_TEST(test_throttle_full_switch_and_brake);
    RUN_TEST(test_brake_on_at_once_off_after_quorum);
    RUN_TEST(test_turn_signal_needs_two_frames);
    RUN_TEST(test_hazards_from_two_pushes_100_to_800_ms_apart);
    RUN_TEST(test_p_menu_change_waits_until_slow_with_throttle_released);
    RUN_TEST(test_p_menu_change_while_stopped_applies_after_three_frames);
    RUN_TEST(test_brake_light_flashes_while_braking);
    RUN_TEST(test_turn_signal_flashes_by_turning_off_when_lights_are_on);
    RUN_TEST(test_horn_cuts_off_and_rearms_after_release);
    RUN_TEST(test_outputs_start_low);
    return UNITY_END();
}
