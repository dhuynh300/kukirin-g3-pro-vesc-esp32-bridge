// Speed shown on the display for a given speedRaw (KukirinG3ProProtocol.h).
//
// The bench notes say speedRaw was stepped through every value from 1 to 2299 (km/h) and 1 to 1352 (mph)
// with P03 = 80 (8.0 in wheel) and the LCD compared by eye, with no mismatches; the per-value records were
// not kept. These tests pin the values and cutoffs listed in those notes, so a change to the model that
// disagrees with them fails here. They are due to be re-checked (docs/protocol/verification.md).
#include <unity.h>
#include "KukirinDisplay.h"

using namespace KukirinG3Pro;

static const float P03_80 = 8.0f;  // wheel diameter setting used for the sweeps

void setUp() {}
void tearDown() {}

struct Reading { uint16_t speedRaw; uint16_t mph; };

// mph values at P03 = 80 from the table in the earlier protocol notes, listed there as LCD readings. Only
// speedRaw 2 separates 287.275 from pi x 0.0254 x 3600. The last 1 mph value is speedRaw 1351: the raw
// sweep note says 1352 and above showed 0 on the LCD.
static const Reading kMphTable[] = {
    {2, 714}, {7, 204}, {23, 62}, {46, 31}, {62, 22}, {95, 14}, {142, 10}, {237, 5},
    {284, 4}, {354, 3}, {470, 2}, {697, 1}, {1351, 1}, {1352, 0}, {3500, 0},
};

void test_mph_matches_notes() {
    for (const Reading& r : kMphTable) {
        char msg[32];
        snprintf(msg, sizeof(msg), "speedRaw %u", r.speedRaw);
        TEST_ASSERT_EQUAL_UINT16_MESSAGE(r.mph, decodeSpeedRawMph(r.speedRaw, P03_80), msg);
    }
}

void test_kmh_zero_cutoff() {
    // Last speedRaw that shows 1 km/h, and the first that shows 0
    TEST_ASSERT_EQUAL_UINT16(1, decodeSpeedRawKmh(2298, P03_80));
    TEST_ASSERT_EQUAL_UINT16(0, decodeSpeedRawKmh(2299, P03_80));
}

void test_mph_zero_cutoff() {
    TEST_ASSERT_EQUAL_UINT16(1, decodeSpeedRawMph(1351, P03_80));
    TEST_ASSERT_EQUAL_UINT16(0, decodeSpeedRawMph(1352, P03_80));
}

void test_stopped_and_invalid_values_show_zero() {
    TEST_ASSERT_EQUAL_UINT16(0, decodeSpeedRawMph(0, P03_80));
    TEST_ASSERT_EQUAL_UINT16(0, decodeSpeedRawKmh(0, P03_80));
    TEST_ASSERT_EQUAL_UINT16(0, decodeSpeedRawMph(SPEED_RAW_STATIONARY, P03_80));
    TEST_ASSERT_EQUAL_UINT16(0, decodeSpeedRawKmh(SPEED_RAW_STATIONARY, P03_80));
}

void test_shown_speed_never_increases_with_speedraw() {
    // A longer wheel period can never show a higher speed
    uint16_t prev_kmh = 0xFFFF, prev_mph = 0xFFFF;
    for (uint16_t r = 1; r < SPEED_RAW_STATIONARY; r++) {
        const uint16_t kmh = decodeSpeedRawKmh(r, P03_80);
        const uint16_t mph = decodeSpeedRawMph(r, P03_80);
        TEST_ASSERT_TRUE(kmh <= prev_kmh);
        TEST_ASSERT_TRUE(mph <= prev_mph);
        prev_kmh = kmh;
        prev_mph = mph;
    }
}

void test_encoded_speed_is_shown_within_one_unit() {
    // The firmware sends encodeSpeedRawMph(speed); the display must show that speed within 1 mph.
    // speedRaw is a whole number of ms, so resolution drops at high speed on small wheels: with an 8 in
    // wheel, 56 mph already shows 54. The scooter uses 10 in wheels; 8 in is checked up to 50 mph.
    struct Case { float wheel; int max_mph; };
    const Case cases[] = {{10.0f, 60}, {16.0f, 60}, {8.0f, 50}};
    for (const Case& c : cases) {
        const float wheel = c.wheel;
        for (int mph = 1; mph <= c.max_mph; mph++) {
            const uint16_t raw = encodeSpeedRawMph(static_cast<float>(mph), wheel);
            const int shown = decodeSpeedRawMph(raw, wheel);
            char msg[48];
            snprintf(msg, sizeof(msg), "%d mph, wheel %.1f in", mph, wheel);
            TEST_ASSERT_INT_WITHIN_MESSAGE(1, mph, shown, msg);
        }
    }
}

void test_encode_below_minimum_sends_stopped() {
    TEST_ASSERT_EQUAL_UINT16(SPEED_RAW_STATIONARY, encodeSpeedRawMph(0.0f));
    TEST_ASSERT_EQUAL_UINT16(SPEED_RAW_STATIONARY, encodeSpeedRawMph(0.1f));
    TEST_ASSERT_EQUAL_UINT16(SPEED_RAW_STATIONARY, encodeSpeedRawMph(NAN));
    TEST_ASSERT_EQUAL_UINT16(SPEED_RAW_STATIONARY, encodeSpeedRaw(0.2f));
}

void test_pole_count_does_not_change_kmh_encoding() {
    // P04 (magnet count) does not affect the speed the display shows
    TEST_ASSERT_EQUAL_UINT16(encodeSpeedRaw(25.0f, 30, 10.0f), encodeSpeedRaw(25.0f, 2, 10.0f));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_mph_matches_notes);
    RUN_TEST(test_kmh_zero_cutoff);
    RUN_TEST(test_mph_zero_cutoff);
    RUN_TEST(test_stopped_and_invalid_values_show_zero);
    RUN_TEST(test_shown_speed_never_increases_with_speedraw);
    RUN_TEST(test_encoded_speed_is_shown_within_one_unit);
    RUN_TEST(test_encode_below_minimum_sends_stopped);
    RUN_TEST(test_pole_count_does_not_change_kmh_encoding);
    return UNITY_END();
}
