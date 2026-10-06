// VESCSafetySupervisor: brake-to-start, killswitch output, brake priority, lockouts, throttle rise limit,
// jitter hold, mode scaling and the fault shown on the display.
#include <unity.h>
#include "VESCBridge.h"

using namespace VESCBridge;

static const int KILL_PIN = 2;
static const float DT = 0.020f;  // control frame period

void setUp() {
    fake::reset_pins();
    fake::set_ms(1000);
}
void tearDown() {}

// Brake held with throttle released and the vehicle stopped, for the full hold time
static void arm(VESCSafetySupervisor& s) {
    const uint32_t t0 = millis();
    s.update_b2s(true, 0, t0);
    s.update_b2s(true, 0, t0 + PowertrainConfig::B2S_HOLD_REQUIRED_MS);
    TEST_ASSERT_TRUE(s.is_armed());
}

// Throttle request through the supervisor as the firmware does it: brake off, dual on, front online, VESC online
static uint16_t drive(VESCSafetySupervisor& s, uint16_t throttle, float dt = DT) {
    return s.compute_safe_control(throttle, 0, false, true, false, true, dt).throttle;
}

// --- Killswitch and brake-to-start ---

void test_killswitch_output_low_after_begin() {
    VESCSafetySupervisor s(KILL_PIN);
    fake::pin_levels()[KILL_PIN] = HIGH;
    s.begin();
    TEST_ASSERT_EQUAL(OUTPUT, fake::pin_modes()[KILL_PIN]);
    TEST_ASSERT_EQUAL(LOW, fake::pin_levels()[KILL_PIN]);
    TEST_ASSERT_FALSE(s.is_armed());
}

void test_brake_to_start_arms_after_hold_time() {
    VESCSafetySupervisor s(KILL_PIN);
    s.begin();
    s.update_b2s(true, 0, 1000);
    TEST_ASSERT_FALSE(s.update_b2s(true, 0, 1000 + PowertrainConfig::B2S_HOLD_REQUIRED_MS - 1));
    TEST_ASSERT_EQUAL(LOW, fake::pin_levels()[KILL_PIN]);
    TEST_ASSERT_TRUE(s.update_b2s(true, 0, 1000 + PowertrainConfig::B2S_HOLD_REQUIRED_MS));
    TEST_ASSERT_EQUAL(HIGH, fake::pin_levels()[KILL_PIN]);
}

void test_brake_to_start_needs_released_throttle() {
    VESCSafetySupervisor s(KILL_PIN);
    s.begin();
    for (uint32_t t = 0; t <= 1000; t += 20) {
        s.update_b2s(true, PowertrainConfig::THROTTLE_NEUTRAL_COUNTS, t);
    }
    TEST_ASSERT_FALSE(s.is_armed());
    // Throttle released: the full hold time starts again
    s.update_b2s(true, 0, 1020);
    TEST_ASSERT_FALSE(s.is_armed());
    s.update_b2s(true, 0, 1000 + PowertrainConfig::B2S_HOLD_REQUIRED_MS);
    TEST_ASSERT_TRUE(s.is_armed());
}

void test_brake_to_start_needs_vehicle_stopped() {
    VESCSafetySupervisor s(KILL_PIN);
    s.begin();
    for (uint32_t t = 0; t <= 1000; t += 20) {
        s.update_b2s(true, 0, t, PowertrainConfig::SPEED_STATIONARY_MPH, 0.0f);
    }
    TEST_ASSERT_FALSE(s.is_armed());
    for (uint32_t t = 1000; t <= 2000; t += 20) {
        s.update_b2s(true, 0, t, 0.0f, static_cast<float>(PowertrainConfig::SPEED_STATIONARY_ERPM));
    }
    TEST_ASSERT_FALSE(s.is_armed());
}

void test_brake_to_start_restarts_when_brake_released() {
    VESCSafetySupervisor s(KILL_PIN);
    s.begin();
    s.update_b2s(true, 0, 0);
    s.update_b2s(true, 0, 150);
    s.update_b2s(false, 0, 160);
    s.update_b2s(true, 0, 170);
    s.update_b2s(true, 0, 300);
    TEST_ASSERT_FALSE(s.is_armed());
    s.update_b2s(true, 0, 170 + PowertrainConfig::B2S_HOLD_REQUIRED_MS);
    TEST_ASSERT_TRUE(s.is_armed());
}

void test_disarm_drops_killswitch_and_requires_brake_to_start() {
    VESCSafetySupervisor s(KILL_PIN);
    s.begin();
    arm(s);
    s.disarm();
    TEST_ASSERT_EQUAL(LOW, fake::pin_levels()[KILL_PIN]);
    TEST_ASSERT_EQUAL_UINT16(0, drive(s, 5000));
}

void test_unarmed_sends_no_throttle() {
    VESCSafetySupervisor s(KILL_PIN);
    s.begin();
    TEST_ASSERT_EQUAL_UINT16(0, drive(s, 5000));
}

// --- Brake priority ---

void test_brake_zeroes_throttle() {
    VESCSafetySupervisor s(KILL_PIN);
    s.begin();
    arm(s);
    TEST_ASSERT_TRUE(drive(s, 1000) > 0);
    TEST_ASSERT_EQUAL_UINT16(0, s.compute_safe_control(1000, 0, true, true, false, true, DT).throttle);
    TEST_ASSERT_EQUAL_UINT16(0, s.compute_safe_control(1000, 4000, false, true, false, true, DT).throttle);
}

void test_brake_released_with_throttle_open_waits_for_release() {
    VESCSafetySupervisor s(KILL_PIN);
    s.begin();
    arm(s);
    s.compute_safe_control(3000, 0, true, true, false, true, DT);   // braking
    TEST_ASSERT_EQUAL_UINT16(0, drive(s, 3000));                     // brake let go, throttle still open
    TEST_ASSERT_TRUE(s.is_brake_release_locked());
    TEST_ASSERT_EQUAL_UINT16(0, drive(s, 3000));
    drive(s, 0);                                                     // throttle released
    TEST_ASSERT_FALSE(s.is_brake_release_locked());
    TEST_ASSERT_TRUE(drive(s, 1000) > 0);
}

// --- Throttle rise limit and jitter hold ---

void test_throttle_rises_at_pa_rate_and_releases_at_once() {
    VESCSafetySupervisor s(KILL_PIN);
    s.begin();
    arm(s);
    // Default PA 3: 60,000 counts/s, so 1200 counts per 20 ms
    TEST_ASSERT_EQUAL_UINT16(1200, drive(s, 10000));
    TEST_ASSERT_EQUAL_UINT16(2400, drive(s, 10000));
    for (int i = 0; i < 5; i++) drive(s, 10000);   // 3600 .. 8400
    TEST_ASSERT_EQUAL_UINT16(9600, drive(s, 10000));
    TEST_ASSERT_EQUAL_UINT16(10000, drive(s, 10000));
    TEST_ASSERT_EQUAL_UINT16(0, drive(s, 0));
}

void test_pa_level_sets_rise_rate() {
    VESCSafetySupervisor s(KILL_PIN);
    s.begin();
    arm(s);
    s.set_accel_level_pa(5);
    TEST_ASSERT_EQUAL_UINT16(2000, drive(s, 10000));
    s.set_accel_level_pa(1);
    TEST_ASSERT_EQUAL_UINT16(2400, drive(s, 10000));
}

void test_long_gap_cannot_release_a_large_step() {
    VESCSafetySupervisor s(KILL_PIN);
    s.begin();
    arm(s);
    // A 1 s gap counts as 100 ms: at most 6000 counts at PA 3
    TEST_ASSERT_EQUAL_UINT16(6000, drive(s, 10000, 1.0f));
}

void test_jitter_hold_ignores_small_changes() {
    VESCSafetySupervisor s(KILL_PIN);
    s.begin();
    arm(s);
    for (int i = 0; i < 5; i++) drive(s, 4000);
    TEST_ASSERT_EQUAL_UINT16(4000, drive(s, 4000));
    TEST_ASSERT_EQUAL_UINT16(4000, drive(s, 4030));  // moved 30: held
    TEST_ASSERT_EQUAL_UINT16(4060, drive(s, 4060));  // moved 60: sent
    TEST_ASSERT_EQUAL_UINT16(4060, drive(s, 4040));  // small drop: held
    TEST_ASSERT_EQUAL_UINT16(0, drive(s, 0));        // release always goes through
}

void test_mode_ceiling_is_reached_exactly() {
    VESCSafetySupervisor s(KILL_PIN);
    s.begin();
    arm(s);
    for (int i = 0; i < 5; i++) drive(s, PowertrainConfig::MODE_1_MAX_COUNTS - 30);
    TEST_ASSERT_EQUAL_UINT16(PowertrainConfig::MODE_1_MAX_COUNTS - 30, drive(s, PowertrainConfig::MODE_1_MAX_COUNTS - 30));
    // 30 counts below the jitter hold, but the mode ceiling is always sent
    TEST_ASSERT_EQUAL_UINT16(PowertrainConfig::MODE_1_MAX_COUNTS, drive(s, PowertrainConfig::MODE_1_MAX_COUNTS));
}

// --- Lockouts: torque off, killswitch stays high, released throttle clears ---

void test_vesc_fault_locks_until_cleared_and_throttle_released() {
    VESCSafetySupervisor s(KILL_PIN);
    s.begin();
    arm(s);
    s.set_foc_fault(3);
    TEST_ASSERT_EQUAL_UINT16(0, drive(s, 3000));
    TEST_ASSERT_EQUAL(HIGH, fake::pin_levels()[KILL_PIN]);
    s.set_foc_fault(0);
    TEST_ASSERT_EQUAL_UINT16(0, drive(s, 3000));  // fault gone, throttle still open
    drive(s, 0);
    TEST_ASSERT_FALSE(s.is_fault_recovery_locked());
    TEST_ASSERT_TRUE(drive(s, 3000) > 0);
}

void test_vesc_link_loss_locks_until_back_and_throttle_released() {
    VESCSafetySupervisor s(KILL_PIN);
    s.begin();
    arm(s);
    TEST_ASSERT_EQUAL_UINT16(0, s.compute_safe_control(3000, 0, false, true, false, false, DT).throttle);
    TEST_ASSERT_EQUAL_UINT16(0, drive(s, 3000));
    TEST_ASSERT_EQUAL(HIGH, fake::pin_levels()[KILL_PIN]);
    drive(s, 0);
    TEST_ASSERT_TRUE(drive(s, 3000) > 0);
}

void test_display_stall_locks_until_back_and_throttle_released() {
    VESCSafetySupervisor s(KILL_PIN);
    s.begin();
    arm(s);
    s.update_display_recovery(true, 3000);
    TEST_ASSERT_EQUAL_UINT16(0, drive(s, 3000));
    s.update_display_recovery(false, 3000);
    TEST_ASSERT_EQUAL_UINT16(0, drive(s, 3000));
    s.update_display_recovery(false, 0);
    TEST_ASSERT_TRUE(drive(s, 3000) > 0);
    TEST_ASSERT_EQUAL(HIGH, fake::pin_levels()[KILL_PIN]);
}

void test_front_loss_keeps_rear_throttle_and_drops_dual() {
    VESCSafetySupervisor s(KILL_PIN);
    s.begin();
    arm(s);
    s.set_can_desync(true);
    const TXPayloadControl lost = s.compute_safe_control(1000, 0, false, true, true, true, DT);
    TEST_ASSERT_EQUAL_UINT8(0, lost.dual_mode);
    TEST_ASSERT_EQUAL_UINT16(1000, lost.throttle);  // not reduced
    // Front back: dual drive returns only after the throttle is released
    s.set_can_desync(false);
    TEST_ASSERT_EQUAL_UINT8(0, s.compute_safe_control(1000, 0, false, true, false, true, DT).dual_mode);
    s.compute_safe_control(0, 0, false, true, false, true, DT);
    TEST_ASSERT_EQUAL_UINT8(1, s.compute_safe_control(1000, 0, false, true, false, true, DT).dual_mode);
}

void test_speed_limit_is_passed_to_the_vesc() {
    VESCSafetySupervisor s(KILL_PIN);
    s.begin();
    arm(s);
    const TXPayloadControl out = s.compute_safe_control(1000, 0, false, true, false, true, DT,
                                                        PowertrainConfig::MODE_1_SPEED_LIMIT_CMS);
    TEST_ASSERT_EQUAL_UINT16(PowertrainConfig::MODE_1_SPEED_LIMIT_CMS, out.speed_limit_cms);
}

// --- Mode scaling and regen ---

void test_mode_scaling() {
    TEST_ASSERT_EQUAL_UINT16(PowertrainConfig::MODE_1_MAX_COUNTS, VESCSafetySupervisor::compute_mode_scaled_throttle(1.0f, 1));
    TEST_ASSERT_EQUAL_UINT16(PowertrainConfig::MODE_2_MAX_COUNTS, VESCSafetySupervisor::compute_mode_scaled_throttle(1.0f, 2));
    TEST_ASSERT_EQUAL_UINT16(PowertrainConfig::MODE_3_MAX_COUNTS, VESCSafetySupervisor::compute_mode_scaled_throttle(1.0f, 3));
    TEST_ASSERT_EQUAL_UINT16(2500, VESCSafetySupervisor::compute_mode_scaled_throttle(0.5f, 1));
    TEST_ASSERT_EQUAL_UINT16(5000, VESCSafetySupervisor::compute_mode_scaled_throttle(0.5f, 3));
    TEST_ASSERT_EQUAL_UINT16(0, VESCSafetySupervisor::compute_mode_scaled_throttle(0.0f, 3));
}

void test_speed_governor_inactive_at_speed_zero() {
    // The firmware passes speed 0, so the ESP32 governor never reduces throttle (the VESC script limits speed)
    TEST_ASSERT_EQUAL_FLOAT(1.0f, VESCSafetySupervisor::compute_speed_governor_scale(0.0f, PowertrainConfig::MODE_1_SPEED_LIMIT_MPH));
}

void test_regen_from_pb_level() {
    TEST_ASSERT_EQUAL_UINT16(6000, VESCSafetySupervisor::compute_pb_regen(true, 3));
    TEST_ASSERT_EQUAL_UINT16(0, VESCSafetySupervisor::compute_pb_regen(false, 3));
    TEST_ASSERT_EQUAL_UINT16(0, VESCSafetySupervisor::compute_pb_regen(true, 0));
    TEST_ASSERT_EQUAL_UINT16(10000, VESCSafetySupervisor::compute_pb_regen(true, 9));  // clamped to PB 5
}

// --- Fault shown on the display ---

void test_lost_vesc_link_overrides_other_faults() {
    VESCSafetySupervisor s(KILL_PIN);
    s.begin();
    TEST_ASSERT_EQUAL(static_cast<int>(SafetyFault::VescOffline), static_cast<int>(s.evaluate_fault(false, true, 0, 500)));
}

void test_unarmed_shows_disarmed() {
    VESCSafetySupervisor s(KILL_PIN);
    s.begin();
    TEST_ASSERT_EQUAL(static_cast<int>(SafetyFault::KillswitchDisarmed), static_cast<int>(s.evaluate_fault(true, false, 0, 500)));
    arm(s);
    TEST_ASSERT_EQUAL(static_cast<int>(SafetyFault::None), static_cast<int>(s.evaluate_fault(true, false, 0, 500)));
}

void test_two_faults_alternate_every_period() {
    VESCSafetySupervisor s(KILL_PIN);
    s.begin();  // unarmed, plus front lost
    const uint32_t p = PowertrainConfig::FAULT_CYCLE_PERIOD_MS;
    TEST_ASSERT_EQUAL(static_cast<int>(SafetyFault::CanDesync), static_cast<int>(s.evaluate_fault(true, true, 0, p / 2)));
    TEST_ASSERT_EQUAL(static_cast<int>(SafetyFault::KillswitchDisarmed), static_cast<int>(s.evaluate_fault(true, true, 0, p + p / 2)));
    TEST_ASSERT_EQUAL(static_cast<int>(SafetyFault::CanDesync), static_cast<int>(s.evaluate_fault(true, true, 0, 0)));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_killswitch_output_low_after_begin);
    RUN_TEST(test_brake_to_start_arms_after_hold_time);
    RUN_TEST(test_brake_to_start_needs_released_throttle);
    RUN_TEST(test_brake_to_start_needs_vehicle_stopped);
    RUN_TEST(test_brake_to_start_restarts_when_brake_released);
    RUN_TEST(test_disarm_drops_killswitch_and_requires_brake_to_start);
    RUN_TEST(test_unarmed_sends_no_throttle);
    RUN_TEST(test_brake_zeroes_throttle);
    RUN_TEST(test_brake_released_with_throttle_open_waits_for_release);
    RUN_TEST(test_throttle_rises_at_pa_rate_and_releases_at_once);
    RUN_TEST(test_pa_level_sets_rise_rate);
    RUN_TEST(test_long_gap_cannot_release_a_large_step);
    RUN_TEST(test_jitter_hold_ignores_small_changes);
    RUN_TEST(test_mode_ceiling_is_reached_exactly);
    RUN_TEST(test_vesc_fault_locks_until_cleared_and_throttle_released);
    RUN_TEST(test_vesc_link_loss_locks_until_back_and_throttle_released);
    RUN_TEST(test_display_stall_locks_until_back_and_throttle_released);
    RUN_TEST(test_front_loss_keeps_rear_throttle_and_drops_dual);
    RUN_TEST(test_speed_limit_is_passed_to_the_vesc);
    RUN_TEST(test_mode_scaling);
    RUN_TEST(test_speed_governor_inactive_at_speed_zero);
    RUN_TEST(test_regen_from_pb_level);
    RUN_TEST(test_lost_vesc_link_overrides_other_faults);
    RUN_TEST(test_unarmed_shows_disarmed);
    RUN_TEST(test_two_faults_alternate_every_period);
    return UNITY_END();
}
