# Known Issues

Open problems in firmware 1.0.0 and what is planned for each. The details are in the linked documents; [pitfalls.md](pitfalls.md) lists every failure mode found so far, open or fixed.

## ESP32-VESC Link

- When telemetry from the VESC is lost, the ESP32 stops sending control frames. The script keeps the last throttle and brake until its 500 ms dead-man cut, so a brake pulled in that time does not reach the motors. Fix planned for 1.0.1 ([pitfalls](pitfalls.md#the-esp32-stops-sending-control-frames-when-telemetry-is-lost)).
- After a dead-man cut, drive resumes at the requested throttle as soon as control frames return, without a ramp and without the throttle being released. Fix planned for 1.0.1 ([pitfalls](pitfalls.md#drive-resumes-at-once-after-a-dead-man-cut)).
- Log frames also reset the script's dead-man timer. Fix planned for 1.0.1 ([pitfalls](pitfalls.md#any-valid-frame-resets-the-dead-man-timer)).
- A long log frame can crowd out a control frame in the VESC's 128-byte receive queue ([pitfalls](pitfalls.md#long-log-frames-can-overflow-the-vescs-receive-queue)).

## Front VESC

- The front VESC is not commanded to zero when the script marks it offline; it stops on its own 500 ms timeout. Fix planned for 1.0.1 ([pitfalls](pitfalls.md#the-front-vesc-is-not-told-to-stop-when-it-is-marked-offline)).
- Front VESC faults are not watched ([pitfalls](pitfalls.md#front-vesc-faults-are-not-watched)).

## Killswitch

- The killswitch line has no pull-down resistor at the VESC end ([hardware](hardware.md#killswitch)).
- One low reading disarms the script until the scooter stops, with no code on the display ([pitfalls](pitfalls.md#one-low-killswitch-reading-disarms-the-script)).

## Braking

- Hill hold applies brake mode to stopped wheels, which can turn a wheel. Fixed in 1.0.1, which is in testing ([powertrain](powertrain.md#hill-hold-known-issue)).
- Why brake mode drove the front wheel near standstill is not confirmed; the regen floor keeps brake mode away from low speeds ([powertrain](powertrain.md#regenerative-braking)).

## Display

- Display frames are lost during high-current launches, sometimes long enough to trip the 500 ms display timeout ([hardware](hardware.md#display-link)).
- The ESP32's mode scaling multiplies with the display's own per-mode throttle limit, so mode 1 delivers about 18 % of the current limit ([powertrain](powertrain.md#the-mode-ratio-compounds-with-the-display-limit)).
- Most display protocol facts, including the speed model sweep, come from bench notes and are not yet re-checked ([protocol](protocol/README.md), [verification](protocol/verification.md)).

## Testing

- The VESC script has no automated tests.
- Test reports with raw data are not written yet.
