# Zero Current Below The Regen Floor Instead Of Brake Mode

Status: Accepted.

## Context

VESC current braking applies current against the sign of the measured speed: `iq = -SIGN(speed) * |iq|` ([mcpwm_foc.c](../../third_party/vesc_firmware_v7/motor/mcpwm_foc.c) line 3437). Near standstill the speed estimate is mostly noise. The firmware guards against this by shorting the phases (duty 0) when the speed or voltage sign changes or the duty cycle is close to zero (lines 3330-3347), yet braking at low speed drove the front wheel forward. The exact cause is not confirmed ([powertrain](../powertrain.md#regenerative-braking)).

## Decision

Below a regen floor speed, the script sends zero current (`set-current 0`, `canset-current 0`) instead of a brake command. Above it, regen fades in linearly with each wheel's own speed up to the full P-menu PB level. The floor and full-regen speeds are constants at the top of [vesc/main.lbm](../../vesc/main.lbm); [powertrain](../powertrain.md) describes the values and how they were chosen on rides.

## Alternatives Considered

- Brake mode at every speed with a small command near standstill: the direction of the current still follows the speed noise.
- A zero brake command (`set-brake-rel 0`) below the floor: leaves the VESC in brake mode; zero current is the plain way to release the motor.

## Evidence

- Seen on the scooter: the front wheel driven forward while braking at low speed, before the floor existed.
- VESC source, as linked above.

## Consequences

- The last few mph of every stop are mechanical braking only.
- Hill hold in v1.0.0 applied brake mode to stopped wheels, against this decision; the fix limits it to wheels that are turning ([powertrain](../powertrain.md)).
