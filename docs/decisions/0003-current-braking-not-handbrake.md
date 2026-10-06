# Electric Braking Uses VESC Current Braking, Not The Handbrake Mode

Status: Accepted.

## Context

The VESC offers two ways to brake with the motor: current braking (`set-brake-rel`, `canset-brake-rel`), which applies current against the direction the motor turns, and a handbrake mode (`set-handbrake`), which applies a fixed current vector in open loop to hold the rotor in place.

## Decision

Use current braking only. The script never calls `set-handbrake`, and [AGENTS.md](../../AGENTS.md) forbids it.

## Alternatives Considered

- Handbrake mode to hold the scooter on a slope: it keeps current in the windings for as long as it is commanded, whether or not the wheel is trying to move ([mcpwm_foc.c](../../third_party/vesc_firmware_v7/motor/mcpwm_foc.c) lines 852 and 3587-3589). A stopped hub motor has little cooling, so the I²R heat builds up. The mechanical brakes hold the scooter instead.

## Evidence

- VESC source, as linked above.

## Consequences

- Holding the scooter still is left to the mechanical brakes.
- Current braking is only reliable while the wheel turns: near standstill its direction follows noise in the speed estimate. That leads to [0004](0004-zero-current-below-regen-floor.md).
