# Current Commands Relative To The VESC Tool Limits

Status: Accepted.

## Context

The VESC current limits are tuned in VESC Tool and change during tuning. Firmware that commands amps would have to be rebuilt to match each change, and documentation quoting amps goes stale.

## Decision

Throttle and brake travel as counts from 0 to 10000 and are sent as relative commands: `set-current-rel` and `canset-current-rel` for drive, where 1.0 is the motor current limit, and `set-brake-rel` and `canset-brake-rel` for braking, where 1.0 is the braking current limit. Documentation gives currents in counts or percent of the limit ([powertrain](../powertrain.md#command-scale)).

## Alternatives Considered

- Absolute currents (`set-current` in amps): every limit change in VESC Tool would need a firmware change, and a mismatch would make the throttle scale wrong.

## Evidence

- Code: the command calls in [vesc/main.lbm](../../vesc/main.lbm).

## Consequences

- Tuning the VESCs needs no firmware change.
- Both motors get the same relative command, so with the same settings they draw about the same current.
- The firmware still contains one unused amp constant and amp figures in comments; they are due to be removed.
