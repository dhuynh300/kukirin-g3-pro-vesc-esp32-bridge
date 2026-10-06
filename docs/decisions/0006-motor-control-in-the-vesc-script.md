# Control That Must Survive A Lost ESP32 Link Runs In The VESC Script

Status: Accepted.

## Context

The ESP32 turns rider input into a throttle and brake request. The rear VESC's LispBM script turns that request into motor current. If the ESP32 crashes, loses power, or its UART link fails, anything the ESP32 alone was doing stops with it.

## Decision

Everything that has to keep working without the ESP32 link runs in the script ([vesc/main.lbm](../../vesc/main.lbm)):

- the dead-man timeout: no valid control frame for 500 ms cuts both motors;
- the per-mode speed limit, applied per wheel with that wheel's own speed (the ESP32 only sends the limit value);
- watching the front VESC and dropping to rear-only drive when it goes silent;
- the regen fade with speed and the low-speed regen floor.

The ESP32 keeps what needs the display: brake-to-start, lockouts after faults, the killswitch line, lights and error codes.

## Alternatives Considered

- Speed limiting on the ESP32: it would stop working when the link fails, and the ESP32 only knows the faster wheel's speed. An ESP32 speed governor from that approach is still in the code but inactive, and is due to be removed.
- All control on the ESP32 with the VESC as a plain current follower: simpler script, but a lost link would leave the VESC on its last command until its own timeout.

## Evidence

- Code: the constants and threads in [vesc/main.lbm](../../vesc/main.lbm); `tools/check_shared_constants.py` checks the values both sides define.

## Consequences

- Two languages hold related logic; shared constants are checked by a script, and [AGENTS.md](../../AGENTS.md) records the rule.
- The script has no automated tests yet; changes to it are checked by review, the bracket check and bench tests.
