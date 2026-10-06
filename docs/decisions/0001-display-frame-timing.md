# Display Link Timeouts Sized For About 8 Frames Per Second

Status: Accepted. The frame rate is an observation; a logged re-capture is planned.

## Context

The display sends a command frame and the controller answers each one. The ESP32 needs to know how often frames arrive to tell a lost frame from normal spacing, and to decide when the link has stalled. Junk495's G2 Pro documentation reports about 16 cycles per second with each frame sent three times. Temporary logging in the ESP32's frame parser on the G3 Pro display showed about 8 frames per second, one frame at a time, with no repeated frames. That log was not kept ([protocol](../protocol/README.md)).

## Decision

Size the display link timing for frames about 125 ms apart:

- a gap of 180 ms is logged as one lost frame, 320 ms as two or more (`DISPLAY_DROP_SINGLE_MS`, `DISPLAY_DROP_STREAK_MS` in `PowertrainConfig`);
- 500 ms without a valid frame counts as a stalled link, about four missed frames (`deadmanTimeoutMs` in `BridgeTimingConfig`), which zeroes throttle.

## Alternatives Considered

- Timing based on 16 to 20 frames per second: every normal 125 ms gap would be logged as a lost frame, and a shorter stall timeout would trip after one or two missed frames.
- A longer stall timeout: fewer false stalls under electrical noise, but the ESP32 would keep sending a stale throttle value for longer when the display really stops.

## Evidence

- Observation with temporary parser logging; the log was not kept.
- The display-verify `timing` test measures frame spacing and writes a CSV ([verification](../protocol/verification.md)); it has not been run yet.

## Consequences

- Rider inputs reach the ESP32 at most about 8 times per second; one lost frame leaves a gap of about 250 ms.
- Brake release is confirmed on the first released frame, because waiting for several frames at this rate delayed the throttle noticeably ([pitfalls](../pitfalls.md)).
- If the re-capture shows a different rate, these values change with it.
