# Control Frames Sent On Each Display Frame And Every 20 ms

Status: Accepted.

## Context

The ESP32 sends a control frame (throttle, brake, dual drive, speed limit) to the rear VESC. New rider input arrives with each display frame, about every 125 ms. The VESC script cuts the motors after 500 ms without a valid frame. It also contains a limit meant to slow throttle increases after a gap of more than 35 ms between frames; that limit never acts ([pitfalls](../pitfalls.md#drive-resumes-at-once-after-a-dead-man-cut)).

## Decision

Send a control frame right after each valid display frame, and otherwise every 20 ms on a fixed grid ([src/main.cpp](../../src/main.cpp), `loop()`). After a display frame the grid restarts from that moment. After a long stall the grid restarts instead of sending a burst of catch-up frames. Log lines for the VESC terminal go 8 to 14 ms after a control frame, halfway between two control frames.

## Alternatives Considered

- Only a fixed 20 ms timer: new input would wait up to 20 ms for the next tick.
- Only on display frames: a lost display frame would leave 250 ms without a control frame, half the dead-man timeout.

## Evidence

- Code: `display_packet_received || keepalive_timeout` in `loop()`. The dispatch itself has no unit test; the frames it sends and the safety logic that builds them are tested (`test/test_vesc_link`, `test/test_safety`).

## Consequences

- Input reaches the VESC as soon as the ESP32 has it.
- Control frames are not evenly spaced: the interval after a display frame can be shorter than 20 ms.
- The VESC dead-man timer is fed every 20 ms even when display frames are lost.
- The ESP32 sends control frames only while it receives telemetry; after 500 ms without telemetry it stops sending them ([pitfalls](../pitfalls.md#the-esp32-stops-sending-control-frames-when-telemetry-is-lost)).
