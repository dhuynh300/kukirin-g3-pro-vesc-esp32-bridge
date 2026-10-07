# Changelog

## Version 1.0.0 (2026-10-06)

First public release. The firmware is the 2026-10-01 build, the first ridden in daily use; every loaded section of the ESP32 image matches that build ([tools/compare_firmware.py](tools/compare_firmware.py)).

- ESP32 bridge between the Kukirin G3 Pro display (TFM13-FEIMI-1) and the rear VESC: display protocol, speed shown from the VESC telemetry, error codes, lights, turn signals, hazards and horn.
- Brake-to-start arming with a killswitch line to the rear VESC, and lockouts after faults, a lost VESC link or a stalled display.
- LispBM script on the rear VESC: current commands for both motors, front VESC over CAN, per-mode speed limits per wheel, regen faded in with speed, hill hold, 500 ms dead-man timeout, rear-only drive when the front VESC is lost.
- Native unit tests, the display verification tool, and checks for shared constants, project rules and firmware identity.

Known issues are listed in [docs/known-issues.md](docs/known-issues.md).

## Development Before Public Release

Dates come from the development history of the earlier private repository.

- July 2026: Junk495's G2 Pro bridge adapted to the G3 Pro display.
- 2026-08-25 to 2026-08-28: display protocol worked out on the bench: register sweeps, error codes, P-menu fields and the speed model ([protocol](docs/protocol/discoveries.md)).
- 2026-09-04: first LispBM script for the rear VESC, with the front VESC commanded over CAN.
- 2026-09-10 to 2026-09-16: battery voltage readings of both VESCs and the display compared with a multimeter; software noise filtering on the display input, and noise tests.
- 2026-09-17: display, ESP32 and VESC running together on the bench; brake-to-start, the killswitch line and the 230400 baud VESC link added.
- 2026-09-21: motor commands enabled; arming tones. Link timeouts during the tones traced to `foc-beep`, which pauses the whole script, and fixed by using `foc-play-tone`.
- 2026-09-26: brake-to-start fixed at 0 mph; per-mode speed limits; speed-scaled regen and hill hold; 1 s off-delay when coasting; front motor spinning up in rear-only mode fixed; control frames sent right after each display frame instead of on a timer.
- 2026-09-27: brake release confirmed on the first frame instead of three; speed limits applied per wheel in the VESC script.
- 2026-09-29 to 2026-10-01: libraries split into standalone APIs; LispBM loop timing fixed and most allocation removed from the loops; all link timeouts set to 500 ms; packet loss logging; display frame rate observed at about 8 per second. The build from 2026-10-01 is the first ridden in daily use and is released as 1.0.0; display link dropouts during high-current launches remain an open issue.
- 2026-10-05: a link timestamp newer than the loop time found to cause false packet-drop logs; the fix ships in 1.0.1.
