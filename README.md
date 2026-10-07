# Kukirin G3 Pro VESC ESP32 Bridge

[![CI](https://github.com/dhuynh300/kukirin-g3-pro-vesc-esp32-bridge/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/dhuynh300/kukirin-g3-pro-vesc-esp32-bridge/actions/workflows/ci.yml)
[![License: GPL-3.0](https://img.shields.io/badge/license-GPL--3.0-blue)](LICENSE)
[![Release](https://img.shields.io/github/v/release/dhuynh300/kukirin-g3-pro-vesc-esp32-bridge)](https://github.com/dhuynh300/kukirin-g3-pro-vesc-esp32-bridge/releases/latest)

ESP32 firmware that lets the Kukirin G3 Pro scooter's original display (TFM13-FEIMI-1) drive two VESC motor controllers, plus the LispBM script that runs the powertrain on the rear VESC.

## What It Does

- Answers the original display like the stock controller did: speed, mode, error codes, and the display's throttle, brake, switches and P-menu settings.
- Turns throttle and brake into current commands for two hub motors, with per-mode speed limits per wheel and regenerative braking that fades in with speed.
- Arms the motors only after brake-to-start, through a killswitch line to the rear VESC, and locks drive after faults or a lost link until the throttle is released.
- Stops the motors when a link fails, with 500 ms timeouts on each link: display to ESP32, ESP32 to VESC, and the CAN link to the front VESC.
- Runs the lights, turn signals, hazards, brake light and horn from the handlebar controls.

## Why I Built This

I wanted to add more features to the scooter, like Bluetooth motor speakers, ARGB accent lighting and better throttle control, and to learn circuit design and more embedded programming.

## How It Works

The display talks to the ESP32 over UART at 9600 baud, about 8 frames per second. The ESP32 sends a control frame to the rear VESC at least every 20 ms over UART at 230400 baud. The rear VESC's LispBM script commands its own motor and the front VESC over CAN, and sends speed and fault telemetry back. [docs/architecture.md](docs/architecture.md) has the diagram, the threads, the arming sequence and what happens on each failure.

| Part | Responsible for |
|---|---|
| Display | Throttle, brake and switches; shows speed, mode and error codes; holds the P-menu settings |
| ESP32 | Display protocol, throttle and brake request, brake-to-start, lockouts, killswitch line, lights and horn |
| Rear VESC script | Current commands for both motors, speed limits, regen, dead-man timeout, watching the front VESC |
| Front VESC | Follows CAN commands; its own timeout stops the motor if they stop |

## Results

| Result | How it was established |
|---|---|
| The released ESP32 image matches the ridden 2026-10-01 build in every loaded section | `tools/compare_firmware.py --ignore-build-time` on the two ELF files, built with the same version string; the build date and time strings are masked |
| 77 unit tests pass on a PC | `pio test -e native` |
| The display speed model matched the LCD in a sweep of speedRaw 1 to 2299 (km/h) and 1 to 1352 (mph), with P03 = 80 | Stepped through each value and compared by eye; per-value records not kept, re-check planned ([protocol](docs/protocol/README.md#speed)) |
| The display sends about 8 frames per second | Observed with temporary logging; not kept, re-capture planned |

Test reports with raw data will be added under `docs/test-reports/` as the [verification runs](docs/protocol/verification.md) and release tests are done.

## Engineering Highlights

- Display protocol: worked out on the bench with an ESP32 test firmware that changed one status-frame bit per step while the LCD was read. The speed shown follows `floor(287.275 * P03 / speedRaw)` tenths, and the constant is within 0.003 % of pi x 0.0254 x 3600, so speedRaw behaves as the wheel revolution period in milliseconds (the exact constant rests on one reading that is to be re-checked). Each fact is marked with how sure it is ([protocol](docs/protocol/README.md), [how it was found](docs/protocol/discoveries.md)).
- Display timing: the G3 Pro display sends about 8 frames per second, one at a time; Junk495's G2 Pro documentation reports about 16 cycles per second with repeated frames, which may reflect a different display model. The link timeouts are sized for the observed rate ([decision 0001](docs/decisions/0001-display-frame-timing.md)).
- VESC CAN off-delay: in VESC firmware 7.00, `canset-current` with an off-delay is sent and received with its fields in a different order, so a 1.0 s off-delay arrives as 1.0 A. The script uses `canset-current-rel` instead ([decision 0005](docs/decisions/0005-can-off-delay-workaround.md)).
- Layered safety: brake-to-start and lockouts on the ESP32, a killswitch line, and a dead-man timeout and speed limits in the VESC script that keep working if the ESP32 link fails ([decision 0006](docs/decisions/0006-motor-control-in-the-vesc-script.md)).

## Hardware

Kukirin G3 Pro with its stock hub motors and battery, two Spintend Ubox Single 100 controllers running VESC firmware 7.00, and an ESP32-WROVER-IE DevKitC on a custom carrier board. Pins, level shifting, the killswitch line and the light outputs are in [docs/hardware.md](docs/hardware.md); part details and controller settings in [docs/system_spec.md](docs/system_spec.md).

## Building And Flashing

- ESP32 firmware: `pio run -e esp-wrover-kit`, then `pio run -e esp-wrover-kit -t upload`.
- VESC script: open `vesc/main.lbm` in VESC Tool's LispBM scripting page, connected to the rear VESC, and upload it.
- Unit tests on a PC: `pio test -e native` (needs gcc on PATH).
- Checks: `python tools/check_lisp_parens.py`, `python tools/check_shared_constants.py`, `python tools/lint_invariants.py`.
- Display verification: see [docs/protocol/verification.md](docs/protocol/verification.md).

## Repository Layout

| Path | Contents |
|---|---|
| `src/main.cpp` | ESP32 application |
| `src/display_verify.cpp` | Bench firmware for re-checking the display protocol |
| `lib/KukirinDisplay/` | Display protocol and driver |
| `lib/VESCBridge/` | VESC serial link and safety supervisor |
| `lib/BTAudio/` | Planned motor-coil audio, not built into the firmware |
| `vesc/main.lbm` | Rear VESC script |
| `test/` | Unit tests run on a PC |
| `tools/` | Checks, firmware comparison, display verification script |
| `docs/` | Documentation, decision records, protocol notes |
| `third_party/` | Pinned upstream sources: VESC firmware and VESC Tool, LispBM, Junk495's G2 Pro work |

## Roadmap

- 1.0.1: fixes for hill hold, regen tuning and the ESP32-VESC link handling (in development).
- More of the control processing moved into the VESC script, with a new ESP32-VESC protocol (in design).
- Throttle scaled by the display alone, and simpler fault recovery.
- Bluetooth motor speakers through the motor coils ([lib/BTAudio](lib/BTAudio/README.md)).
- ARGB accent lighting on the four spare LED strip outputs.
- Fixes for the display link noise under load.

## How This Was Built

I designed the architecture and the hardware, worked out the display protocol on the bench, and did all bench and road testing. I used coding agents for parts of the implementation and documentation, under the rules in [AGENTS.md](AGENTS.md), and checked changes with the unit tests, the project checks and the scooter itself.

The public history starts on 2026-10-06, when the project moved here from an earlier private repository; the development timeline before that is in the [changelog](CHANGELOG.md#development-before-public-release).

## Limitations And Safety

This firmware controls a 52 V vehicle with two high-current motor controllers. It is written for this one scooter and is still in development; open issues and planned fixes are listed in [docs/known-issues.md](docs/known-issues.md), and failure modes found during development in [docs/pitfalls.md](docs/pitfalls.md). Wiring or configuration mistakes can cause injury. It is provided without warranty; see the license.

## License

GPL-3.0. See [LICENSE](LICENSE).

## Acknowledgements

- [VESC](https://github.com/vedderb/bldc) firmware and [VESC Tool](https://github.com/vedderb/vesc_tool) by Benjamin Vedder.
- [LispBM](https://github.com/svenssonjoel/lispBM) by Joel Svensson.
- Junk495's [Kukirin G2 Pro ESP32 bridge](https://github.com/junk495/KukirinG2Pro-ESP32-Bridge) and [G2 Pro protocol documentation](https://github.com/junk495/Kukirin-G2-Pro---UART-Communication-Protocol), the starting point for this project.
