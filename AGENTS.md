# Agent Guide

Instructions for coding agents working in this repository.

## Project

ESP32-WROVER firmware that sits between the Kukirin G3 Pro display (TFM13-FEIMI-1, UART) and the rear VESC motor controller (UART), plus the LispBM script that runs on the rear VESC and drives the front VESC over CAN. It controls a 52 V vehicle with two high-current motor controllers: mistakes can cause injury.

## Layout

- `src/main.cpp`: firmware application.
- `lib/KukirinDisplay/`: display protocol library.
- `lib/VESCBridge/`: VESC UART link and safety supervisor.
- `lib/BTAudio/`: planned feature, not built into the firmware.
- `vesc/main.lbm`: LispBM powertrain script for the rear VESC.
- `tools/`: checks and helper scripts.
- `test/`: native unit tests (Unity).
- `third_party/`: pinned upstream sources (VESC firmware, VESC Tool, LispBM, Junk495's G2 Pro work). Read-only reference; never edit.

## Commands

- Build: `pio run`
- Unit tests on the PC: `pio test -e native` (needs a host gcc on PATH; libraries run unchanged against the stand-ins in test/support)
- LispBM bracket check: `python tools/check_lisp_parens.py`
- ESP32 and LispBM constants agree: `python tools/check_shared_constants.py`
- Project rules (forbidden calls, log format, doc style, secrets): `python tools/lint_invariants.py`
- Compare two firmware builds: `python tools/compare_firmware.py --ignore-build-time A.elf B.elf`

## Rules

- Never upload or flash firmware (`pio run -t upload` or similar). Uploads are done manually.
- Facts come from code and measured data. If a doc and the code disagree, report it; do not pick one silently.
- Do not invent measurements, dates, test results or motivations. Ask.
- LispBM safety:
  - Never use `set-handbrake`: it holds current at standstill and heats the motor coils. Braking uses passive current braking.
  - Never use `foc-beep`: it stops the motor and sleeps the LispBM interpreter for the whole beep, halting every script thread. Use `foc-play-tone`.
  - Never use the 3-argument `canset-current` (with off-delay): VESC firmware 7.00 decodes those CAN fields in the wrong order. Use `canset-current-rel` with an off-delay instead.
- ESP32 control paths: no `delay()`, no heap allocation (`new`, `malloc`, `String` growth).
- Control that must keep working when the ESP32 link fails (timeouts, speed limits, motor commands) belongs in the LispBM script, not on the ESP32.
- Write each fact once. Constants are documented where they are defined; docs link to them.
- Runtime log tokens contain no square brackets.

## Writing Style

- Plain technical English, short sentences. Say what the code does and why.
- Every number has units and a source.
- No emojis, no numbered headings, no marketing words, no ALL-CAPS rule names.
- Diagrams in Mermaid, not ASCII art.
- Commit messages: Conventional Commits describing the technical change, for example `fix(display): ...`.
