# Protocol Test Notes

Raw notes from the protocol work, kept as written at the time (only the file names changed). They were made between 2026-08-25 and 2026-08-28.

Method for the register sweeps: a test firmware on the ESP32 answered the display with status frames in which one field or bit was changed per step. Stepping was manual from the serial console; after each step the display was read by eye and the result typed into the file. "single only" / "dual only" mean the display did not accept the frame and kept its last state. The test firmware is not part of this repository.

Some early interpretations in these notes were later corrected, and some write-ups from that time were drafted with a coding assistant and contain mistakes. [../discoveries.md](../discoveries.md) gives the final reading and [../README.md](../README.md) the status of each fact.

| File | Contents |
|---|---|
| [2026-08-25-lcd-layout-tests.md](2026-08-25-lcd-layout-tests.md) | First tests: LCD layout, command frame fields (mode, throttle range, throttle limit per mode, switches, P-menu), dual drive indicator, first speed calibration |
| [register-sweep-run1.txt](register-sweep-run1.txt) | 88 steps: every status frame byte, with the old checksum (many results are E-006 because of it) |
| [register-sweep-run2.txt](register-sweep-run2.txt) | 60 steps after the checksum fix: bytes 3-6 stopped and at 11 mph, braking |
| [register-sweep-run3.txt](register-sweep-run3.txt) | 45 steps: bit by bit through bytes 3-6, sweeps of the unknown bytes, fault combinations, motion |
| [register-sweep-run4.txt](register-sweep-run4.txt) | 7 steps: sweeps of bytes 7, 10, 11 and speed control tests |
| [register-sweep-run5.txt](register-sweep-run5.txt) | 14 steps: speed targets, E-011 and E-017, remaining bits |
| [register-sweep-run6.txt](register-sweep-run6.txt) | 36 steps: re-check of all codes, single vs dual mode, two faults at once, unknown bits, E-031 |
| [register-sweep-run7.txt](register-sweep-run7.txt) | 28 steps: second-controller codes in single mode, search for codes between the known ones, fault combinations within one byte, E-00 recovery attempts |
| [speed-sweep-mph.txt](speed-sweep-mph.txt) | Short summary of the mph speed sweep (P03 = 80), written afterwards: no mismatches from speedRaw 1 to 1352; 1352 and above showed 0. The per-value records were not kept |

The km/h speed sweep (speedRaw 1 to 2299), the P-menu mapping and the averaging test left no separate notes; their results are described in [../discoveries.md](../discoveries.md).
