# How the Protocol Was Worked Out

This is the account from the August 2026 bench notes. Parts of the original write-ups were drafted with a coding assistant and some details proved wrong, so the status of each fact (in use, from notes, or wrong) is tracked in [README.md](README.md), and [verification.md](verification.md) describes how the notes will be re-checked.

The starting point was Junk495's G2 Pro bridge and protocol documentation ([third_party](../../third_party/)), adapted for the G3 Pro in July 2026. The G3 Pro display answered to the same framing, but several fields, the error codes and the speed calculation had to be found by testing. Dates come from the commit history; raw notes are in [captures/](captures/).

## Method

An ESP32 test firmware played the controller: it read the display's command frames and answered with status frames, changing one byte or bit per step. Steps were advanced by hand from the serial console, and after each step the LCD was read and the result written down. Command frame fields were found the other way round: change one control or P-menu setting on the scooter and see which byte changes.

## First Tests (2026-08-25)

The command frame was decoded first: mode byte, throttle (0 to 1000, big-endian), light, turn signal, brake and horn bits, and the P-menu fields. The display limits the throttle value per mode: at full throttle it sent at most 360 in mode 1, 680 in mode 2 and 1000 in mode 3. Setting byte 4 bit 3 and byte 6 bit 3 switched the LCD between SINGLE and DUAL.

## Checksum (Runs 1 and 2)

Run 1 walked through every byte of the status frame. Most changes produced E-006, and one change to byte 6 produced E-017, which was at first read as that byte's meaning. The cause was the checksum: the code carried over from the G2 Pro bridge computed it from the two speed bytes and the constant `0xCD`, so any other change made the frame invalid.

The G3 Pro checksum is the XOR of bytes 0-11. The G2 Pro formula is the same XOR with all other bytes at their idle values, which is why it worked as long as only the speed changed. With the full XOR, Run 2 could change any field without E-006.

## Error Codes (Runs 2 to 7)

- Runs 2 and 3 mapped byte 3 (E-001, E-002, E-003, E-005), byte 4 (E-007, E-009, brake icon, dual drive) and byte 5 (E-013, E-015).
- Run 3 also saw E-00 in several situations (unknown bits, two faults at once, a fault while moving) and concluded that the display hides faults while moving. Runs 6 and 7 showed that was wrong: with valid frames, two faults alternate on the display and faults are shown while moving.
- Run 5 found E-011 (byte 5 `0x40`) and E-017 (byte 6 `0x10`) and first saw them only in dual mode; Runs 6 and 7 showed them in single mode too.
- Run 6 found that byte 5 `0x20` makes the display flash E-031 and then show E-00. Run 7 showed that E-00 stays until the display is power-cycled; no frame sent over the link cleared it.
- Run 7 set every remaining bit in bytes 3-6: none produced a code, so there are no codes between the known ones (no E-004, E-008, E-010 and so on).

## Unused Fields (Run 4)

Sweeping bytes 7, 10 and 11 through all 256 values changed nothing on the LCD. The display has no power or current bar, and its battery gauge and voltage reading do not come from the status frame, so it appears to measure its own supply.

## P-Menu (Run 8)

Each P-menu setting was changed on the display and the command frame captured. P02 (battery) is sent as a low-voltage cutoff in 0.1 V (but a capture taken with P02 = 60 V shows the same bytes as 52 V, so this needs a re-check), P03 as the wheel diameter in 0.1 in, P04 as a magnet count, PA and PB share one byte, and P07 is a plain percentage. P01, P05 and P08 stay inside the display. The full mapping is in [README.md](README.md).

## Speed (Runs 4, 5 and 9)

Speed is sent as speedRaw, which falls as speed rises. Early tests fitted a constant to hit a few target speeds. The final model came from asking how a small microcontroller would compute speed with integer math: a speed in 0.1 km/h from the wheel size and speedRaw, then a conversion to km/h or mph with integer division. The constant that matches the display, 287.275, is pi x 0.0254 x 3600 within 0.003 %, which is what it would be if speedRaw were the wheel's revolution time in milliseconds. Among the values in the notes, only speedRaw 2 (714 mph) tells 287.275 apart from 287.267, so the exact constant rests on that one reading until it is re-checked.

Run 9 tested the model with P03 = 80: speedRaw was stepped through every value from 1 to 2299 in km/h and from 1 to 1352 in mph, and each LCD reading was compared by eye with the prediction. No mismatches were noted, but the per-value records were not kept; only a short summary of the mph sweep exists. Changing P04 did not change the speed shown. Other P03 values were not swept. The LCD has three digits, so the km/h values the model puts above 999 (speedRaw 1 and 2) cannot have been shown as such.

An older table in these notes put the last 1 mph value at speedRaw 1352. That contradicts its own formula and the raw sweep note, which says 1352 and above showed 0; the last 1 mph value is 1351 (corrected 2026-10-06 when the unit tests were written).

## No Averaging (Run 10)

speedRaw was alternated between two values every few frames. The LCD never showed a value in between: it either settled on one of the two or jumped between them. The display shows each new speed directly.

## Frame Rate (2026-10-01)

Timing the display's command frames with temporary logging showed about 8 frames per second, one frame at a time, without repeated frames. The log was not kept, so this is an observation rather than a measurement. Junk495's G2 Pro documentation describes about 16 cycles per second with each frame sent three times; whether the difference comes from the display model or from the controller answering it is not known.
