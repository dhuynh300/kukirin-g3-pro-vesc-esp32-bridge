# Kukirin G3 Pro Display Protocol

Serial protocol between the Kukirin G3 Pro display (TFM13-FEIMI-1) and the motor controller, as implemented in [KukirinG3ProProtocol.h](../../lib/KukirinDisplay/src/KukirinG3ProProtocol.h). It was worked out by sending controlled values to the display and reading the LCD; [discoveries.md](discoveries.md) tells how, and [captures/](captures/) holds the raw notes.

It builds on Junk495's documentation of the G2 Pro protocol ([third_party/junk495_g2_pro_protocol](../../third_party/junk495_g2_pro_protocol)): the frame layout, start bytes and most field positions carried over to the G3 Pro.

## How Sure Each Fact Is

Each fact below has a status:

- **In use**: the ridden firmware depends on it and the scooter works as expected, so it is confirmed in practice.
- **Notes**: from the bench notes of August 2026, written up partly with a coding assistant and not re-checked since. Most are expected to hold, but some details in the old write-ups turned out wrong, so these are treated as unconfirmed until re-checked with the procedure in [verification.md](verification.md).
- **Verified**: re-checked with the procedure in [verification.md](verification.md); the row links to the data. (None yet.)
- **Wrong**: found to be incorrect; kept here so older notes are not trusted on that point.

## Link

| Fact | Status |
|---|---|
| UART, 9600 baud, 8N1 | In use |
| The display is the master: it sends a command frame and the controller answers each one with a status frame | In use |
| About 8 command frames per second, one at a time, no repeated frames. Seen with temporary logging on 2026-10-01; no log was kept. Junk495's G2 Pro documentation reports about 16 cycles per second with each frame sent three times; whether the difference comes from the display model or from the answering controller is not known | Notes |
| The display shows E-006 when valid answers stop for about 2.5 s | Notes |
| The display's TX line is 5 V and reaches the ESP32 (GPIO 35) through a 1.0 kOhm / 1.8 kOhm divider; the ESP32's 3.3 V TX drives the display's RX directly | In use |

## Command Frame (Display to Controller, 20 Bytes)

| Byte | Content | Status |
|---|---|---|
| 0 | `0x01` start | In use |
| 1 | `0x14` length (20) | In use |
| 2 | `0x01` | In use (frames with another value are rejected) |
| 3 | `0x02` in normal operation | Notes |
| 4 | Mode: `0x05` mode 1, `0x0A` mode 2, `0x0F` mode 3 | In use |
| 5 | Bit 5 (`0x20`) lights on | In use |
| 5 | Bit 6 (`0x40`) P06 kick-start; bit 7 seen set in one capture, meaning unknown | Notes |
| 6-7 | P04 magnet count, little-endian (30 = 15 pole pairs) | Notes |
| 8 | P03 wheel diameter in 0.1 in (80 = 8.0 in, 95 = 9.5 in, 100 = 10.0 in) | In use (the firmware uses it to encode speed) |
| 9 | `0x03` in all captures | Notes |
| 10 | High nibble PB regen level 0-5, low nibble PA acceleration level 1-5 | In use |
| 11 | `0x00` | Notes |
| 12 | P07 speed limit in percent | Notes |
| 13 | `0x0C` in all captures | Notes |
| 14-15 | P02 low-voltage cutoff in 0.1 V, big-endian | Notes |
| 16-17 | Throttle, 0 (released) to 1000, big-endian | In use |
| 18 | Bit 3 (`0x08`) left turn, bit 4 (`0x10`) right turn, bit 5 (`0x20`) brake lever, bit 7 (`0x80`) horn | In use |
| 18 | Bits 0 and 2 seen set in one capture, meaning unknown | Notes |
| 19 | XOR of bytes 0-18 | In use |

Example capture, mode 3, P03 = 80, P04 = 29, PA = 2, PB = 4, P07 = 100 %:

```
01 14 01 02 0F 80 1D 00 50 03 42 00 64 0C 01 CC 00 00 05 35
```

**Throttle limit per mode.** The display limits the throttle value according to the selected mode: at full throttle it sends at most 360 in mode 1, 680 in mode 2 and 1000 in mode 3. The limiting itself is confirmed on the scooter; the exact maximums come from the 2026-08-25 notes ([captures/2026-08-25-lcd-layout-tests.md](captures/2026-08-25-lcd-layout-tests.md)). Status: Notes for the values. In firmware v1.0.0 the ESP32 applies its own mode scaling on top of this, so the modes deliver less than intended; see [powertrain](../powertrain.md).

The firmware treats a throttle value above 1050 as a wiring fault. 1050 is the firmware's own safety margin above the display's maximum of 1000, not a value the display sends.

**P-menu.** The display stores the P-menu and sends the controller-relevant settings in every frame.

| Setting | Sent in | Status |
|---|---|---|
| P01 units (km/h or mph) | Not sent; applied by the display | Notes |
| P02 battery | Bytes 14-15 as a cutoff in 0.1 V. Logged values: 36 V setting = 31.0 V, 52 V = 46.0 V, 72 V = 65.0 V. Older notes also list 48 V = 42.0 V and 60 V = 54.0 V, but one capture made with P02 = 60 V shows the 52 V bytes, so those two are unconfirmed | Notes |
| P03 wheel diameter | Byte 8; the display also uses it for its own speed calculation | In use |
| P04 magnets | Bytes 6-7; reported not to affect the speed shown | Notes |
| P05 cruise control | Not sent | Notes |
| P06 kick-start | Byte 5 bit 6; not used by this firmware | Notes |
| P07 speed limit | Byte 12; not used by this firmware (mode speed limits are set in code) | Notes |
| P08 sleep timer | Not sent | Notes |
| PA acceleration | Byte 10 low nibble; sets the throttle rise rate | In use |
| PB regen | Byte 10 high nibble; sets the regen strength | In use |

## Status Frame (Controller to Display)

The length byte says 14. The firmware sends 16 bytes, the last two repeating the start and length bytes, and the display accepts them (In use).

| Byte | Content | Status |
|---|---|---|
| 0 | `0x02` start | In use |
| 1 | `0x0E` length | In use |
| 2 | `0x01` total odometer, `0x02` trip odometer | Notes |
| 3 | `0x00`; `0x80` on keepalive frames; error bits (below) | In use |
| 4 | `0xC0` base; bit 5 (`0x20`) brake icon; bit 3 (`0x08`) dual drive | In use |
| 4 | Bit 4 E-007, bit 0 E-009 | Notes |
| 5 | Error bits for the second controller (below) | Notes |
| 6 | Bit 3 (`0x08`) dual drive; bit 4 (`0x10`) E-017 | In use |
| 7 | `0x00` | In use |
| 8-9 | speedRaw, big-endian (see Speed) | In use |
| 10-11 | `0x00`; reported to have no visible effect | Notes |
| 12 | XOR of bytes 0-11 | In use |
| 13 | `0x00` | In use |
| 14-15 | `0x02 0x0E` | In use |

Junk495's G2 Pro documentation gives the checksum as `(speedRaw_H ^ speedRaw_L) ^ 0xCD`. That is the same XOR: with bytes 2-7 and 10-11 at their idle values, bytes 0-11 XOR to `0xCD ^ speedRaw_H ^ speedRaw_L`. The full XOR is needed once error bits or other flags are set.

The firmware sets the keepalive flag (byte 3 = `0x80`) on the second reply after start-up and every 50th reply after that, when no error is shown.

## Error Codes

Names as in the G3 Pro user manual. Each code is shown while its bit is set.

| Code | Bit | Used by this firmware for | Status |
|---|---|---|---|
| E-001 rear motor hall | byte 3 `0x40` | | Notes |
| E-002 | byte 3 `0x20` | Not armed, or a lockout waiting for the throttle to be released | In use |
| E-003 main controller | byte 3 `0x10` | Link to the rear VESC lost | In use |
| E-005 voltage | byte 3 `0x08` | | Notes |
| E-006 receive error | none: shown when replies stop or the checksum is wrong | | Notes |
| E-007 sending error | byte 4 `0x10` | | Notes |
| E-009 controller temperature | byte 4 `0x01` | | Notes |
| E-011 front motor hall | byte 5 `0x40` | | Notes |
| E-013 secondary controller | byte 5 `0x10` | | Notes |
| E-015 | byte 5 `0x08` | | Notes |
| E-017 sub controller receiving | byte 6 `0x10` | Front VESC lost | In use |
| E-031, then E-00 | byte 5 `0x20` | Avoid: reported to flash E-031 and then show E-00 until the display is power-cycled | Notes |

Also from the notes, not re-checked: with two or more bits set the display alternates between the codes; no other bit in bytes 3-6 produces a code (no E-004, E-008, and so on).

## Speed

The status frame does not carry a speed. It carries speedRaw, and the display computes the speed it shows from speedRaw and P03 with integer arithmetic. The model used by the firmware:

```
speed_tenths = floor(287.275 * P03 / speedRaw)        speed in 0.1 km/h, P03 in 0.1 in
km/h shown   = floor(speed_tenths / 10)
mph shown    = floor(speed_tenths * 6214 / 100000)
```

speedRaw behaves as the time for one wheel turn in milliseconds: with the diameter in 0.1 in and the period in ms, the factor for 0.1 km/h is pi x 0.0254 x 3600 = 287.267, within 0.003 % of the fitted 287.275. At P03 = 80, the only value in the notes that tells the two apart is speedRaw 2, listed as 714 mph (287.267 would give 713); it is to be re-checked. The display shows 0 for speedRaw 3500 and above; the firmware sends 3500 when stopped.

What supports the model:

- In use: the firmware converts the VESC's wheel speed to speedRaw with this model on every frame. The shown speed has not yet been compared with GPS.
- Notes: in late August 2026, with P03 = 80, speedRaw was stepped through every value from 1 to 2299 in km/h and 1 to 1352 in mph, and each LCD reading was compared by eye with the model; no mismatches were found. The per-value records were not kept; [captures/speed-sweep-mph.txt](captures/speed-sweep-mph.txt) is a short summary written afterwards. The last values that still show 1 are 2298 (km/h) and 1351 (mph). Other P03 values were not swept. The LCD has three digits, so the km/h values the model puts above 999 (speedRaw 1 and 2) cannot have been shown as such; what the display does there is to be re-checked.
- Notes: alternating between two speedRaw values made the display show one of them or jump between them, never a value in between, so it does not average.
- Wrong: an older table put the last 1 mph value at 1352, which contradicts its own formula and the summary note.
- Wrong: the 2026-08-25 bench notes give a speed formula with factors 285/16 (mph) and 459/16 (km/h), marked as calibrated; the integer model above replaced it.
- Wrong: the same notes give P03 as 658 + 2 x (diameter in 0.1 in) over bytes 8 and 9. Byte 8 is the diameter in 0.1 in.

The cutoffs and the values listed in the notes are encoded in [test/test_speed_codec](../../test/test_speed_codec/test_main.cpp), so a model change that disagrees with them fails the tests.

To show a speed, the firmware sends `speedRaw = round(17.8512 * P03 / mph)`, the inverse of the model (17.8512 = 287.275 x 0.06214). Because speedRaw is a whole number of ms, the shown speed can be 1 mph lower than sent; with a 10 in wheel this holds up to 60 mph.
