# Re-Checking the Display Protocol

Several facts in [README.md](README.md) come from bench notes that were never re-checked, and some details in those notes proved wrong. This page describes how they are re-checked, so each fact can move from "Notes" to "Verified" with data behind it.

## Tools

- Firmware `display-verify` ([src/display_verify.cpp](../../src/display_verify.cpp)): answers the display like a controller, with every status frame field set from the USB console. It does not start the VESC link and holds the killswitch output low, so the motors stay off even with the VESCs powered.
- Script [tools/display_verify.py](../../tools/display_verify.py): runs each test, sends the values, asks what the LCD shows, and writes one CSV row per step into [captures/](captures/). It never shows the expected value while asking, so it cannot bias the reading; predictions are written next to the readings only in the file.

## Setup

1. Scooter on the stand, powered on (the display needs battery power).
2. Flash the verification firmware: `pio run -e display-verify -t upload`.
3. Run the script with PlatformIO's Python, which includes pyserial:
   `%USERPROFILE%\.platformio\penv\Scripts\python.exe tools/display_verify.py --port COM6 <test>`
4. Afterwards, flash the bridge firmware again: `pio run -e esp-wrover-kit -t upload`.

## Tests

| Test | What it answers | Manual work | Time |
|---|---|---|---|
| `timing` | Frame spacing of the display (the "about 8 per second" observation) | None | 1 min |
| `survey` | Which command frame bytes change for each control and P-menu setting: throttle maximum per mode, switch bits, all five P02 settings, P03, P04, P06, P07, PA, PB, and whether P01, P05 and P08 are sent | Follow 30 short prompts | 10 min |
| `speed` | The speed model: both sides of every change of the shown value up to 40 mph (or 60 km/h), the speedRaw values where the fitted constant 287.275 and the physical constant 287.267 disagree, and the stopped edge | Type the number on the LCD, about 75-105 steps per run | 5 min per run |
| `errors` | Each bit of status bytes 3-6 alone (second-controller bits in dual and single mode), three pairs, a code while moving, and the bad-checksum timeout (E-006). The E-031/E-00 latch only with `--include-latch`, as the last step | Type what the display shows | 10 min |
| `average` | Whether the display averages two alternating speeds | Describe the LCD | 3 min |

Suggested speed runs: P03 = 100 (the scooter's wheel) in mph and km/h, plus P03 = 80 and P03 = 160 in mph. `--seed` changes the step order.

`plan` prints the speed test points without a connection, to check a run before starting it.

## Recording Results

Each run writes a dated CSV into [captures/](captures/) with what was sent, what was read and, for speed, both model predictions. A fact in [README.md](README.md) changes to "Verified" only with a link to the file that supports it. A fact that the data contradicts is corrected in the code, the tests and the docs together.
