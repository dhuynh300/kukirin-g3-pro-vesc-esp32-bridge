# Pitfalls

Failure modes found while building this system, and known hazards the code is written to avoid, with how it avoids each one. Status says where each comes from:

- Seen: happened on this scooter or showed up in this code.
- Source: confirmed in the VESC or LispBM source, or the ESP32 documentation.
- Notes: from the development notes, not re-checked.
- Precaution: a known hazard of this kind of design; not seen on this scooter.

Open means the fix is not done yet.

## LispBM Script

### Reading past the end of a buffer stops the thread

- Problem: `uart-read` returns an eval error when the requested length plus offset is larger than the buffer ([lispif_vesc_extensions.c](../third_party/vesc_firmware_v7/lispBM/lispif_vesc_extensions.c) line 3502). An eval error ends the thread that hit it.
- Symptom: the powertrain thread stops; the motors get no new commands until the VESC's own 500 ms timeout stops them.
- Fix: buffers are allocated once at load time with room for the largest frame (`rx-frame` is 135 bytes, payloads are limited to 120), and every read is bounded by the frame length.
- Status: Source.

### Reading one frame per pass lets commands fall behind

- Problem: control frames arrive every 20 ms. If the script handles one frame per pass and anything delays a pass, frames queue up in the UART buffer and every later frame is handled late.
- Symptom: would show as throttle and brake responding with a growing delay.
- Fix: each pass reads up to 11 frames and acts on the newest control frame.
- Status: Precaution.

### Allocating inside the loops causes garbage collection pauses

- Problem: building strings or lists inside the 20 ms loop allocates memory that the garbage collector must later reclaim, which pauses the interpreter.
- Symptom: would show as irregular loop timing.
- Fix: buffers are allocated once at load time. One exception remains: parsing the ESP32 version string allocates; it runs only when a version line arrives and is due to move out of the loop.
- Status: Precaution.

### Integer division truncates

- Problem: dividing two integers gives an integer result, so `(/ 15 2)` is 7 ([fundamental.c](../third_party/vesc_firmware_v7/lispBM/lispBM/src/fundamental.c) line 146).
- Symptom: ratios and scales silently become 0 or step in whole numbers.
- Fix: convert with `to-float` before dividing whenever a fraction is needed, as the script does for throttle, brake and speed values.
- Status: Source.

### foc-beep halts the whole script

- Problem: `foc-beep` stops the motor, reconfigures FOC and sleeps the LispBM interpreter for the length of the beep ([mcpwm_foc.c](../third_party/vesc_firmware_v7/motor/mcpwm_foc.c), `mcpwm_foc_beep` from line 2104), not only the thread that called it.
- Symptom: every script thread, including the dead-man check, stops for the beep.
- Fix: the arming chime uses `foc-play-tone` in its own thread.
- Status: Source.

### canset-current with an off-delay sends the fields in the wrong order

- Problem: in VESC firmware 7.00, the sender packs current then off-delay ([comm_can.c](../third_party/vesc_firmware_v7/comm/comm_can.c) lines 523-530), but the receiver reads off-delay then current (lines 1613-1619). A 1.0 s off-delay arrives as 1.0 A of current.
- Symptom: the front motor gets a current that was never commanded.
- Fix: the script uses `canset-current-rel` whenever an off-delay is needed; its packet has the fields in the same order on both sides.
- Status: Source.

### Drive resumes at once after a dead-man cut

- Problem: the script stores the arrival time of each frame before it measures the gap since the previous one ([vesc/main.lbm](../vesc/main.lbm), `last-rx-time` and `delta-rx`), so the gap is always close to zero on a pass that received a frame. The limit of 2000 counts per pass after a gap never acts. The dead-man cut is not reported in telemetry, and nothing requires the throttle to be released afterwards.
- Symptom: if control frames stop for more than 500 ms while telemetry still reaches the ESP32, the motors stop and then return straight to the requested throttle when frames resume, with no code on the display.
- Fix: planned for v1.0.1: measure the gap before storing the new arrival time, and require the throttle to be released after a dead-man cut.
- Status: Source, open.

### Any valid frame resets the dead-man timer

- Problem: the script resets its dead-man timer on every valid frame, log frames included. The ESP32 sends some log lines even while it is not sending control frames.
- Symptom: each such log frame keeps the last throttle in force for another 500 ms.
- Fix: planned for v1.0.1: reset the timer only on control frames.
- Status: Source, open.

### One low killswitch reading disarms the script

- Problem: the script reads ADC2 once per 20 ms pass without filtering, and one reading below 1.65 V disarms it. It arms again by itself once the rear wheel is below 300 ERPM with the throttle released. Its armed state is not in telemetry.
- Symptom: noise or a loose contact while riding cuts drive and regen until the scooter stops, with no code on the display; the ESP32 shows E-002 only when it drove the line low itself. The script then arms again without a new brake-to-start.
- Fix: planned: filter the reading over several passes and report the armed state in telemetry.
- Status: Source, open.

### Long log frames can overflow the VESC's receive queue

- Problem: the VESC firmware's serial receive queue holds 128 bytes (`SERIAL_BUFFERS_SIZE`, [halconf.h](../third_party/vesc_firmware_v7/halconf.h) line 294). A log frame of up to 126 bytes and a control frame do not both fit if the script misses a read pass.
- Symptom: would show as lost control frames.
- Fix: planned: shorter log frames.
- Status: Source, open.

## Braking

### Brake mode can drive a wheel near standstill

- Problem: VESC brake mode applies current against the sign of the measured speed (`iq = -SIGN(speed) * |iq|`, [mcpwm_foc.c](../third_party/vesc_firmware_v7/motor/mcpwm_foc.c) line 3437), and near standstill the speed estimate is mostly noise. The firmware shorts the phases (duty 0) when the sign changes or the duty cycle is close to zero (lines 3330-3347), so how the wheel was still driven is not confirmed.
- Symptom: the front wheel was driven forward while braking at low speed.
- Fix: below the regen floor the script sends zero current instead of a brake command ([powertrain](powertrain.md#regenerative-braking)).
- Status: Seen.

### Hill hold braked stopped wheels

- Problem: hill hold applied full brake mode once the scooter had been stopped for 1 s, putting brake mode at exactly the standstill the regen floor avoids. The exact cause of the movement is not confirmed ([powertrain](powertrain.md#hill-hold-known-issue)).
- Symptom: when regen came back on at a stop, the motors drew current and turned to a fixed position; rarely, a wheel moved backwards or forwards.
- Fix: v1.0.1 (in testing) brakes a wheel during hill hold only while it turns faster than 0.6 mph ([powertrain](powertrain.md)).
- Status: Seen.

### The VESC handbrake mode heats a stopped motor

- Problem: `set-handbrake` applies a fixed current vector in open loop ([mcpwm_foc.c](../third_party/vesc_firmware_v7/motor/mcpwm_foc.c) lines 852 and 3587-3589), holding current in the windings for as long as it is commanded.
- Symptom: I²R heating in a stationary hub motor with little cooling.
- Fix: the script never uses it; braking uses `set-brake-rel` and `canset-brake-rel`.
- Status: Source.

## Front VESC

### The front VESC must stop by itself when commands stop

- Problem: the front VESC only follows CAN commands from the rear one. If the CAN link or the rear VESC fails, nothing tells the front to stop.
- Symptom: the front motor would keep its last commanded current.
- Fix: the front VESC's own timeout (500 ms, set in VESC Tool) is reset by each CAN current command ([comm_can.c](../third_party/vesc_firmware_v7/comm/comm_can.c), `timeout_reset` in the command handlers) and stops the motor when commands stop.
- Status: Source.

### The front VESC is not told to stop when it is marked offline

- Problem: once the script marks the front VESC offline (no status message for 500 ms), it stops sending it commands, zero current included (`cut-motors` and the drive and brake paths check `front-vesc-online`).
- Symptom: if only the front's status messages are lost, the front keeps its last command until its own 500 ms timeout stops it; a brake pulled in that time gets no regen from the front.
- Fix: planned for v1.0.1: send zero current to the front whether or not it is marked online.
- Status: Source, open.

### Front VESC faults are not watched

- Problem: telemetry carries only the rear VESC's fault code.
- Symptom: a front fault stops the front motor for the VESC's fault stop time; then it follows the script's commands again, with no lockout and no code on the display.
- Fix: planned: read the front's fault code over CAN and handle it like a rear fault.
- Status: Source, open.

## ESP32

### The ESP32 stops sending control frames when telemetry is lost

- Problem: after 500 ms without telemetry, the ESP32 stops sending control frames ([src/main.cpp](../src/main.cpp), `if (vesc_online)` around `send_control`), although the frame it builds then already has zero throttle. The script keeps the last throttle and brake until its own dead-man timeout, up to 500 ms later.
- Symptom: a brake pulled in that time does not reach the motors, and drive continues at the last throttle for up to about 1 s after telemetry stopped.
- Fix: planned for v1.0.1: send control frames whether or not telemetry arrives.
- Status: Source, open.

### GPIO 16 and 17 belong to the PSRAM

- Problem: on the ESP32-WROVER module, GPIO 16 and 17 are wired to the PSRAM.
- Symptom: using them for UART or GPIO breaks PSRAM access.
- Fix: no pin assignment uses them; the display UART uses GPIO 35 and 33 ([hardware](hardware.md)).
- Status: Source.

### GPIO 34 to 39 have no pull resistors

- Problem: these pins are inputs only and have no internal pull-up or pull-down; `INPUT_PULLUP` has no effect on them.
- Symptom: an input that is not driven floats and reads at random.
- Fix: the dual/single button on GPIO 39 has a 10 kOhm pull-up on the board; the UART inputs on GPIO 34 and 35 are always driven.
- Status: Source.

### The display's TX line is 5 V

- Problem: the display drives its TX line to 5 V; ESP32 inputs take at most 3.6 V.
- Symptom: risk of damaging the ESP32 input.
- Fix: a 1.0 kOhm / 1.8 kOhm divider brings it to 3.2 V ([hardware](hardware.md#display-link)).
- Status: Source.

### Log frames can delay control frames

- Problem: the ESP32 sends log lines to the VESC script over the same UART as the control frames. A long log frame sent just before a control frame delays it.
- Symptom: would show as control frames arriving late.
- Fix: log frames are sent between control frames, at least 2 ms after the previous frame. The wait is currently a busy wait inside the transmit lock and is due to be replaced.
- Status: Precaution.

### A link timestamp can be newer than the loop's time

- Problem: the loop reads `millis()` once at the top; the VESC and display updates then store the time of frames received afterwards. Subtracting gave a negative age that wrapped to about 4.29e9 ms.
- Symptom: false packet-drop log lines.
- Fix: v1.0.1 (in testing) treats a timestamp newer than the loop time as age 0 (`link_age_ms`, with tests).
- Status: Seen.

### The killswitch line has no pull-down

- Problem: the killswitch output from GPIO 2 to the rear VESC's ADC2 input has no pull-down resistor.
- Symptom: if the wire breaks or comes unplugged, ADC2 floats and may not read as "killed".
- Fix: planned, a pull-down resistor at the VESC end ([hardware](hardware.md#killswitch)). Until then the other layers still apply: zero throttle until armed, and the 500 ms dead-man timeout in the script.
- Status: Open.

## Display

### Confirming brake release over several frames delays throttle

- Problem: the brake reads as released only after a number of consecutive display frames show it released. With 3 frames at about 8 frames per second, that took 250 to 375 ms after letting go of the lever.
- Symptom: a noticeable delay between releasing the brake and the throttle working again.
- Fix: the brake now counts as released on the first frame that shows it (`brakeReleaseQuorum` = 1 in `BridgeTimingConfig`), 0 to 125 ms after letting go. Brake on still takes effect on the first frame.
- Status: Seen.

### The display already limits throttle per mode

- Problem: the ESP32's mode ratios were written for a raw throttle that reaches 1000 in every mode, but the display already limits it per mode.
- Symptom: the two reductions multiply; mode 1 delivers about 18 % of the current limit and feels weak.
- Fix: planned for v1.1, remove the ESP32 mode ratio ([powertrain](powertrain.md#the-mode-ratio-compounds-with-the-display-limit)).
- Status: Seen.

### One status bit latches an error until power-off

- Problem: setting byte 5 bit `0x20` in the status frame makes the display show E-031 and then E-00 until it is power-cycled.
- Symptom: the display stays in E-00.
- Fix: the firmware never sets that bit ([protocol](protocol/README.md)).
- Status: Notes.

### Display frames are lost under high motor current

- Problem: during high-current launches, frames on the display link are lost or corrupted; the VESC link is not affected.
- Symptom: display timeouts, which zero throttle until the link recovers and the throttle is released.
- Fix: planned steps are in [hardware](hardware.md#display-link).
- Status: Seen, open.
