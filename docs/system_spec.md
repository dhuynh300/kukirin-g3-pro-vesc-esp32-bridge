# System Specification

Facts about the vehicle, the hardware and the controller configuration that are not defined in this repository's code.

Firmware constants are documented once, where they are defined:

- ESP32: `PowertrainConfig` in [lib/VESCBridge/src/VESCSafety.h](../lib/VESCBridge/src/VESCSafety.h), pins and baud rates in `HardwareConfig` in [src/main.cpp](../src/main.cpp), display protocol constants in [lib/KukirinDisplay/src/KukirinG3ProProtocol.h](../lib/KukirinDisplay/src/KukirinG3ProProtocol.h).
- VESC script: the constants block at the top of [vesc/main.lbm](../vesc/main.lbm).
- Values defined on both sides are checked by `python tools/check_shared_constants.py`.

Source column: **Measured** (method linked), **Observed** (seen during testing, no saved data), **Configured** (set in VESC Tool or hardware), **Hardware** (fitted part or marking), **Datasheet**, **Calculated**. Status says whether the row is confirmed.

## Vehicle

| Item | Value | Source | Status |
|---|---|---|---|
| Vehicle | Kukirin G3 Pro, dual hub motors | Hardware | Confirmed |
| Display | TFM13-FEIMI-1 (integrated throttle and handlebar controls), at the top of the stem | Hardware marking | Confirmed |
| Tyres | 10 x 3.0-6 tubeless | Hardware | Confirmed |
| Brakes | Hydraulic discs, 140 mm rotors; brake lever switches are active high | Hardware | Confirmed |
| Accessory supply | 52 V to 12 V step-down converter for lights and horn | Hardware | Confirmed |

## Battery

| Item | Value | Source | Status |
|---|---|---|---|
| Pack | Original factory pack, 14S8P, LG INR18650-MJ1 cells | Hardware | Confirmed |
| Battery current limits | Set in VESC Tool from the cell datasheet's recommended charge and discharge rates | Configured, datasheet | Confirmed |

## Motors

| Item | Value | Source | Status |
|---|---|---|---|
| Type | Stock brushless hub motors, 1200 W nominal, front and rear | Seller listing | Not confirmed |
| Pole pairs | 15 (30 magnets) | Counted, VESC detection | Confirmed |
| Rear: R, L, flux linkage | 38.8 mOhm, 68.16 uH, 11.283 mWb | VESC Tool motor detection | Confirmed |
| Front: R, L, flux linkage | 38.3 mOhm, 65.63 uH, 11.286 mWb | VESC Tool motor detection | Confirmed |

## Motor Controllers

| Item | Value | Source | Status |
|---|---|---|---|
| Controllers | 2 x Spintend Ubox Single 100 | Hardware | Confirmed |
| Firmware | VESC 7.00 with the standard (not `no_limits`) Ubox Single 100 hardware configuration; source pinned in third_party/ | Configured | Exact build not confirmed |
| Roles | Rear = CAN ID 26, runs the LispBM script, UART to the ESP32. Front = CAN ID 123, no script | Configured | Confirmed (code) |
| Controller hardware limits | Motor current -135..135 A, battery current -135..135 A, absolute current 180 A, input voltage 11..95 V (`hwconf/Ubox/100v/hw_ubox_100_core.h`) | VESC firmware source | Confirmed |
| Current settings | Not listed: the firmware commands a fraction of the VESC Tool limits (`set-current-rel`), so tuning them needs no firmware change | Configured | - |
| Timeout (App Settings, General) | 500 ms on both controllers | Configured | Confirmed |
| Timeout brake current | 0 A (coast on timeout) | Configured | Confirmed |
| Killswitch | Rear: ADC2 input, low = killed | Configured | Confirmed |
| CAN bus | Bit rate and termination | Configured, hardware | Not confirmed |
| CAN status message rate | 50 Hz (VESC default) | Configured | Confirmed |
| Zero vector frequency | 40 kHz (20 kHz switching, 20 kHz current sampling in V0 mode) | Configured | Confirmed |

## ESP32 Board

| Item | Value | Source | Status |
|---|---|---|---|
| Module | ESP32-WROVER-IE on an Espressif DevKitC board, on a custom carrier board with one ground plane for all logic grounds | Module marking | Confirmed |
| Flash / PSRAM | Flash size to be read with `esptool.py flash_id`; the build is configured for 4 MB | Configured | Not confirmed |
| Display RX level shifting | Divider R1 = 1.0 kOhm, R2 = 1.8 kOhm (5 V to about 3.2 V) and a filter capacitor of about 600-750 pF, next to the ESP32; 4.7 nF planned | Hardware | Confirmed |
| Display cable | Through the stem, Julet connector bundle with 26-28 AWG wires; controllers and ESP32 board at the bottom | Hardware | Confirmed |

## Observed And Measured Behaviour

| Item | Value | Source | Status |
|---|---|---|---|
| Display frame rate | About 8 frames per second, one 20-byte frame at a time, no repeated bursts | Observed (temporary parser logging; no saved data) | Re-capture planned |
| Display speed model | Integer model in `KukirinG3ProProtocol.h` matched the LCD in a sweep of speedRaw 1 to 2299 (km/h) and 1 to 1352 (mph) with P03 = 80; last value showing 1: 2298 and 1351 | Observed: stepped through each value and compared the LCD by eye; no mismatches noted, per-value records not kept ([protocol](protocol/README.md#speed)) | Not confirmed |
| Display link under load | Display frames are lost or corrupted during high-current launches, sometimes long enough for the 500 ms display timeout; the VESC link is not affected | Observed on rides | Open issue |
