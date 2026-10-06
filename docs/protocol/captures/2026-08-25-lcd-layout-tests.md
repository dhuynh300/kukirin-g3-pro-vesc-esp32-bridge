# Kukirin G3 Pro Display (TFM13-FEIMI-1) Protocol Test Results

**Date**: 2026-08-25  
**Target Hardware**: Kukirin G3 Pro (`TFM13-FEIMI-1` Display, ESP32-WROVER bridge on COM6)  
**Firmware Under Test**: `tests/test_display_protocol.cpp`  

---

## 1. Verified Hardware & LCD Layout
- **Center Speed Display**: 3 large digits (`000` to `999` with `mph`/`km/h` selector).
- **Drive Mode Indicator (Top-Left)**: Displays `SINGLE` or `DUAL`.
- **Mode Indicator (Right)**: Bordered box showing `Mode 1`, `Mode 2`, `Mode 3`.
- **Battery Gauge (Bottom-Left)**: Voltage readout (e.g. `53.4V`) and 8-segment battery "fuel gauge" outline.
- **Power/Current Meter**: **None**. There are no real-time wattage/current bar animations on the `TFM13-FEIMI-1` LCD.
- **Odometer (Bottom)**: Multi-digit `ODO` / `Trip` display.

---

## 2. Phase 1: Inbound Display Tests (Listen / Receive First)

| Feature / Signal | Command / Action | Result | Protocol Finding |
|---|---|---|---|
| **Gear Selector** | Short-press power button | **PASS** | Byte `[4]` reports gear state: `0x05` (Gear 1 / Eco), `0x0A` (Gear 2 / Mid), `0x0F` (Gear 3 / Sport). |
| **Throttle ADC Dynamic Range** | Squeeze throttle 0% $\to$ 100% | **PASS** | 16-bit big-endian value in `throttle_H` (`Byte[16]`) and `throttle_L` (`Byte[17]`). Range: `0` (idle neutral) to `1000` (`0x03E8`, full saturation). |
| **Gear Throttle Clamping** | Measure peak ADC per gear | **PASS** | Display firmware clamps peak throttle per gear:<br>• **Gear 1 (Eco)**: max **`360`** (`0x0168`, **36.0%**)<br>• **Gear 2 (Mid)**: max **`680`** (`0x02A8`, **68.0%**)<br>• **Gear 3 (Sport)**: max **`1000`** (`0x03E8`, **100.0%**). |
| **TX Frame Checksum** | Cumulative XOR verification | **PASS** | Byte `[19]` is a cumulative XOR checksum over the payload: `Byte[19] = 0x35 ^ Payload_Diff`. |
| **Headlight Switch** | Flip headlight switch | **PASS (Verified)** | `Byte[5]` (`functionBitmask`), Bit 5 (`0x20`) toggles high (`Light: 1`). |
| **Left Turn Signal** | Slide blinker switch Left | **PASS (Verified)** | `Byte[18]` (`indicator_L`), Bit 3 (`0x08`) toggles high (`Turn: L-`). |
| **Right Turn Signal** | Slide blinker switch Right | **PASS (Verified)** | `Byte[18]` (`indicator_L`), Bit 4 (`0x10`) toggles high (`Turn: -R`). |
| **Horn Button** | Press horn push-button | **PASS (Verified)** | `Byte[18]` (`indicator_L`), Bit 7 (`0x80`) toggles high (`Horn: 1`). |
| **Brake Lever Switch** | Pull brake lever | **PASS (Verified)** | `Byte[18]` (`indicator_L`), Bit 5 (`0x20`) toggles high (`Brk: 1`), display illuminates brake warning icon. |
| **Handlebar D/S Switch** | Press physical red D/S switch | **PASS (Verified)** | Physical switch on ESP32 GPIO 39 pulls to GND (active-low) with hardware pull-up. Contact bounce observed; software 50ms low-pass debounce filter required. |
| **P-Menu Parameters** | Read 20-byte baseline frame | **PASS** | Stock configuration decoded:<br>• P02 Battery: `0x01CC` (52V)<br>• P04 Poles: `0x001E` (30 magnetic poles)<br>• P03 Circumference: `0x035F`<br>• PA/PB (Accel/Regen): `0x44`<br>• Speed Profile: `0x0C64` (Open / Unlocked). |
| **P-Menu Live Modifications** | Modify P01–PB in P-menu | **PASS (Verified)** | • **P02 Voltage**: `Byte[14..15]` (LE) = $(10 \times V) - 60$. Tested 36V (`0x0136`), 52V (`0x01CC`), 72V (`0x028A`).<br>• **P03 Wheel Size**: `Byte[8..9]` (LE) = $658 + (2 \times \text{in tenths})$. Tested 9.5" (`0x0350` = 848) and 10.25" (`0x035F` = 863).<br>• **P04 Poles**: `Byte[6..7]` (LE) = Exact magnet count. Tested 30 (`0x001E`) and 100 (`0x0064`).<br>• **P05 / P06**: `Byte[5]` Bit 6 (`0x40`) sets on kick-start/cruise.<br>• **P07 Limiter**: `Byte[12..13]` (LE) = `0x0C00 | Percentage`. Tested 100% (`0x0C64`) and 99% (`0x0C63`).<br>• **PA / PB Accel/Regen**: `Byte[10]` = `(PB << 4) | PA`. Tested PA=1/PB=0 (`0x01`), PA=4/PB=4 (`0x44`), PA=5/PB=5 (`0x55`).<br>• **P01 Units / P08 Sleep**: Handled internally by display rendering without controller UART diff.<br>• **TX Checksum**: `Byte[19]` auto-updates cleanly across all modifications. |


---

## 3. Phase 2: Outbound Display Tests (Send / Control Second)

| Feature / Signal | Command Tested | Result | Protocol Finding |
|---|---|---|---|
| **Dual Motor Mode** | `dual 1` / `dual 0` | **PASS (Verified)** | `systemStatus |= 0x08` and `speedField[1] |= 0x08` successfully switches LCD top-left indicator between `SINGLE` and `DUAL`. |
| **Handlebar D/S Synchronization** | Physical red button on GPIO 39 | **PASS (Verified)** | Hardware-debounced (50ms) active-low edge detector on GPIO 39 toggles `SINGLE` $\leftrightarrow$ `DUAL` in real time on the LCD screen on every button click. |
| **Universal Speed Calibration** | `speed 10` .. `speed 285 mph`<br>`speed 10` .. `speed 459 km/h` | **PASS (Calibrated)** | **Exact Fixed-Point Dyadic Formulas**:<br>• **mph**: $\text{speedRaw} = \text{round}\left(\frac{285 \times \text{P03}}{16 \times \text{mph}}\right)$<br>• **km/h**: $\text{speedRaw} = \text{round}\left(\frac{459 \times \text{P03}}{16 \times \text{km/h}}\right)$<br>• P04 (Poles) is ignored by LCD; P03 directly sets numerator factor.<br>• Triple-digit display verified up to **`285 mph`** / **`459 km/h`**.<br>• Resolution quantization follows $\Delta \text{Speed} \approx \frac{\text{Speed}^2}{K}$. |
| **Response Frame Identifier** | `Byte[2]` = `0x01` | **PASS (Verified)** | Standard response frame identifier (Fixed `0x01`; `0x02` accepted with $+1$ checksum offset). |
| **Official Manual Error Codes** | Live Verification in Progress | **Ground-Truth Error Code Status**:<br>• **VERIFIED ON DISPLAY**: **`E-006`** (Receiver Error), **`E-007`** (Master Sending Error, `Byte[4]=0x10`), **`E-009`** (Master Over-Temp, `Byte[4]=0x01`), **`E-013`** (Secondary Controller Failure, `Byte[5]=0x10`), **`E-017`** (Sub-Controller Receiving Failure, `Byte[6]=0x10`).<br>• **UNVERIFIED / CURRENTLY INVESTIGATING**: **`E-001`** (Rear Motor Hall), **`E-003`** (Main Controller Error), **`E-005`** (Voltage Error), **`E-011`** (Front Motor Hall).<br>• **P02 Voltage Observation**: Setting P02 to 72V on 52V pack does not trigger E-005 (threshold likely well below 36V or register-triggered). |

---

## 4. Phase 3: Bluetooth BLE & Mobile App Reverse-Engineering (TODO)
- **Blinking Bluetooth Icon**: LCD displays a blinking BLE icon when unpaired.
- **Companion App**: Kukirin / Feimi Bluetooth companion mobile application discovered.
- **Test Tasks**:
  1. Capture BLE advertisement packets & GATT service UUIDs.
  2. Map read/write characteristics for P-menu settings, lock/unlock, and speed limiter.
  3. Reverse-engineer Android APK or fuzz BLE GATT table to identify scooter control packets.
