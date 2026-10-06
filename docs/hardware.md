# Hardware

The ESP32 sits on a custom carrier board at the bottom of the scooter, next to the two VESCs; a 5 V buck converter module powers it. The display is at the top of the stem, runs from battery voltage, and reaches the board through the scooter's Julet connector bundle. Part numbers, motor data and controller settings are in [system_spec.md](system_spec.md); pin numbers are defined in `HardwareConfig` in [src/main.cpp](../src/main.cpp).

## ESP32 Connections

| GPIO | Signal | Direction | Connects to |
|---|---|---|---|
| 35 | Display RX | In | Display TX, through the divider and filter below |
| 33 | Display TX | Out | Display RX |
| 34 | VESC RX | In | Rear VESC UART TX |
| 32 | VESC TX | Out | Rear VESC UART RX |
| 2 | Killswitch | Out | Rear VESC ADC2 input |
| 39 | Dual/single button | In | Handlebar button, pressed = low; 10 kOhm pull-up to 3.3 V on the board |
| 18 | Left turn signal | Out | 12 V high-side switch and factory light module |
| 19 | Right turn signal | Out | 12 V high-side switch and factory light module |
| 21 | Brake light | Out | 12 V high-side switch and factory light module |
| 22 | Head and tail light | Out | 12 V high-side switch and factory light module |
| 23 | Horn | Out | 12 V high-side switch |
| 4, 25, 26, 27, 14 | Spare (one plain output, four for addressable LED strips) | Out | Unused, held low |

All logic grounds share one ground plane on the board. Every output is set low before it is made an output, so nothing switches on briefly at boot.

Pins to avoid on this module: GPIO 16 and 17 are wired to the WROVER module's PSRAM. GPIO 34 to 39 are inputs only and have no internal pull resistors, so the button on GPIO 39 has its own 10 kOhm pull-up on the board. GPIO 2 is a boot strapping pin; it must be low or floating to enter the serial bootloader, which the firmware's low-at-boot killswitch output allows.

## Display Link

UART at 9600 baud. The display sends a command frame about 8 times a second and the ESP32 answers each one ([protocol](protocol/README.md)).

The display's TX is a 5 V signal and the ESP32's inputs take at most 3.6 V, so it passes through a resistor divider next to the ESP32: R1 = 1.0 kOhm in series, R2 = 1.8 kOhm to ground. A high level becomes 5 V x 1.8 / 2.8 = 3.21 V. The ESP32's input thresholds are 0.75 x 3.3 V = 2.48 V (high) and 0.25 x 3.3 V = 0.83 V (low) per its datasheet, so the margins are 0.74 V and 0.83 V at the pin, or about 1.1 V and 1.3 V at the display end before the divider. The divider looks like a 643 Ohm source (1.0 k parallel 1.8 k), and the filter capacitor from the pin to ground forms a low-pass filter with it:

| Capacitor | Time constant | -3 dB | Delay to the input threshold | Share of one 104 us bit |
|---|---|---|---|---|
| 680 pF (fitted, about 600-750 pF) | 0.44 us | 364 kHz | 0.6 us | 0.6 % |
| 4.7 nF (planned) | 3.0 us | 53 kHz | 4.4 us | 4.3 % |
| 10 nF (upper limit) | 6.4 us | 25 kHz | 9.4 us | 9.1 % |

Rising and falling edges are delayed by almost the same amount, so bit widths barely change. Much above 10 nF, slow edges spend long enough near the threshold for noise to cause extra transitions. The 1.0 kOhm resistor limits the current the display's output supplies into the capacitor to 5 mA whatever its value.

The ESP32's TX drives the display's RX directly at 3.3 V. This works in practice; the display's input threshold is not known, so the margin is not known either.

Open issue: during high-current launches, display frames are lost or corrupted, sometimes for longer than the 500 ms display timeout, which zeroes throttle until the link recovers and the throttle is released. The VESC link is not affected. The filter capacitor only removes short switching spikes; the 20 kHz switching frequency itself is below the filter's corner. Likely paths are coupling from the motor phase and battery wires into the long cable up the stem, and noise currents through the metal frame. Planned steps: the 4.7 nF capacitor, a ferrite on the display cable bundle, ferrites and twisting on the motor phase wires, routing the display cable away from them, and checking that logic ground does not also reach the frame. Which direction fails can be read from the ESP32's log lines (`E:DispRej` for corrupted frames, `E:DispStall` for none arriving) in the VESC Tool terminal, or from its `diag` console command.

## VESC Link

UART at 230400 baud between the ESP32 and the rear VESC, 3.3 V logic on both sides; both are at the bottom of the scooter. The ESP32 sends a control frame every 20 ms and the VESC answers with telemetry ([architecture](architecture.md)). The front VESC is reached over CAN from the rear VESC.

## Killswitch

GPIO 2 drives the rear VESC's ADC2 input. The script reads it every 20 ms and treats anything below 1.65 V as "motors off"; the VESC's own killswitch setting is also ADC2, low = killed ([system_spec.md](system_spec.md)). The ESP32 holds the line low from boot until brake-to-start, and keeps it high after that; lockouts after faults zero the throttle instead.

Known gap: the line has no pull-down resistor. While the ESP32 drives it, that does not matter. If the wire breaks or comes unplugged, the ADC2 input floats, and whether it then reads low depends on the input circuit inside the VESC, which has not been checked. The other layers still apply in that case: the ESP32 sends zero throttle until armed, and the script cuts the motors after 500 ms without control frames. Planned fix: a resistor from ADC2 to ground at the VESC end of the wire (10 to 47 kOhm; the ESP32 output drives 10 kOhm at 0.33 mA), then a check in VESC Tool that ADC2 reads close to 0 V with the wire unplugged and above 3 V when armed.

## Lights And Horn

The lights and horn run from a 52 V to 12 V converter. High-side MOSFET switches with gate drivers and protection diodes on the carrier board switch 12 V to the added higher-power lights and the horn. The same 12 V supply powers the scooter's factory light module, which runs the smaller stock lights (side lights, brake light, turn signals). Each lighting output goes to both: it drives the high-side switch for the added light and the factory light module's input for the same function, so one handlebar control works the stock and added lights together. The spare LED strip outputs are the only ones without a stock counterpart. Behaviour:

- Head and tail light: on while the display's light switch is on.
- Brake light: shows the tail light state normally and flashes (250 ms on, 250 ms off) while the brake lever is pulled.
- Turn signals: 400 ms on, 400 ms off. Pushing the same turn switch twice within 800 ms turns on the hazard lights.
- Horn: on while the horn button is held, cut after 3 s held continuously, and usable again after the button has been released for 100 ms.

Timing values are in `BridgeTimingConfig` in [KukirinG3ProProtocol.h](../lib/KukirinDisplay/src/KukirinG3ProProtocol.h).
