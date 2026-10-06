"""Check that values defined on both sides of the ESP32-VESC link agree.

The ESP32 firmware (C++) and the VESC script (LispBM) each define some of the
same numbers: CAN IDs, framing bytes, message types, wheel geometry, speed
thresholds and timeouts. Neither side can include the other's source, so this
script reads both and fails if they drift apart. It also checks a few values
that the C++ code stores twice in different units.

Usage: python tools/check_shared_constants.py [--root PATH]
Exit status is 0 when every check passes, 1 otherwise.
"""
import argparse
import os
import re
import sys

MPS_PER_MPH = 0.44704

ESP32_FILES = {
    "safety": "lib/VESCBridge/src/VESCSafety.h",
    "bridge": "lib/VESCBridge/src/VESCUARTBridge.h",
    "app": "src/main.cpp",
}
LISPBM_FILE = "vesc/main.lbm"


class Source:
    def __init__(self, root):
        self.root = root
        self.text = {}

    def read(self, rel):
        if rel not in self.text:
            with open(os.path.join(self.root, rel), encoding="utf-8") as f:
                self.text[rel] = f.read()
        return self.text[rel]

    def cpp(self, key, name):
        """Value of `NAME = number` (constexpr, const or enum member) in a C++ file."""
        rel = ESP32_FILES[key]
        m = re.search(rf"\b{re.escape(name)}\s*=\s*(0x[0-9A-Fa-f]+|-?[0-9.]+)f?\s*(?:[;,}}]|//|/\*|$)", self.read(rel), re.M)
        if not m:
            raise LookupError(f"{name} not found in {rel}")
        return parse_number(m.group(1))

    def lbm(self, name):
        """Value of `(def NAME number)` in the LispBM script."""
        m = re.search(rf"\(def\s+{re.escape(name)}\s+(-?[0-9.]+)\s*\)", self.read(LISPBM_FILE))
        if not m:
            raise LookupError(f"(def {name} ...) not found in {LISPBM_FILE}")
        return parse_number(m.group(1))

    def lbm_literal(self, pattern, label):
        """Number captured by `pattern` in the LispBM script, for values written inline."""
        m = re.search(pattern, self.read(LISPBM_FILE))
        if not m:
            raise LookupError(f"{label} not found in {LISPBM_FILE} (pattern {pattern!r})")
        return parse_number(m.group(1))

    def cpp_literal(self, key, pattern, label):
        rel = ESP32_FILES[key]
        m = re.search(pattern, self.read(rel))
        if not m:
            raise LookupError(f"{label} not found in {rel} (pattern {pattern!r})")
        return parse_number(m.group(1))


def parse_number(text):
    return int(text, 16) if text.lower().startswith("0x") else float(text)


def checks(s):
    """Each check: (description, function returning (esp32_value, lispbm_or_derived_value), tolerance)."""
    return [
        # Link identity and framing
        ("CAN ID, rear VESC", lambda: (s.cpp("bridge", "CAN_ID_REAR_VESC"), s.lbm("CAN-ID-REAR-VESC")), 0),
        ("CAN ID, front VESC", lambda: (s.cpp("bridge", "CAN_ID_FRONT_VESC"), s.lbm("CAN-ID-FRONT-VESC")), 0),
        ("VESC UART baud rate", lambda: (s.cpp("app", "VescUart"), s.lbm("VESC-UART-BAUD")), 0),
        ("Frame start byte 1", lambda: (s.cpp("bridge", "START_BYTE_1"), s.lbm("START-1")), 0),
        ("Frame start byte 2", lambda: (s.cpp("bridge", "START_BYTE_2"), s.lbm("START-2")), 0),
        ("Message type: heartbeat", lambda: (s.cpp("bridge", "MSG_TX_HEARTBEAT"), s.lbm("MSG-HEARTBEAT")), 0),
        ("Message type: control", lambda: (s.cpp("bridge", "MSG_TX_CONTROL"), s.lbm("MSG-CONTROL")), 0),
        ("Message type: audio", lambda: (s.cpp("bridge", "MSG_TX_AUDIO"), s.lbm("MSG-AUDIO")), 0),
        ("Message type: telemetry", lambda: (s.cpp("bridge", "MSG_RX_TELEMETRY"), s.lbm("MSG-TELEM")), 0),
        ("Message type: debug log", lambda: (s.cpp("bridge", "MSG_TX_DEBUG_LOG"), s.lbm("MSG-DEBUG-LOG")), 0),
        ("Debug log payload limit (bytes)",
         lambda: (s.cpp_literal("bridge", r"if \(len > ([0-9]+)\) len =", "debug log clamp"),
                  s.lbm_literal(r"\(<= msg-len ([0-9]+)\)", "payload length limit")), 0),

        # Vehicle geometry and thresholds
        ("Wheel diameter (in)", lambda: (s.cpp("safety", "WHEEL_DIAMETER_INCHES"), s.lbm("WHEEL-DIAMETER-INCHES")), 0),
        ("Motor pole pairs", lambda: (s.cpp("safety", "MOTOR_POLE_PAIRS"), s.lbm("MOTOR-POLE-PAIRS")), 0),
        ("Stationary speed (mph)", lambda: (s.cpp("safety", "SPEED_STATIONARY_MPH"), s.lbm("SPEED-STATIONARY-MPH")), 0),
        ("Stationary speed (ERPM)", lambda: (s.cpp("safety", "SPEED_STATIONARY_ERPM"), s.lbm("SPEED-STATIONARY-ERPM")), 0),
        ("Throttle full scale (counts)", lambda: (s.cpp("safety", "THROTTLE_SCALE_MAX_COUNTS"), s.lbm("THROTTLE-SCALE-MAX")), 0),
        ("Neutral throttle (counts)", lambda: (s.cpp("safety", "THROTTLE_NEUTRAL_COUNTS"), s.lbm("THROTTLE-NEUTRAL-COUNTS")), 0),
        ("Regen fade-in start (m/s)",
         lambda: (s.cpp("safety", "SPEED_REGEN_MIN_MPH") * MPS_PER_MPH, s.lbm("SPEED-REGEN-MIN-MPS")), 0.001),
        ("Regen full strength (m/s)",
         lambda: (s.cpp("safety", "SPEED_REGEN_MAX_MPH") * MPS_PER_MPH, s.lbm("SPEED-REGEN-MAX-MPS")), 0.001),
        ("Speed limit taper window (m/s)",
         lambda: (s.cpp("safety", "SPEED_LIMIT_TAPER_WINDOW_MPH") * MPS_PER_MPH, s.lbm("SPEED-LIMIT-TAPER-MPS")), 0.001),

        # Timing
        ("Control period (s)",
         lambda: (s.cpp("safety", "VESC_TICK_INTERVAL_SEC"),
                  s.lbm_literal(r"\(< elapsed ([0-9.]+)\)", "powertrain loop period")), 0),
        ("Dead-man timeout, ESP32 link (s)",
         lambda: (s.cpp("safety", "VESC_DEADMAN_TIMEOUT_MS") / 1000.0,
                  s.lbm_literal(r"\(>= delta-rx ([0-9.]+)\)", "dead-man timeout")), 0),
        ("Front VESC loss timeout equals dead-man timeout (s)",
         lambda: (s.cpp("safety", "VESC_DEADMAN_TIMEOUT_MS") / 1000.0,
                  s.lbm_literal(r"\(> front-age ([0-9.]+)\)", "front CAN timeout")), 0),

        # Values the C++ code stores twice in different units
        ("Mode 1 speed limit, mph vs cm/s",
         lambda: (round(s.cpp("safety", "MODE_1_SPEED_LIMIT_MPH") * MPS_PER_MPH * 100), s.cpp("safety", "MODE_1_SPEED_LIMIT_CMS")), 0),
        ("Mode 2 speed limit, mph vs cm/s",
         lambda: (round(s.cpp("safety", "MODE_2_SPEED_LIMIT_MPH") * MPS_PER_MPH * 100), s.cpp("safety", "MODE_2_SPEED_LIMIT_CMS")), 0),
        ("Speed limit taper, mph vs cm/s",
         lambda: (round(s.cpp("safety", "SPEED_LIMIT_TAPER_WINDOW_MPH") * MPS_PER_MPH * 100), s.cpp("safety", "SPEED_LIMIT_TAPER_CMS")), 0),
        ("Mode 1 ceiling, ratio vs counts",
         lambda: (s.cpp("safety", "MODE_1_SCALE_RATIO") * s.cpp("safety", "THROTTLE_SCALE_MAX_COUNTS"), s.cpp("safety", "MODE_1_MAX_COUNTS")), 0),
        ("Mode 2 ceiling, ratio vs counts",
         lambda: (s.cpp("safety", "MODE_2_SCALE_RATIO") * s.cpp("safety", "THROTTLE_SCALE_MAX_COUNTS"), s.cpp("safety", "MODE_2_MAX_COUNTS")), 0),
        ("Mode 3 ceiling, ratio vs counts",
         lambda: (s.cpp("safety", "MODE_3_SCALE_RATIO") * s.cpp("safety", "THROTTLE_SCALE_MAX_COUNTS"), s.cpp("safety", "MODE_3_MAX_COUNTS")), 0),
        ("Dead-man timeout, ms vs ticks",
         lambda: (s.cpp("safety", "VESC_DEADMAN_TIMEOUT_MS"),
                  s.cpp("safety", "DEADMAN_MISSED_TICKS") * s.cpp("safety", "VESC_TICK_INTERVAL_MS")), 0),
    ]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--root", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
    args = parser.parse_args()

    source = Source(args.root)
    failures = 0
    for description, get, tolerance in checks(source):
        try:
            a, b = get()
        except LookupError as e:
            print(f"FAIL  {description}: {e}")
            failures += 1
            continue
        ok = abs(a - b) <= tolerance
        failures += 0 if ok else 1
        print(f"{'ok  ' if ok else 'FAIL'}  {description}: {a:g} vs {b:g}")

    print(f"{failures} failure(s)" if failures else "all shared constants agree")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
