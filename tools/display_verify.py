"""Re-check display protocol facts with the display-verify firmware, reading the LCD by eye.

Flash the verification firmware first:  pio run -e display-verify -t upload
Run with PlatformIO's Python, which has pyserial:
  %USERPROFILE%\\.platformio\\penv\\Scripts\\python.exe tools/display_verify.py --port COM6 <test>

Tests:
  timing            measure the display's frame spacing for a while (no reading needed)
  survey            guided checklist: you operate a control or change a P-menu setting, the tool records
                    which command frame bytes change (throttle maximum per mode, switches, P02, P03, ...)
  speed             step speedRaw through the values where the display should change, you type what it shows
  errors            set one status bit at a time (then a few pairs), you type what the display shows
  average           alternate two speeds and describe what the display does
  plan              print the speed test points without a connection (dry run)

Expected values are never shown while you read, so they cannot bias the reading. Results are written as CSV
(one row per step, with what was sent, what you read and, for speed, what the model predicts) into --out,
by default docs/protocol/captures/.
"""
import argparse
import csv
import datetime
import os
import random
import sys
import time
from collections import Counter

FITTED_K = 287.275    # constant used by the firmware (KukirinG3ProProtocol.h)
PHYSICS_K = 287.267   # pi * 0.0254 * 3600: speedRaw as wheel revolution time in ms
STOPPED = 3500


# ---------------------------------------------------------------------------
# Speed model (same arithmetic as KukirinG3ProProtocol.h, with the constant as a parameter)

def numerator(k, p03):
    return int(k * p03 + 1e-9)


def shown(r, p03, units, k=FITTED_K):
    if r <= 0 or r >= STOPPED:
        return 0
    tenths = numerator(k, p03) // r
    return tenths // 10 if units == "kmh" else (tenths * 6214) // 100000


def speed_points(p03, units, max_value, seed):
    """speedRaw values to test: both sides of every change of the shown value up to max_value, every value
    where the fitted and the physical constant disagree, and the edges of the stopped range."""
    points = set()
    last_r_for = {}
    for r in range(1, STOPPED):
        last_r_for[shown(r, p03, units)] = r   # r rises, so this keeps the last r showing each value
    for v in range(1, max_value + 1):
        r = last_r_for.get(v)
        if r is not None:
            points.add(r)
            points.add(r + 1)
    for r in range(1, STOPPED):
        if shown(r, p03, units, FITTED_K) != shown(r, p03, units, PHYSICS_K):
            points.update((r, r + 1))
    points.update((STOPPED - 1, STOPPED))
    ordered = sorted(p for p in points if 1 <= p <= STOPPED)
    random.Random(seed).shuffle(ordered)
    return ordered


# ---------------------------------------------------------------------------
# Link to the verification firmware

class Bench:
    def __init__(self, port):
        try:
            import serial
        except ImportError:
            sys.exit("pyserial missing: run this with PlatformIO's Python (see the top of this file)")
        self.ser = serial.Serial(port, 115200, timeout=0.2)
        time.sleep(2.0)  # the board may reset when the port opens
        self.ser.reset_input_buffer()
        self.command("ping")

    def lines(self, seconds):
        end = time.time() + seconds
        out = []
        while time.time() < end:
            raw = self.ser.readline()
            if raw:
                out.append(raw.decode("ascii", "replace").strip())
        return out

    def command(self, text, expect="OK"):
        self.ser.write((text + "\n").encode("ascii"))
        end = time.time() + 2.0
        while time.time() < end:
            line = self.ser.readline().decode("ascii", "replace").strip()
            if line.startswith(expect) or line.startswith("ERR"):
                if line.startswith("ERR"):
                    sys.exit(f"firmware refused '{text}': {line}")
                return line
        sys.exit(f"no answer to '{text}': is the display-verify firmware running and the display on?")

    def frame(self):
        return bytes.fromhex(self.command("frame", expect="FRAME").split()[1])

    def frames_for(self, seconds):
        self.command("log 1")
        lines = self.lines(seconds)
        self.command("log 0")
        return [bytes.fromhex(l.split(",")[2]) for l in lines if l.startswith("F,")]


# ---------------------------------------------------------------------------
# Output

def out_path(args, name):
    os.makedirs(args.out, exist_ok=True)
    return os.path.join(args.out, f"{datetime.date.today().isoformat()}-{name}.csv")


def write_csv(path, header_notes, fieldnames, rows):
    with open(path, "w", newline="", encoding="utf-8") as f:
        for note in header_notes:
            f.write(f"# {note}\n")
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(rows)
    print(f"saved {path}")


def ask(prompt):
    try:
        return input(prompt).strip()
    except EOFError:
        return "q"


# ---------------------------------------------------------------------------
# Tests

def test_timing(bench, args):
    bench.command("stat reset")
    print(f"measuring frame spacing for {args.seconds} s ...")
    time.sleep(args.seconds)
    stat = bench.command("stat", expect="STAT")
    print(stat)
    fields = dict(kv.split("=") for kv in stat.split()[1:])
    fields["seconds"] = str(args.seconds)
    write_csv(out_path(args, "display-frame-timing"),
              ["Display command frame spacing, measured by display-verify when each frame's last byte is read "
               "(1 ms loop polling)."],
              list(fields), [fields])


SURVEY = [
    ("baseline", "Mode 1, throttle released, brake released, lights off, switches centred."),
    ("throttle-max-mode1", "Mode 1: hold FULL throttle for the next 3 s."),
    ("throttle-max-mode2", "Mode 2: hold FULL throttle for the next 3 s."),
    ("throttle-max-mode3", "Mode 3: hold FULL throttle for the next 3 s."),
    ("brake", "Mode 1, release throttle. Hold the brake lever."),
    ("lights", "Release brake. Turn the lights on."),
    ("left", "Lights off. Turn signal left."),
    ("right", "Turn signal right."),
    ("horn", "Centre the turn signal. Hold the horn."),
    ("p01-other-units", "Release horn. Change P01 to the other unit (km/h or mph)."),
    ("p02-36", "Set P02 to 36 V."), ("p02-48", "Set P02 to 48 V."), ("p02-52", "Set P02 to 52 V."),
    ("p02-60", "Set P02 to 60 V."), ("p02-72", "Set P02 to 72 V."),
    ("p03-80", "Set P02 back to your normal value. Set P03 to 80."), ("p03-100", "Set P03 to 100."),
    ("p03-160", "Set P03 to 160."),
    ("p04-1", "Set P03 back to your normal value. Set P04 to 1."), ("p04-100", "Set P04 to 100."),
    ("p05-toggle", "Set P04 back. Toggle P05 (cruise)."),
    ("p06-toggle", "Set P05 back. Toggle P06 (kick-start)."),
    ("p07-0", "Set P06 back. Set P07 to 0 %."), ("p07-50", "Set P07 to 50 %."), ("p07-100", "Set P07 to 100 %."),
    ("p08-change", "Change P08 (sleep timer)."),
    ("pa-1", "Set P08 back. Set PA to 1."), ("pa-5", "Set PA to 5."),
    ("pb-0", "Set PA back. Set PB to 0."), ("pb-5", "Set PB to 5."),
    ("end", "Set PB back to your normal value."),
]


def test_survey(bench, args):
    rows = []
    baseline = None
    for label, instruction in SURVEY:
        if ask(f"\n[{label}] {instruction}\nPress Enter when done (s = skip, q = stop): ") in ("q", "Q"):
            break
        frames = bench.frames_for(3.0 if label.startswith("throttle") else 1.5)
        if not frames:
            print("  no frames received")
            continue
        common = Counter(frames).most_common(1)[0][0]
        throttle_max = max((f[16] << 8) | f[17] for f in frames)
        if baseline is None:
            baseline = common
        changed = [f"{i}:{baseline[i]:02X}>{common[i]:02X}" for i in range(19) if common[i] != baseline[i]]
        print(f"  frame {common.hex(' ').upper()}  throttle max {throttle_max}  changed {' '.join(changed) or 'none'}")
        rows.append({"step": label, "instruction": instruction, "frame": common.hex(" ").upper(),
                     "frames_seen": len(frames), "distinct_frames": len(set(frames)),
                     "throttle_max": throttle_max, "changed_vs_baseline": " ".join(changed)})
    write_csv(out_path(args, "command-frame-survey"),
              ["Command frame survey: most common frame in each step, throttle maximum seen, bytes that differ "
               "from the baseline frame (byte:before>after)."],
              ["step", "instruction", "frame", "frames_seen", "distinct_frames", "throttle_max", "changed_vs_baseline"],
              rows)


def test_speed(bench, args):
    frame = bench.frame()
    p03 = frame[8]
    if args.p03 and args.p03 != p03:
        sys.exit(f"display reports P03 = {p03}, not {args.p03}; change the P-menu or the option")
    points = speed_points(p03, args.units, args.max_value, args.seed)
    print(f"P03 = {p03} (from the display), units {args.units}, {len(points)} steps, order seed {args.seed}.")
    print("For each step, type the number on the LCD. r = resend, q = stop and save.")
    rows = []
    for i, r in enumerate(points, 1):
        bench.command(f"speedraw {r}")
        time.sleep(0.6)  # several display frames
        while True:
            reading = ask(f"step {i}/{len(points)}: LCD shows? ")
            if reading.lower() == "r":
                bench.command(f"speedraw {r}")
                continue
            if reading == "":
                continue
            break
        if reading.lower() == "q":
            break
        rows.append({"step": i, "speedraw": r, "p03": p03, "units": args.units, "reading": reading,
                     "model_fitted": shown(r, p03, args.units, FITTED_K),
                     "model_physics": shown(r, p03, args.units, PHYSICS_K)})
    bench.command(f"speedraw {STOPPED}")
    bad = [row for row in rows if row["reading"] != str(row["model_fitted"])]
    phys_bad = [row for row in rows if row["reading"] != str(row["model_physics"])]
    print(f"{len(rows)} readings: {len(bad)} differ from the fitted model, {len(phys_bad)} from the physics constant")
    for row in bad[:20]:
        print(f"  speedRaw {row['speedraw']}: read {row['reading']}, fitted model {row['model_fitted']}")
    write_csv(out_path(args, f"speed-p03-{p03}-{args.units}"),
              [f"Speed readings, P03 = {p03}, units {args.units}, order seed {args.seed}. model_fitted uses "
               f"{FITTED_K}, model_physics uses {PHYSICS_K}. Readings typed without seeing the prediction."],
              ["step", "speedraw", "p03", "units", "reading", "model_fitted", "model_physics"], rows)


def error_steps(include_latch):
    """(label, dual, overrides) for each step. Byte 4 keeps its 0xC0 base."""
    steps = []
    for byte, base in ((3, 0x00), (4, 0xC0), (5, 0x00), (6, 0x00)):
        for bit in range(8):
            if byte == 4 and bit >= 6:
                continue  # part of the 0xC0 base
            if byte == 5 and bit == 5:
                continue  # E-031 latch, run last if asked
            for dual in ((1, 0) if byte in (5, 6) else (1,)):
                value = base | (1 << bit) | (0x08 if (byte in (4, 6) and dual) else 0)  # keep the dual bits
                steps.append((f"byte{byte}-bit{bit}{'-single' if not dual else ''}", dual, {byte: value}))
    steps += [
        ("pair-E001-E013", 1, {3: 0x40, 5: 0x10}),
        ("pair-E002-E003", 1, {3: 0x30}),
        ("pair-E007-E009", 1, {4: 0xC0 | 0x08 | 0x10 | 0x01}),
        ("E003-while-moving", 1, {3: 0x10, "speedraw": 200}),
    ]
    if include_latch:
        steps.append(("byte5-bit5-E031-latch", 1, {5: 0x20}))
    return steps


def test_errors(bench, args):
    steps = error_steps(args.include_latch)
    print(f"{len(steps)} steps. For each, type what the display shows, e.g. 'none', 'E2', 'E2/E3 alternating', "
          "'E0'. r = resend, q = stop and save.")
    rows = []
    for i, (label, dual, overrides) in enumerate(steps, 1):
        if "latch" in label and ask("Next step may latch E-00 until a power cycle. Continue? (y/n) ") != "y":
            break
        bench.command("clear")
        bench.command(f"dual {dual}")
        for key, value in overrides.items():
            bench.command(f"speedraw {value}" if key == "speedraw" else f"set {key} {value}")
        time.sleep(1.0)
        reading = ask(f"step {i}/{len(steps)} ({label}): display shows? ")
        while reading.lower() == "r":
            time.sleep(1.0)
            reading = ask(f"step {i}/{len(steps)} ({label}): display shows? ")
        if reading.lower() == "q":
            break
        rows.append({"step": i, "label": label, "dual": dual,
                     "sent": " ".join(f"{k}={v}" for k, v in overrides.items()), "reading": reading})
    bench.command("clear")
    if ask("Last check: the checksum will be inverted. Press Enter, then press Enter again when E-006 "
           "appears (q to skip): ") != "q":
        bench.command("badsum 1")
        start = time.time()
        ask("")
        rows.append({"step": len(rows) + 1, "label": "bad-checksum", "dual": 1, "sent": "badsum=1",
                     "reading": f"E-006 after about {time.time() - start:.1f} s (manual timing)"})
        bench.command("clear")
    write_csv(out_path(args, "error-bits"),
              ["Error bit readings: status frame built normally, then the listed bytes forced to the listed "
               "values (byte=value). Readings typed in by hand."],
              ["step", "label", "dual", "sent", "reading"], rows)


def test_average(bench, args):
    rows = []
    for a, b in ((28, 30), (18, 36)):
        for n in (1, 2, 3, 10):
            bench.command(f"alt {a} {b} {n}")
            time.sleep(1.0)
            p03 = bench.frame()[8]
            print(f"\nAlternating speedRaw {a} and {b} every {n} frame(s). With P03 = {p03} these would show "
                  f"{shown(a, p03, args.units)} and {shown(b, p03, args.units)} {args.units}.")
            reading = ask("Describe what the LCD shows (one value / jumps between two / values in between): ")
            if reading.lower() == "q":
                break
            rows.append({"a": a, "b": b, "every_frames": n, "p03": p03, "reading": reading})
    bench.command("clear")
    write_csv(out_path(args, "speed-averaging"),
              ["Two speedRaw values alternated every N display frames; description of the LCD, typed in by hand."],
              ["a", "b", "every_frames", "p03", "reading"], rows)


def main():
    p = argparse.ArgumentParser(description="Re-check display protocol facts (see the top of this file).")
    p.add_argument("test", choices=["timing", "survey", "speed", "errors", "average", "plan"])
    p.add_argument("--port", help="serial port of the ESP32, e.g. COM6")
    p.add_argument("--out", default=os.path.join("docs", "protocol", "captures"))
    p.add_argument("--seconds", type=int, default=60, help="timing: how long to measure")
    p.add_argument("--units", choices=["mph", "kmh"], default="mph", help="speed: the display's P01 setting")
    p.add_argument("--p03", type=int, help="speed/plan: wheel setting (speed reads it from the display)")
    p.add_argument("--max-value", type=int, default=40, help="speed: highest shown value to test")
    p.add_argument("--seed", type=int, default=1, help="speed: order of the steps")
    p.add_argument("--include-latch", action="store_true", help="errors: also set byte 5 bit 5 (E-031/E-00)")
    args = p.parse_args()

    if args.test == "plan":
        p03 = args.p03 or 100
        pts = speed_points(p03, args.units, args.max_value, args.seed)
        print(f"P03 {p03}, {args.units}: {len(pts)} steps")
        print(" ".join(str(r) for r in sorted(pts)))
        return 0
    if not args.port:
        p.error("--port is required for this test")
    bench = Bench(args.port)
    {"timing": test_timing, "survey": test_survey, "speed": test_speed,
     "errors": test_errors, "average": test_average}[args.test](bench, args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
