"""Compare the loaded sections of two ESP32 firmware ELF files.

Used to show that a change does not alter the compiled firmware, for
example that a release built from this repository matches the firmware that
was ridden. Every section that is loaded onto the chip (code, read-only data,
initialized data) is compared byte for byte.

With --ignore-build-time, the __DATE__ ("Oct  6 2026") and __TIME__
("00:40:06") strings in read-only data are masked before comparing, since
they change on every build. Build both images with the same FW_VERSION
(set KUKIRIN_FW_VERSION) so that string matches too.

Usage:
  python tools/compare_firmware.py [--ignore-build-time] A.elf B.elf
Exit status is 0 when the images match, 1 otherwise.
"""
import argparse
import os
import re
import subprocess
import sys
import tempfile

TOOLCHAIN_BIN = os.path.join(os.path.expanduser("~"), ".platformio", "packages",
                             "toolchain-xtensa-esp32", "bin")
OBJDUMP = os.path.join(TOOLCHAIN_BIN, "xtensa-esp32-elf-objdump")
OBJCOPY = os.path.join(TOOLCHAIN_BIN, "xtensa-esp32-elf-objcopy")

BUILD_TIME_PATTERNS = [
    re.compile(rb"(?<=\x00)\d\d:\d\d:\d\d(?=\x00)"),
    re.compile(rb"(?<=\x00)[A-Z][a-z]{2} [ \d]\d \d{4}(?=\x00)"),
]


def loaded_sections(elf):
    """Return {name: (size, vma)} for sections with ALLOC and CONTENTS flags."""
    out = subprocess.run([OBJDUMP, "-h", elf], capture_output=True, text=True, check=True).stdout
    lines = out.splitlines()
    sections = {}
    for i, line in enumerate(lines[:-1]):
        fields = line.split()
        if len(fields) >= 7 and fields[0].isdigit():
            flags = lines[i + 1]
            if "ALLOC" in flags and "CONTENTS" in flags:
                sections[fields[1]] = (int(fields[2], 16), int(fields[3], 16))
    return sections


def section_bytes(elf, name):
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "section.bin")
        subprocess.run([OBJCOPY, "-O", "binary", f"--only-section={name}", elf, path], check=True)
        with open(path, "rb") as f:
            return f.read()


def mask_build_time(data):
    for pattern in BUILD_TIME_PATTERNS:
        data = pattern.sub(lambda m: b"#" * len(m.group(0)), data)
    return data


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--ignore-build-time", action="store_true")
    parser.add_argument("a")
    parser.add_argument("b")
    args = parser.parse_args()

    sections_a, sections_b = loaded_sections(args.a), loaded_sections(args.b)
    match = True
    for name in sorted(set(sections_a) | set(sections_b)):
        if name not in sections_a or name not in sections_b:
            print(f"{name}: only in {'A' if name in sections_a else 'B'}")
            match = False
            continue
        if sections_a[name] != sections_b[name]:
            (size_a, vma_a), (size_b, vma_b) = sections_a[name], sections_b[name]
            print(f"{name}: layout differs (A {size_a} bytes at {vma_a:#x}, B {size_b} bytes at {vma_b:#x})")
            match = False
            continue
        data_a, data_b = section_bytes(args.a, name), section_bytes(args.b, name)
        if args.ignore_build_time:
            data_a, data_b = mask_build_time(data_a), mask_build_time(data_b)
        diffs = [i for i, (x, y) in enumerate(zip(data_a, data_b)) if x != y]
        size, vma = sections_a[name]
        if diffs:
            print(f"{name}: {len(diffs)} of {size} bytes differ, first at {vma + diffs[0]:#x}")
            match = False
        else:
            print(f"{name}: identical ({size} bytes)")

    print("MATCH" if match else "DIFFERENT")
    return 0 if match else 1


if __name__ == "__main__":
    sys.exit(main())
