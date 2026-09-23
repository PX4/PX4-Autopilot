#!/usr/bin/env python3
"""Compare flash image and static RAM usage in bare-metal firmware ELFs."""

import argparse
import json
import os
from pathlib import Path
import subprocess

# The commit hash compiled into px4_firmware_version_binary() changes its code
# size, so identical sources on two commits can differ by up to 16 B.
MIN_REPORTED_DELTA = 30


def memory_usage(elf: Path) -> dict[str, int]:
    # Sections rather than program headers: ld may map the ELF header into the
    # first LOAD segment, below the flash origin, depending on its page size.
    headers = subprocess.check_output(
        ["arm-none-eabi-objdump", "--section-headers", "--wide", str(elf)],
        text=True,
        env={**os.environ, "LC_ALL": "C"},
    )
    image_start = None
    image_end = 0
    ram = 0

    for line in headers.splitlines():
        fields = line.split(maxsplit=7)
        if len(fields) < 8 or not fields[0].isdigit():
            continue

        size, vma, lma = (int(value, 16) for value in fields[2:5])
        flags = {flag.strip() for flag in fields[7].split(",")}
        if "ALLOC" not in flags or not size:
            continue

        in_image = {"LOAD", "CONTENTS"} <= flags
        if in_image:
            image_start = lma if image_start is None else min(image_start, lma)
            image_end = max(image_end, lma + size)

        # Only code and constants execute in place. Initialized data and RAM
        # functions are copied out of the image; NOLOAD sections reserve RAM.
        if not (in_image and vma == lma):
            ram += size

    if image_start is None:
        raise ValueError(f"{elf}: no flash load image found")

    # Include alignment gaps in the programmed image, as objcopy -O binary does.
    return {"flash": image_end - image_start, "ram": ram}


def indicator(delta: int) -> str:
    if delta > 1000:
        return "🔴 "
    if delta > 100:
        return "🟡 "
    if delta < -100:
        return "🟢 "
    return ""


def format_change(before: int, after: int) -> str:
    delta = after - before
    percentage = f"{delta / before:+.2%}" if before else "n/a"
    return f"{indicator(delta)}{delta:+,} B ({percentage})"


def summarize(before: dict[str, int], after: dict[str, int]) -> dict:
    return {
        "flash": format_change(before["flash"], after["flash"]),
        "ram": format_change(before["ram"], after["ram"]),
        "changed": any(abs(after[key] - before[key]) >= MIN_REPORTED_DELTA
                       for key in ("flash", "ram")),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--before", type=Path, required=True)
    parser.add_argument("--after", type=Path, required=True)
    args = parser.parse_args()
    before = memory_usage(args.before)
    after = memory_usage(args.after)
    print(json.dumps(summarize(before, after)))


if __name__ == "__main__":
    main()
