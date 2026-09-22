#!/usr/bin/env python3
"""Compare flash image and static RAM usage in bare-metal firmware ELFs."""

import argparse
import json
import os
from pathlib import Path
import subprocess


def memory_usage(elf: Path, flash_origin: int, flash_size: int) -> dict[str, int]:
    headers = subprocess.check_output(
        ["arm-none-eabi-readelf", "--program-headers", "--wide", str(elf)],
        text=True,
        env={**os.environ, "LC_ALL": "C"},
    )
    flash_end = flash_origin + flash_size
    image_end = flash_origin
    ram = 0

    for line in headers.splitlines():
        fields = line.split()
        if not fields or fields[0] != "LOAD":
            continue

        vma, lma, file_size, memory_size = (int(value, 16) for value in fields[2:6])

        if file_size:
            if not flash_origin <= lma < lma + file_size <= flash_end:
                raise ValueError(f"{elf}: load image outside the configured flash region")
            image_end = max(image_end, lma + file_size)

        # Initialized data and RAM functions have a flash LMA and a RAM VMA.
        # NOLOAD segments (BSS, retained buffers) reserve RAM without file bytes.
        if not flash_origin <= vma < flash_end:
            ram += memory_size

    if image_end == flash_origin:
        raise ValueError(f"{elf}: no flash load image found")

    # Include alignment gaps in the programmed image, as objcopy -O binary does.
    return {"flash": image_end - flash_origin, "ram": ram}


def format_change(before: int, after: int) -> str:
    delta = after - before
    percentage = f"{delta / before:+.2%}" if before else "n/a"
    return f"{delta:+,} B ({percentage})"


def summarize(before: dict[str, int], after: dict[str, int]) -> dict:
    return {
        "flash": format_change(before["flash"], after["flash"]),
        "ram": format_change(before["ram"], after["ram"]),
        "changed": before != after,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--before", type=Path, required=True)
    parser.add_argument("--after", type=Path, required=True)
    parser.add_argument("--flash-origin", type=lambda value: int(value, 0), required=True)
    parser.add_argument("--flash-size", type=lambda value: int(value, 0), required=True)
    args = parser.parse_args()
    before = memory_usage(args.before, args.flash_origin, args.flash_size)
    after = memory_usage(args.after, args.flash_origin, args.flash_size)
    print(json.dumps(summarize(before, after)))


if __name__ == "__main__":
    main()
