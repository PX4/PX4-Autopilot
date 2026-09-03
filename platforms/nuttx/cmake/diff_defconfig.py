#!/usr/bin/env python3
"""Write a NuttX defconfig fragment: assignments in `new` that differ from `base`."""

import sys


def parse(path):
    keys = {}
    with open(path) as f:
        for line in f:
            s = line.strip()
            if s.startswith("CONFIG_"):
                k = s.split("=", 1)[0]
            elif s.startswith("# CONFIG_") and s.endswith(" is not set"):
                k = s[2:].split()[0]
            else:
                continue
            keys[k] = s
    return keys


def fragment_lines(base, new):
    lines = []
    for k, line in new.items():
        if base.get(k) != line:
            lines.append(line)
    for k, line in base.items():
        if k not in new and line.startswith("CONFIG_"):
            lines.append(f"# {k} is not set")
    return lines


def main():
    if len(sys.argv) != 4:
        sys.stderr.write("usage: diff_defconfig.py BASE NEW OUT\n")
        return 2
    lines = fragment_lines(parse(sys.argv[1]), parse(sys.argv[2]))
    with open(sys.argv[3], "w") as f:
        if lines:
            f.write("\n".join(lines) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
