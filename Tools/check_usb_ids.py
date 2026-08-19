#!/usr/bin/env python3
"""Check board USB IDs against the Dronecode USB ID registry.

Boards declaring the Dronecode USB vendor ID (0x3643) in their NuttX
defconfigs must use a product ID registered to their manufacturer in
https://github.com/Dronecode/usb-ids. Boards using other vendor IDs are
ignored.

Usage:
  check_usb_ids.py check <defconfig> [<defconfig> ...]
  check_usb_ids.py lookup <0xNNNN | boards/<vendor>[/...]>

The registry is fetched from the usb-ids repo main branch by default;
use --registry <file> to check against a local copy.

Only dependency: PyYAML.
"""

import argparse
import re
import sys
import urllib.request

import yaml

REGISTRY_URL = "https://raw.githubusercontent.com/Dronecode/usb-ids/main/usb-ids.yaml"

CONFIG_RE = re.compile(
    r'^CONFIG_CDCACM_(VENDORID|PRODUCTID|VENDORSTR|PRODUCTSTR)=("?)(.*?)\2\s*$'
)


def load_registry(args):
    if args.registry:
        with open(args.registry, encoding="utf-8") as f:
            doc = yaml.safe_load(f)
    else:
        with urllib.request.urlopen(args.registry_url, timeout=30) as r:
            doc = yaml.safe_load(r.read())

    registry = {
        "vid": int(doc["vid"], 16),
        "vendor_string": doc["vendor_string"],
        "pids": {},  # pid int -> {"manufacturer", "board", "px4_vendor"}
        "vendors": {},  # px4_vendor slug -> manufacturer name
    }
    for mfr in doc["manufacturers"]:
        slug = mfr.get("px4_vendor")
        if slug:
            registry["vendors"][slug] = mfr["name"]
        for entry in mfr["pids"]:
            registry["pids"][int(entry["pid"], 16)] = {
                "manufacturer": mfr["name"],
                "board": entry["board"],
                "px4_vendor": slug,
            }
    return registry


def parse_defconfig(path):
    values = {}
    with open(path, encoding="utf-8") as f:
        for line in f:
            m = CONFIG_RE.match(line)
            if m:
                values[m.group(1)] = m.group(3)
    return values


def board_vendor(path):
    """Vendor directory name from a path like boards/<vendor>/<board>/..."""
    parts = path.replace("\\", "/").split("/")
    try:
        return parts[parts.index("boards") + 1]
    except (ValueError, IndexError):
        return None


def cmd_check(registry, paths):
    violations = []
    checked = 0
    for path in paths:
        values = parse_defconfig(path)
        vid = values.get("VENDORID")
        if vid is None or int(vid, 16) != registry["vid"]:
            continue
        checked += 1

        pid_raw = values.get("PRODUCTID")
        if pid_raw is None:
            violations.append(f"{path}: VID 0x3643 set but no CONFIG_CDCACM_PRODUCTID")
            continue
        pid = int(pid_raw, 16)

        entry = registry["pids"].get(pid)
        if entry is None:
            violations.append(
                f"{path}: PID {pid_raw} is not registered in the Dronecode USB ID "
                "registry (https://github.com/Dronecode/usb-ids)"
            )
            continue

        vendor = board_vendor(path)
        if entry["px4_vendor"] is None:
            violations.append(
                f"{path}: PID {pid_raw} is registered to '{entry['manufacturer']}' "
                "but the registry has no px4_vendor mapping for them; add it to "
                "usb-ids.yaml first"
            )
        elif vendor != entry["px4_vendor"]:
            violations.append(
                f"{path}: PID {pid_raw} is registered to '{entry['manufacturer']}' "
                f"(boards/{entry['px4_vendor']}/), not to boards/{vendor}/"
            )

        vendorstr = values.get("VENDORSTR")
        allowed = {registry["vendor_string"], entry["manufacturer"]}
        if vendorstr not in allowed:
            violations.append(
                f"{path}: CONFIG_CDCACM_VENDORSTR \"{vendorstr}\" must be one of: "
                + ", ".join(f'"{s}"' for s in sorted(allowed))
            )

    for v in violations:
        print(f"error: {v}", file=sys.stderr)
    print(
        f"checked {checked} defconfig(s) with VID 0x3643, "
        f"{len(violations)} violation(s)"
    )
    return 1 if violations else 0


def cmd_lookup(registry, query):
    if query.lower().startswith("0x"):
        entry = registry["pids"].get(int(query, 16))
        if entry is None:
            print(f"{query}: not registered")
            return 1
        print(f"{query}: {entry['manufacturer']}, board: {entry['board']}")
        return 0

    vendor = board_vendor(query) or query
    name = registry["vendors"].get(vendor)
    if name is None:
        print(f"boards/{vendor}/: no manufacturer registered for this vendor directory")
        return 1
    pids = sorted(p for p, e in registry["pids"].items() if e["manufacturer"] == name)
    pid_list = ", ".join(f"0x{p:04X}" for p in pids)
    print(f"boards/{vendor}/: {name}, PIDs: {pid_list}")
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--registry", help="path to a local usb-ids.yaml")
    parser.add_argument("--registry-url", default=REGISTRY_URL)
    sub = parser.add_subparsers(dest="command", required=True)
    p_check = sub.add_parser("check", help="check defconfig files")
    p_check.add_argument("paths", nargs="+")
    p_lookup = sub.add_parser("lookup", help="look up a PID or board path")
    p_lookup.add_argument("query")
    args = parser.parse_args()

    registry = load_registry(args)
    if args.command == "check":
        return cmd_check(registry, args.paths)
    return cmd_lookup(registry, args.query)


if __name__ == "__main__":
    sys.exit(main())
