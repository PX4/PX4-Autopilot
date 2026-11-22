#!/usr/bin/env python3
"""Check board USB IDs against the Dronecode USB ID registry.

Every NuttX defconfig under boards/<vendor>/<board>/ that declares the
Dronecode USB vendor ID (0x3643) must use a product ID registered in
https://github.com/Dronecode/usb-ids whose px4_board is exactly
<vendor>/<board>. Boards using other vendor IDs are ignored. The USB
vendor string is not checked; the registry does not govern it.

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
from typing import Dict, List, Optional, TypedDict

import yaml

REGISTRY_URL = (
    "https://raw.githubusercontent.com/Dronecode/usb-ids/main/usb-ids.yaml"
)

CONFIG_RE = re.compile(
    r'^CONFIG_CDCACM_(VENDORID|PRODUCTID)=("?)(.*?)\2\s*$'
)


class PidEntry(TypedDict):
    manufacturer: str
    board: str
    px4_board: Optional[str]


class Registry(TypedDict):
    vid: int
    pids: Dict[int, PidEntry]  # pid -> registry entry
    vendors: Dict[str, str]  # px4_vendor slug -> manufacturer name


def load_registry(path: Optional[str] = None,
                  url: str = REGISTRY_URL) -> Registry:
    if path:
        with open(path, encoding="utf-8") as f:
            doc = yaml.safe_load(f)
    else:
        with urllib.request.urlopen(url, timeout=30) as r:
            doc = yaml.safe_load(r.read())

    registry: Registry = {
        "vid": int(doc["vid"], 16),
        "pids": {},
        "vendors": {},
    }
    for mfr in doc["manufacturers"]:
        slug = mfr.get("px4_vendor")
        if slug:
            registry["vendors"][slug] = mfr["name"]
        for entry in mfr["pids"]:
            registry["pids"][int(entry["pid"], 16)] = {
                "manufacturer": mfr["name"],
                "board": entry["board"],
                "px4_board": entry.get("px4_board"),
            }
    return registry


def parse_defconfig(path: str) -> Dict[str, str]:
    values: Dict[str, str] = {}
    with open(path, encoding="utf-8") as f:
        for line in f:
            m = CONFIG_RE.match(line)
            if m:
                values[m.group(1)] = m.group(3)
    return values


def parse_hex(value: str) -> Optional[int]:
    try:
        return int(value, 16)
    except ValueError:
        return None


def board_path(path: str) -> Optional[str]:
    """'<vendor>/<board>' from a path like boards/<vendor>/<board>/..."""
    parts = path.replace("\\", "/").split("/")
    try:
        i = parts.index("boards")
    except ValueError:
        return None
    return "/".join(parts[i + 1:i + 3]) or None


def check_defconfig(registry: Registry, path: str,
                    values: Dict[str, str]) -> Optional[str]:
    """Return the violation for a defconfig using the Dronecode VID.

    None means the defconfig passes.
    """
    pid_raw = values.get("PRODUCTID")
    if pid_raw is None:
        return "VID 0x3643 set but no CONFIG_CDCACM_PRODUCTID"
    pid = parse_hex(pid_raw)
    if pid is None:
        return f"CONFIG_CDCACM_PRODUCTID \"{pid_raw}\" is not a hex value"

    entry = registry["pids"].get(pid)
    if entry is None:
        return (
            f"PID {pid_raw} is not registered in the Dronecode USB ID "
            "registry (https://github.com/Dronecode/usb-ids)"
        )

    board = board_path(path)
    if entry["px4_board"] is None:
        return (
            f"PID {pid_raw} ({entry['manufacturer']}, "
            f"\"{entry['board']}\") has no px4_board in the registry; "
            f"set it to \"{board}\" in usb-ids.yaml"
        )
    if entry["px4_board"] != board:
        return (
            f"PID {pid_raw} is mapped to boards/{entry['px4_board']}/ in the "
            f"registry, not boards/{board}/"
        )
    return None


def cmd_check(registry: Registry, paths: List[str]) -> int:
    violations: List[str] = []
    checked = 0
    for path in paths:
        values = parse_defconfig(path)
        vid_raw = values.get("VENDORID")
        if vid_raw is None:
            continue
        vid = parse_hex(vid_raw)
        if vid is None:
            violations.append(
                f"{path}: CONFIG_CDCACM_VENDORID \"{vid_raw}\" "
                "is not a hex value"
            )
            continue
        if vid != registry["vid"]:
            continue
        checked += 1

        violation = check_defconfig(registry, path, values)
        if violation:
            violations.append(f"{path}: {violation}")

    for v in violations:
        print(f"error: {v}", file=sys.stderr)
    print(
        f"checked {checked} defconfig(s) with VID 0x3643, "
        f"{len(violations)} violation(s)"
    )
    return 1 if violations else 0


def cmd_lookup(registry: Registry, query: str) -> int:
    if query.lower().startswith("0x"):
        entry = registry["pids"].get(int(query, 16))
        if entry is None:
            print(f"{query}: not registered")
            return 1
        print(f"{query}: {entry['manufacturer']}, board: {entry['board']}")
        return 0

    vendor = (board_path(query) or query).split("/")[0]
    name = registry["vendors"].get(vendor)
    if name is None:
        print(
            f"boards/{vendor}/: no manufacturer registered "
            "for this vendor directory"
        )
        return 1
    pids = sorted(
        p for p, e in registry["pids"].items() if e["manufacturer"] == name
    )
    pid_list = ", ".join(f"0x{p:04X}" for p in pids)
    print(f"boards/{vendor}/: {name}, PIDs: {pid_list}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--registry", help="path to a local usb-ids.yaml")
    parser.add_argument("--registry-url", default=REGISTRY_URL)
    sub = parser.add_subparsers(dest="command", required=True)
    p_check = sub.add_parser("check", help="check defconfig files")
    p_check.add_argument("paths", nargs="+")
    p_lookup = sub.add_parser("lookup", help="look up a PID or board path")
    p_lookup.add_argument("query")
    args = parser.parse_args()

    registry = load_registry(args.registry, args.registry_url)
    if args.command == "check":
        return cmd_check(registry, args.paths)
    return cmd_lookup(registry, args.query)


if __name__ == "__main__":
    sys.exit(main())
