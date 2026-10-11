#!/usr/bin/env python3
"""Check firmware and bootloader IDs against the PX4-Bootloader registry.

Run from any directory with Python 3.10+; no build or third-party packages needed:
    python3 Tools/ci/check_board_ids.py
    python3 Tools/ci/check_board_ids.py --registry ../PX4-Bootloader/board_types.txt

By default, download board_types.txt at the revision pinned in board_ids.json.
--registry uses a local copy instead (including for offline use).
--strict also fails on the documented, grandfathered inconsistencies.

When adding a board, map its boards/<vendor>/<model> path to its registry name in
board_ids.json. Sharing a registry entry explicitly declares ID compatibility;
do not reuse another board's entry just to make this check pass. Update the
pinned revision when adopting newly registered IDs. Legacy entries freeze exact
existing values, not entire boards: remove them when correcting those values.

This checks source metadata, not installed or prebuilt bootloader binaries.
"""

import argparse
import json
from pathlib import Path
import re
import sys
from urllib.error import URLError
from urllib.request import urlopen


ROOT = Path(__file__).resolve().parents[2]
POLICY = Path(__file__).with_name("board_ids.json")


def unique_keys(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"Duplicate JSON key: {key}")
        result[key] = value
    return result


def load_json(path):
    return json.loads(path.read_text(encoding="utf-8"), object_pairs_hook=unique_keys)


def parse_registry(text):
    registry = {}
    for number, line in enumerate(text.splitlines(), 1):
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        match = re.fullmatch(r"(.+?)\s+([0-9]+)", line)
        if not match:
            raise ValueError(f"board_types.txt:{number}: invalid registry entry")
        name = " ".join(match[1].split())
        value = int(match[2])
        if name in registry:
            raise ValueError(f"board_types.txt:{number}: duplicate name {name}")
        registry[name] = value
    if not registry:
        raise ValueError("Board ID registry is empty")
    return registry


def read_board_ids(board):
    prototype = board / "firmware.prototype"
    board_id = load_json(prototype)["board_id"]
    if type(board_id) is not int or not 0 < board_id <= 0xFFFFFFFF:
        raise ValueError(f"{prototype}: board_id must be a positive uint32")

    header = board / "src/hw_config.h"
    if not header.exists():
        if (board / "bootloader.px4board").exists():
            raise ValueError(f"{header}: missing bootloader configuration")
        return board_id, None

    text = re.sub(r"/\*.*?\*/|//[^\n]*", "", header.read_text(encoding="utf-8"), flags=re.S)
    definitions = re.findall(r"^\s*#\s*define\s+BOARD_TYPE\b([^\n]*)", text, re.M)
    if len(definitions) != 1:
        raise ValueError(f"{header}: expected exactly one BOARD_TYPE definition")
    value = definitions[0].strip()
    if not re.fullmatch(r"(?:0[xX][0-9a-fA-F]+|[1-9][0-9]*)", value):
        raise ValueError(f"{header}: BOARD_TYPE must be a literal integer, got {value!r}")
    board_type = int(value, 16 if value.lower().startswith("0x") else 10)
    if not 0 < board_type <= 0xFFFFFFFF:
        raise ValueError(f"{header}: BOARD_TYPE must be a positive uint32")
    return board_id, board_type


def check_boards(root, policy, registry):
    errors = []
    warnings = []
    registered = policy["boards"]
    legacy = policy["legacy_boards"]
    exceptions = policy["bootloader_exceptions"]
    boards = {
        str(path.parent.relative_to(root / "boards")): path.parent
        for path in (root / "boards").glob("*/*/firmware.prototype")
    }
    if not boards:
        return ["No firmware.prototype files found"], warnings

    for name in sorted(registered.keys() & legacy.keys()):
        errors.append(f"{name}: listed in both boards and legacy_boards")
    for name in sorted((registered.keys() | legacy.keys() | exceptions.keys()) - boards.keys()):
        errors.append(f"{name}: stale board_ids.json entry; board does not exist")
    for name in sorted(exceptions.keys() - registered.keys()):
        errors.append(f"{name}: bootloader exception requires a registered board")
    for path in (root / "boards").glob("*/*/bootloader.px4board"):
        if not (path.parent / "firmware.prototype").exists():
            errors.append(f"{path.parent}: missing firmware.prototype")

    for name, board in sorted(boards.items()):
        try:
            board_id, board_type = read_board_ids(board)
        except (OSError, ValueError, KeyError) as error:
            errors.append(f"{name}: {error}")
            continue

        if name in legacy:
            entry = legacy[name]
            if not entry["reason"].strip():
                errors.append(f"{name}: legacy entry requires a reason")
            if (board_id, board_type) != (entry["board_id"], entry["board_type"]):
                errors.append(
                    f"{name}: legacy IDs changed: firmware={board_id}, bootloader={board_type}; "
                    "register the correct identity and remove the legacy entry"
                )
            else:
                warnings.append(f"{name}: {entry['reason']}")
            continue

        if name not in registered:
            errors.append(f"{name}: add its registry name to Tools/ci/board_ids.json")
            continue
        registry_name = registered[name]
        if registry_name not in registry:
            errors.append(f"{name}: {registry_name!r} is missing from the pinned registry")
            continue
        expected = registry[registry_name]
        if board_id != expected:
            errors.append(
                f"{name}/firmware.prototype: board_id={board_id}, "
                f"expected {expected} ({registry_name})"
            )

        if name in exceptions:
            entry = exceptions[name]
            if not entry["reason"].strip():
                errors.append(f"{name}: bootloader exception requires a reason")
            if entry["board_type"] == expected:
                errors.append(f"{name}: unnecessary bootloader exception")
            if board_type != entry["board_type"]:
                errors.append(f"{name}: bootloader ID changed; remove or resolve its exact exception")
            else:
                warnings.append(f"{name}: {entry['reason']}")
        elif board_type is not None and board_type != expected:
            errors.append(
                f"{name}/src/hw_config.h: BOARD_TYPE={board_type}, "
                f"expected {expected} ({registry_name})"
            )

    return errors, warnings


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--root", type=Path, default=ROOT, help="PX4-Autopilot checkout to inspect")
    parser.add_argument("--registry", type=Path, help="Local board_types.txt (no network access)")
    parser.add_argument("--strict", action="store_true", help="Also fail on known legacy inconsistencies")
    args = parser.parse_args()

    try:
        policy = load_json(POLICY)
        revision = policy["registry_revision"]
        if not re.fullmatch(r"[0-9a-f]{40}", revision):
            raise ValueError("registry_revision must be a full commit SHA")
        if args.registry:
            text = args.registry.read_text(encoding="utf-8")
        else:
            url = f"https://raw.githubusercontent.com/PX4/PX4-Bootloader/{revision}/board_types.txt"
            with urlopen(url, timeout=30) as response:
                text = response.read().decode("utf-8")
        errors, warnings = check_boards(args.root, policy, parse_registry(text))
    except (OSError, ValueError, KeyError, URLError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1

    for warning in warnings:
        print(f"KNOWN: {warning}")
    for error in errors:
        print(f"ERROR: {error}", file=sys.stderr)
    if errors or (args.strict and warnings):
        print(f"Board ID check failed: {len(errors)} errors, {len(warnings)} known inconsistencies.")
        return 1
    print(f"Board ID check passed ({len(warnings)} documented legacy inconsistencies).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
