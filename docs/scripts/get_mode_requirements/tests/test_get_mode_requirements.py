#!/usr/bin/env python3
"""
Regression tests for get_mode_requirements.py.

Two golden JSON files in test_data/ capture the expected parser output:

  test_data/requirement_defns_golden.json
      The static requirement_defns mapping (flag name → text/sensor/detail).
      Update this when adding or editing entries in requirement_defns.

  test_data/vehicle_modes_golden.json
      The parsed output of mode_requirements.cpp: per-vehicle-type,
      per-mode lists of requirement flags (sorted alphabetically).
      Update this when mode_requirements.cpp changes.

Running the tests
-----------------
From the get_mode_requirements/ directory:
  pytest                   # discover and run all tests
  pytest tests/ -v         # verbose

Updating the golden files after intentional changes
----------------------------------------------------
  python tests/test_get_mode_requirements.py --update-golden

This rewrites both golden files with the current parser output and exits.

"""

import argparse
import json
import sys
from pathlib import Path

# ---------------------------------------------------------------------------
# Import the module under test
# ---------------------------------------------------------------------------
_TESTS_DIR = Path(__file__).resolve().parent
_SCRIPT_DIR = _TESTS_DIR.parent  # get_mode_requirements/

# Ensure the script directory is on sys.path when run directly.
# conftest.py does the same when running under pytest.
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))

from get_mode_requirements import parse_requirements, requirement_defns  # noqa: E402

# ---------------------------------------------------------------------------
# Golden file paths
# ---------------------------------------------------------------------------
_TEST_DATA = _TESTS_DIR / "test_data"
_GOLDEN_REQUIREMENT_DEFNS = _TEST_DATA / "requirement_defns_golden.json"
_GOLDEN_VEHICLE_MODES = _TEST_DATA / "vehicle_modes_golden.json"


# ---------------------------------------------------------------------------
# Helper
# ---------------------------------------------------------------------------

def _vehicle_modes_as_sorted_lists(vehicle_modes: dict) -> dict:
    """Convert the sets returned by parse_requirements() to sorted lists."""
    return {
        vtype: {mode: sorted(reqs) for mode, reqs in modes.items()}
        for vtype, modes in vehicle_modes.items()
    }


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def test_requirement_defns_matches_golden():
    """requirement_defns must match the golden snapshot.

    Fails when a flag is added, removed, or its text/sensor/detail is edited
    without also updating the golden file.
    """
    with _GOLDEN_REQUIREMENT_DEFNS.open(encoding="utf-8") as fh:
        expected = json.load(fh)

    assert requirement_defns == expected, (
        "requirement_defns has diverged from the golden file.\n"
        "If this change is intentional run:\n"
        "  python tests/test_get_mode_requirements.py --update-golden"
    )


def test_vehicle_modes_matches_golden():
    """parse_requirements() applied to the checked-in mode_requirements.cpp
    must produce output matching the golden snapshot.

    Fails when mode_requirements.cpp is edited in a way that adds, removes, or
    changes the requirement flags for any mode/vehicle-type combination.
    """
    with _GOLDEN_VEHICLE_MODES.open(encoding="utf-8") as fh:
        expected = json.load(fh)

    vehicle_modes = parse_requirements()
    actual = _vehicle_modes_as_sorted_lists(vehicle_modes)

    # Collect all differences for a readable failure message
    diffs = []

    all_vtypes = set(expected) | set(actual)
    for vtype in sorted(all_vtypes):
        if vtype not in actual:
            diffs.append(f"  Vehicle type removed: {vtype}")
            continue
        if vtype not in expected:
            diffs.append(f"  Vehicle type added: {vtype}")
            continue

        all_modes = set(expected[vtype]) | set(actual[vtype])
        for mode in sorted(all_modes):
            exp_reqs = expected[vtype].get(mode)
            act_reqs = actual[vtype].get(mode)
            if exp_reqs != act_reqs:
                diffs.append(
                    f"  {vtype} / {mode}:\n"
                    f"    expected: {exp_reqs}\n"
                    f"    actual:   {act_reqs}"
                )

    assert not diffs, (
        "Parsed vehicle modes have diverged from the golden file:\n"
        + "\n".join(diffs)
        + "\nIf this change is intentional run:\n"
        "  python tests/test_get_mode_requirements.py --update-golden"
    )


def test_all_requirement_flags_are_defined():
    """Every requirement flag that appears in parse_requirements() output must
    have an entry in requirement_defns.

    This catches the case where a new flag is added to mode_requirements.cpp
    but not documented in requirement_defns.
    """
    vehicle_modes = parse_requirements()
    missing = set()
    for modes in vehicle_modes.values():
        for reqs in modes.values():
            for req in reqs:
                if req not in requirement_defns:
                    missing.add(req)

    assert not missing, (
        "The following requirement flags are used in mode_requirements.cpp but "
        "have no entry in requirement_defns:\n  " + "\n  ".join(sorted(missing))
    )


# ---------------------------------------------------------------------------
# Golden-file updater (run directly, not via pytest)
# ---------------------------------------------------------------------------

def _update_golden():
    _TEST_DATA.mkdir(exist_ok=True)

    # requirement_defns
    with _GOLDEN_REQUIREMENT_DEFNS.open("w", encoding="utf-8") as fh:
        json.dump(requirement_defns, fh, indent=2, ensure_ascii=False)
        fh.write("\n")
    print(f"Updated {_GOLDEN_REQUIREMENT_DEFNS}")

    # vehicle_modes
    vehicle_modes = parse_requirements()
    vehicle_modes_sorted = _vehicle_modes_as_sorted_lists(vehicle_modes)
    with _GOLDEN_VEHICLE_MODES.open("w", encoding="utf-8") as fh:
        json.dump(vehicle_modes_sorted, fh, indent=2, ensure_ascii=False)
        fh.write("\n")
    print(f"Updated {_GOLDEN_VEHICLE_MODES}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Manage golden files for get_mode_requirements tests.")
    parser.add_argument(
        "--update-golden",
        action="store_true",
        help="Rewrite the golden JSON files with the current parser output.",
    )
    args = parser.parse_args()

    if args.update_golden:
        _update_golden()
    else:
        parser.print_help()
