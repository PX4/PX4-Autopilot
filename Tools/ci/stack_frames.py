#!/usr/bin/env python3
"""Collect GCC -fstack-usage records without claiming task stack bounds.

These are individual compiled function frames, not a call-chain total. The
records must come from the same compiler, target, and optimization settings as
the firmware being compared; this collector cannot establish that provenance.

GCC's ``static`` byte count is fixed. ``dynamic,bounded`` includes the maximum
bounded dynamic adjustment. Bare ``dynamic`` only gives the bounded portion:
its byte count is NOT an upper bound. See GCC's -fstack-usage documentation:
https://gcc.gnu.org/onlinedocs/gcc/Developer-Options.html
"""

import argparse
from collections import defaultdict
import json
from pathlib import Path
import re


_LOCATION = re.compile(r"^(.*?):([0-9]+):([0-9]+):(.+)$")
# GCC 13 can emit incomplete names such as ')' or '0(args)' for optimized
# functions. These cannot identify a function, even if byte counts agree.
_INCOMPLETE_NAME = re.compile(r"^(?:\)|[0-9]+\()")
_KINDS = {
    frozenset({"static"}): "static",
    frozenset({"dynamic", "bounded"}): "dynamic_bounded",
    frozenset({"dynamic"}): "dynamic_unbounded",
}


def _parse_record(record, source_root):
    fields = record.rsplit("\t", 2)
    if len(fields) != 3:
        raise ValueError("expected location, byte count, and qualifiers separated by tabs")

    location, byte_count, qualifiers = fields
    match = _LOCATION.fullmatch(location)
    if not match or not match[1]:
        raise ValueError("expected source:line:column:function")
    if not byte_count.isascii() or not byte_count.isdecimal():
        raise ValueError("expected a nonnegative integer byte count")

    qualifier_list = [value.strip() for value in qualifiers.split(",")]
    qualifier_set = frozenset(qualifier_list)
    if len(qualifier_list) != len(qualifier_set) or qualifier_set not in _KINDS:
        raise ValueError(f"unsupported stack qualifier: {qualifiers!r}")

    source = Path(match[1])
    if source_root is not None and source.is_absolute():
        # Do not guess the working directory for relative compiler paths.
        try:
            source = source.resolve().relative_to(source_root)
        except ValueError:
            pass

    return {
        "source": source.as_posix(),
        "function": match[4],
        "bytes": int(byte_count),
        "kind": _KINDS[qualifier_set],
        "line": int(match[2]),
    }


def collect_stack_frames(root, source_root=None):
    """Read .su files recursively, deduplicating by source and function.

    Conflicting observations for one identity (e.g. an inline function compiled
    differently in several translation units) are excluded from ``frames`` and
    preserved in ``ambiguities``, as are incomplete compiler function names.
    Line numbers are diagnostic, not identities.
    Malformed input invalidates the collection rather than producing a partial
    successful report. Coverage counts describe .su inputs, not all firmware
    functions: assembly, libraries, inlining and linker removal limit coverage.
    """
    root = Path(root)
    source_root = Path(source_root).resolve() if source_root is not None else None
    result = {
        "status": "unavailable",
        "reason": None,
        "frames": [],
        "ambiguities": [],
        "errors": [],
        "coverage": {
            "files": 0,
            "records": 0,
            "functions": 0,
            "static": 0,
            "dynamic_bounded": 0,
            "dynamic_unbounded": 0,
            "ambiguous_functions": 0,
            "invalid_records": 0,
        },
    }
    if not root.is_dir():
        result["reason"] = "Stack-usage directory does not exist or is not a directory"
        return result

    files = sorted(path for path in root.rglob("*.su") if path.is_file())
    result["coverage"]["files"] = len(files)
    if not files:
        result["reason"] = "No .su files; build with -fstack-usage to collect function frames"
        return result

    observations = defaultdict(list)
    for path in files:
        try:
            records = path.read_text(encoding="utf-8").splitlines()
        except (OSError, UnicodeError) as error:
            result["errors"].append({"file": str(path.relative_to(root)), "reason": str(error)})
            continue

        for number, record in enumerate(records, 1):
            if not record.strip():
                continue
            try:
                frame = _parse_record(record, source_root)
            except ValueError as error:
                result["coverage"]["invalid_records"] += 1
                result["errors"].append({
                    "file": str(path.relative_to(root)), "line": number, "reason": str(error)
                })
                continue
            result["coverage"]["records"] += 1
            observations[(frame["source"], frame["function"])].append(frame)

    if result["errors"]:
        result["reason"] = "Invalid or unreadable .su input; stack-frame comparison is unavailable"
        return result
    if not observations:
        result["reason"] = "The .su files contain no function records"
        return result

    for (source, function), frames in sorted(observations.items()):
        variants = {}
        for frame in frames:
            key = (frame["bytes"], frame["kind"])
            if key not in variants or frame["line"] < variants[key]["line"]:
                variants[key] = frame
        incomplete_name = _INCOMPLETE_NAME.match(function) is not None
        if len(variants) != 1 or incomplete_name:
            result["ambiguities"].append({
                "source": source,
                "function": function,
                "reason": ("Compiler supplied an incomplete function name" if incomplete_name else
                           "Conflicting compiler records; no single comparable frame size"),
                "variants": [variants[key] for key in sorted(variants)],
            })
            result["coverage"]["ambiguous_functions"] += 1
            continue

        frame = next(iter(variants.values()))
        result["frames"].append(frame)
        result["coverage"][frame["kind"]] += 1

    result["coverage"]["functions"] = len(observations)
    result["status"] = "ok"
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", required=True, type=Path, help="Build directory containing .su files")
    parser.add_argument("--source-root", type=Path, help="Strip this prefix from absolute source paths")
    args = parser.parse_args()
    result = collect_stack_frames(args.root, args.source_root)
    print(json.dumps(result, indent=2, sort_keys=True))
    # Missing data may be advisory to a caller, but must not look like success
    # when this collector is run directly in a required CI step.
    return 0 if result["status"] == "ok" else 1


if __name__ == "__main__":
    raise SystemExit(main())
