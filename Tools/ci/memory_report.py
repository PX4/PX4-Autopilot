#!/usr/bin/env python3
"""Compare firmware sections, individual stack frames and selected object sizes."""

import argparse
import html
import json
from pathlib import Path
import subprocess

from firmware_size import MIN_REPORTED_DELTA, indicator, memory_usage
from module_object_sizes import collect_object_sizes
from stack_frames import collect_stack_frames


LIMITATIONS = [
    "Function frames are not task stack peaks. Calls, recursion, indirect calls and interrupt/context overhead are not combined.",
    "Dynamic unbounded frames have no compiler-proven upper bound; their recorded bytes must not be treated as the maximum.",
    "Coverage is the collected .su records, not every linked function. Inlining, linker removal, assembly and external libraries limit coverage.",
    "Object sizes include inline members and padding, but exclude pointed-to allocations, allocator overhead and instance counts.",
    "Embedded objects are already included in their owner. These categories must not be added into a total RAM estimate.",
    "Runtime heap peaks, fragmentation, allocation failures and task watermarks require separate target tests.",
]


def collect_snapshot(elf, stack_root, source_root, manifest, target, revision, toolchain):
    entries = manifest["types"]
    names = [entry["name"] for entry in entries]
    if len(names) != len(set(names)):
        raise ValueError("Manifest contains duplicate type names")
    try:
        sections = {"status": "ok", "reason": None, **memory_usage(elf)}
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        sections = {"status": "unavailable", "reason": str(error)}
    return {
        "schema_version": 1, "target": target, "revision": revision, "toolchain": toolchain,
        "manifest": entries, "static": sections,
        "objects": collect_object_sizes(elf, names),
        "stack": collect_stack_frames(stack_root, source_root),
    }


def _availability(before, after):
    reasons = [f"{side}: {data.get('reason') or 'unavailable'}"
               for side, data in (("Before", before), ("After", after))
               if data["status"] != "ok"]
    return {"status": "unavailable" if reasons else "ok", "reason": "; ".join(reasons) or None}


def _stack_map(collection):
    records = {(frame["source"], frame["function"]): frame for frame in collection["frames"]}
    ambiguous = {(frame["source"], frame["function"]): frame
                 for frame in collection.get("ambiguities", [])}
    return records, ambiguous


def _stack_observation(frame, ambiguity):
    """Compare compiler evidence without line numbers or record ordering."""
    if ambiguity is not None:
        return ("ambiguous", tuple(sorted({(item["bytes"], item["kind"])
                                          for item in ambiguity.get("variants", [])})))
    return ("known", frame["bytes"], frame["kind"]) if frame is not None else None


def compare_snapshots(before, after, label="Firmware build comparison"):
    for key in ("schema_version", "target", "toolchain", "manifest"):
        if before[key] != after[key]:
            raise ValueError(f"Cannot compare snapshots with different {key}")
    if before["schema_version"] != 1:
        raise ValueError("Unsupported snapshot schema")
    sections = _availability(before["static"], after["static"])
    for key in ("flash", "ram"):
        sections[key] = None
        if sections["status"] == "ok":
            old, new = before["static"][key], after["static"][key]
            sections[key] = {"before": old, "after": new, "delta": new - old}

    objects = {**_availability(before["objects"], after["objects"]), "rows": []}
    for entry in after["manifest"]:
        name = entry["name"]
        old, new = (snapshot["objects"]["types"][name] for snapshot in (before, after))
        available = objects["status"] == "ok" and old["status"] == new["status"] == "ok"
        old_size = old["bytes"] if before["objects"]["status"] == old["status"] == "ok" else None
        new_size = new["bytes"] if after["objects"]["status"] == new["status"] == "ok" else None
        objects["rows"].append({
            **entry, "included_in": entry.get("included_in"),
            "before": old_size, "after": new_size,
            "delta": new_size - old_size if available else None,
            "status": ("unchanged" if old_size == new_size else "changed") if available else "unavailable",
            "reason": None if available else objects["reason"] or _availability(old, new)["reason"],
        })

    stack = {**_availability(before["stack"], after["stack"]), "rows": [],
             "coverage": {"before": before["stack"]["coverage"], "after": after["stack"]["coverage"]}}
    if stack["status"] == "ok":
        old_frames, old_ambiguous = _stack_map(before["stack"])
        new_frames, new_ambiguous = _stack_map(after["stack"])
        for identity in sorted(old_frames.keys() | new_frames.keys() | old_ambiguous.keys() | new_ambiguous.keys()):
            old, new = old_frames.get(identity), new_frames.get(identity)
            if _stack_observation(old, old_ambiguous.get(identity)) == _stack_observation(new, new_ambiguous.get(identity)):
                continue
            ambiguous = identity in old_ambiguous or identity in new_ambiguous
            status = "ambiguous" if ambiguous else "added" if old is None else "removed" if new is None else (
                "unchanged" if (old["bytes"], old["kind"]) == (new["bytes"], new["kind"]) else "changed")
            comparable = (not ambiguous and old is not None and new is not None
                          and old["kind"] != "dynamic_unbounded" and new["kind"] != "dynamic_unbounded")
            ambiguity_reasons = dict.fromkeys(
                entry.get("reason") or "Conflicting compiler records; no single comparable frame size"
                for entry in (old_ambiguous.get(identity), new_ambiguous.get(identity)) if entry is not None)
            stack["rows"].append({
                "source": identity[0], "function": identity[1], "before": old, "after": new,
                "delta": new["bytes"] - old["bytes"] if comparable else None,
                "status": status, "measurement_changed": True,
                "reason": "; ".join(ambiguity_reasons) if ambiguous else None,
            })

    # Missing data remains visible. There is no pass/fail memory-safety claim.
    changed = (any(part["status"] != "ok" for part in (sections, objects, stack))
               or any(row["status"] != "unchanged" for row in objects["rows"] + stack["rows"])
               or any(sections[key] and abs(sections[key]["delta"]) >= MIN_REPORTED_DELTA
                      for key in ("flash", "ram")))
    return {
        "schema_version": 1, "target": after["target"], "changed": changed,
        "provenance": {"before": before["revision"], "after": after["revision"],
                       "toolchain": after["toolchain"], "label": label},
        "static": sections, "objects": objects, "stack": stack, "limitations": LIMITATIONS,
    }


def _md(value, max_length=None):
    value = str(value)
    if max_length is not None and len(value) > max_length:
        value = value[:max_length - 1] + "…"
    return html.escape(value).replace("|", "&#124;").replace("`", "&#96;").replace("\n", " ").replace("\r", " ")


def _bytes(value, signed=False):
    return "Unavailable" if value is None else f"{indicator(value)}{value:+,} B" if signed else f"{value:,} B"


def _frame(frame):
    if frame is None:
        return "Unavailable"
    if frame["kind"] == "dynamic_unbounded":
        return f"{frame['bytes']:,} B + dynamic ⚠️"
    suffix = " (bounded dynamic)" if frame["kind"] == "dynamic_bounded" else ""
    return _bytes(frame["bytes"]) + suffix


def _changed_stack_rows(report):
    return [row for row in report["stack"]["rows"]
            if row["status"] in ("changed", "added", "removed")
            or (row["status"] == "ambiguous" and row.get("measurement_changed"))]


def _largest_delta(rows):
    deltas = [row["delta"] for row in rows if row.get("delta") is not None]
    # An increase must not be hidden by a larger decrease elsewhere.
    increases = [value for value in deltas if value > 0]
    return max(increases) if increases else min(deltas, default=0)


def render_details_summary(report):
    """One short, escaped heading visible while the bot's details are closed."""
    objects = report["objects"]
    owners = [row for row in objects["rows"] if not row.get("included_in") and row["status"] == "changed"]
    missing = sum(row["status"] == "unavailable" for row in objects["rows"])
    object_text = f"{indicator(_largest_delta(owners))}{len(owners)} module object{'s' if len(owners) != 1 else ''} changed"
    if owners:
        object_text += f" (largest {_largest_delta(owners):+,} B)"
    if missing or objects["status"] != "ok":
        object_text += "; ⚠️ object data unavailable"
    stack = report["stack"]
    rows = _changed_stack_rows(report)
    stack_text = f"{indicator(_largest_delta(rows))}{len(rows)} stack change{'s' if len(rows) != 1 else ''}"
    if stack["status"] != "ok":
        stack_text = "⚠️ stack data unavailable"
    elif any(row["status"] == "ambiguous" or (row.get("after") or {}).get("kind") == "dynamic_unbounded" for row in rows):
        stack_text += "; ⚠️ unknown bounds"
    return _md(f"{report['target']} — {object_text}; {stack_text}")


def render_markdown(report, limit=12):
    lines = ["| Module object | Before | After | Δ per instance |", "| --- | ---: | ---: | ---: |"]
    for row in report["objects"]["rows"]:
        owner = f" (included in {_md(row['included_in'])})" if row["included_in"] else ""
        lines.append(f"| {_md(row['name'])}{owner} | {_bytes(row['before'])} | {_bytes(row['after'])} | {_bytes(row['delta'], True)} |")
    reasons = [f"- {_md(row['name'])}: {_md(row['reason'])}" for row in report["objects"]["rows"] if row["reason"]]
    lines += [""]
    stack = report["stack"]
    if stack["status"] != "ok":
        lines += ["⚠️ Stack data unavailable."]
        reasons.append(f"- Stack: {_md(stack['reason'])}")
    else:
        changed = _changed_stack_rows(report)
        # Keep unknown dynamic bounds visible even when the comment is truncated.
        changed.sort(key=lambda row: (
            not any((row[side] or {}).get("kind") == "dynamic_unbounded" for side in ("before", "after")),
            -(abs(row["delta"] or 0)), row["function"]))
        if changed:
            lines += ["| Function frame | Before | After | Δ |", "| --- | ---: | ---: | ---: |"]
            for row in changed[:limit]:
                note = f" — {_md(row['status'])}"
                delta = _bytes(row["delta"], True) if row["delta"] is not None else "⚠️ Not comparable"
                lines.append(f"| {_md(row['source'], 100)}: {_md(row['function'], 160)}{note} | {_frame(row['before'])} | {_frame(row['after'])} | {delta} |")
            if len(changed) > limit:
                lines += ["", f"Showing {limit} of {len(changed)} changed records; full list in the report artifact."]
        else:
            lines += ["No frame changes."]
    lines += ["", "<details>", "<summary>How to read this</summary>", "",
              "- Object sizes exclude separate allocations. Embedded members are included in their owner; do not add these rows.",
              "- Stack values are individual frames, not task peaks. `+ dynamic` has no known upper bound.",
              "- Δ colours: 🟡 >+100 B; 🔴 >+1,000 B; 🟢 decrease >100 B. Change indicators, not safety limits; ⚠️ means unknown.",
              "- Only changed stack records are listed. Full records are in the artifacts."]
    if reasons:
        lines += ["", "Measurement details:", ""] + reasons
    cov = stack["coverage"]
    lines += ["", f"Coverage: {cov['before'].get('files', 0)} → {cov['after'].get('files', 0)} files; "
              f"{cov['before'].get('functions', 0)} → {cov['after'].get('functions', 0)} functions; "
              f"{cov['before'].get('ambiguous_functions', 0)} → {cov['after'].get('ambiguous_functions', 0)} ambiguous.",
              "", f"Compared `{_md(report['provenance']['before'])}` → `{_md(report['provenance']['after'])}`.",
              "", "</details>", ""]
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    collect = commands.add_parser("collect")
    for flag in ("elf", "stack-root", "source-root", "manifest", "output"):
        collect.add_argument(f"--{flag}", required=True, type=Path)
    for flag in ("target", "revision", "toolchain"):
        collect.add_argument(f"--{flag}", required=True)
    compare = commands.add_parser("compare")
    for flag in ("before", "after", "json-output", "markdown-output", "html-output"):
        compare.add_argument(f"--{flag}", required=True, type=Path)
    compare.add_argument("--label", default="Firmware build comparison")
    compare.add_argument("--summary-output", type=Path, help="Escaped heading for the bot's expandable details")
    args = parser.parse_args()
    if args.command == "collect":
        result = collect_snapshot(args.elf, args.stack_root, args.source_root,
                                  json.loads(args.manifest.read_text()), args.target, args.revision, args.toolchain)
        args.output.write_text(json.dumps(result, indent=2) + "\n")
    else:
        from memory_report_html import render_html
        result = compare_snapshots(json.loads(args.before.read_text()), json.loads(args.after.read_text()), args.label)
        args.json_output.write_text(json.dumps(result, indent=2) + "\n")
        args.markdown_output.write_text(render_markdown(result))
        args.html_output.write_text(render_html(result))
        if args.summary_output:
            args.summary_output.write_text(render_details_summary(result) + "\n")


if __name__ == "__main__":
    main()
