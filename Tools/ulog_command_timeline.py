#!/usr/bin/env python3
"""Reconstruct vehicle_command / vehicle_command_ack timeline from a ulog → HTML."""

import argparse
import html
import math
import os
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

from pyulog import ULog

SCRIPT_DIR = Path(__file__).resolve().parent
MSG_DIR = SCRIPT_DIR.parent / "msg" / "versioned"
MAVLINK_XML_DIR = SCRIPT_DIR.parent / "src" / "modules" / "mavlink" / "mavlink" / "message_definitions" / "v1.0"


# Known sub-enum group prefixes (longest-match-first). Constants with these
# prefixes get bucketed; the rest fall into their own raw group keyed by full name.
KNOWN_PREFIXES = [
    "VEHICLE_CMD_RESULT",
    "ARM_AUTH_DENIED_REASON",
    "VEHICLE_MOUNT_MODE",
    "VEHICLE_ROI",
    "PARACHUTE_ACTION",
    "FAILURE_UNIT",
    "FAILURE_TYPE",
    "SPEED_TYPE",
    "ORBIT_YAW_BEHAVIOUR",
    "ARMING_ACTION",
    "GRIPPER_ACTION",
    "PREFLIGHT_CALIBRATION",
    "COMPONENT_MODE_EXECUTOR",
    "VEHICLE_CMD",  # catch-all for command IDs themselves
]


def parse_enum_file(path):
    """Return dict: prefix -> {int_value: suffix_after_prefix_underscore}."""
    if not path.exists():
        return {}
    groups = {}
    line_re = re.compile(r"^\s*(?:u?int\d+|int\d+)\s+([A-Z][A-Z0-9_]*)\s*=\s*(-?\d+)")
    for line in path.read_text().splitlines():
        m = line_re.match(line)
        if not m:
            continue
        name, val = m.group(1), int(m.group(2))
        for prefix in KNOWN_PREFIXES:
            if name == prefix or name.startswith(prefix + "_"):
                suffix = name[len(prefix) + 1:] if name != prefix else name
                groups.setdefault(prefix, {})[val] = suffix
                break
    return groups


def parse_param_labels(path):
    """Return dict: command_short_name -> [label1..label7] from .msg comments.

    Line shape: `uint16 VEHICLE_CMD_FOO = 22 # desc |p1 text|p2 text|...|p7 text|`
    """
    if not path.exists():
        return {}
    labels = {}
    line_re = re.compile(
        r"^\s*u?int\d+\s+VEHICLE_CMD_([A-Z0-9_]+)\s*=\s*\d+\s*#[^|]*\|(.*)$"
    )
    for line in path.read_text().splitlines():
        m = line_re.match(line)
        if not m:
            continue
        name = m.group(1)
        parts = [p.strip() for p in m.group(2).split("|")]
        parts = [p for p in parts if p != ""]
        if not parts:
            continue
        # truncate each label to short hint
        labels[name] = [shorten_label(p) for p in parts[:7]]
    return labels


def shorten_label(s):
    # strip [units] and @enum markers; keep first ~24 chars of meaningful text
    s = re.sub(r"\[@enum [^\]]+\]", "", s)
    s = re.sub(r"\[@range[^\]]+\]", "", s)
    s = re.sub(r"\s+", " ", s).strip(" .,")
    if len(s) > 28:
        s = s[:28].rstrip() + "…"
    return s


def build_enums():
    global MAV_COMPONENT
    cmd_enums = parse_enum_file(MSG_DIR / "VehicleCommand.msg")
    ack_enums = parse_enum_file(MSG_DIR / "VehicleCommandAck.msg")
    param_labels = parse_param_labels(MSG_DIR / "VehicleCommand.msg")
    MAV_COMPONENT = parse_mavlink_enum(MAVLINK_XML_DIR, "MAV_COMPONENT") or dict(MAV_COMPONENT_FALLBACK)

    # command_id -> short name (suffix after VEHICLE_CMD_)
    cmd_id_to_name = dict(cmd_enums.get("VEHICLE_CMD", {}))

    # ack result enum: VEHICLE_CMD_RESULT group
    result_map = {}
    if "VEHICLE_CMD_RESULT" in ack_enums:
        for v, suffix in ack_enums["VEHICLE_CMD_RESULT"].items():
            result_map[v] = suffix

    arm_denied_map = {}
    if "ARM_AUTH_DENIED_REASON" in ack_enums:
        for v, suffix in ack_enums["ARM_AUTH_DENIED_REASON"].items():
            arm_denied_map[v] = suffix

    # sub-enums in VehicleCommand.msg, keyed by prefix
    sub_enums = cmd_enums  # full dict of groups

    return cmd_id_to_name, result_map, arm_denied_map, sub_enums, param_labels


# Minimal fallback when MAVLink XML submodule isn't checked out.
MAV_COMPONENT_FALLBACK = {0: "ALL", 1: "AUTOPILOT1", 190: "MISSIONPLANNER", 191: "ONBOARD_COMPUTER"}


def parse_mavlink_enum(xml_dir, enum_name, _seen=None):
    """Walk MAVLink XML dialects (following <include> tags) for `enum_name`.

    Returns {int_value: short_name_without_common_prefix}. Empty dict on failure.
    """
    if _seen is None:
        _seen = set()
    result = {}
    # Search every .xml in dir; prefer minimal.xml first if enum is MAV_COMPONENT.
    xml_files = sorted(xml_dir.glob("*.xml")) if xml_dir.is_dir() else []
    # priority order: minimal, common, then rest
    order = {"minimal.xml": 0, "common.xml": 1}
    xml_files.sort(key=lambda p: order.get(p.name, 2))

    for xml_path in xml_files:
        if xml_path in _seen:
            continue
        _seen.add(xml_path)
        try:
            tree = ET.parse(xml_path)
        except ET.ParseError:
            continue
        root = tree.getroot()
        # follow includes (relative paths)
        for inc in root.findall("include"):
            inc_path = (xml_path.parent / inc.text.strip()).resolve()
            if inc_path.is_file() and inc_path not in _seen:
                _seen.add(inc_path)
                # parse same-dir include implicitly below; nothing extra here
        for enum in root.findall(".//enum"):
            if enum.get("name") != enum_name:
                continue
            for entry in enum.findall("entry"):
                try:
                    v = int(entry.get("value"))
                except (TypeError, ValueError):
                    continue
                name = entry.get("name") or ""
                # strip common prefix patterns: MAV_COMP_ID_, MAV_CMD_, MAV_RESULT_
                for p in ("MAV_COMP_ID_", "MAV_CMD_", "MAV_RESULT_"):
                    if name.startswith(p):
                        name = name[len(p):]
                        break
                result[v] = name
        if result:
            return result
    return result


# Loaded lazily by build_enums()
MAV_COMPONENT = {}


def fmt_comp(comp_id):
    name = MAV_COMPONENT.get(comp_id)
    return f"{comp_id}({name})" if name else str(comp_id)


# (command_short_name, param_index 1..7) -> sub-enum prefix in VehicleCommand.msg
PARAM_ENUMS = {
    ("COMPONENT_ARM_DISARM", 1): "ARMING_ACTION",
    ("DO_ORBIT", 3): "ORBIT_YAW_BEHAVIOUR",
    ("NAV_ROI", 1): "VEHICLE_ROI",
    ("DO_SET_ROI", 1): "VEHICLE_ROI",
    ("DO_MOUNT_CONFIGURE", 1): "VEHICLE_MOUNT_MODE",
    ("DO_PARACHUTE", 1): "PARACHUTE_ACTION",
    ("DO_CHANGE_SPEED", 1): "SPEED_TYPE",
    ("INJECT_FAILURE", 1): "FAILURE_UNIT",
    ("INJECT_FAILURE", 2): "FAILURE_TYPE",
    ("DO_GRIPPER", 2): "GRIPPER_ACTION",
}


def fmt_enum(val, name_map):
    if val in name_map:
        return f"{name_map[val]} ({val})"
    return f"? ({val})"


def fmt_param(cmd_name, idx, raw, sub_enums, param_labels):
    try:
        v = float(raw)
    except (TypeError, ValueError):
        return f"p{idx}={raw}"
    if math.isnan(v) or v == 0.0:
        return None
    label_list = param_labels.get(cmd_name, [])
    label = label_list[idx - 1] if idx - 1 < len(label_list) else ""
    tag = f"p{idx}"
    if label and not re.match(r"^[Uu]nused", label):
        tag = f"p{idx}[{label}]"
    elif label and re.match(r"^[Uu]nused", label):
        # explicitly unused param with non-zero value — still show, mark
        tag = f"p{idx}[unused?]"
    key = (cmd_name, idx)
    if key in PARAM_ENUMS:
        prefix = PARAM_ENUMS[key]
        if prefix in sub_enums and v.is_integer():
            iv = int(v)
            if iv in sub_enums[prefix]:
                return f"{tag}={sub_enums[prefix][iv]} ({iv})"
    if abs(v) >= 1e6 or abs(v) < 1e-3:
        return f"{tag}={v:.6g}"
    return f"{tag}={v:g}"


def fmt_time_rel(us):
    if us < 0:
        us = 0
    s = us / 1_000_000.0
    h = int(s // 3600)
    m = int((s % 3600) // 60)
    sec = s - h * 3600 - m * 60
    return f"{h:02d}:{m:02d}:{sec:06.3f}"


def extract_events(ulog, cmd_id_to_name, result_map, sub_enums, param_labels):
    events = []
    t0 = ulog.start_timestamp

    cmd_data = next((d for d in ulog.data_list if d.name == "vehicle_command"), None)
    if cmd_data is not None:
        d = cmd_data.data
        n = len(d["timestamp"])
        for i in range(n):
            ts = int(d["timestamp"][i])
            cmd_id = int(d["command"][i])
            name = cmd_id_to_name.get(cmd_id, f"UNKNOWN_{cmd_id}")
            params = []
            for p_idx in range(1, 8):
                key = f"param{p_idx}"
                if key in d:
                    out = fmt_param(name, p_idx, d[key][i], sub_enums, param_labels)
                    if out is not None:
                        params.append(out)
            events.append({
                "ts": ts,
                "rel": ts - t0,
                "kind": "CMD",
                "name": name,
                "cmd_id": cmd_id,
                "src": f"{int(d.get('source_system', [0]*n)[i])}/{fmt_comp(int(d.get('source_component', [0]*n)[i]))}",
                "tgt": f"{int(d.get('target_system', [0]*n)[i])}/{fmt_comp(int(d.get('target_component', [0]*n)[i]))}",
                "params": params,
                "from_external": bool(d["from_external"][i]) if "from_external" in d else False,
                "confirmation": int(d["confirmation"][i]) if "confirmation" in d else 0,
                "result": "",
                "result_class": "cmd",
            })

    ack_data = next((d for d in ulog.data_list if d.name == "vehicle_command_ack"), None)
    if ack_data is not None:
        d = ack_data.data
        n = len(d["timestamp"])
        for i in range(n):
            ts = int(d["timestamp"][i])
            cmd_id = int(d["command"][i])
            name = cmd_id_to_name.get(cmd_id, f"UNKNOWN_{cmd_id}")
            result_val = int(d["result"][i])
            result_txt = fmt_enum(result_val, result_map)
            rp1 = int(d["result_param1"][i]) if "result_param1" in d else 0
            rp2 = int(d["result_param2"][i]) if "result_param2" in d else 0
            extras = []
            if rp1:
                extras.append(f"result_param1={rp1}")
            if rp2:
                extras.append(f"result_param2={rp2}")
            events.append({
                "ts": ts,
                "rel": ts - t0,
                "kind": "ACK",
                "name": name,
                "cmd_id": cmd_id,
                "src": "",
                "tgt": f"{int(d.get('target_system', [0]*n)[i])}/{fmt_comp(int(d.get('target_component', [0]*n)[i]))}",
                "params": extras,
                "from_external": bool(d["from_external"][i]) if "from_external" in d else False,
                "confirmation": 0,
                "result": result_txt,
                "result_class": ack_class(result_val, result_map),
            })

    events.sort(key=lambda e: (e["ts"], 0 if e["kind"] == "CMD" else 1))
    return events


def ack_class(val, result_map):
    name = result_map.get(val, "")
    if name == "ACCEPTED":
        return "ack-ok"
    if name == "IN_PROGRESS":
        return "ack-progress"
    if name in ("DENIED", "FAILED", "UNSUPPORTED", "TEMPORARILY_REJECTED", "CANCELLED"):
        return "ack-bad"
    return "ack-other"


HTML_TEMPLATE = """<!doctype html>
<html><head><meta charset="utf-8"><title>{title}</title>
<style>
body {{ font-family: ui-monospace, SFMono-Regular, Menlo, monospace; margin: 1rem; background:#fafafa; color:#111; }}
h1 {{ font-size: 1.1rem; margin: 0 0 .5rem 0; }}
.meta {{ color:#555; margin-bottom:.75rem; font-size:.85rem; }}
input#flt {{ width: 100%; padding: .4rem .6rem; font-family: inherit; margin-bottom:.5rem; box-sizing:border-box; }}
table {{ border-collapse: collapse; width: 100%; font-size: .8rem; }}
th, td {{ border-bottom: 1px solid #ddd; padding: 4px 8px; vertical-align: top; text-align: left; white-space: nowrap; }}
th {{ position: sticky; top: 0; background:#eee; z-index:1; }}
td.params {{ white-space: normal; max-width: 50rem; }}
tr.cmd {{ background:#eef5ff; }}
tr.ack-ok {{ background:#e6f7e6; }}
tr.ack-progress {{ background:#fff7d6; }}
tr.ack-bad {{ background:#fde2e2; }}
tr.ack-other {{ background:#f0f0f0; }}
.dim {{ color:#666; font-size:.75rem; }}
</style></head><body>
<h1>{title}</h1>
<div class="meta">{meta}</div>
<input id="flt" placeholder="filter rows (substring, e.g. ARM, DENIED, COMPONENT_ARM_DISARM)" />
<table id="t"><thead><tr>
<th>time</th><th>boot µs</th><th>kind</th><th>command</th><th>result</th>
<th>src</th><th>tgt</th><th>params / details</th><th>ext</th><th>conf</th>
</tr></thead><tbody>
{rows}
</tbody></table>
<script>
const inp = document.getElementById('flt');
const rows = document.querySelectorAll('#t tbody tr');
inp.addEventListener('input', () => {{
  const q = inp.value.toLowerCase();
  rows.forEach(r => {{ r.style.display = r.textContent.toLowerCase().includes(q) ? '' : 'none'; }});
}});
</script>
</body></html>
"""


def render_row(e):
    cls = e["result_class"]
    params = " ".join(html.escape(p) for p in e["params"])
    return (
        f'<tr class="{cls}">'
        f'<td>{fmt_time_rel(e["rel"])}</td>'
        f'<td>{e["ts"]}</td>'
        f'<td>{e["kind"]}</td>'
        f'<td>{html.escape(e["name"])} <span class="dim">({e["cmd_id"]})</span></td>'
        f'<td>{html.escape(e["result"])}</td>'
        f'<td>{html.escape(e["src"])}</td>'
        f'<td>{html.escape(e["tgt"])}</td>'
        f'<td class="params">{params}</td>'
        f'<td>{"1" if e["from_external"] else ""}</td>'
        f'<td>{e["confirmation"] or ""}</td>'
        f'</tr>'
    )


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("ulog", help="path to .ulg file")
    ap.add_argument("-o", "--output", help="output HTML path (default: <ulog>_commands.html)")
    args = ap.parse_args()

    ulog_path = args.ulog
    if not os.path.isfile(ulog_path):
        sys.exit(f"not a file: {ulog_path}")

    out = args.output or os.path.splitext(ulog_path)[0] + "_commands.html"

    cmd_id_to_name, result_map, _arm_denied, sub_enums, param_labels = build_enums()

    ulog = ULog(ulog_path, message_name_filter_list=["vehicle_command", "vehicle_command_ack"])
    events = extract_events(ulog, cmd_id_to_name, result_map, sub_enums, param_labels)

    if not events:
        sys.exit("no vehicle_command or vehicle_command_ack messages found in log")

    n_cmd = sum(1 for e in events if e["kind"] == "CMD")
    n_ack = sum(1 for e in events if e["kind"] == "ACK")
    duration_s = (events[-1]["rel"] - events[0]["rel"]) / 1e6

    title = f"Command timeline — {html.escape(os.path.basename(ulog_path))}"
    meta = (f"{n_cmd} commands, {n_ack} acks, span {duration_s:.2f}s. "
            f"start_timestamp={ulog.start_timestamp} µs.")
    rows = "\n".join(render_row(e) for e in events)
    out_html = HTML_TEMPLATE.format(title=title, meta=meta, rows=rows)

    with open(out, "w") as f:
        f.write(out_html)

    print(f"wrote {out} ({n_cmd} cmd, {n_ack} ack)")


if __name__ == "__main__":
    main()
