#!/usr/bin/env python3
"""
Parses mode requirements in
https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/ModeUtil/mode_requirements.cpp
and uses the result to generate Markdown documentation.

Mode requirements are injected directly into parent flight-mode docs, wrapped in sentinel
comments so subsequent runs update the content in-place.
"""
import re
import argparse
import sys
from pathlib import Path
import json

# ---------------------------------------------------------------------------
# Regex patterns for inline-injection mode
# ---------------------------------------------------------------------------

# Matches a VitePress include directive pointing at a mode_requirements snippet.
# Captures the snippet stem (e.g. mode_requirements_fixed_wing_manual).
_INCLUDE_RE = re.compile(
    r'<!--@include:\s*\.\./flight_modes_(?:fw|mc)/(mode_requirements_\w+)\.md-->'
)

# Matches an existing sentinel block for a given stem (built dynamically per stem).
def _sentinel_re(stem: str) -> re.Pattern:
    return re.compile(
        r'<!-- AUTO-GENERATED: ' + re.escape(stem) + r' -->.*?'
        r'<!-- END AUTO-GENERATED: ' + re.escape(stem) + r' -->',
        re.DOTALL,
    )

# Load requirement definitions from the JSON file alongside this script.
_DEFNS_PATH = Path(__file__).resolve().parent / "requirement_defns.json"
try:
    with _DEFNS_PATH.open() as _f:
        requirement_defns: dict = json.load(_f)
except FileNotFoundError:
    sys.exit(f"ERROR: requirement definitions file not found: {_DEFNS_PATH}")
except json.JSONDecodeError as e:
    sys.exit(f"ERROR: requirement definitions file is not valid JSON ({_DEFNS_PATH}): {e}")

# Load nav-state / vehicle-type metadata from the JSON file alongside this script.
_NAV_STATE_DEFNS_PATH = Path(__file__).resolve().parent / "mode_nav_state_defns.json"
try:
    with _NAV_STATE_DEFNS_PATH.open() as _f:
        mode_nav_state_defns: dict = json.load(_f)
except FileNotFoundError:
    sys.exit(f"ERROR: mode nav state definitions file not found: {_NAV_STATE_DEFNS_PATH}")
except json.JSONDecodeError as e:
    sys.exit(f"ERROR: mode nav state definitions file is not valid JSON ({_NAV_STATE_DEFNS_PATH}): {e}")


def _get_nav_state_def(vehicle_type: str, nav_state: str) -> dict:
    """Return the nav-state definition for *(vehicle_type, nav_state)*.

    Emits a WARNING to stderr if the pair has no entry in mode_nav_state_defns.json.
    Returns an empty dict for unknown entries so callers can use .get() safely.
    """
    vt_map = mode_nav_state_defns.get(vehicle_type)
    if vt_map is None:
        print(
            f"WARNING: vehicle type '{vehicle_type}' has no entries in "
            "mode_nav_state_defns.json.",
            file=sys.stderr,
        )
        return {}
    entry = vt_map.get(nav_state)
    if entry is None:
        print(
            f"WARNING: no entry in mode_nav_state_defns.json for "
            f"({vehicle_type}, {nav_state}).",
            file=sys.stderr,
        )
        return {}
    return entry


def _get_req_def(flag: str) -> dict:
    """Return the requirement definition for *flag*, warning if it is unknown."""
    if flag not in requirement_defns:
        print(
            f"WARNING: requirement '{flag}' found in mode_requirements.cpp "
            "but has no entry in requirement_defns.json — text/detail will be empty.",
            file=sys.stderr,
        )
        return {"text": "", "sensor": "", "detail": ""}
    return requirement_defns[flag]


# 1. Resolve the absolute path of the current script file.
script_path = Path(__file__).resolve()
# 2. Go up three directories from the script's location (docs/scripts/get_mode_requirements/ -> PX4-Autopilot/).
# This finds the repository root.
repo_root = script_path.parent.parent.parent.parent
# 3. Construct the target file path relative to the repository root.
SRC = repo_root / "src/modules/commander/ModeUtil/mode_requirements.cpp"

docs_output_path_base = repo_root / "docs/en/"
docs_output_path_flight_modes = docs_output_path_base / "flight_modes"


# Tokeniser for getModeRequirements() function body.
#
# Walks the body left-to-right and emits one of six token kinds:
#   comment      //…          – ignored
#   if_vtype     if (vehicle_type == X) {   – push condition onto stack
#   else_open    } else {                   – flip top of stack to else-side
#   close        }                          – pop stack
#   open         {                          – push None (non-conditional block)
#   req          setRequirement(nav, flag)  – record requirement
#
# \s* in the req pattern intentionally matches newlines so that multi-line
# setRequirement() calls (e.g. NAVIGATION_STATE_ALTITUDE_CRUISE) are handled.
_TOKEN = re.compile(
    r'(?P<comment>//[^\n]*)'
    r'|(?P<if_vtype>if\s*\(\s*vehicle_type\s*==\s*vehicle_status_s::'
        r'(?P<vtype>VEHICLE_TYPE_\w+)\s*\)\s*\{)'
    r'|(?P<else_open>\}\s*else\s*\{)'
    r'|(?P<close>\})'
    r'|(?P<open>\{)'
    r'|(?P<req>setRequirement\(\s*'
        r'(?P<nav_state>vehicle_status_s::\w+)\s*,\s*'
        r'flags\.(?P<req_flag>\w+)\s*\))'
)


def parse_requirements():
    try:
        text = SRC.read_text()
    except FileNotFoundError:
        print(f"Error: Source file not found at {SRC.resolve()}")
        return {}

    # Extract getModeRequirements() body
    m = re.search(r"getModeRequirements\s*\([^)]*\)\s*\{(.*)\}", text, re.S)
    if not m:
        raise RuntimeError("Could not find getModeRequirements() body")
    body = m.group(1)

    # Discover all vehicle types mentioned in if-conditions
    all_vtypes = set(re.findall(r'vehicle_status_s::(VEHICLE_TYPE_\w+)', body))
    vehicle_modes = {vt: {} for vt in all_vtypes}

    # Single-pass token scan with a condition stack.
    #
    # cond_stack entries:
    #   None              – inside a non-vehicle-type block (or function body)
    #   (vtype, False)    – inside  if (vehicle_type == vtype) { … }
    #   (vtype, True)     – inside  } else { … }  (i.e. NOT vtype)
    cond_stack = []

    for tok in _TOKEN.finditer(body):
        kind = tok.lastgroup

        if kind == 'comment':
            pass  # skip

        elif kind == 'if_vtype':
            cond_stack.append((tok.group('vtype'), False))

        elif kind == 'else_open':
            if cond_stack and cond_stack[-1] is not None:
                # Flip the current vehicle-type condition to its else side
                cond_stack[-1] = (cond_stack[-1][0], True)
            else:
                # else on a non-vehicle-type if – treat as a plain block
                if cond_stack:
                    cond_stack.pop()
                cond_stack.append(None)

        elif kind == 'close':
            if cond_stack:
                cond_stack.pop()

        elif kind == 'open':
            cond_stack.append(None)

        elif kind == 'req':
            mode = tok.group('nav_state').replace('vehicle_status_s::', '')
            req  = tok.group('req_flag')

            # The innermost vehicle-type condition determines which types apply
            active = next((c for c in reversed(cond_stack) if c is not None), None)

            if active is None:
                vtypes = all_vtypes                  # default: all types
            elif not active[1]:
                vtypes = {active[0]}                 # if-branch: only this type
            else:
                vtypes = all_vtypes - {active[0]}    # else-branch: all other types

            for vt in vtypes:
                vehicle_modes[vt].setdefault(mode, set()).add(req)

    return vehicle_modes

def build_snippet_to_parent_map(docs_base: Path) -> dict:
    """Scan flight-mode doc directories and return {snippet_stem: parent_file_path}.

    Recognises both the original VitePress include directive and existing
    AUTO-GENERATED sentinel blocks so that re-runs work correctly.
    """
    mapping = {}
    search_dirs = [
        docs_base / "flight_modes",
        docs_base / "flight_modes_fw",
        docs_base / "flight_modes_mc",
        docs_base / "advanced_features",
    ]
    for search_dir in search_dirs:
        for md_file in sorted(search_dir.glob("*.md")):
            text = md_file.read_text(encoding="utf-8")
            # Check for @include directives
            for m in _INCLUDE_RE.finditer(text):
                stem = m.group(1)
                mapping[stem] = md_file
            # Check for existing sentinel blocks (covers re-runs after first inline)
            for m in re.finditer(r'<!-- AUTO-GENERATED: (mode_requirements_\w+) -->', text):
                stem = m.group(1)
                mapping[stem] = md_file
    return mapping


def inline_requirements_into_doc(parent_path: Path, snippet_stem: str, content: str) -> None:
    """Inject *content* into *parent_path*, replacing an @include directive or
    an existing AUTO-GENERATED sentinel block for *snippet_stem*.

    The injected region is wrapped in sentinel HTML comments so subsequent runs
    can find and replace it without any manual markers remaining in the file.
    """
    sentinel_start = f'<!-- AUTO-GENERATED: {snippet_stem} -->'
    sentinel_end   = f'<!-- END AUTO-GENERATED: {snippet_stem} -->'
    replacement = f'{sentinel_start}\n\n{content}\n{sentinel_end}'

    original = parent_path.read_text(encoding="utf-8")

    # Try existing sentinel block first (re-run case)
    updated, n = _sentinel_re(snippet_stem).subn(replacement, original)
    if n == 0:
        # Try @include directive (first-time case)
        include_pattern = re.compile(
            r'<!--@include:\s*\.\./flight_modes_(?:fw|mc)/' + re.escape(snippet_stem) + r'\.md-->'
        )
        updated, n = include_pattern.subn(replacement, original)

    if n == 0:
        print(
            f"⚠️  Warning: could not update '{parent_path}' for '{snippet_stem}'.\n"
            f"  The doc was found but contains neither an AUTO-GENERATED sentinel block\n"
            f"  nor an @include directive for this snippet.\n"
            f"  Add an injection point manually or remove the stale mapping."
        )
        return

    if updated != original:
        parent_path.write_text(updated, encoding="utf-8")
        print(f"✅ Injected {snippet_stem} into {parent_path}")
    # else: already up to date, no output needed


if __name__ == "__main__":

    _parser = argparse.ArgumentParser(
        description="Generate PX4 flight-mode requirement documentation."
    )
    _parser.add_argument(
        '--suppress_warnings',
        action='store_true',
        help='Suppress warnings about modes not listed in INVALID_MODES, NO_DOC_MODES, '
             'or the doc mapping. Resolve warnings by adding the mode to one of those '
             'lists or by providing a doc page with an injection point.',
    )
    args = _parser.parse_args()

    vehicle_modes = parse_requirements()

    # Convert sets to sorted lists for JSON output - i.e. all reqs are listed in the same order.
    vehicle_modes_sorted = {
        vtype: {mode: sorted(list(reqs)) for mode, reqs in modes.items()}
        for vtype, modes in vehicle_modes.items()
    }

    print(json.dumps(vehicle_modes_sorted, indent=4))

    # Warn about requirements defined in the JSON but absent from the parsed C++ file.
    all_found_flags = {
        req
        for vtype in vehicle_modes_sorted.values()
        for reqs in vtype.values()
        for req in reqs
    }
    for flag in requirement_defns:
        if flag not in all_found_flags:
            print(
                f"WARNING: '{flag}' is defined in requirement_defns.json "
                "but was not found in mode_requirements.cpp.",
                file=sys.stderr,
            )

    requirement_table = "| Requirement | Example |\n| --- | --- |\n"
    for requirement, info in requirement_defns.items():
        sensor_info = f" ({info['sensor']})" if info['sensor'] else ''
        detail = f" <br><br>{info['detail']}" if info['detail'] else ''
        requirement_table += (
            f'| <a id="{requirement}">`{requirement}`</a>'
            f' | {info["text"]}{sensor_info}{detail} |\n'
        )

    mode_requirements_markdown = f"""# Mode Requirements

::: info
This documentation was auto-generated from the source code (see [docs/scripts/get_mode_requirements](https://github.com/PX4/PX4-Autopilot/tree/main/docs/scripts/get_mode_requirements)).
:::

Mode requirements define the set of conditions that must be met in order to arm in a particular flight mode, or to switch to the mode if it is already armed.

Requirements are defined for internal modes in [mode_requirements.cpp](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/ModeUtil/mode_requirements.cpp), and for ROS 2 external modes in [requirement_flags.hpp](https://github.com/Auterion/px4-ros2-interface-lib/blob/main/px4_ros2_cpp/include/px4_ros2/common/requirement_flags.hpp) (Github `Auterion/px4-ros2-interface-lib` repository).
The mode requirements are the same in both cases.

The following sections provide an overview of the requirements and what modes they are used in.

## Requirements Definitions

{requirement_table}
### Naming Conventions

In general requirement flag names are abstracted from specific sensors.
This is done because particular requirements can often be met by several sensors.
For example, GNSS is the most common source of global position, but it isn't the only one.

The requirements include frame and accuracy information hints in their names:

- `global` means an absolute world frame, such as that provided by GNSS.
- `local` means a frame that relative to an initialization point, such as the position of an IMU on boot.
- `relaxed` means that the mode does not require or rely on accurate data: as long as sensors are providing some data the state is considered valid.
  Relaxed conditions are used for modes where some sensor data is considered more important than none at all, such as when calculating position via optical flow velocity measurements.
  By contrast, a position mode that is not relaxed requires reliable sensor data, and will block arming if inaccuracy is detected.

Note that a global position requirement can be met if you have a valid _local position_, by mapping the local frame to a global position.
This can be done by setting the global position of the local origin using the MAVLink message [SET_GPS_GLOBAL_ORIGIN](https://mavlink.io/en/messages/common.html#SET_GPS_GLOBAL_ORIGIN), either directly or via a GCS (see [External Position Estimate > Enabling Auto Modes with a Local Position](../ros/external_position_estimation#enabling-auto-modes-with-a-local-position)).
Similarly, if the vehicle has `mode_req_local_position_relaxed`, then you can map to a global position in order to meet the `global_position_relaxed` requirement.
This allows PX4 automatic flight modes that require a global position to be used locally, such as Mission and Return.

"""



    for vehicle_type in vehicle_modes_sorted:
        if vehicle_type == 'VEHICLE_TYPE_ROTARY_WING':
            mode_requirements_markdown += f"\n## Multicopter ({vehicle_type})\n"
        elif vehicle_type == 'VEHICLE_TYPE_FIXED_WING':
            mode_requirements_markdown += f"\n## Fixed-wing ({vehicle_type})\n"
        else:
            mode_requirements_markdown += f"\n## {vehicle_type}\n"


        undocumented_modes = []

        for flight_mode in vehicle_modes_sorted[vehicle_type]:
            defn  = _get_nav_state_def(vehicle_type, flight_mode)
            label = defn.get("label")
            doc   = defn.get("doc")

            # Modes with no doc page have no meaningful (or no implemented) behaviour to
            # document on this vehicle type: list them in a trailing summary instead of
            # giving them a full heading in the main list.
            if not doc:
                undocumented_modes.append((flight_mode, defn.get("status")))
                continue

            if label:
                mode_heading_markdown = f"\n### [{label}]({doc}) ({flight_mode})\n\n"
            else:
                mode_heading_markdown = f"\n### {flight_mode}\n\n"

            mode_requirements_markdown += mode_heading_markdown

            for requirement in vehicle_modes_sorted[vehicle_type][flight_mode]:
                #mode_requirements_markdown += f"- {requirement}: {requirement_defns[requirement]}\n"
                mode_requirements_markdown += f"- [`{requirement}`](#{requirement})\n"

        if undocumented_modes:
            mode_requirements_markdown += "\n### Modes Without a Dedicated Page\n\n"
            mode_requirements_markdown += (
                "The following internal navigation states have no distinct user-facing "
                "behaviour or documentation page on this frame type:\n\n"
            )
            for flight_mode, status in undocumented_modes:
                if not status:
                    print(
                        f"WARNING: no 'status' text in mode_nav_state_defns.json for "
                        f"({vehicle_type}, {flight_mode}) — it has no doc and will show "
                        "with no explanation in the 'Modes Without a Dedicated Page' summary.",
                        file=sys.stderr,
                    )
                    status = "Not currently documented."
                mode_requirements_markdown += f"- **{flight_mode}** — {status}\n"


    # 2. Define the filename for the overview topic: mode_requirements
    filename = "mode_requirements.md"
    filename = docs_output_path_flight_modes / filename
    try:
        with open(filename, 'w', encoding='utf-8') as f:
            # 'w' stands for 'write' mode, which creates the file or overwrites an existing one.
            f.write(mode_requirements_markdown)
        
        print(f"✅ Successfully wrote content to {filename}")

    except Exception as e:
        print(f"❌ An error occurred: {e}")


    # Generate per-doc mode requirements, injected inline into parent docs

    snippet_to_parent = build_snippet_to_parent_map(docs_output_path_base)

    # Display labels for vehicle types used in shared-doc headings.
    _VT_DISPLAY_LABEL = {
        "VEHICLE_TYPE_FIXED_WING": "Fixed-Wing",
        "VEHICLE_TYPE_ROTARY_WING": "Multicopter",
    }

    for vehicle_type in vehicle_modes_sorted:
        for flight_mode in vehicle_modes_sorted[vehicle_type]:
            vehicle_part = vehicle_type.split("VEHICLE_TYPE_")[-1]
            mode_part = flight_mode.split("NAVIGATION_STATE_")[-1]

            snippet_stem = f"mode_requirements_{vehicle_part}_{mode_part}".lower()

            print(f"Processing: {snippet_stem}")

            # Skip modes that have warn=false and no doc in mode_nav_state_defns.json.
            # This replaces the former INVALID_MODES / NO_DOC_MODES hard-coded sets.
            nav_defn = mode_nav_state_defns.get(vehicle_type, {}).get(flight_mode, {})
            if not nav_defn.get("warn", True) and not nav_defn.get("doc"):
                print(f"  Skipping {snippet_stem} (warn=false, no doc in mode_nav_state_defns.json)")
                continue

            # Resolve parent doc early so we can choose the right heading.
            parent = snippet_to_parent.get(snippet_stem)

            # Use a labeled heading when injecting into a shared doc (flight_modes/)
            # so readers know which vehicle type each section applies to.
            is_shared_doc = parent is not None and parent.parent.name == "flight_modes"
            if is_shared_doc:
                vt_label = _VT_DISPLAY_LABEL.get(vehicle_type, vehicle_type)
                req_heading = f"### Mode Requirements — {vt_label}"
            else:
                req_heading = "### Mode Requirements"

            vehicle_mode_markdown = f"""{req_heading}

The following requirements must be met to arm in this mode, or to switch to this mode when it is armed.

"""

            for requirement in vehicle_modes_sorted[vehicle_type][flight_mode]:
                text = _get_req_def(requirement)['text']
                vehicle_mode_markdown += f"- [`{requirement}`](../flight_modes/mode_requirements.md#{requirement}) — {text}\n"

            if parent:
                inline_requirements_into_doc(parent, snippet_stem, vehicle_mode_markdown)
            else:
                if not args.suppress_warnings and nav_defn.get("warn", True):
                    print(
                        f"⚠️  Missing mode: '{snippet_stem}' is not mapped to a doc.\n"
                        f"  Resolve by:\n"
                        f"    • Adding a doc page with an injection point for this mode\n"
                        f"    • Setting \"warn\": false in mode_nav_state_defns.json for "
                        f"({vehicle_type}, {flight_mode})\n"
                        f"    • Re-running with --suppress_warnings to silence this warning"
                    )






