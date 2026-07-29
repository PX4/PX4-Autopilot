# get_mode_requirements

## Purpose

`get_mode_requirements.py` parses the PX4 flight mode requirements defined in
`src/modules/commander/ModeUtil/mode_requirements.cpp` and generates Markdown documentation.

It produces two kinds of output under `docs/en/`:

- `flight_modes/mode_requirements.md` — a consolidated overview table for all vehicle types.
- Per-mode requirement text injected directly into individual flight-mode pages using include markers.

## Running

From the repo root:

```sh
python3 docs/scripts/get_mode_requirements/get_mode_requirements.py
```

Or from the `docs/` directory:

```sh
python3 scripts/get_mode_requirements/get_mode_requirements.py
```

## Inline Injection

The script locates each parent flight-mode doc (e.g. `flight_modes_fw/manual.md`) by scanning
`docs/en/flight_modes/`, `docs/en/flight_modes_fw/`, `docs/en/flight_modes_mc/`, and
`docs/en/advanced_features/` for a VitePress `<!--@include:-->` directive (or an existing
`AUTO-GENERATED` sentinel block) that references the corresponding snippet name
(`build_snippet_to_parent_map()`). This directory scan is what actually determines where a
mode's requirements get injected — it is **independent** of the `doc` field in
`mode_nav_state_defns.json`, which only controls headings/links in `mode_requirements.md`. A
mode can have a `doc` pointing at a page that isn't scanned (or has no injection marker on it
yet) and still trigger a "not mapped to a doc" warning; fix this by adding an
`<!-- AUTO-GENERATED: snippet_stem --><!-- END AUTO-GENERATED: snippet_stem -->` marker pair
(content between them can be empty — the script fills it in) to the actual doc page, and if
that page lives outside the four scanned directories, add its directory to `search_dirs` in
`build_snippet_to_parent_map()`.

A single doc page can contain multiple injection points for different nav-states (e.g.
`flight_modes/offboard.md` has one per vehicle type; `flight_modes_mc/manual_stabilized.md`
has one for `NAVIGATION_STATE_STAB` and one for `NAVIGATION_STATE_MANUAL`, since those are
distinct nav-states with different requirements that happen to share one doc page). Each
generated block always starts its own `### Mode Requirements` heading, so when adding a second
block to the same page, add distinguishing surrounding text/labels manually to avoid duplicate
headings, then hand-write the marker pair; the script replaces the content and heading between
the markers.

It replaces that directive with the rendered content, wrapped in sentinel HTML comments:

```markdown
<!-- AUTO-GENERATED: mode_requirements_fixed_wing_manual -->
### Mode Requirements

The following requirements must be met to arm in this mode, or to switch to this mode when it is armed.

- [`mode_req_manual_control`](xxx) - Requires stick input

<!-- END AUTO-GENERATED: mode_requirements_fixed_wing_manual -->
```

On subsequent runs the sentinels are detected and the content between them is updated in-place,
so the script is safe to re-run whenever `mode_requirements.cpp` changes.

**Do not edit the text between the sentinel comments manually** — it will be overwritten on the
next run.  Make changes in `requirement_defns.json` or in `mode_requirements.cpp`.

## Path Calculation

The script resolves the repository root at runtime using `Path(__file__).resolve()` and
navigating **three** parent directories up:

```
docs/scripts/get_mode_requirements/get_mode_requirements.py
  -> docs/scripts/get_mode_requirements/   (parent)
  -> docs/scripts/                         (parent.parent)
  -> docs/                                 (parent.parent.parent)
  -> PX4-Autopilot/                        (parent.parent.parent.parent)  ← repo root
```

If this script is ever moved, update the `repo_root` line accordingly.

## Adding or Changing Requirement Definitions

`requirement_defns.json` (in the same directory as the script) maps each `mode_req_*` flag
name to a human-readable `text`, an example `sensor`, and an extended `detail` string.  Add
new entries here when new requirement flags are introduced in `mode_requirements.cpp`.

If the script encounters a flag in `mode_requirements.cpp` that has no entry in
`requirement_defns.json`, it prints a `WARNING:` line to stderr and continues with empty
`text`, `sensor`, and `detail` strings.  If the JSON file is missing or malformed the script
exits immediately with a clear `ERROR:` message.

## Nav-State / Vehicle-Type Metadata

`mode_nav_state_defns.json` (in the same directory as the script) maps every
`(VEHICLE_TYPE_*, NAVIGATION_STATE_*)` pair to display metadata used when generating
`mode_requirements.md` section headings and controlling per-mode warning behaviour.

Each entry may contain:

| Key | Type | Default | Meaning |
|-----|------|---------|---------|
| `label` | string | — | Human-readable mode name (e.g. `"Mission Mode"`) |
| `doc` | string | — | Relative doc path used as a heading hyperlink (e.g. `"../flight_modes_fw/mission.md"`) |
| `warn` | bool | `true` | Set to `false` to suppress warnings for this mode |
| `status` | string | — | Short implementation-status note, used only when `doc` is absent (see below) |

**Heading generation** (in `mode_requirements.md`):
A mode is only given a full heading + requirement list in the main per-vehicle-type list if it
has a `doc`. Modes with no `doc` are **not** given a bare heading in the main list — instead
they are collected into a trailing `### Modes Without a Dedicated Page` summary for that
vehicle type, rendered as `- **NAV_STATE** — {status}`. Use `status` to explain *why* there's
no dedicated page — e.g. the mode isn't implemented for this vehicle type, or it behaves
identically to another mode already documented, or it's an internal/failsafe state that isn't
user-selectable. If a no-doc mode has no `status` text, the script prints a `WARNING:` to
stderr and falls back to "Not currently documented."

**Skipping snippet injection:** if an entry has `"warn": false` and no `"doc"` key, the script
skips snippet processing for that mode without printing a warning.  Use this for modes that
are not real operational modes (e.g. `TERMINATION`) or that deliberately have no doc page.
(This is orthogonal to `status`: `warn`/`doc` control snippet-injection behaviour, while
`status` controls what's shown in the `mode_requirements.md` summary.)

If the script encounters a `(vehicle_type, nav_state)` pair that has no entry at all in
`mode_nav_state_defns.json`, it prints a `WARNING:` line to stderr.  Add a new entry whenever
a new navigation state is introduced in `mode_requirements.cpp`.

## Parser design

`parse_requirements()` uses a single-pass token scanner.  A single compiled regex (`_TOKEN`)
identifies all meaningful tokens in the `getModeRequirements()` body in left-to-right order:
single-line comments (skipped), `if (vehicle_type == X) {` (push condition), `} else {` (flip
condition), `}` (pop), `{` (push None for non-conditional blocks), and `setRequirement(…)`
calls.  A condition stack tracks the innermost active vehicle-type condition; requirements are
assigned to the appropriate vehicle-type set based on whether we are inside an if-branch, an
else-branch, or unconditional code.

This replaces an earlier two-pass regex approach that was fragile: its non-greedy `[\s\S]*?`
pattern would span across unrelated blocks when a standalone `if` appeared before an `if-else`,
producing silently wrong output for several modes.

## Tests

The test suite lives in `tests/` and is a pytest regression suite that guards against
unintended changes to the parser output.  It compares the live output of the parser against
golden JSON snapshots:

```
tests/
  __init__.py
  test_get_mode_requirements.py   ← the test module
  test_data/
    requirement_defns_golden.json ← expected requirement_defns contents
    vehicle_modes_golden.json     ← expected parse_requirements() output
```

- `test_data/requirement_defns_golden.json` — expected contents of the `requirement_defns`
  dict (flag name → text / sensor / detail).
- `test_data/vehicle_modes_golden.json` — expected output of `parse_requirements()` applied
  to the checked-in `mode_requirements.cpp`: a per-vehicle-type, per-mode mapping of sorted
  requirement flag names.

`conftest.py` at the package root adds this directory to `sys.path` so pytest can import
`get_mode_requirements` without any install step.

The suite contains three tests:

| Test | What it checks |
|---|---|
| `test_requirement_defns_matches_golden` | `requirement_defns` exactly matches the golden snapshot |
| `test_vehicle_modes_matches_golden` | `parse_requirements()` output matches the golden snapshot; prints a per-mode diff on failure |
| `test_all_requirement_flags_are_defined` | Every flag found in `mode_requirements.cpp` has an entry in `requirement_defns` |

### Running the tests

From this directory (`get_mode_requirements/`):

```sh
pytest tests/ -v
```

### When to run and update

**Run the tests** whenever you change:
- the parser logic in `get_mode_requirements.py`, or
- `src/modules/commander/ModeUtil/mode_requirements.cpp`.

**Update the golden files** after any intentional change to either of the above so that the
snapshots stay in sync with the new expected output:

```sh
python tests/test_get_mode_requirements.py --update-golden
```

Then re-run the tests to confirm they pass, and commit the updated golden files alongside the
parser or C++ changes.
