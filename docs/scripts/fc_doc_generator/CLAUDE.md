# fc_doc_generator

Auto-generates flight controller documentation sections for PX4-Autopilot
from board source files (`boards/<vendor>/<board>/`).

## What it does

Parses PX4 board C/Kconfig source files and generates Markdown sections
inserted into `docs/en/flight_controller/*.md` docs. Sections generated:

- `## Specifications` — processor, sensors, interfaces
- `## PWM Outputs` — timer groups, DShot/BDShot capability per output
- `## Serial` — UART → /dev/ttyS* mapping with labels and flow control
- `## Radio Control` — RC input protocols and ports
- `## GPS & Compass` — GPS/safety connector info
- `## Power` — power input ports and monitor type
- `## Telemetry Radios` — TELEM port listing
- `## SD Card` — presence/absence

## File layout

```
fc_doc_generator/
├── fc_doc_generator.py      # Main script (~3700 lines)
├── pytest.ini               # testpaths = tests
├── metadata/                # Per-board cached JSON (data + wizard overrides)
│   ├── <vendor>_<board>_data.json    # Parsed board data
│   └── <vendor>_<board>_wizard.json  # User-supplied wizard overrides
└── tests/
    ├── conftest.py          # snapshot fixture + board_* path fixtures
    ├── fixtures/            # Minimal fake board trees
    │   ├── stm32h7_all_dshot/
    │   ├── stm32h7_mixed_io/
    │   ├── stm32h7_ppm_shared/
    │   ├── stm32h7_capture_channels/   # 8 regular outputs + 8 initIOTimerChannelCapture inputs (excluded)
    │   ├── stm32f4_no_dshot/
    │   └── imxrt_all_dshot/
    ├── snapshots/           # Expected markdown output (.md files)
    ├── test_parsers.py      # Unit tests for parse_* functions
    ├── test_compute.py      # Unit tests for compute_groups / compute_bdshot
    ├── test_generators.py   # Snapshot tests for generate_*_section functions
    ├── test_helpers.py      # Unit tests for helper functions
    └── test_wizard.py       # Tests for wizard cache and generate_full_template
```

## Running the script

Run from the repo root (requires the PX4 `boards/` tree to be present):

```sh
# Generate metadata JSON + fc_sections.md (no file edits):
python docs/scripts/fc_doc_generator/fc_doc_generator.py

# Apply sections to existing FC docs:
python docs/scripts/fc_doc_generator/fc_doc_generator.py --apply

# Apply a single section only:
python docs/scripts/fc_doc_generator/fc_doc_generator.py --apply --section pwm_outputs

# Apply all sections to a single doc only (stem or filename, implies --apply):
python docs/scripts/fc_doc_generator/fc_doc_generator.py --doc cuav_x25-evo.md

# Apply a single section to a single doc:
python docs/scripts/fc_doc_generator/fc_doc_generator.py --doc cuav_x25-evo.md --section pwm_outputs

# Create a new stub FC doc (interactive wizard):
python docs/scripts/fc_doc_generator/fc_doc_generator.py --new-doc

# Check a single doc against quality specs:
python docs/scripts/fc_doc_generator/fc_doc_generator.py --check-doc docs/en/flight_controller/holybro_kakuteh7.md

# Check all FC docs:
python docs/scripts/fc_doc_generator/fc_doc_generator.py --check-all
```

Via yarn (from the `docs/` directory): `cd docs && yarn gen_fc_sections`

## Running tests

From `docs/scripts/fc_doc_generator/`:

```sh
pytest                        # run all tests
pytest --update-snapshots     # regenerate snapshot files after intentional changes
pytest tests/test_generators.py   # specific test file
```

## Snapshot tests

Tests in `test_generators.py` use the `snapshot` fixture from `conftest.py`.
- Snapshot files live in `tests/snapshots/*.md`
- To add a new snapshot test: call `snapshot("my_name.md", result)` — then run `pytest --update-snapshots` to create the file
- After intentional generator changes: run `pytest --update-snapshots` then review diffs with `git diff tests/snapshots/`

## Extension pattern (adding a new section)

1. Write `parse_<thing>(board_path: Path) -> dict` and call it in `gather_board_data()`, merging into the entry
2. Write `generate_<thing>_section(board_key, entry) -> str`
3. Register both in `SECTION_GENERATORS` and `SECTION_ORDER`
4. Add snapshot tests in `test_generators.py`
5. Re-run `cd docs && yarn gen_fc_sections` to regenerate metadata JSON + `fc_sections.md`

## Key architecture notes

- **Parsers** read from `boards/<vendor>/<board>/` source files:
  - `nuttx-config/nsh/defconfig` — chip family, enabled UARTs, SD card
  - `src/board_config.h` — PWM count, IO board presence, GPIOs
  - `src/timer_config.cpp` — timer groups and channels; `initIOTimerChannelCapture`
    entries are **excluded** from the FMU output count — these are timer-capture
    inputs (e.g. RC PPM, pulse-width measurement), not usable as PWM outputs.
    They are stored separately in `capture_channels` in the metadata JSON for use
    in other documentation sections.
  - `default.px4board` — Kconfig board settings (serial labels, RC, GPS, drivers)
  - `nuttx-config/include/board.h` — flow control GPIO definitions
  - `init/rc.board_sensors` — sensor driver start commands; comments immediately
    preceding a sensor start line are parsed for port labels (e.g.
    `# External compass on GPS1/I2C1:` → `port_label: 'GPS1'` on that sensor entry);
    power monitor drivers (INA226/INA228/INA238) are also captured in
    `sensor_bus_info['power_monitor']`
  - `src/i2c.cpp` — authoritative I2C bus routing:
    `initI2CBusExternal(N)` = external connector; `initI2CBusInternal(N)` = on-board only.
    When present, stored in `i2c_bus_config` and enables the detailed per-bus I2C output.

- **`DSHOT_UNSUPPORTED_FAMILIES`** — constant listing chip families where PX4 firmware
  does not support DShot (currently `stm32f4`, `stm32f7`). Both use DMA IP V1; PX4's
  `dshot.c` skips DMA V1 MCUs entirely. `compute_groups()` suppresses DShot for all
  groups when the board's family is in this set, regardless of DMA presence in
  `timer_config.cpp`.

- **Metadata JSON** in `metadata/` caches parsed data (`*_data.json`) and
  wizard-supplied overrides (`*_wizard.json`). Wizard data persists across runs
  and provides connector types, sensor names, dimensions, etc.

- **`BOARD_TO_DOC`** — static mapping from `vendor/board` key to doc filename.
  Boards mapped to `None` have no existing doc page yet.

- **Section insertion** — `_apply_section()` finds existing headings and
  replaces them, or inserts before anchor headings like `## Where to Buy`.
  - The `specifications` section always preserves hand-written content and
    appends generated content as a `<!-- specifications-proposed -->` comment.
  - All other sections check whether the existing heading contains the
    `{#section_key}` anchor (e.g. `{#pwm_outputs}`). If it does, the section
    was previously generated and is safely replaced. If not, the section is
    hand-written: it is preserved and the proposed content is appended as a
    `<!-- section_key-proposed -->` comment, with a console warning.

- **Wizard** — `--new-doc` runs an interactive prompts session and caches
  answers to `metadata/<stem>_wizard.json` for future re-use.

## Conventions

- British English in doc output
- Asset files lowercase with underscores; asset folder named after doc stem
- Section generators emit embedded `<!-- *-source-data ... -->` JSON comments
  so the raw parsed values are visible in the doc for manual verification
- `TODO:` placeholders are left wherever data cannot be auto-detected

## Development rules

**When modifying `fc_doc_generator.py`:**
1. All existing tests must pass: run `pytest` from `docs/scripts/fc_doc_generator/`
2. New functionality must have new tests (unit tests and/or snapshot tests as appropriate)
3. Update this `CLAUDE.md` if the change affects how to run the script, the architecture, extension patterns, or conventions
4. **Wizard JSON backward compatibility** — `metadata/*_wizard.json` files are persisted user
   data. A new tool version must work correctly with an old wizard JSON:
   - **Adding** a new field to `_WIZARD_CACHE_FIELDS`: safe — missing keys are read via
     `cached.get(...)` which returns `None`, triggering re-prompting.
   - **Renaming** an existing field or **changing its structure** (e.g. the shape of
     `i2c_buses_wizard` entries): **breaking change** — old data is silently lost or
     misinterpreted. This must be explicitly flagged in the plan and requires a migration
     strategy (e.g. read both old and new key names, or add a one-time upgrade step).
