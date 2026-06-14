# Target-Class Build System Redesign

Working tracker for the target-class build/airframe redesign. Supersedes PR #27539.

## 0. RESUME HERE

Branch `feat/airframe-class-dirs` (this draft PR is a staging/handoff interface — not for
merge yet). **Phases 1 & 2 are implemented, verified, and committed.** A fresh session:

1. Read §3 (locked decisions), §4 (class catalog), §5 (phase log), §6 (migration spec).
2. Sanity-check current state (all should pass, exit 0):
   - `make px4_fmu-v6x_default romfs_gen_files_target` → `build/.../etc/init.d/airframes/`
     is flat with generics only (no product frames, no class subdirs).
   - `make ark_pi6x_copter romfs_gen_files_target` → copter generics only.
   - `make ark_pi6x_rover romfs_gen_files_target` → the 3 rover generics only (no copter).
3. **Next: Phase 3** (scripted 115-board migration). Concrete first steps:
   - Write `Tools/migrate_px4board.py` per §6 (split each `default.px4board` → `base` +
     `<class>`, infer class from enabled controllers; mc+fw+vtol → single `vtol`).
   - Add remaining `boards/common/<class>.px4board` bases — plane, vtol (MC+FW+VTOL
     superset), uuv, spacecraft, airship, cannode, bootloader, sitl, infra (§4).
   - Migrate one family end-to-end first (e.g. `ark/*`), build-verify, then fan out.
   - Defer Makefile/CI grammar + cutover to Phase 4.

NOT done: only ark/pi6x is class-migrated; `default.px4board` still exists everywhere
(legacy merge path intact, so all current targets still build); Makefile/CI/docs grammar
unchanged; products dangle globally (claim per-board as needed). Nothing merged to main.
The unrelated `boards/ark/fpv/extras/ark_fpv_bootloader.bin` working-tree change is
deliberately excluded from this PR.

## 1. Core model

Every buildable target belongs to exactly one **target class**. Classes are peers:

- **Vehicle classes:** `copter`, `plane`, `vtol`, `rover`, `uuv`, `spacecraft`, `airship`
- **System classes:** `cannode`, `bootloader`, `sitl`, `infra`

A target is composed by merging, in order (later overrides earlier):

```
boards/<v>/<b>/base.px4board          # 1. board hardware foundation (NOT buildable)
boards/common/<class>.px4board        # 2. global class base (modules + airframe toggles)
boards/<v>/<b>/<class>.px4board       # 3. board's overrides for that class (often 1 line)
boards/<v>/<b>/<class>.<variant>.px4board  # 4. optional variant delta
```

Filename grammar: `<class>[.<variant>].px4board`. Class always first; variant a free
string. Target name: `<vendor>_<model>_<class>[_<variant>]` (dot → underscore).

- `base.px4board` (renamed from `default.px4board`) holds vehicle-agnostic hardware
  only. No controllers, no airframes. Building `<board>`, `<board>_default`, or
  `<board>_base` is an **error** (no controllers ⇒ unusable binary).

### Airframes are filed by directory, not by header
- Generic airframes live in **`ROMFS/px4fmu_common/init.d/airframes/<class>/`**. The
  directory *is* the class — no `@class` header to remember. A class dir is included
  iff `CONFIG_AIRFRAMES_<class>=y` (set by `common/<class>.px4board`).
- `vtol` is the air superset: `common/vtol.px4board` turns on
  `AIRFRAMES_COPTER + AIRFRAMES_PLANE + AIRFRAMES_VTOL` (one binary flies all air,
  exactly as today). `rover` is one dir for all of diff/ackermann/mecanum.
- **Product/vendor airframes stay in the flat `airframes/` root → never globbed →
  dangling/unwired by construction.** Claimed by moving the file into a board's
  `boards/<v>/<b>/init/airframes/` (globbed for that target). Cross-board duplication ok.
- `srcparser.py` derives the QGC `@class` from the directory (map `uuv`→"Underwater
  Robot", `vtol`→"VTOL", else Title-case). Files keep `@name`/`@type`/`@maintainer`;
  the `@class` line is dropped.

## 2. Current-state data (the why)

`boards/*/*/default.px4board` = **115**. Controller-set distribution:

| count | controllers | migrates to |
|---|---|---|
| 49 | mc + fw + vtol | `vtol` (superset = generalist air) |
| 26 | mc + fw + vtol + uuv | `vtol` + `uuv` |
| 23 | none (romfs=cannode) | `cannode` |
| 12 | mc only | `copter` |
| 4  | mc + fw + vtol + airship | `vtol` + `airship` |
| 1  | all (px4_sitl) | `sitl` |
| 1  | all (radiolink_PIX6) | `vtol` |

- **80/92 flight boards are multi-vehicle today** → single `vtol` target (decision §3.1).
- 12 single-type (mc-only): `ark_pi6x`, `atl_mantis-edu`, `bitcraze_crazyflie{,21}`,
  `diatone_mamba-f405-mk2`, `flywoo_gn-f405`, `holybro_kakutef7`, `omnibus_f4sd`,
  `px4_fmu-v2`, `raspberrypi_pico`, `spracing_h7extreme`, `uvify_core` → `copter`.
- 95 existing variant labels — `rover`×17, `multicopter`×4, `uuv`×3, `spacecraft`×3
  already class-named; rest (`test`, `mavlink-dev`, `encrypted_logs`, …) → `<class>.<variant>`.
- 70 `bootloader`/`canbootloader` → `bootloader` class (special: savedefconfig, no merge).
- `px4_sitl`: all controllers + 12 variant files → `sitl` class + `sitl.<variant>`.

## 3. Decisions

1. **Multi-vehicle policy — LOCKED (A):** generalist boards → single `vtol` target.
2. **Class names — LOCKED:** `copter`/`plane`/`vtol`/`rover`/`uuv`/`spacecraft`/`airship`
   (not `multicopter`/`fixedwing`).
3. **Classification — LOCKED:** directory-based (§1), not `@class` header.
4. **Minor (proceeding, easy to revert):**
   - Generic boundary: keep `4050_generic_250`, `2507_cloudship`, `70000_atmos`,
     `18001_TF-B1` as generic. Autogyro has no generic frame → `autogyro/` ships none;
     `17002/17003` stay dangling.
   - Per-class inclusion via `file(GLOB)` of the class dir (drop-in works, no list to edit).

## 4. Target-class catalog

| class | `common/<class>.px4board` enables | airframe dirs pulled (Tier 1) |
|---|---|---|
| copter | MC_RATE_CONTROL, CONTROL_ALLOCATOR | `copter/` |
| plane | FW_RATE_CONTROL | `plane/` |
| vtol | MC + FW + VTOL_ATT_CONTROL (superset) | `copter/` + `plane/` + `vtol/` |
| rover | ROVER_DIFFERENTIAL + ACKERMANN + MECANUM | `rover/` |
| uuv | UUV_ATT_CONTROL | `uuv/` |
| spacecraft | SPACECRAFT | `spacecraft/` |
| airship | AIRSHIP_ATT_CONTROL | `airship/` |
| cannode | cannode romfs, uavcannode drivers | none |
| bootloader | bl defconfig path | none |
| sitl | all controllers (runtime-selected) | `sitl/` + all (exempt from prune) |
| infra | inherits a vehicle class + CI tweak | per base class |

**Tier-1 generics → move into `airframes/<class>/`:**
- `copter/`: 4001 quad_x, 5001 quad_+, 6001/7001 hexa, 8001/9001 octo, 11001/12001 cox,
  24001 dodeca, 14001 mc_tilt, 16001 heli, 4050 generic_250
- `plane/`: 2100 standard_plane, 3000 generic_wing
- `vtol/`: 13000/13030/13100/13200 generic_vtol_*
- `rover/`: 50000/51000/52000 generic_rover_*
- `uuv/`: 60000 uuv_generic · `spacecraft/`: 70000 atmos · `airship/`: 2507 cloudship
- `balloon/`: 18001 TF-B1 · `sitl/`: 1001/1002/1100–1105 *.hil

**Dangling (~37, stay in flat root, unwired):** `holybro_*`, `nxp_*`, `crazyflie21`,
`albatross`, `clover4`, `dexi_5`, `beta75x`, `qav250`, `kopis2`, `ifo*`, `x500_v2`,
`s500`, `hovergames`, `mantis_edu`, `draco_r`, `aion_*`, `hiwonder_*`, `axial_*`,
`bluerov2`, `hippocampus`, `TF-AutoG2/G2`.

## 5. Phases (each keeps `main` green)

### Phase 1 — Airframe mechanism — DONE (verified), branch `feat/airframe-class-dirs`, uncommitted
Verified: (a) generated airframes.xml + rc.autostart byte-identical to baseline (metadata
neutral); (b) real `px4_fmu-v6x_default` ROMFS build clean — 19 generics included, runtime
tree flat (flatten works), all product frames excluded, autostart wired. Intentional
behavior change: product/vendor frames no longer shipped (dangle until board-claimed).
Also verified `px4_sitl_default` clean (exit 0) — shared srcparser/Kconfig changes
regression-free; init.d-posix untouched, init.d/airframes flattened correctly.
- `ROMFS/Kconfig` (new): `CONFIG_AIRFRAMES_<class>`, each `default y if MODULES_<ctrl>`
  (sim → `default y if MODULES_SIMULATION_PWM_OUT_SIM`). Source from top-level `Kconfig`.
- Move Tier-1 generics into `airframes/<class>/` (git mv, preserve history); drop their
  `@class` line; products stay in flat root.
- `airframes/CMakeLists.txt`: `if(CONFIG_AIRFRAMES_<class>) px4_add_romfs_glob(<class>/)`.
- `Tools/px4airframes/srcparser.py`: derive class from parent dir; class display-name map.
- `ROMFS/CMakeLists.txt`: glob `${PX4_BOARD_DIR}/init/airframes/*` into generated tree;
  retire the `rc.board_airframes` whitelist path.
- **Verify:** diff generated `etc/init.d/airframes/` set + `rc.autostart` + `airframes.xml`
  before/after for `px4_sitl`, `px4_fmu-v6x`, `ark_pi6x` (+rover). Supersedes #27539.

### Phase 2 — N-overlay merge + class bases + pilot — IN PROGRESS (config-verified)
- `merge_config.py`: now N-ary; base (first fragment) loaded verbatim, every overlay
  after it gets the unset-revert → existing default+label 2-fragment path unchanged.
- `kconfig.cmake`: class label (leading dot-component) → if `base.px4board` +
  `boards/common/<class>.px4board` exist, merge `base → common/<class> →
  [board <class> overlay if variant] → resolved label`; else legacy default+label.
- `boards/common/{copter,rover}.px4board` (controllers only; airframes follow Kconfig
  defaults). Pilot ark/pi6x: `base.px4board` (= default − MC controllers), empty
  `copter.px4board`, slim `rover.px4board` (just roboclaw + hiwonder_emm drivers).
- VERIFIED end-to-end: standalone merge (copter modules == pi6x default; rover composes
  with zero copter controllers, AIRFRAMES_ROVER=y only) AND real ROMFS builds (exit 0) —
  `ark_pi6x_copter` ships copter generics, `ark_pi6x_rover` ships only the 3 rover
  generics, no cross-contamination. #27539's bug solved structurally.
- Remaining: other `common/<class>` bases land with Phase 3 migration; `default.px4board`
  kept during transition (legacy labels still use it).

### Phase 3 — Migrate boards (scripted, per family)
- `Tools/migrate_px4board.py` splits each `default` → `base` + `<class>` (policy A).
- Relabel variants → `<class>.<variant>`; formalize cannode/bootloader/sitl/infra.
- Update Makefile + CI enumeration; both paths valid during transition.

### Phase 4 — Cutover
- `px4_config.cmake`: drop "omit default" rule; error on base/default/bare.
- `Makefile`: rework target lists; `all:` → a sitl class. Sweep workflows/Tools/docs (§7).

## 6. Migration script spec (`Tools/migrate_px4board.py`)
Input each `default.px4board`; infer class from controllers (mc+fw+vtol → single `vtol`;
romfs=cannode → `cannode`); emit `base.px4board` (minus controller `MODULES_*`/whitelist)
+ `<class>.px4board` (ideally just inherits `common/<class>`); per-board diff report; never
silently drop config.

## 7. Ripple checklist
**Grammar:** `Makefile` (ALL_CONFIG_TARGETS, `_default` omission, `all: px4_sitl_default`),
`cmake/px4_config.cmake`, `cmake/kconfig.cmake`.
**Tools:** `kconfig/{merge_config,updateconfig}.py`, `ci/generate_board_targets_json.py`,
`ci/build_all_config.yml` (`excluded_labels`, `special_labels`), `ci/metadata_sync.sh`,
`ci/package_build_artifacts.sh`, `ci/generate_sbom.py`, `ci/run-clang-tidy-pr.py`,
`px4_uploader.py`, `simulation/*/sitl_multiple_run.sh`, `simulation/gz/tests/test_model.py`,
`packaging/Dockerfile.gazebo`.
**Workflows (~15):** `.github/workflows/{compile_ubuntu,compile_macos,sitl_tests,clang-tidy,
docs-orchestrator,ros_integration_tests}.yml`, `.github/actions/{build-deb,build-gazebo-sitl}`.
**Docs:** `.github/instructions/system.instructions.md`, user-guide build refs. **Close #27539.**

## 8. Risks
- `vtol`-as-superset naming wart. · Flash unchanged under (A); narrow classes later for savings.
- `file(GLOB)` won't re-cmake on new airframe until reconfigure (matches existing extras).
- Transition: dual grammar (default + class) coexists through Phase 3.
