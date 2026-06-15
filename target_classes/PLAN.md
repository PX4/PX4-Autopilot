# Target-Class Build System Redesign

Working tracker for the target-class build/airframe redesign. Supersedes PR #27539.

## 0. RESUME HERE

Branch `feat/airframe-class-dirs` — draft PR #27667 (staging/handoff interface, not for
merge). **Phases 1 & 2 done; Phase 3 migration done for 109 boards — 102 vehicle + 2 cannode
(esp32, mr-canhubk3) + 5 Linux SBC flight controllers. Remaining: 6 special boards, all
deferred to Phase 4 (cutover).** The change is purely additive — `default.px4board` is kept
everywhere, so every legacy `<board>_default` still builds next to the new `<board>_<class>`
targets.

A fresh session: read §3 (locked decisions), §4 (class catalog), §5 (phase log), §6 (script
spec), §7 (Phase-4 ripple list).

### State check (all exit 0)
- `python3 Tools/migrate_px4board.py` (dry run, writes nothing) → `0 ok, 109 skip,
  6 manual, 0 error` — everything migrated; the 6 manual = the Phase-4 specials below.
- `make px4_fmu-v6x_vtol romfs_gen_files_target` → `boardconfig` byte-identical to
  `px4_fmu-v6x_default` except the `# Label:` comment.
- `make emlid_navio2_linux romfs_gen_files_target` → `boardconfig` byte-identical to
  `emlid_navio2_default` except the `# Label:` comment (all 5 Linux boards verified).
- `make cubepilot_cubeyellow_vtol …` and `… _uuv` → their enabled-symbol sets UNION to
  `cubepilot_cubeyellow_default` (multi-vehicle split: nothing lost, nothing added).

### Done (Phase 3 — vehicle + cannode + Linux boards)
- `Tools/migrate_px4board.py`: splits `default.px4board` → vehicle-agnostic `base.px4board`
  + per-class `<class>.px4board` overlays inheriting `target_classes/<class>`. Single-class
  → one target; multi-vehicle → one target per class (vtol + uuv …) sharing one base, whose
  UNION reproduces `default`. Self-verifies symbol-for-symbol before writing; refuses to
  guess specials. The pre-write gate tolerates any class-base symbol a board reverts off
  (controllers AND e.g. cannode `DRIVERS_UAVCANNODE`; all Kconfig default-n).
- Class bases in top-level `target_classes/` (out of `boards/`, vendors-only): copter, plane,
  vtol, rover, uuv, airship, spacecraft, cannode, **linux**. cannode base provides
  `DRIVERS_UAVCANNODE`; `linux` = `PLATFORM_POSIX + BOARD_LINUX_TARGET` + the mc+fw+vtol+uuv
  controller superset (one all-vehicle SBC binary, runtime-selected airframe via init.d-posix).
- **102 vehicle boards migrated.** Empty overlays self-document via an "inherits" comment;
  the air boards' pre-existing `rover` variants were slimmed. Build-verified per class
  (copter/vtol/cannode == legacy `_default` modulo `# Label:`; multi-class unions to default;
  flipped rover/uuv/spacecraft variant labels byte-identical). Full per-board build coverage
  is left to CI.
- **2 cannode boards** (`espressif/esp32`, `nxp/mr-canhubk3`) migrated — both omit the
  uavcannode driver the class base force-enables, so each carries a 1-line revert overlay
  (`# CONFIG_DRIVERS_UAVCANNODE is not set`). Both `_cannode` boardconfigs verified ==
  legacy `_default` (modulo Label; mr-canhubk3 also shows the unrelated, build-order
  dependent zenoh topic Kconfig regen). mr-canhubk3's `fmu`/`sysview` vehicle variants
  keep building via the legacy path.
- **5 Linux SBC flight controllers** (`beaglebone/blue`, `bluerobotics/navigator`,
  `emlid/navio2`, `px4/raspberrypi`, `scumaker/pilotpi`) migrated to the `linux` class. The
  first four match the class controller set exactly (empty overlay); pilotpi reverts the two
  UUV controllers. All five `_linux` boardconfigs verified byte-identical to `_default`
  (modulo Label, arm-linux-gnueabihf). `voxl2` (aarch64 muorb/QURT) is refused — no apps-side
  controllers — and deferred. The aarch64 `_arm64` variants stay on the legacy path.
- Fixed a Phase-1 regression: `flatten_classes.py` crashed every cannode `romfs_gen` (ROMFS
  root with no `airframes/`). `Makefile` no longer enumerates `base.px4board` as a `_base`.

### NOT done (next sessions)
1. **6 special boards still legacy → all deferred to Phase 4** (the tool refuses them):
   - `px4/sitl` (PLATFORM_POSIX, all controllers) → `sitl` class.
   - 3 IO-firmware blobs (`px4/io-v2`, `cubepilot/io-v2`, `modalai/voxl2-io`; NuttX cortex-m3,
     `PX4IOFIRMWARE`, `ROMFSROOT=""`) → `io` class.
   - `px4/ros2` (`PLATFORM_ROS2`) → `ros2` class.
   - `modalai/voxl2` (aarch64 muorb/QURT; controllers run on the SLPI DSP, apps side carries
     none) → special; its `slpi` variant is the controller-bearing half.
   Taxonomy LOCKED (dakejahl 2026-06-14): distinct `sitl`/`io`/`ros2` classes, NOT one
   `infra` catch-all. These are singletons/blobs with no cross-board sharing and (sitl) deep
   CI entanglement, so they are best classed during the cutover when grammar/CI/naming change
   together — not migrated additively now (which would only add awkward duplicate targets like
   `px4_sitl_sitl`).
2. **Phase 4 — cutover (big, CI-facing; do deliberately, §7):** error on base/default/bare
   (`cmake/px4_config.cmake`), Makefile target-list rework + `all:`, relabel pre-redesign
   variants → `<class>.<variant>`, ~15 workflows, Tools scripts, docs. `default.px4board`
   stays until this lands.

### Housekeeping
- The unrelated `boards/ark/fpv/extras/ark_fpv_bootloader.bin` working-tree change is
  deliberately excluded — do NOT commit it.
- Harmless transitional warts: `<vendor>/cannode` → `ark_cannode_cannode`; a few
  pre-redesign variant overlays still carry now-redundant `=n` lines (byte-identical,
  slimmable later).

## 1. Core model

Every buildable target belongs to exactly one **target class**. Classes are peers:

- **Vehicle classes:** `copter`, `plane`, `vtol`, `rover`, `uuv`, `spacecraft`, `airship`
- **System classes:** `cannode`, `bootloader`, `sitl`, `infra`

A target is composed by merging, in order (later overrides earlier):

```
boards/<v>/<b>/base.px4board          # 1. board hardware foundation (NOT buildable)
target_classes/<class>.px4board        # 2. global class base (modules + airframe toggles)
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
  iff `CONFIG_AIRFRAMES_<class>=y` (set by `target_classes/<class>.px4board`).
- `vtol` is the air superset: `target_classes/vtol.px4board` turns on
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

1. **Multi-vehicle policy — LOCKED (A):** a generalist air board (mc+fw+vtol) → single
   `vtol` target. A board that carries an EXTRA vehicle class on top of the air stack
   (vtol+uuv, vtol+airship, vtol+rover) → **one target PER class** (e.g. `<b>_vtol` AND
   `<b>_uuv`), each an empty-overlay board claiming its class base. Preserves all
   capability; no superset binary, no silent drop. (Decided 2026-06-14, dakejahl.)
2. **Class names — LOCKED:** `copter`/`plane`/`vtol`/`rover`/`uuv`/`spacecraft`/`airship`
   (not `multicopter`/`fixedwing`).
3. **Classification — LOCKED:** directory-based (§1), not `@class` header.
   Class bases live in top-level **`target_classes/`** (NOT `boards/`, whose children are
   all vendors). cannode base also provides `CONFIG_DRIVERS_UAVCANNODE=y` (boards that
   lack it — `espressif/esp32`, `nxp/mr-canhubk3` — revert it in their overlay).
4. **Minor (proceeding, easy to revert):**
   - Generic boundary: keep `4050_generic_250`, `2507_cloudship`, `70000_atmos`,
     `18001_TF-B1` as generic. Autogyro has no generic frame → `autogyro/` ships none;
     `17002/17003` stay dangling.
   - Per-class inclusion via `file(GLOB)` of the class dir (drop-in works, no list to edit).
5. **Linux / special-board taxonomy — LOCKED (dakejahl 2026-06-14):**
   - Linux SBC flight controllers (`PLATFORM_POSIX + BOARD_LINUX_TARGET` with vehicle
     controllers) → one shared **`linux`** class: an all-vehicle binary that runtime-selects
     its airframe via init.d-posix (the SITL model, cross-compiled). Done now — the one
     special group with real cross-board sharing.
   - The non-vehicle specials get **distinct** classes, not an `infra` catch-all:
     `sitl` (simulator), `io` (PX4IO coprocessor firmware), `ros2` (ROS2 platform build).
     Deferred to Phase 4 (singletons/blobs, sitl is CI-entangled — class them when the
     grammar changes, not additively now).
   - `modalai/voxl2` is special (muorb/QURT; vehicle controllers live in its `slpi` variant,
     not the apps-side default) → handled separately in Phase 4, not forced into `linux`.

## 4. Target-class catalog

| class | `target_classes/<class>.px4board` enables | airframe dirs pulled (Tier 1) |
|---|---|---|
| copter | MC_RATE_CONTROL, CONTROL_ALLOCATOR | `copter/` |
| plane | FW_RATE_CONTROL | `plane/` |
| vtol | MC + FW + VTOL_ATT_CONTROL (superset) | `copter/` + `plane/` + `vtol/` |
| rover | ROVER_DIFFERENTIAL + ACKERMANN + MECANUM | `rover/` |
| uuv | UUV_ATT_CONTROL | `uuv/` |
| spacecraft | SPACECRAFT | `spacecraft/` |
| airship | AIRSHIP_ATT_CONTROL | `airship/` |
| cannode | cannode romfs, uavcannode drivers | none |
| linux | POSIX + LINUX_TARGET + mc/fw/vtol/uuv superset (DONE) | none (init.d-posix) |
| bootloader | bl defconfig path | none |
| sitl | all controllers, runtime-selected (Phase 4) | `sitl/` + all (exempt from prune) |
| io | PX4IOFIRMWARE coprocessor blob (Phase 4) | none |
| ros2 | PLATFORM_ROS2 build (Phase 4) | none |

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

### Phase 1 — Airframe mechanism — DONE (verified + committed)
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

### Phase 2 — N-overlay merge + class bases + pilot — DONE (committed)
- `merge_config.py`: now N-ary; base (first fragment) loaded verbatim, every overlay
  after it gets the unset-revert → existing default+label 2-fragment path unchanged.
- `kconfig.cmake`: class label (leading dot-component) → if `base.px4board` +
  `target_classes/<class>.px4board` exist, merge `base → target_classes/<class> →
  [board <class> overlay if variant] → resolved label`; else legacy default+label.
- `target_classes/{copter,rover}.px4board` (controllers only; airframes follow Kconfig
  defaults). Pilot ark/pi6x: `base.px4board` (= default − MC controllers), empty
  `copter.px4board`, slim `rover.px4board` (just roboclaw + hiwonder_emm drivers).
- VERIFIED end-to-end: standalone merge (copter modules == pi6x default; rover composes
  with zero copter controllers, AIRFRAMES_ROVER=y only) AND real ROMFS builds (exit 0) —
  `ark_pi6x_copter` ships copter generics, `ark_pi6x_rover` ships only the 3 rover
  generics, no cross-contamination. #27539's bug solved structurally.
- `default.px4board` kept during transition (legacy labels still use it).

### Phase 3 — Migrate boards (scripted) — DONE for 109 boards (6 specials + cutover remain, see §0)
- `Tools/migrate_px4board.py` — DONE. Splits each `default` → `base` (= default minus the
  symbols `target_classes/<class>` provides minus the board's controllers) + `<class>` overlay
  (the board's delta vs `target_classes/<class>`; empty when the board matches the class base).
  Class(es) inferred from controllers (+ `ROMFSROOT="cannode"`); air collapses to `vtol`
  (policy A), each extra vehicle class becomes its own target (multi-class → shared base +
  N overlays, UNION-verified). Before writing it self-verifies that the merge reproduces the
  original `default` symbol-for-symbol (only tolerated delta: an omitted controller the
  overlay pins off — all controllers are `default n`). Refuses POSIX/no-controller specials.
- `target_classes/{vtol,plane,uuv,airship,spacecraft,cannode,linux}.px4board` — DONE (added
  to the `copter,rover` from Phase 2). `vtol` = the mc5+fw5+VTOL modal set (38/47 boards match
  exactly). `cannode` = just `BOARD_ROMFSROOT="cannode"`. `linux` = POSIX + LINUX_TARGET +
  mc/fw/vtol/uuv superset. (sitl/io/ros2/bootloader deferred — special; bootloader stays on
  the savedefconfig path, never merged.)
- **All 102 vehicle boards migrated** — DONE. ark/* (single-class) + `cubepilot/cubeyellow`
  (multi-class) as pilots, then the rest in one additive sweep. Spot build-verified per class
  (== legacy `_default` modulo `# Label:`; multi-class unions to default; flipped variant
  labels byte-identical). The air boards' pre-existing `rover` overlays were slimmed.
- Fixed `Tools/px4airframes/flatten_classes.py` crash when the ROMFS root has no
  `airframes/` dir (a Phase-1 regression that broke every cannode `romfs_gen_files_target`).
- `Makefile`: `base.px4board` excluded from `ALL_CONFIG_TARGETS` (no spurious `_base` target).
- 2 cannode boards (esp32, mr-canhubk3) migrated with a `DRIVERS_UAVCANNODE` revert overlay;
  `target_classes/linux.px4board` added and 5 Linux SBC flight controllers migrated (voxl2
  refused — muorb/QURT). All build-verified == legacy `_default`.
- TODO (next): the 6 specials (sitl/io/ros2 classes + voxl2, all deferred to Phase 4, §0);
  relabel pre-redesign variants → `<class>.<variant>`; then Phase 4 cutover.

### Phase 4 — Cutover
- `px4_config.cmake`: drop "omit default" rule; error on base/default/bare.
- `Makefile`: rework target lists; `all:` → a sitl class. Sweep workflows/Tools/docs (§7).

## 6. Migration script (`Tools/migrate_px4board.py`) — IMPLEMENTED
Per `default.px4board`: infer the class(es) — air (mc/fw/vtol) collapses to `vtol`, each
extra vehicle class is its own target, `ROMFSROOT="cannode"` → `cannode`,
`PLATFORM_POSIX + BOARD_LINUX_TARGET` with vehicle controllers → `linux`. Emit one
`base.px4board` (= default minus ALL controllers and every symbol the class bases provide)
+ one `<class>.px4board` overlay per class (the board's delta vs `target_classes/<class>`;
self-documenting comment when empty). Self-verifies the layered merge reproduces `default`
symbol-for-symbol (UNION of the per-class targets for multi-class) before writing, tolerating
any class-base symbol the board reverts off (controllers AND e.g. cannode `DRIVERS_UAVCANNODE`;
all `default n`). Refuses (MANUAL, never writes) plain-POSIX SITL, muorb/QURT Linux boards
with no apps-side controllers (voxl2), no-controller blobs (IO firmware, ros2), and class-base
mismatches.
`--apply` writes; default is a dry-run report; `--force` overwrites an existing base.
Usage: `Tools/migrate_px4board.py [--apply] [--force] [vendor/model ...]`.

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
