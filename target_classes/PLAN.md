# Target-Class Build System Redesign

Working tracker for the target-class build/airframe redesign. Supersedes PR #27539.

## 0. RESUME HERE

Branch `feat/airframe-class-dirs` — draft PR #27667 (staging/handoff interface, not for
merge); pushed to fork `jake` (`dakejahl/PX4-Autopilot`, NOT `origin`=upstream). `git log` for
the head.

**Status (2026-06-15) — PHASE-4 CUTOVER LANDED (commit `f914c21643`) + BOARD-CONFIG MODEL
RE-DOCUMENTED (commit `0090c24b99`); both pushed to `jake`.** The target-class grammar is now the
SOLE build vocabulary; the legacy `default` grammar is gone (all 114 `default.px4board` deleted)
and the `docs/en` board-config model is migrated to match. A bare board name resolves to its sole
class; a multi-class board names a class explicitly (`px4_fmu-v6x_vtol`); `default`/`base` error.
**The epic is functionally complete — the only open items are (a) the ko/uk/zh translation sync
(PX4's Crowdin workflow regenerates these from `docs/en`; NOT our work) and (b) optional cosmetic
slimming of 17 pre-existing rover overlays (§NOT-done #2).** What landed in the cutover:
- `px4_config.cmake` resolves bare→sole-class, errors on default/base/ambiguous, and exports
  `PX4_TARGET_CLASS` (a bare variant like `px4_sitl_test` attaches to the board's sole class).
  `kconfig.cmake` dropped the legacy `default.px4board` merge; its savedefconfig dev-tooling now
  diffs a generated base+class baseline (`board_label_baseline.px4board`).
- 41 pre-redesign variants relabeled to `<class>.<variant>`; `multicopter`→`copter`
  (+`AIRFRAMES_COPTER`, verified copter-only airframes); `px4/sitl` made sole-class (its uuv/
  spacecraft sims → `sitl.uuv`/`sitl.spacecraft`, so `px4_sitl`/`px4_sitl_test` stay clean).
- **voxl2 carve-out DONE**: now a sole-class `voxl2` board — `base.px4board` (= old apps config)
  + tag-only `target_classes/voxl2.px4board` + bare `slpi` variant; `TARGET_CLASS_VOXL2` added to
  `Kconfig.target_classes`. No resolver special-case; behavior preserved.
- CI enumerator / `updateconfig.py` / `Makefile` (sole-class→bare-name collapse via awk, `all:
  px4_sitl`, `_deb`→`$(subst _deb,,…)`, check-lists, IO-firmware copy) / `build_all_config.yml`
  seeders reworked to merge `base.px4board`. Workflows, actions, CI scripts, `Dockerfile.gazebo`,
  `generate_sbom.py`, `px4_uploader.py`, and the build-command docs (`building_px4.md` + ko/zh/uk
  + ~97 docs/en pages, 249 target-name replacements) follow the new names. `build-deb/action.yml`
  maps the `default` input → `px4_sitl` via `SITL_TARGET`.
- IO blob filenames (`px4_io-v2_default.bin`, `cubepilot_io-v2_default.bin`) intentionally KEPT —
  they are runtime artifact paths in `rcS`/`board_common.h` (`PX4IO_FW_SEARCH_PATHS`), not build
  labels. The `px4io_update` target's source path is updated, the blob dest names are not.

Verified: `make px4_sitl` builds end-to-end (`build/px4_sitl/bin/px4`); resolver error+happy
cases (real cmake configures); enumerator output (no `_default`/`_base`, sole-class bare, voxl2
deb grouped); enumerator↔Makefile parity (all 293 CI targets make-buildable, make-only set =
CI-excluded labels); `px4_fmu-v6x_copter` ships 26 copter / 0 plane-vtol airframes.

**REMAINING — none in docs/en; one cross-language follow-up.** The board-config MODEL
re-documentation LANDED (2026-06-15): all 31 `docs/en` files that referenced `default.px4board`
as a CONCEPT now describe the base + `target_classes` + class-overlay model. The porting guide
got the real rewrite — `porting_guide_config.md` §"PX4 Board Configuration Files" (was "Kconfig
Label Inheritance") now spells out the four-layer merge (base → `target_classes/<class>` → board
`<class>` overlay → `<class>.<variant>`); `porting_guide.md`, `porting_guide_nuttx.md`,
`serial_port_mapping.md` follow. Per-case routing held: driver/hardware/serial/system/example/
middleware keys → `base.px4board` (anchors re-pinned to the new files: hold auterion/fmu-v6s L46,
log_encryption fmu-v4 L76→L64, mavlink sitl L36→L70, uorb fmu-v6x release/1.15 L100→main L88,
zenoh fmu-v6xrt L91→L81; INS `#L25` anchors dropped since fmu-v6c base lacks the key); vehicle-
controller pages (`config_rover`, `mc_neural_network_control`) route to the board's class overlay
and the `copter`-vs-`vtol` target choice; `cube_yellow` stale `hex/cube-orange` link repointed to
`cubepilot/cubeyellow`; `serial_port_mapping`/`make px4_fmu-v5`→`_vtol` (bare is now ambiguous).
The build-COMMAND docs were already done. **Only follow-up:** the ko/uk/zh translations (~90
files) still carry `default.px4board` — left to PX4's translation (Crowdin) sync, which
regenerates them from `docs/en` (the English `building_px4.md` is already clean while its
translations lag — same pattern). Not hand-edited: judgment-based prose in three languages belongs
to the translation workflow, not a manual sweep.

Class names are canonical `copter`/`fixedwing`/... (the `plane`→`fixedwing` rename + #25414
`_VEHICLE_LABELS` alignment, commit `3503a0006a`, landed earlier — see §9). Pre-cutover phase
history is in §5.

**Design reworked 2026-06-15 (dakejahl) — self-documenting, no hidden coupling. Controller
membership moved back into `target_classes/` fragments 2026-06-15 (dakejahl), modules decoupled:**
- A class target = `base.px4board` + `target_classes/<class>.px4board` + `<board>/<class>.px4board`.
  The class base fragment defines the class: `CONFIG_TARGET_CLASS_<X>=y` (a declarative tag) plus
  the explicit controller `CONFIG_MODULES_*=y` set (linux also POSIX + LINUX_TARGET, cannode the
  romfs + uavcannode driver); cmake merges it by class name. The board overlay carries the explicit
  `CONFIG_AIRFRAMES_<Y>=y` set plus a `# CONFIG_MODULES_X is not set` for any class controller it
  drops (overrides the fragment's `=y`). Module Kconfigs carry NO class coupling — plain `default
  n`; the rework's `default y if TARGET_CLASS_<X>` on each module (+ the top-level Kconfig
  POSIX/LINUX_TARGET/ROMFSROOT defaults + uavcannode) is reverted. `TARGET_CLASS_*` symbols remain
  as tags only (Kconfig.target_classes). The `AIRFRAMES default y if MODULES_*` coupling stays gone.
- Airframes are flat in `init.d/airframes/` with their `@class` header; a frame ships iff its
  `@class` maps to an enabled `CONFIG_AIRFRAMES_<class>` (init.d/airframes/CMakeLists.txt globs
  + parses `@class`; `.hil` → AIRFRAMES_SIMULATION; unmapped e.g. Autogyro → dangles). Products
  ship with their class again. `BALLOON` stays a global `default y` (was unconditional).
- Override mechanics: a board's `# CONFIG_MODULES_X is not set` wins over the fragment's `=y` via
  merge_config's unset-revert (16 vtol + 8 copter boards drop a controller); with modules at plain
  `default n` the revert resolves cleanly to n. The fragment approach is why modules need no
  `default y if`/select/imply at all — membership is explicit data in one file per class.
- **Transitional caveat:** decoupling AIRFRAMES means legacy `_default` targets (which don't
  set AIRFRAMES_*) now ship only the balloon frame, not vehicle airframes. The class targets
  are the supported path; `_default` goes away at Phase-4 cutover. SITL/Linux unaffected
  (init.d-posix airframes, a separate flat ungated list).

A fresh session: read §3 (locked decisions), §4 (class catalog), §6 (script spec), §7 (Phase-4),
§9 (PR #25414 firmware-manifest alignment — the two-axis role/artifact model).

### State check (all exit 0)
- `python3 Tools/migrate_px4board.py` (dry run) → `0 ok, 114 skip, 1 manual, 0 error` (skips =
  already migrated; the 1 manual is the deferred `modalai/voxl2`); `… --force` re-verifies the
  merge model for every board → `114 ok, 1 manual, 0 error`.
- Specials ground-truth (real cmake configure, boardconfig diff): `px4_sitl_sitl` ==
  `px4_sitl_default` excluding the decoupled `TARGET_CLASS_*`/`AIRFRAMES_*` lines;
  `px4_io-v2_io` == `px4_io-v2_default` excluding `# Label:`; `px4_ros2_ros2` ==
  `px4_ros2_default` excluding `# Label:` (both ros2 configures fail identically later for lack
  of a colcon/ROS2 env — pre-existing, not the migration). cubepilot/voxl2-io share byte-
  identical io configs (tool self-verify).
- `make px4_fmu-v6x_vtol romfs_gen_files_target` → `boardconfig` == `px4_fmu-v6x_default`
  EXCLUDING `# Label:` and the now-decoupled `CONFIG_AIRFRAMES_*`/`CONFIG_TARGET_CLASS_*`
  lines; the generated `etc/init.d/airframes/` set is the 36 Copter+Plane+VTOL+Balloon frames
  (products included, `.hil` excluded) = the @class rule.
- Override canary `make holybro_kakutef7_copter …` → the dropped MC_AUTOTUNE/MC_HOVER are
  `# … is not set` in the resolved boardconfig (proves the fragment's `=y` is overridable by the
  board overlay; modules are plain `default n` so the unset-revert resolves to n).
- `make cubepilot_cubeyellow_vtol …` and `… _uuv` → enabled-symbol sets UNION to `_default`.

### Done
- **Board-config MODEL re-documented across all 31 `docs/en` files (2026-06-15, dakejahl).**
  Porting guide rewritten for the four-layer merge (`porting_guide_config.md` §"PX4 Board
  Configuration Files" + `porting_guide{,_nuttx}.md` + `serial_port_mapping.md`); ~25 sensor/
  driver/middleware/system pages repointed `default.px4board` → `base.px4board` with anchors
  re-pinned to the live files; `config_rover` + neural pages route controllers to the class
  overlay / `copter`-vs-`vtol` choice. 0 `default.px4board` left in `docs/en`; new
  `#px4-board-configuration-files` anchor + 4 inbound cross-links verified. Translations (ko/uk/
  zh) deferred to Crowdin sync (see §0 REMAINING).
- **5 specials migrated to `sitl`/`io`/`ros2` classes (2026-06-15, dakejahl — additive).**
  Created `target_classes/{sitl,io,ros2}.px4board` (`sitl` = POSIX + the full mc/fw/vtol/uuv/
  rover/airship controller set; `io` = ROMFSROOT="" + PX4IOFIRMWARE; `ros2` = PLATFORM_ROS2) +
  the `TARGET_CLASS_{SITL,IO,ROS2}` tag symbols in `Kconfig.target_classes`. Extended
  `migrate_px4board.py` to classify them (PLATFORM_ROS2→ros2; plain PLATFORM_POSIX+controllers→
  sitl; no-controller+PX4IOFIRMWARE→io; CLASS_PROVIDES + CLASS_FAMILIES entries) and migrated
  `px4/sitl`, `px4/io-v2`, `cubepilot/io-v2`, `modalai/voxl2-io`, `px4/ros2`. `default.px4board`
  kept everywhere (additive). Verified at the real-merge level (see State check). The sitl
  overlay carries `AIRFRAMES_COPTER/PLANE/VTOL/ROVER/UUV/AIRSHIP` (harmless on POSIX, which
  ships init.d-posix; matches the linux precedent); io/ros2 overlays are empty (inherits
  comment); `px4/ros2`'s base is empty (everything moved to the class).
- **Controller membership reverted to `target_classes/` fragments (2026-06-15, dakejahl).**
  Recreated the 9 `target_classes/<class>.px4board` fragments (`TARGET_CLASS_<X>=y` tag + the
  controller/platform/romfs/driver set); restored the `class_base` merge in `cmake/kconfig.cmake`;
  stripped `default y if TARGET_CLASS_<X>` from the 19 modules + uavcannode + the 3 top-level
  Kconfig defaults (modules now plain `default n`, fully class-agnostic); regenerated the 137
  overlays to drop the now-redundant `TARGET_CLASS` line (cannode no-delta overlays get the
  self-doc comment); reworked `migrate_px4board.py` to match. Verified at the real-merge level:
  v6x_vtol/linux/cannode == legacy `_default`; kakutef7_copter override; cubeyellow vtol+uuv
  union; orphan rover/uuv/spacecraft secondaries resolve to clean single-class targets. The 17
  pre-existing "slimmed" rover overlays now correctly pull rover controllers via the fragment
  (were controller-less under the Kconfig-coupling design).
- **All 109 boards migrated**, overlays regenerated to the explicit format; build-verified per
  class (module/controller set == legacy `_default`; @class airframe set; override boards;
  multi-class union; linux; cannode; SITL + airframes.xml metadata regenerate). `base.px4board`
  files are unchanged by the rework.
- `Tools/migrate_px4board.py` reworked to emit `TARGET_CLASS_<X>` + `AIRFRAMES_<Y>` + controller
  reverts and to model `TARGET_CLASS_<X>` → its controllers (replacing the deleted fragments)
  for its pre-write self-verification.
- 5-commit rework `13ede295f5..9dded1fae5`: Kconfig decoupling; merge_config compound-cond fix;
  drop fragments + simplify merge; @class airframe revert (flat, srcparser reads @class,
  flatten_classes.py + 4 orphaned rc.board_airframes deleted); regenerate overlays.
- Earlier this branch: the 2 cannode boards (esp32, mr-canhubk3 — uavcannode revert) and the 5
  Linux SBC FCs were migrated; `voxl2` (aarch64 muorb/QURT, no apps-side controllers) refused +
  deferred. mr-canhubk3 `fmu`/`sysview` + the linux `_arm64` variants stay on the legacy path.

### NOT done (next sessions)
1. **ko/uk/zh translation sync (~90 files).** The translated docs still reference
   `default.px4board` as a concept; they regenerate from `docs/en` via PX4's translation (Crowdin)
   workflow, so leave them to that sync rather than hand-editing judgment-based prose in three
   languages. (The `docs/en` MODEL re-documentation itself is DONE — see §0 / the Done list.)
2. **Optional — slim the 17 "slimmed" rover overlays** (`px4/fmu-v6x|v6c|v6u|v6xrt|v5x|v4|v3|v2`,
   `ark/{fmu-v6s,fmu-v6x,fpv,pi6x}`, `auterion/{fmu-v6s,fmu-v6x}`, `x-mav/ap-h743r1`): pre-redesign
   rover variants on air boards; under the fragment merge they pull the rover controllers by the
   `rover` filename and resolve to a clean rover. Either canonicalize them (drop the now-redundant
   explicit rover-controller / air-disable lines the fragment supplies) or leave as-is — cosmetic.

### DONE — Phase 4 cutover (commit `f914c21643`, 2026-06-15)
The breaking cutover landed: grammar/resolver (`px4_config.cmake` bare→sole-class + `PX4_TARGET_CLASS`),
`kconfig.cmake` legacy-merge removal, 41 variant relabels + `multicopter`→`copter`, all 114
`default.px4board` deleted, the **`modalai/voxl2` carve-out** (last legacy board → sole-class
`voxl2`), the CI enumerator / `updateconfig.py` / Makefile / seeders rework, and the
workflow/action/script/command-doc ripple. Full breakdown + verification in §0.

### Housekeeping
- The unrelated `boards/ark/fpv/extras/ark_fpv_bootloader.bin` working-tree change is
  deliberately excluded — do NOT commit it.
- Harmless transitional warts: `<vendor>/cannode` → `ark_cannode_cannode`; a few
  pre-redesign variant overlays still carry now-redundant `=n` lines (byte-identical,
  slimmable later).

## 1. Core model

Every buildable target belongs to exactly one **target class**. Classes are peers:

- **Vehicle classes:** `copter`, `fixedwing`, `vtol`, `rover`, `uuv`, `spacecraft`, `airship`
- **System classes:** `cannode`, `linux`, `bootloader`, `sitl`, `io`, `ros2`

A target is composed by merging, in order (later overrides earlier):

```
boards/<v>/<b>/base.px4board               # 1. board hardware foundation (NOT buildable)
target_classes/<class>.px4board            # 2. class definition: TARGET_CLASS_<X>=y + controllers
boards/<v>/<b>/<class>.px4board            # 3. AIRFRAMES_<Y>=y + board deltas (drops)
boards/<v>/<b>/<class>.<variant>.px4board  # 4. optional variant delta
```

The class controllers ARE a merged fragment: `target_classes/<class>.px4board` lists the class's
`CONFIG_MODULES_*=y` (cmake merges it by class name) plus `CONFIG_TARGET_CLASS_<X>=y` as a
declarative tag. Module Kconfigs are plain `default n` with no class coupling; a board drops a
controller with `# CONFIG_MODULES_X is not set` in its overlay. Airframes are the explicit
`CONFIG_AIRFRAMES_<class>=y` set, decoupled from controllers and from each other.

Filename grammar: `<class>[.<variant>].px4board`. Class always first; variant a free
string. Target name: `<vendor>_<model>_<class>[_<variant>]` (dot → underscore).

- `base.px4board` (renamed from `default.px4board`) holds vehicle-agnostic hardware
  only. No controllers, no airframes. Building `<board>`, `<board>_default`, or
  `<board>_base` is an **error** (no controllers ⇒ unusable binary).

### Airframes: flat layout, classified by `@class` header (reworked 2026-06-15)
- All airframes live flat in **`ROMFS/px4fmu_common/init.d/airframes/`**, each with its
  `@class` header. `init.d/airframes/CMakeLists.txt` globs the dir, reads each file's
  `@class`, and ships the frame iff the mapped `CONFIG_AIRFRAMES_<class>` is set (display
  name → option: Copter→COPTER, "Underwater Robot"→UUV, …). `*.hil` → AIRFRAMES_SIMULATION
  regardless of vehicle `@class`. A frame whose `@class` has no option (Autogyro) dangles.
- A board's overlay sets the explicit `AIRFRAMES_<class>` set: a `vtol` target emits
  `AIRFRAMES_COPTER + AIRFRAMES_PLANE + AIRFRAMES_VTOL` (one binary flies all air). `BALLOON`
  stays a global `default y`. No `default y if MODULES_*` coupling, no directory magic.
- **Product/vendor frames carry `@class` too and ship with their class again** (the
  dangling/tiering experiment was reverted). The `boards/<v>/<b>/init/airframes/` per-board
  claim path is still available (unused).
- `srcparser.py` reads `@class` from the file (reverted to the pre-redesign behavior; aborts
  if a frame has no `@class`).

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
2. **Class names — LOCKED:** `copter`/`fixedwing`/`vtol`/`rover`/`uuv`/`spacecraft`/`airship`
   (`copter` not `multicopter`; `fixedwing` kept over `plane` — revised 2026-06-15 for
   PR #25414 manifest alignment, since `variant` = class name).
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
     DONE additively 2026-06-15 (kept on the dual grammar; the CI-facing rename + cutover is
     still Phase 4). Their transient `_<class>` doubled target names collapse to the bare board
     name at the cutover (bare → sole class).
   - `modalai/voxl2` is special (muorb/QURT; vehicle controllers live in its `slpi` variant,
     not the apps-side default) → handled separately in Phase 4, not forced into `linux`.

## 4. Target-class catalog

| class | `target_classes/<class>.px4board` enables | airframe dirs pulled (Tier 1) |
|---|---|---|
| copter | MC_RATE_CONTROL, CONTROL_ALLOCATOR | `copter/` |
| fixedwing | FW_RATE_CONTROL | `plane/` (Plane frames) |
| vtol | MC + FW + VTOL_ATT_CONTROL (superset) | `copter/` + `plane/` + `vtol/` |
| rover | ROVER_DIFFERENTIAL + ACKERMANN + MECANUM | `rover/` |
| uuv | UUV_ATT_CONTROL | `uuv/` |
| spacecraft | SPACECRAFT | `spacecraft/` |
| airship | AIRSHIP_ATT_CONTROL | `airship/` |
| cannode | cannode romfs, uavcannode drivers | none |
| linux | POSIX + LINUX_TARGET + mc/fw/vtol/uuv superset (DONE) | none (init.d-posix) |
| bootloader | bl defconfig path | none |
| sitl | POSIX + full controller set, runtime-selected (DONE) | init.d-posix; overlay also sets AIRFRAMES_* |
| io | ROMFSROOT="" + PX4IOFIRMWARE blob (DONE) | none |
| ros2 | PLATFORM_ROS2 build (DONE) | none |

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
- `target_classes/{vtol,fixedwing,uuv,airship,spacecraft,cannode,linux}.px4board` — DONE (added
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

### Phase 4 — Cutover — DONE (commit `f914c21643`, 2026-06-15; see §0)
- **Naming LOCKED (dakejahl 2026-06-15): bare board name → its SOLE class.** `px4_config.cmake`
  drops the "omit default" rule and errors on the literal `default`/`base` labels and on a bare
  multi-class board (ambiguous), BUT a bare board name resolves to its one class when exactly one
  exists — so `px4_sitl`/`px4_io-v2`/`px4_ros2`/`holybro_kakutef7` keep working and the transient
  `px4_sitl_sitl` doubled names go away. (Relaxes the earlier §1 "bare is always an error" line.)
- `Makefile`: rework target lists; `all:` → the sitl class (`px4_sitl`). Sweep workflows/Tools/
  docs (§7). The CI enumerator `Tools/ci/generate_board_targets_json.py` is the linchpin (its
  `default.px4board`-merge branch + `<mfr>_<board>_<label>` naming + `excluded_labels`).

## 6. Migration script (`Tools/migrate_px4board.py`) — IMPLEMENTED
Per `default.px4board`: infer the class(es) — air (mc/fw/vtol) collapses to `vtol`, each
extra vehicle class is its own target, `ROMFSROOT="cannode"` → `cannode`,
`PLATFORM_POSIX + BOARD_LINUX_TARGET` with vehicle controllers → `linux`, `PLATFORM_ROS2` →
`ros2`, plain `PLATFORM_POSIX` + controllers → `sitl`, no-controller + `PX4IOFIRMWARE` → `io`.
Emit one
`base.px4board` (= default minus ALL controllers and every symbol the class bases provide)
+ one `<class>.px4board` overlay per class (the board's delta vs `target_classes/<class>`;
self-documenting comment when empty). Self-verifies the layered merge reproduces `default`
symbol-for-symbol (UNION of the per-class targets for multi-class) before writing, tolerating
any class-base symbol the board reverts off (controllers AND e.g. cannode `DRIVERS_UAVCANNODE`;
all `default n`). Refuses (MANUAL, never writes) only muorb/QURT Linux boards with no
apps-side controllers (voxl2) and class-base mismatches; sitl/io/ros2 are now handled
(`CLASS_PROVIDES`/`CLASS_FAMILIES` entries added).
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

## 9. PR #25414 (firmware manifest) alignment
PR #25414 (mrpollo / Ramon Roche — "feat(manifest): embed per-build metadata and publish
unified firmware manifest"; QGC companion qgc#13966) generates per-build metadata (`variant`,
`firmware_category`, `label_pretty`, hardware IDs), embeds it in each `.px4`, and aggregates a
unified S3 manifest QGC consumes. It currently DERIVES the taxonomy from label-string
heuristics: `_VEHICLE_LABELS = {multicopter, fixedwing, vtol, rover, uuv, spacecraft}` (old
vocabulary — `multicopter` vs our `copter`, **`airship` missing**, `fixedwing` shared →
micoair boards mis-file as `dev` + warn), `ROMFSROOT=="cannode"` → peripheral, else `dev`; and
sets `CONFIG_BOARD_LABEL_PRETTY` by hand (their own follow-up = ~258 `.px4board` files). It
already added an `artifact_type` field (foreseeing "VOXL2 `.deb`").

**Reframe — two orthogonal axes; this redesign owns both sources of truth:**
- **role** = `CONFIG_TARGET_CLASS_<X>` → manifest `firmware_category`.
- **artifact** = `CONFIG_PLATFORM_*` (+ `BOARD_LINUX_TARGET` / `cmake/package.cmake`) →
  manifest `artifact_type`. Not every target is a QGC-flashable MCU `.px4`:

| target group | PLATFORM | emits | QGC-flashable | category |
|---|---|---|---|---|
| copter/fixedwing/vtol/rover/uuv/spacecraft/airship | nuttx | `.px4` MCU | yes (serial) | vehicle |
| cannode | nuttx | `.px4` MCU | yes | peripheral |
| io | nuttx (cortex-m3) | MCU blob | embedded in FMU | peripheral |
| bootloader | nuttx | `.px4` (bl) | yes | bootloader |
| sitl | posix | host exe | no | dev |
| ros2 | ros2 | colcon libs | no | dev |
| linux SBC | posix+linux | cross exe/tarball | no (copy to SBC) | vehicle |
| voxl2 | posix+linux + qurt | `.deb` (on-device) | no | vehicle |

**Alignment actions (do NOT edit #25414 unprompted — coordinate with mrpollo):**
1. `detect_firmware_category` reads `CONFIG_TARGET_CLASS_<X>` first (authoritative); keep the
   string heuristics only as the fallback for un-migrated boards, then drop `_VEHICLE_LABELS`.
2. Carry `CONFIG_BOARD_LABEL_PRETTY` (+ `CONFIG_BOARD_FIRMWARE_CATEGORY` for non-vehicle) IN the
   `target_classes/<class>.px4board` fragments — the fragment already defines the target's
   identity, so every board in a class gets correct manifest metadata for free; their ~258-file
   follow-up collapses to ~13 class entries.
3. `variant` = class name (aligned post-cutover). **DONE 2026-06-15:** #25414's `_VEHICLE_LABELS`
   set to `{copter, fixedwing, vtol, rover, uuv, spacecraft, airship}` (multicopter→copter,
   +airship, `fixedwing` kept over `plane`) — commit `3503a0006a`, pushed by dakejahl, now the
   head of PR #25414.
4. Add NO new `artifact_type` symbol — it derives from `PLATFORM`; the manifest publishes the
   distributable subset keyed on the two axes.

**Status 2026-06-15:** Decided `fixedwing` over `plane` and `copter` over `multicopter`; our
redesign's `plane` class is renamed `plane → fixedwing` (fragment + `TARGET_CLASS_FIXEDWING` +
tool + this doc). The #25414 vocab edit landed on the PR (action 3). The deeper alignment (manifest
reads `TARGET_CLASS`/`PLATFORM`) waits until the redesign merges — those symbols don't exist on
`main` yet.

**Cutover coupling (the one hard dependency):** `modalai/voxl2` is a `.deb`/on-device distributed
build (SLPI DSP `slpi` + ARM64 apps `default`), not a vehicle class — keep the carve-out, but its
apps target needs a **non-`default` label by the cutover** (the cutover errors on `default`); when
the manifest reaches it, set `artifact_type=deb`, `firmware_category=vehicle`. ros2/sitl are `dev`
artifacts (not firmware, not QGC-distributable) — their class is just a platform marker.
