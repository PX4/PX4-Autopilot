# Sees Fork: v1.13.1-dev → v1.17.0-dev Migration Assessment

**Date:** 2026-09-03
**Branches:** `origin/v1.13.1_dev` (old baseline) → `origin/v1.17.0-dev` (new baseline)
**Scope:** (1) what upstream PX4 changed between these baselines, (2) what Sees changed on top of v1.13.1, (3) a per-commit recommendation for bringing our changes onto v1.17.0-dev.

---

## 1. Upstream PX4 changes, v1.13.1 → v1.17.0

`v1.13.1_dev` is `v1.13.1` + 41 Sees commits (`git describe` → `v1.13.1-0.0.41`). `v1.17.0-dev` is `v1.17.0` + a handful of newer commits (`git describe` → `v1.17.0-0.0.1-6-g6e2a8da15f`). This spans **4 upstream PX4 releases**:

| Tag | Release date |
|---|---|
| v1.13.1 | 2022-09-28 |
| v1.14.0 | 2023-08-10 |
| v1.15.0 | 2024-08-23 |
| v1.16.0 | 2025-08-05 |
| v1.17.0 | 2026-01-16 |

**Scale:** 7,718 commits, 12,265 files touched (6,745 excluding `docs/`), ~533k insertions / ~167k deletions in firmware code. Of the 7,622 non-merge commits in range, 7,589 are stock upstream PX4 commits (top authors: Pavel Kirienko, Daniel Agar, Silvan Fuhrer, Matthias Grob, bresch) — this is fundamentally a full upstream re-sync, not an incremental bump.

### Major architectural changes to be aware of

These are the changes most likely to affect where our custom commits land:

- **Fixed-wing control rewrite**: `fw_pos_control_l1` → replaced by `fw_lateral_longitudinal_control` + `fw_mode_manager` + `fw_rate_control`. (_DONT_CARE_)
- **Rover control rewrite**: `rover_pos_control` → replaced by `rover_ackermann` / `rover_differential` / `rover_mecanum`. (_DONT_CARE_)
- **Comms/middleware overhaul**: `microdds_client` / `micrortps_bridge` → replaced by `uxrce_dds_client`; new `zenoh` transport module. (_DONT_CARE_ for now unless we directly use zenoh instead of mavlink)
- **SITL restructuring**: `sih` / `simulator` → replaced by a new `simulation` module tree. (_DONT_CARE_)
- **Commander failsafe rewrite**: `state_machine_helper.cpp` → replaced by `commander/failsafe/{failsafe.cpp, framework.cpp}`.
- **Pre-arm checks rewrite**: `Arming/PreFlightCheck/PreFlightCheck.cpp` → replaced by `HealthAndArmingChecks/checks/*.cpp`.
- **Battery library rewrite**: voltage-lookup-table SoC estimation → RLS internal-resistance estimator (`interpolate(v_empty, v_charged)`).
- **Rate control library**: `mc_rate_control/RateControl` → moved to shared `src/lib/rate_control/`, used by MC/FW/rover/spacecraft, with a plain `update()` signature (no D-term/drag-compensation hooks).
- **Manual control**: message schema moved to `msg/versioned/ManualControlSetpoint.msg`; `ManualControlSelector` rewritten to an enum-based `RcInMode`. (_Will add our own SEES_* on top_)
- **UAVCAN GNSS channel assignment**: node-ID ordering hacks replaced by `get_channel_index_for_node()`. (Decide whether we want to backport their changes from latest master into v1.17. GPS issues still exist in v1.17. Some have been backported by William)
- **New driver/module categories**: `cyphal`, `gnss`, `ins`, `rc`, `gpio`, `transponder`, `wind_sensor`, `payload_deliverer`, `internal_combustion_engine_control`, `mc_nn_control`. (We benefit from newer GPS module support)
- **~50 new board targets** added (Auterion fmu-v6x/v6s, CUAV x25-evo, Holybro kakuteh7 family, ModalAI VOXL2, NXP mr-tropic, PX4 fmu-v6xrt, Espressif esp32, etc).
- Heavy churn throughout EKF2, MAVLink, Navigator, Commander, VTOL, TECS, NuttX platform code. (Could impact us - beware - do flight tests)

---

## 2. Sees custom changes on `v1.13.1_dev`

Identified by diffing `v1.13.1_dev` against upstream PX4 at their merge-base, then filtering to commits merged as fork-native PRs (`#26`–`#99`) into the Sees repo directly (as opposed to backports of upstream PX4 PRs, which were excluded).

**57 commits, PR #26 (2022-12-29) → PR #99 (2026-03-20).** Authors: mostly `DavidPaddy97` (David Patrick, 47 commits — matches the "Dp ..." commit-message prefix convention), plus `wsolichin@sees.ai`/`wsolichin-sees`, `ealdaz-seesai`/`sees-dev` (Eduardo Aldaz), `dpatrick@sees.ai`, `mrossi@sees.ai`/`m.rossi@live.com`, `github@richardhopkirk.net`, and one commit each from `abamforth`.

**Important:** cross-checked against `v1.17.0-dev`'s own non-upstream commits — **none of these 57 have been ported yet.** The only genuinely Sees-authored commits already on `v1.17.0-dev` are two unrelated newer items by `ealdaz@sees.ai` ("Reenabled standard option to allow QGC to send RP position" and "feat(remoteid): DroneCAN arm_status keepalive + rate and system message fixes"), confirmed not to overlap with any of the 57 below. The rest of `v1.17.0-dev`'s non-upstream-main commits are backports cherry-picked from PX4's own `release/1.17` maintenance branch (authored by PX4 maintainers, e.g. `mrpollo@gmail.com`, `dakejahl`, `Claudio-Chies`), not Sees originals.

### Feature groupings

Most of the 57 commits aren't independent — they're installments of five long-running features, built up incrementally over 3+ years:

1. **Safety-pilot RC/Mavlink control-source selector** — `c392d6d1`, `fc334f30`, `988f4bc0`, `60142574`, `72da9ff2`, `bb02f594` (+ telemetry hooks in `1ecc00e7`, `e9b776fe`). Lets ops toggle/monitor RC vs. Mavlink-joystick control source, with FrSky/SYS_STATUS feedback.
2. **Battery SOC / chemistry curve** — `39110b7b`, `b7eed0cd`, `5f316bfa`. Coulomb-counting / voltage-lookup-table SOC estimation tuned to specific packs (25Ah Tattu HV).
3. **Dual-CAN-GPS node-ID ordering / Rover-ID** — `0efcb970`, `e116be69`, `77b5150e`, `b7aa8c4b`, `f918007e`. Ensures stable uORB instance ordering when two CAN GPS units are on the bus.
4. **Custom magnetometer filtering + noise metric** — `b932235e`, `106674e8`, `7ae25eee`. Two-stage low-pass filter + RMS noise calculation on mag data.
5. **FRSky/Horus telemetry customization** — `2337435c`, `e9b776fe`. Custom Smartport DIY streams (dual-GPS, RC/Mav status, flight mode).

(There may be other little ones like geo-fencing. Commits may be missing)

Everything else is a standalone fix, tuning value, or CI/tooling change.

---

## 3. Porting report: bringing the 57 commits onto `v1.17.0-dev`

Each commit was checked against the `v1.17.0-dev` tip: whether its touched files/functions still exist, whether the same need is already met by an independent upstream change, and whether the surrounding architecture moved enough to require a rewrite rather than a cherry-pick.

### Summary

| Verdict | Count | Meaning |
|---|---|---|
| **Port as-is** | 16 | File/logic essentially unchanged upstream — clean cherry-pick |
| **Port with adaptation** | 26 | Still needed, but surrounding code was rewritten — needs reimplementation |
| **Likely obsolete/superseded** | 15 | Subsystem removed, or upstream independently fixed/added the same thing |
| **Total** | **57** | |

### Porting strategy

- :done: Assign IER score to each item
  - Impact : low/med/high
  - Effort : low/med/high
  - Risk   : low/med/high
- :done: Assign action for each item
  - NO_PORT: ignore our change
  - DEFER_UNTIL_TEST: Test upstream behaviour first. If upstream fix is good, then NO_PORT, otherwise PORT
  - PORT: Implement our change
- Flight test to resolve all DEFER_UNTIL_TEST to NO_PORT/PORT(with IER score).
  - Define a pass/fail criteria for each such item
- Port L/L/L tasks to get familiar with firmware codebase, flight test
- Port H/L/L tasks next, flight test
- Port H/M/L tasks, flight test
- Port remaining high impact tasks, flight test
- Port remaining tasks, flight test


### 3.1 Portable as-is (16) — clean, low-risk cherry-picks

| Commit | PR | Subject | Note | I/E/R score, Action |
|---|---|---|---|--|
| `5f1e7223` | #32 | `CONFIG_DRIVERS_HYGROMETER_SHT3x=y` on cubeorange board | One-line board config | PORT - L/L/L |
| `fcecf06e` | #37 | Enable `GPS_RTCM_DATA` mavlink stream in onboard mode | Stream exists upstream, still not auto-enabled there | PORT - M/L/L |
| `46cd8f26` | #40 | Precision-land "hold position" fallback instead of falling through to normal landing | Safety feature, state machine structurally unchanged | NO_PORT |
| `b1983220` | #47 | Start `landing_target_estimator` on `SENS_EN_IRLOCK=1` | `rcS` still lacks this; still needed for precision-land HW | NO_PORT |
| `8b92bd69` | #48 | ICM20602 accelerometer error debug logging | Pure debug logging, code paths unchanged | PORT - L/L/L |
| `a80433c3` | #54 | Fix stuck UAVCAN param queue (missing `dequeue_uavcan_request()`) | Identical surrounding code, bug still present upstream | PORT - H/L/L |
| `7982691f` + `ccd58586` | #56, #57 | batmon deci-current fallback for saturated current reads (apply together) | File essentially identical; note original diff used spaces not tabs — run `make format` | NO_PORT |
| `1dd11de2` | #59 | Sanity-guard `NAV_ACC_RAD` in `get_default_acceptance_radius()` to stop uncontrolled yaw on auto-takeoff | Function unchanged upstream | DEFER_UNTIL_TEST |
| `6508e103` | #60 | Audible tone on kill-switch / AUTO_RTL entry (BVLOS compliance) | Confirm tone choice still matches regulatory requirement | PORT - H/L/L|
| `20cec0d3` | #63 | Bump mavlink instances 3→4 | `module.yaml` unchanged | PORT - H/L/L |
| `020a4815` + `a931f3e5` | #66, #67 | Broadcast STATUSTEXT to onboard controller without requiring GCS | Apply together | PORT - L/L/L |
| `5a450f6a` | #74 | In-flight GPS failure injector (`VEHICLE_CMD_INJECT_FAILURE`) | No upstream equivalent; also enable `CONFIG_SYSTEMCMDS_FAILURE` on the 2 boards | DEFER_UNTIL_TEST - L/M/H |
| `28e74b0b` | #84 | Offboard acceleration frame fix (use yaw-only rotation, matching velocity handling) | Bug still present and unfixed upstream | DEFER_UNTIL_TEST - H/M/H |
| `e680fcb8` | #86 | Default-enable IRLock/landing-target-pose logging | Same file/line, still optional/off upstream | NO_PORT |

### 3.2 Port with adaptation (18) — real features, need reimplementation

| Commit | PR | Subject | What changed upstream | I/E/R score, Action |
|---|---|---|---|---|
| `dc5376bc` | #26 | Drag-compensated rate controller (DragEstimator module + filtered D-term) | `mc_rate_control` moved to shared `src/lib/rate_control/`; `drag_estimator` module and `LowPassFilter1p.hpp` both gone — full rebuild against new lib | PORT - H/H/H |
| `2337435c` | #29 | FRSky telemetry: custom RADIO_STATUS stream, flight-mode remap, secondary-GPS fix-type | `RADIO_STATUS.hpp` stream must be recreated; `vehicle_gps_position_s` fully replaced by `sensor_gps_s` | PORT - H/L/L |
| `e9b776fe` | #45 | FRSky Horus custom DIY Smartport streams | Depends on #29/#38 landing first, then low-conflict reapply | PORT WITH #29 - H/L/L|
| `39110b7b` | #30 | Coulomb-counting SOC estimator + low-cell-voltage warning | `battery.cpp` rewritten (RLS estimator); rebuild against new API | DEFER_UNTIL_TEST - H/M/M |
| `b7eed0cd` | #61 | 25Ah Tattu HV SoC lookup table + critical-voltage warning | Confirm curve/pack still applies; battery mechanism was replaced upstream | REVIEW WITH #30 (SOC) - MAY NOT BE REQUIRED BUT LUT MAY STILL BE IN USE, CRITICAL WARNING STILL VALID |
| `5f316bfa` | #95 | Battery chemistry curve v1.2 + SOC tests | Same as above; also depends on `39110b7b`'s prerequisite being re-ported first | REVIEW WITH #30 (SOC) |
| `0efcb970` | #31 | GPS driver submodule swap + timeout warning via `isFallbackAllowed()` | Blending logic rewritten, no equivalent concept present | REVIEW UPSTREAM MASTER FIRST, PORT - H/L/L |
| `b932235e` + `106674e8` | #33, #36 | Two-stage mag low-pass filter + RMS noise metric + per-instance init fix | `VehicleMagnetometer.cpp` has none of this; depends on removed `LowPassFilter1p.hpp` | PORT - H/M/M |
| `fc334f30` | #38 | "OBManualControl" — `COM_RC_IN_MODE=5` toggle between RC/Mavlink-joystick | `ManualControlSelector` now enum-based (`RcInMode`); reimplement as a new case — mode-number choice needs domain review | PORT - H/H/H |
| `c392d6d1` | #58 | Safety-pilot control-source selector (core feature, see §2) | Must be rebuilt against new versioned msg schema + current `ManualControl`/`sPort_data.cpp` | PORT WITH #38 - H/H/H |
| `988f4bc0` | #62 | Manual-control refactor (adds `sees_manual_control_data` topic + counters) | `ManualControl.cpp` is pure vanilla upstream; message doesn't exist — rebuild from scratch | PORT WITH #38 - H/H/H |
| `60142574` | #68 | Use `sees_manual_control_data` counters instead of `rc_channels.signal_lost` in SYS_STATUS | Fold into #62's reimplementation | PORT WITH #38 - H/H/H |
| `72da9ff2` | #72 | Recheck manual-control inputs on source toggle to avoid spurious "control lost" | Fold into #62's reimplementation (navigator part is moot, see §3.3) | PORT WITH #38 H/H/H |
| `e116be69` | #50 | Delay GPS node-124 publish until node-125 (Rover) claims uORB instance 0 | Confirm dual-CAN-GPS node-ID assumptions still match fleet config; upstream now uses a different channel-index mechanism | DEFER_UNTIL_TEST, POSSIBLY BACKPORT FROM LATEST MASTER |
| `77b5150e` | #69 | `UAVCAN_ROVER_ID` param for configurable CAN node ID | Verify whether the instance-ordering problem still exists under the new upstream mechanism before rebuilding | PORT WITH #50 - H/L/L |
| `1ecc00e7` | #52 | New outbound `GPS_INPUT` mavlink stream (NED velocity + uncertainty) | Stream mechanism intact, but file must be recreated (no conflicting upstream feature) | NEEDS MAVLINK UPDATE - USE PX4 MAVLINK - PORT - H/L/L |
| `7ae25eee` | #53 | Per-instance mag-filter warning (only alert on primary) | Depends on the undocumented base mag-filtering feature (`b932235e`) — confirm that feature is still wanted before building this on top | PORT - L/L/L |
| `6d28a6dd` | #64 | "AWS" audio triggers for external BVLOS system on kill/disarm/landing/critical-SoC | Confirm the external AWS integration is still required before rebuilding against the restructured Commander tune logic | PORT - H/L/L |
| `72657ad9` | #65 | GPS-loss failsafe: fall to Altitude/Manual before Loiter; force RC-aware offboard-loss response | `state_machine_helper.cpp` replaced by `commander/failsafe/{failsafe.cpp,framework.cpp}` — reimplement against new framework | PORT - H/M/H |
| `b90eb056` | #89 | `MC_YAW_ACC_MAX` param via SlewRate limiter on yaw stick | `StickYaw` moved to `src/lib/stick_yaw/`, rewritten to an LPF/error-convergence approach — no SlewRate member left | DEFER_UNTIL_TEST - H/L/L |
| `b7aa8c4b` + `f918007e` | #94, #96 | `UAVCAN_COMPID_1/2` auto-detection for param-management tooling | Needs the (unported) rover-ID prerequisite logic re-anchored first | DEFER_UNTIL_TEST - H/L/L |
| `4ab4a120` | #92 | GPS submodule: F9P rate 7Hz, UART2 baud 921600 | Needs re-pointing the GPS driver submodule to a Sees fork rebased on new upstream — hardware tuning, not a cherry-pick | REVIEW - BACKPORT FROM 1.17 or MASTER M/L/L |
| `bb02f594` | #97 | `num_channels_lost` field + <900µs signal-lost detection; refactor RC/Mav input reassessment | `rc_channels.msg` path moved; also depends on unported manual-control-selector prerequisite | PORT - H/L/L |
| `445f7a3a` | #41 | RFD868 link-budget mavlink stream/rate tuning | RF-engineering call — reapply against current defaults once decided | NO_PORT - UNUSED |

### 3.3 Likely obsolete/superseded (15) — verify, then drop

| Commit | PR | Subject | Why it's moot | I/E/R score, Action |
|---|---|---|---|---|
| `1d5bc57f` | #28 | Mavlink shell fflush fix | Upstream already carries the equivalent fix | DEFER_UNTIL_TEST (CHECK IF OUR PR IS A BACKPORT) |
| `d8b4aeb7` | #46 | CAN GPS hardcoded-node stop-gap | Superseded by the author's own follow-up `e116be69` (#50) | DEFER_UNTIL_TEST |
| `f9e0294f` | #44 | Mag-always-log + rate tuning | `logged_topics.cpp` rewritten; mag already logged by default | DEFER_UNTIL_TEST |
| `b7ad225f` | #51 | Param/log cleanup (`drag_estimator` default, mavlink_log spam removal, `PLD_TARGET_YAW`, `SENS_MAG_LP_CUT`) | `drag_estimator` module removed; both params removed upstream; log cleanup already done independently | DEFER_UNTIL_TEST |
| `53a4ad59` | #55 | Unlimited high-rate topics for FFT | Upstream defaults already log these near-unlimited | DEFER_UNTIL_TEST |
| `287fb4ba` | #39 | Blank-logs-need-refresh fix + stdio redirect | Log handler fully rewritten (old bug pattern gone); stdio fix already present upstream | NO_PORT |
| `758d575d` | #82 | CMake `BOARD_LINUX`→`BOARD_LINUX_TARGET` rename | Identical rename already present upstream | NO_PORT |
| `a98af32c` | #81 | GPS_INPUT accuracy fields + mavlink submodule bump | Stream removed entirely upstream; submodule already newer | DEFER_UNTIL_TEST |
| `8786adcc` | #70 | ADS-B vertical separation + time-to-flyby params | Traffic avoidance rewritten with equivalent `NAV_TRAFF_A_VER`/`NAV_TRAFF_COLL_T` params | DEFER_UNTIL_TEST M/?/? |
| `319cdf8d` | #88 | Keep offboard-acceleration mode alive on position loss | New gating condition (`attitude_invalid` vs `local_velocity_invalid`) already achieves this | DEFER_UNTIL_TEST |
| `57ca578a` | #93 | `GF_SEES_STOP` — geofence loiter holds at current position | New `GF_PREDICT=0` default already produces the same stop-and-hold behavior | DEFER_UNTIL_TEST - H/M/H |
| `f2d54bbc` | #85 | High-rate actuator/torque logging for FFT plots | Already logged at max rate unconditionally upstream | DEFER_UNTIL_TEST |
| `253ec2c0` | #90 | CI action/runner version bumps | Stale; CI has long since moved past these versions | DEFER_UNTIL_TEST L/L/L |
| `218c1095` | #98 | Fix discarded-return-value bug in distance-sensor pre-arm check | `PreFlightCheck.cpp` replaced by `HealthAndArmingChecks/`; new code doesn't have this bug | DEFER_UNTIL_TEST - M/L/L |
| `7d2ac4b9` | #99 | Custom-DSDL `noise_per_ms` field exposed over UAVCAN | `v1.17.0-dev` already carries `noise_per_ms` end-to-end via a different (submodule-free) mechanism | NO_PORT - BACKPORTED |
