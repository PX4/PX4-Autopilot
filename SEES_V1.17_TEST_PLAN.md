# Sees Fork: Test Plan for Porting Custom Commits onto v1.17.0-dev

**Companion to:** `SEES_V1.17_MIGRATION.md` (architecture/porting assessment)
**Purpose:** test-driven release — for every Sees-specific behavior change carried over from `v1.13.1_dev`, define a test that proves the behavior on `v1.17.0-dev` matches (or knowingly improves on) the behavior on `v1.13.1_dev`, before we ship.
**Source:** commit diffs (`v1.13.1_dev` vs `v1.17.0-dev`) + the original GitHub PR descriptions for PRs #26–#99 in `SEESAI/Firmware`, which included problem statements, linked Jira tickets, and (for several) prior Notion test write-ups.

---

## 1. Methodology

For each PR we're porting, a test case defines: the behavior, how to exercise it, what "pass" looked like on `v1.13.1_dev`, and what "pass" must look like on `v1.17.0-dev` post-port. Four execution lanes, cheapest first:

| Lane | What it covers | Tooling |
|---|---|---|
| **Unit** | Pure logic (filters, conversions, param clamping) | `make tests`, gtest (Sees already has `SeesSOCTest.cpp` precedent from PR #95) |
| **SITL** | Module interaction, failsafe/mode-transition logic, mavlink streams | `make px4_sitl`, MAVSDK/pymavlink scripts, `simulator_mavlink` failure injection |
| **HITL/Bench** | Driver-level behavior needing real peripherals (CAN GPS, batmon, IMU, radios) | Bench rig with the specific sensor/radio, `nsh` console, log inspection (`ulog`) |
| **Flight** | End-to-end behavior only observable in flight (GPS RTK quality, magnetic interference, RF link budget) | Test flight per existing Sees flight-test procedure |

**Pass criterion, generally:** behavior on `v1.17.0-dev` == behavior on `v1.13.1_dev`, captured as a `ulog` diff or MAVSDK assertion, *unless* the migration assessment already found the behavior superseded by an upstream mechanism — in which case the test proves the *outcome* is preserved even though the *implementation* differs (see §5).

**Baseline capture:** before writing/running any "after" test, run it against `v1.13.1_dev` first (or against existing flight logs where the bug/fix is already documented) to pin down the actual current behavior — several of these PRs reference incidents (e.g. #97's 3m altitude drop) that are worth reproducing once on the old baseline so the "after" assertion is concrete, not assumed.

---

## 2. Test case catalog, by feature area

### 2.1 Manual-control source selector (RC ↔ Mavlink-joystick toggle) — highest priority

PRs: #38, #58, #62, #68, #72, #97. This is Sees' core safety-pilot handover mechanism (local RC pilot ↔ backup GCS joystick) and carries the highest-consequence bugs found in the whole set — #97 documents a **3m altitude drop** and an **Offboard-control steal** from RC-channel recovery races. `ManualControl.cpp` and the message schema were both rewritten upstream, so this is a full reimplementation, not a cherry-pick — write these tests *before* reimplementing, as the spec for the new code.

| ID | Behavior (source PR) | Test | Pre-port (v1.13.1) expectation |
|---|---|---|---|
| MC-01 | Toggle RC↔Mav control source via RC switch and via Mavlink command (#38) | SITL: start with RC source active, trigger switch/command toggle, assert `manual_control_setpoint.data_source` flips within 1 cycle | Source flips cleanly, no intermediate invalid state |
| MC-02 | Control-source telemetry reflects actual state incl. "No Source" (#58) | Bench/SITL: present only RC (not desired source) → assert FRSky/telemetry field shows current real source, not last-valid | Fixed in #58 (was a stale-value display bug) |
| MC-03 | Kill switch works while in "No Source" state (#62, Jira S1-4385) | SITL: force both sources invalid → assert kill switch still disarms | Must not be silently ignored |
| MC-04 | QGC/backup-GCS connection status only counts joystick-providing Mavlink connections (#62, Jira S1-4467) | SITL: connect a passive Mavlink client (e.g. simulated RTK sender, no joystick) alongside the real backup-GCS joystick → assert `sees_manual_control_data` connected-count reflects only the joystick source | Passive connections must not be counted as "backup GCS present" |
| MC-05 | PX4 switches to local RC automatically on Taranis mode change (#62, Jira S1-3792) | Bench: with Mav source active, change flight mode via Taranis switch → assert source reverts to RC | |
| MC-06 | MAV/RC transition status message content correct (#62, Jira S1-4401) | SITL: trigger a transition, capture STATUSTEXT/telemetry message → assert message matches actual transition, not stale/wrong direction | |
| MC-07 | RC-connected indicator doesn't falsely show "connected" when RC was never powered on (#68, Jira S1-4689) | Bench: boot vehicle with RC receiver powered off throughout → assert `valid_rc_setpoint_count`-derived indicator stays "not connected" (old `signal_lost`-flag logic never initializes and would falsely show connected) | Regression test — this was the actual bug being fixed |
| MC-08 | No spurious "Manual Control Lost" on desired-source toggle (#72, Jira S1-4768) | SITL: toggle desired source repeatedly at various points in the assessment cycle → assert no single-cycle "lost" event fires during the handover | |
| MC-09 | Flight-mode-switch-triggered toggle doesn't cause a 1-cycle dropout flag (#97 Issue 1) | SITL: toggle control source via flight-mode switch specifically (not the dedicated control-source switch) → assert immediate reassessment prevents stale-invalid publish | |
| MC-10 | **Staggered RC channel recovery does not produce spurious stick/mode-switch inputs** (#97 Issue 2 — safety-critical, caused a 3m altitude drop and an Offboard-control steal in the field) | Bench: simulate RC signal loss then recovery where individual channel values return to valid PWM (900–2000) at slightly different times → assert RC-signal-lost flag is held until **all** channels are within bounds, and no transient max-deflection stick or flight-mode-switch value is acted on during partial recovery | **Must reproduce the original incident conditions as a regression test** before considering this port complete |
| MC-11 | `input_rc`/`rc_channels`/`manual_control_input` logged at increased rate (#97) | Log inspection: confirm logging rates (10Hz) present post-port for post-incident diagnosability | |

> Recommend writing MC-10 as a SITL/bench regression test *first*, independent of the port, since it's the highest-severity behavior in the whole set.

### 2.2 Battery SOC / chemistry curve

PRs: #30, #56, #57, #61, #95. Upstream rewrote `battery.cpp` (RLS estimator replacing the lookup-table approach), so ports here need domain sign-off (see migration doc §3.4) before test cases can be finalized — but the *behavioral* tests below should hold regardless of implementation.

| ID | Behavior | Test | Notes |
|---|---|---|---|
| BAT-01 | Disarmed SOC estimate is voltage-based via lookup/curve (#30) | Bench: apply known cell voltages via bench PSU, disarmed → assert reported SOC matches expected curve value within tolerance | Needs current pack curve (25Ah Tattu HV per #61, or Reference Chemistry Curve v1.2 per #95 — **confirm which is current with Sees battery team**) |
| BAT-02 | Armed SOC estimate = initial SOC − coulomb-counted discharge (#30) | Bench: discharge known mAh through the monitored path while armed → assert SOC delta matches within tolerance | |
| BAT-03 | Low-cell-voltage warning fires at correct threshold (#30, #61) | Bench: drop average cell voltage below threshold → assert mavlink warning fires; confirm it does **not** fire for the retired 20Ah Overlander thresholds if those packs are decommissioned (#61 explicitly warns old packs are unsafe with new curve) | Confirm Overlander packs are actually retired fleet-wide before dropping their curve |
| BAT-04 | Batmon current reading doesn't clip at high current (#56) | Bench: draw >32.7A (mA-register range) through Batmon while logging → assert `BATT_SMBUS_DECI_CURRENT` fallback engages and reported current tracks the bench load correctly (not clipped) | |
| BAT-05 | Deci-current conversion factor correct (#57 — fixes a bug introduced in #56 itself) | Unit/bench: known deci-amp register value → assert conversion divides by 10, not 100 | Regression test for the original off-by-10x bug |
| BAT-06 | SeesSOC unit tests pass (#95 added `SeesSOCTest.cpp`) | Unit: run existing Sees SOC gtest suite against the reimplemented battery lib | Reuse/port the existing test file rather than writing new assertions from scratch |

### 2.3 Dual-CAN-GPS instance ordering & rover ID

PRs: #31, #50, #69, #74, #92, #94, #96. Upstream replaced the node-ID ordering hack with `get_channel_index_for_node()` — **first confirm on the bench whether the original instance-ordering problem (#50/#69) still reproduces under the new upstream mechanism** before porting anything here; if it doesn't reproduce, most of this section becomes a verification-only exercise (§5).

| ID | Behavior | Test | Notes |
|---|---|---|---|
| GPS-01 | **Baseline check — does the dual-GPS uORB-instance race still occur on stock v1.17.0-dev?** | Bench: power up both CAN GPS units simultaneously (varying boot order/timing across repeated trials) on unmodified v1.17.0-dev → record whether Rover consistently lands on `sensor_gps` instance 0 | Run this *first* — determines whether GPS-02/03 are still needed at all |
| GPS-02 | Rover GPS (configurable CAN ID, default 125) always initializes as uORB instance 0 (#50, #69) | Bench: swap which physical GPS unit holds the Rover CAN ID → assert instance-0 assignment follows the ID, not physical port/boot order | Only if GPS-01 shows the race is still present |
| GPS-03 | UART bridge baud rate is 460800, not the PX4-default 230400, for dual-F9P RTK (#31) | Bench: verify actual UART baud on the bridge link; confirm no dropped/garbled RTCM frames at load | |
| GPS-04 | GPS timeout and primary-GPS-switch events produce rate-limited (not spammy) mavlink warnings (#31, and the #36 fix for un-throttled repeats) | Bench: induce a GPS timeout → assert one warning, not a warning storm | |
| GPS-05 | F9P polling at 7Hz doesn't drop fixes (vs. 8Hz causing drops per #92's investigation) | Flight/bench: log fix continuity over a representative session at 7Hz | Check whether the later upstream GPS-driver fix referenced in #92 (PX4-GPSDrivers commit `bbdd576`) has since landed in the pinned driver version — if so, the manual 7Hz workaround may itself be droppable |
| GPS-06 | GPS heading UART2 baud is 921600 (not 460800) (#92) | Bench: verify baud on that link | |
| GPS-07 | `UAVCAN_COMPID_1`/`UAVCAN_COMPID_2` correctly populate with attached CAN GPS component IDs, default `-1` (not stale 124/125) when no GPS attached (#94, #96) | Bench: boot with 0, 1, and 2 CAN GPS units attached → assert param values in each case | |
| GPS-08 | In-flight GPS failure injection (`SYS_FAILURE_EN`) still triggers full GPS dropout on command, for both UART and CAN GPS paths (#74) | SITL/bench: issue the nsh failure-injection command for each GPS path → assert complete signal loss is simulated | Test tooling itself, used to validate failsafe behavior (§2.6) |

### 2.4 Magnetometer filtering

PRs: #33, #36, #53. Depends on rebuilding the base filtering feature (`LowPassFilter1p.hpp`/two-stage LPF), which doesn't exist in v1.17.0-dev at all — confirm this is still needed (powerline/pylon 50Hz interference still a real operating condition) before rebuilding.

| ID | Behavior | Test | Notes |
|---|---|---|---|
| MAG-01 | Mag sampled at 140Hz (not vanilla 50Hz), filtered with two cascaded 1st-order LPFs at 10Hz cutoff (#33) | Bench: inject/observe 50Hz noise (e.g. run near known powerline source, or bench signal injection if available) → assert filtered output suppresses the 50Hz component vs. an unfiltered baseline | Original driver for this: flights near grid infrastructure |
| MAG-02 | Filter cutoff frequency is parameterized and can be disabled without recompiling (#33) | Bench: change the cutoff param at runtime → assert filter response changes accordingly; set to disable value → assert passthrough | |
| MAG-03 | Filter initializes correctly for **all** mag instances, not just one (#36 regression test) | Bench: system with 2+ mag instances → assert both show filtered output, not just instance 0 | This was a real shipped bug — worth a permanent regression test |
| MAG-04 | Filter-disabled warning only alerts for the **primary** mag, not every instance (#53) | Bench: force filter-disable condition on a non-primary (e.g. CAN) mag while primary I2C mag filters fine → assert no repeated warning spam; force it on primary → assert warning does fire | |

### 2.5 FRSky / Horus telemetry

PRs: #29, #45, #41. Standalone from the other stacks but depends on GPS (§2.3) and manual-control (§2.1) data for some fields.

| ID | Behavior | Test | Notes |
|---|---|---|---|
| TLM-01 | Flight-mode numbering on Taranis display matches Sees' remap table (#29) | Bench w/ Taranis: cycle flight modes → assert displayed codes match expected remap, not vanilla PX4 numbering | Remap table itself flagged for domain review in migration doc |
| TLM-02 | Secondary GPS fix type surfaced over FRSky (#29) | Bench: vary secondary GPS fix type → assert Taranis display updates accordingly | |
| TLM-03 | RADIO_STATUS mavlink stream present and populated (#29) | SITL/bench: subscribe to RADIO_STATUS → assert non-null/sane values | |
| TLM-04 | Telem3 (`/dev/ttyS4`) carries FRSky telemetry without conflict from any onboard ADS-B mavlink connection (#29) | Bench: confirm no competing mavlink instance is bound to ttyS4 on boards without ADS-B in use | Original bug: cube-pilot ADS-B carrier board auto-added a conflicting connection |
| TLM-05 | Horus DIY Smartport streams (dual-GPS, RC/Mav status, flight mode) use dedicated poll IDs, not hijacked fields (#45) | Bench w/ Horus: verify each DIY stream ID decodes to the correct semantic field, and no other display field (e.g. Vertical Speed) is being repurposed | |
| TLM-06 | Custom Mavlink mode (RFD868 link) stays within RFD's ~20kbps budget (#41) | Bench/flight: measure actual throughput of `MAVLINK_MODE_CUSTOM` stream set → assert ≤ ~15kbps as originally tuned, or re-tune and document the new figure | Needs RF-engineering re-validation regardless (flagged in migration doc §3.4) since upstream default streams differ now |

### 2.6 Navigator / failsafe behavior

PRs: #59, #65, #40, #64, #60, #84.

| ID | Behavior | Test | Notes |
|---|---|---|---|
| NAV-01 | Garbage `NAV_ACC_RAD` value doesn't cause uncontrolled yaw on auto-takeoff (#59) | SITL: set `NAV_ACC_RAD` to an out-of-range/garbage value → auto-takeoff → assert acceptance radius clamps to 1.0 and yaw stays controlled | Original bug had a documented flight incident |
| NAV-02 | GPS loss in Hold or Offboard falls back to Altitude (if manual control + height source present), else Stabilised (if manual control but no height source), instead of straight to Descend/Terminate (#65) | SITL: simulate GPS loss (using GPS-08's failure injector) while in Hold and while in Offboard, with/without a valid manual-control source → assert correct fallback mode in each combination | 4 combinations to cover: {Hold, Offboard} × {manual source present, absent} |
| NAV-03 | Precision-land holds position after beacon-search retries exhausted, instead of falling through to normal land (#40) | SITL/bench: run precision land with no beacon present → assert vehicle climbs/descends to search altitude and holds, rather than executing a normal land | Safety behavior — verify it doesn't silently regress to old "just land" behavior |
| NAV-04 | AWS audible/telemetry triggers fire in all required BVLOS scenarios (#64): all Land-family modes (Land, Precision Land, Descent), all Low-Battery warning levels including Emergency (<5%), and all kill paths (RC kill, BackupGCS forced-disarm) | SITL: exercise each of the 3 scenario groups individually → assert AWS trigger fires each time, specifically confirming the Emergency/<5% SOC case (explicitly called out as a prior PX4 bug where it didn't) | |
| NAV-05 | Audible tone plays on kill-switch engage and on entering AUTO_RTL (#60) | Bench: trigger kill switch; separately trigger RTL → assert `tune_control` fires in both cases | |
| NAV-06 | Offboard acceleration setpoints under `MAV_FRAME_BODY_NED` are rotated consistently with velocity setpoints (#84) | SITL: send offboard acceleration and velocity setpoints in the same frame with a non-zero vehicle yaw → assert both are transformed the same way (yaw-only rotation, matching current velocity handling) | References open upstream bug PX4#23255 — check if it's been fixed differently upstream before porting |

### 2.7 Standalone driver/system fixes

| ID | PR | Behavior | Test |
|---|---|---|---|
| SYS-01 | #32 | Hygrometer (SHT3x) driver enabled by default on cubeorange | Bench: boot board → `hygrometer status` / `dmesg` shows driver active |
| SYS-02 | #37 | `GPS_RTCM_DATA` stream flags valid RTK correction reception to SI2 | SITL/bench: feed RTCM corrections → assert stream reflects reception |
| SYS-03 | #47 | `landing_target_estimator` auto-starts on boot (not requiring SD-card hotfix) | Bench: fresh boot with `SENS_EN_IRLOCK=1` → assert module running without manual `extras.txt` |
| SYS-04 | #48 | Accelerometer error flags (FIFO overflow/empty, bad transfer/register, temp fault) visible via `dmesg` | Bench: induce each error condition (or mock at driver level) → assert distinct flag/message per error type, not a single generic counter |
| SYS-05 | #54 | UAVCAN param read via Mavlink doesn't get stuck (dequeue-before-enqueue fix) | SITL/bench: issue repeated `PARAM_REQUEST_READ` for a CAN-sourced param → assert no stall/stuck-queue after repeated requests |
| SYS-06 | #63 | 4th mavlink instance available and functions with correct default param values | Bench: configure and connect on the 4th instance → assert normal mavlink traffic; confirm default param values are set per the linked Notion setup notes |
| SYS-07 | #66, #67 | STATUSTEXT delivered to onboard-controller-type (`MAVLINK_MODE_ONBOARD`) connections without requiring a GCS to also be connected | SITL: connect only an onboard-mode client, no GCS → assert STATUSTEXT messages are received |
| SYS-08 | #86 | IRLock / `landing_target_pose` logged by default (not opt-in) | Log inspection: confirm topics present in a default-config log without extra enablement |

---

## 3. Test environment / bench prerequisites

To execute the above without waiting for flight tests, the following bench setups are needed up front:

- **SITL** with MAVSDK/pymavlink scripting — covers most of §2.1, §2.6, parts of §2.2/§2.3/§2.7.
- **Dual CAN-GPS bench rig** (2× F9P or equivalent on CAN, swappable node IDs) — §2.3.
- **Batmon/battery simulator or bench PSU with programmable load** — §2.2.
- **IMU/accelerometer fault-injection capability** (or mock at driver boundary) — SYS-04.
- **FRSky Taranis + Horus radios**, and an **RFD868 link** — §2.5.
- **Precision-land beacon rig** (with the ability to *withhold* the beacon) — NAV-03.
- **A documented current battery pack spec** (cell chemistry/capacity in active fleet use) to settle BAT-01/03's open question before writing final assertions.

---

## 4. Prioritization

Given limited bench/flight time, suggested order:

1. **§2.1 (MC-01…MC-11)** — safety-critical, has a documented field incident (3m altitude drop, Offboard steal). Test MC-10 as a standalone regression case before anything else, even before the reimplementation is designed.
2. **§2.6 NAV-02, NAV-03** — failsafe/precision-land behavior, safety-critical.
3. **§2.3 GPS-01** — determines how much of the GPS section is even still needed; run before investing in GPS-02+.
4. Everything else, roughly in the order listed.

---

## 5. Obsolete/superseded items — verify before dropping, don't port

For the 15 commits the migration assessment marked obsolete/superseded, don't skip testing entirely — run a **one-time verification** that the upstream replacement genuinely preserves the outcome, then drop the old patch for good.

| PR | Original behavior | Verify on v1.17.0-dev |
|---|---|---|
| #28 | Mavlink shell fflush fix | Confirm mavlink shell/console works normally (no hang) — should already pass, upstream fix present |
| #39 | Log list refresh bug + stdio redirect | Confirm log listing doesn't require a "refresh" workaround |
| #44 | Mag always logged + rate tuning | Confirm mag topic present in default log at adequate rate for FFT analysis |
| #46 | CAN GPS node-125-only stopgap | Superseded by #50/#69's approach — covered by GPS-01/02 |
| #51 | Drag-estimator default, log spam removal, `PLD_TARGET_YAW`, `SENS_MAG_LP_CUT` | Confirm precland/LTE don't spam `mavlink_log_info`; confirm removed params aren't referenced anywhere in current configs/scripts |
| #55 | Unlimited high-rate FFT topics | Confirm `vehicle_torque_setpoint`/`vehicle_thrust_setpoint` are logged at effectively unlimited rate by default — covers the same FFT-analysis need as #85 |
| #70 | ADS-B vertical separation + time-to-flyby | Confirm `NAV_TRAFF_A_VER`/`NAV_TRAFF_COLL_T` produce equivalent traffic-avoidance behavior to the old params |
| #81 | GPS_INPUT accuracy fields | Confirm downstream consumers (SI2/MAVSDK) don't depend on the removed outbound stream; if they do, this needs to flip back to "port with adaptation" |
| #82 | CMake ≥3.25 fix | Just confirm the build succeeds with current CMake — no behavioral test needed |
| #85 | High-rate actuator/torque logging for FFT | Confirm equivalent to #55's check above |
| #88 | Offboard-acceleration mode survives position loss | SITL: repeat the original scenario (position loss while in offboard acceleration mode) → assert control is retained via the new `attitude_invalid` gating |
| #90 | CI artifact/runner versions | N/A — CI plumbing only |
| #93 | Geofence predictive-stop (`GF_SEES_STOP`) | SITL: trigger a geofence breach while in a mode that would loiter → confirm vehicle stops/holds at (approximately) current position with default `GF_PREDICT=0`, not a projected point |
| #98 | Distance-sensor pre-arm check discarded-return bug | Bench: remove/fault the distance sensor when it's required → confirm arming is correctly blocked (new `HealthAndArmingChecks` framework should already do this correctly) |
| #99 | Custom-DSDL `noise_per_ms` over UAVCAN | Confirm `sensor_gps.noise_per_ms` is populated end-to-end via the new (non-DSDL) mechanism already on v1.17.0-dev |

---

## 6. Blocked pending a Sees domain decision

These 8 items need an answer from flight-test/ops/RF engineering before a test case can even be written (see `SEES_V1.17_MIGRATION.md` §3.4 for full detail):

| PR | Open question |
|---|---|
| #41 | Is the RFD868 ~20kbps link budget and stream set still current for the radio in use? |
| #50, #69 | Does the dual-CAN-GPS instance-ordering problem still occur under v1.17.0-dev's `get_channel_index_for_node()`? (→ GPS-01 answers this) |
| #92 | Is the F9P submodule fork with the 7Hz/921600-baud tuning still needed, or did the referenced upstream GPS-driver fix land? |
| #61, #95 | Which battery curve is current fleet spec — 25Ah Tattu HV (#61) or the newer Reference Chemistry Curve v1.2 (#95)? Are 20Ah Overlander packs fully retired? |
| #53 | Is the underlying custom mag-filtering feature (§2.4) still wanted at all, given it was never ported and the field condition (powerline interference) may or may not still apply to current ops areas? |
| #64 | Is the external "AWS" BVLOS trigger system still in use? |

---

## Traceability matrix

| PR | Feature area | Test IDs | Verdict (migration doc) |
|---|---|---|---|
| #26 | Rate control / drag estimator | *(needs new test design — full module rebuild, not covered above; add once reimplementation design is settled)* | Port with adaptation |
| #29 | FRSky telemetry | TLM-01…04 | Port with adaptation |
| #30 | Battery SOC | BAT-01, BAT-02, BAT-03 | Port with adaptation |
| #31 | GPS UART bridge | GPS-03, GPS-04 | Port with adaptation |
| #32 | Hygrometer kconfig | SYS-01 | Port as-is |
| #33 | Mag filter | MAG-01, MAG-02 | Port with adaptation |
| #36 | Mag/GPS bug fixes | MAG-03, GPS-04 | Port with adaptation |
| #37 | GPS_RTCM_DATA stream | SYS-02 | Port as-is |
| #38 | Manual control source | MC-01 | Port with adaptation |
| #40 | Precland hold | NAV-03 | Port as-is |
| #41 | RFD stream budget | TLM-06 | Needs domain review |
| #45 | Horus telemetry | TLM-05 | Port with adaptation |
| #47 | LTE autostart | SYS-03 | Port as-is |
| #48 | Accel error flags | SYS-04 | Port as-is |
| #50 | CAN GPS ordering | GPS-01, GPS-02 | Needs domain review |
| #52 | GPS_INPUT UTC velocity stream | *(new stream — add a test once recreated)* | Port with adaptation |
| #53 | Mag filter warning | MAG-04 | Needs domain review |
| #54 | UAVCAN param dequeue | SYS-05 | Port as-is |
| #56, #57 | Batmon deci-current | BAT-04, BAT-05 | Port as-is |
| #58 | Control-source display fix | MC-02 | Port with adaptation |
| #59 | Yaw-on-takeoff | NAV-01 | Port as-is |
| #60 | Kill/RTL beeps | NAV-05 | Port as-is |
| #61 | Battery curve (Tattu HV) | BAT-01, BAT-03 | Needs domain review |
| #62 | Manual control refactor | MC-03…06 | Port with adaptation |
| #63 | 4th mavlink instance | SYS-06 | Port as-is |
| #64 | AWS scenario triggers | NAV-04 | Needs domain review |
| #65 | GPS-loss failsafe fallback | NAV-02 | Port with adaptation |
| #66, #67 | Onboard-controller STATUSTEXT | SYS-07 | Port as-is |
| #68 | RC-connected indicator fix | MC-07 | Port with adaptation |
| #69 | Rover CAN ID param | GPS-01, GPS-02 | Needs domain review |
| #72 | Manual control re-check on toggle | MC-08 | Port with adaptation |
| #74 | In-flight GPS failure injector | GPS-08 | Port as-is |
| #84 | Offboard accel frame fix | NAV-06 | Port as-is |
| #86 | IRLock default logging | SYS-08 | Port as-is |
| #89 | Yaw max acceleration param | *(needs new test — `StickYaw` rebuilt upstream)* | Port with adaptation |
| #92 | GPS submodule F9P/baud tuning | GPS-05, GPS-06 | Needs domain review |
| #94, #96 | UAVCAN COMPID params | GPS-07 | Port with adaptation |
| #95 | Battery chemistry curve v1.2 | BAT-01, BAT-06 | Needs domain review |
| #97 | RC/flight-mode transition fixes | MC-09, MC-10, MC-11 | Port with adaptation |

*(PRs already listed as obsolete in §5, and pure-CI/tooling PRs #82/#90, are intentionally excluded from this matrix — see §5.)*
