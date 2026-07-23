# Detect and Avoid

Detect and Avoid (DAA) runs in the [`navigator`](../modules/modules_controller.md#navigator) module.
Its `DetectAndAvoid` component uses the `adsb` library for conflict evaluation, traffic identification, conflict tracking, and action policy (see [Implementation Structure](#implementation-structure)).
It evaluates cooperative ("transponder-equipped") traffic, such as that from ADS-B, FLARM, or UTM integrations, that is reported through the [`transponder_report`](../msg_docs/TransponderReport.md) topic.
DAA can warn the operator, request navigator actions, or block arming when a conflict requires an automatic response.

:::tip
This page is the detailed technical reference for PX4 developers and advanced integrators looking to maintain and extend the DAA implementation.

For basic setup and configuration of an avoidance system, see [ADS-B/FLARM/UTM Receivers: Air Traffic Avoidance](../peripherals/adsb_flarm.md).
:::

## Scope and Context

DAA is most relevant for operations in shared airspace, especially beyond visual line of sight (BVLOS), where an unmanned aircraft must help maintain safe separation from manned aviation without an onboard pilot performing see-and-avoid.

PX4 currently handles only cooperative traffic: aircraft that broadcast position and related state through ADS-B, FLARM, UTM, or another compatible integration.
If you are not using one of those traffic sources, this page is mainly relevant only if you are developing or integrating DAA itself.

This page uses _encounter_ for a received cooperative-traffic report and _conflict_ when that traffic breaches a configured alert test.

Use this page for understanding:

- The detailed behavior of [Conflict Standards](#conflict-standards)
- Action selection and preflight behavior in [Automated Actions](#automated-actions) and [Arming, Preflight, and Ground Behavior](#arming-preflight-and-ground-behavior)
- Message, logging, and test details in [Operator Messages](#operator-messages), [Analysing a Conflict Using the Logs](#analysing-a-conflict-using-the-logs), and [Testing and Simulation](#testing-and-simulation)
- Extension guidance in [Adding a New Standard](#adding-a-new-standard)

## Загальний огляд

DAA supports two conflict models:

- `Crosstrack` mode raises one conflict level when ownship (the current vehicle) is close to the traffic's predicted track, vertically close, and within a configured collision-time threshold.
  Use it when you want one threshold driven by `NAV_TRAFF_*`.
- `F3442` mode evaluates four alert tests derived from concepts in [ASTM F3442/F3442M-23](https://store.astm.org/f3442_f3442m-23.html).
  Use it when you want staged alerting and per-level actions.

`CONFIG_NAVIGATOR_ADSB` includes DAA in the firmware.
When it is enabled, `CONFIG_NAVIGATOR_ADSB_F3442` selects F3442 mode; otherwise Crosstrack mode is built.

:::warning
F3442 mode handles cooperative traffic only and implements selected alert concepts and thresholds.
It does not by itself establish compliance with ASTM F3442/F3442M-23, which applies to a complete DAA system and its compliance evidence.
The implementation references the 2023 edition and has not been evaluated against the later [ASTM F3442-25](https://store.astm.org/f3442-25.html) edition.
:::

At runtime DAA can:

- Process queued traffic reports from [`transponder_report`](../msg_docs/TransponderReport.md).
- Keep the most important active conflicts in a fixed-size buffer.
- Publish both per-traffic and most-urgent DAA status topics.
- Trigger configured navigator actions when a conflict requires more than a warning.
- Prevent arming if DAA is enabled and the current most urgent conflict requires an automatic action.

## How It Works

Each navigator cycle, DAA:

1. Reads all queued `transponder_report` messages for that cycle.
   - Rejects invalid or stale traffic reports.
   - Filters self-detections using ownship ICAO, ADS-B callsign, and UAS ID.
2. Evaluates the conflict with the built DAA model.
   - Publishes [`detect_and_avoid`](../msg_docs/DetectAndAvoid.md) for reports that create or update a tracked conflict, including the sample that clears one.
     First-seen traffic evaluated at conflict level `NONE` is not published.
3. Applies the result to the active-conflict buffer (`ConflictTracker`) and selects the most urgent conflict.
   - The tracker reports buffer changes (new, level changed, removed, ignored).
   - Publishes the most urgent conflict summary on [`detect_and_avoid_most_urgent`](../msg_docs/DetectAndAvoidMostUrgent.md).
4. Hands the cycle's change records to the notifier (`ConflictNotifier`), which decides all operator warnings for the cycle in one place, and, if configured, requests a navigator action.

Conflict priority is determined by:

- Higher conflict level first.
- Shorter aircraft distance when conflict levels are equal.

## Conflict Standards

### Crosstrack Mode

This mode is used by firmware built without `CONFIG_NAVIGATOR_ADSB_F3442`.

This mode provides a single actionable threshold for track-crossing traffic.
It evaluates the traffic geometry using three gates.
If all three conditions are met, DAA publishes a single conflict level: `HIGH`.

- The ownship must lie within `NAV_TRAFF_A_HOR` of the traffic's projected ground track.
- Vertical separation must be below `NAV_TRAFF_A_VER`.
- Estimated time to collision must be below `NAV_TRAFF_COLL_T`.

#### Metrics and Conflict Test

The crosstrack implementation publishes these geometry outputs in `detect_and_avoid`:

- `aircraft_dist`: the current point-to-point 3D range.
- `aircraft_dist_hor`: the signed crosstrack distance to the projected traffic line when that line can be constructed.
- `aircraft_dist_vert`: the absolute vertical separation.
- `expected_min_dist_time`: a conservative collision-time estimate.

:::details
Click here to view how the crosstrack metrics and gates are computed

The crosstrack implementation computes them as follows:

- It first computes the current point-to-point separations between the ownship and the traffic: $d_{hor}$, $d_{vert}$.

- It projects a traffic ground-track line starting at the reported traffic position and extending in the reported traffic heading direction.
  Let $d_{ext}$ denote the fixed path-extension constant `kTrafficToUavDistanceExtension`.
  Heading is required for this step.
  The line length is:

  $$
  d_{pred} = d_{hor} + d_{ext}
  $$

- It then computes the signed crosstrack error $d_{xt}$ from ownship to that projected line.
  The horizontal gate is:

  $$
  |d_{xt}| < \texttt{NAV\_TRAFF\_A\_HOR}
  $$

  The projected-line gate is only valid when the line solver succeeds and the ownship has not already passed the end of the projected segment.

- Vertical separation is checked directly:

  $$
  |d_{vert}| < \texttt{NAV\_TRAFF\_A\_VER}
  $$

- The time metric uses the current 3D separation and a conservative relative-speed assumption.
  This time calculation intentionally does **not** use heading to derive closure rate.
  It uses a worst-case head-on assumption and sums the ownship and traffic speed magnitudes:

  $$
  d_{xyz} = \sqrt{d_{hor}^2 + d_{vert}^2}
  $$

  $$
  v_{own} = \sqrt{v_{own,hor}^2 + v_{own,vert}^2}, \qquad
  v_{traf} = \sqrt{v_{traf,hor}^2 + v_{traf,vert}^2}
  $$

  $$
  t_{est} =
  \begin{cases}
  \dfrac{d_{xyz}}{v_{own} + v_{traf}}, & v_{own} + v_{traf} > 0 \\
  0, & \text{otherwise}
  \end{cases}
  $$

  The time gate is only valid when the summed speed is greater than zero:

  $$
  v_{own} + v_{traf} > 0 \quad \text{and} \quad t_{est} < \texttt{NAV\_TRAFF\_COLL\_T}
  $$

:::

A crosstrack conflict is raised only when all three gates are true.
If the line projection is valid, `aircraft_dist_hor` publishes $d_{xt}$; otherwise it falls back to the direct horizontal range $d_{hor}$.

#### Data Requirements

- Valid ownship and traffic coordinates, altitude, and velocity, plus a finite traffic heading.

#### Параметри

- [NAV_TRAFF_AVOID](../advanced_config/parameter_reference.md#NAV_TRAFF_AVOID): action requested when Crosstrack raises its single `HIGH` conflict.
- [NAV_TRAFF_A_HOR](../advanced_config/parameter_reference.md#NAV_TRAFF_A_HOR): maximum absolute crosstrack distance from the projected traffic track for the horizontal gate.
- [NAV_TRAFF_A_VER](../advanced_config/parameter_reference.md#NAV_TRAFF_A_VER): maximum absolute vertical separation for the vertical gate.
- [NAV_TRAFF_COLL_T](../advanced_config/parameter_reference.md#NAV_TRAFF_COLL_T): maximum conservative time-to-collision estimate for the time gate.

#### Actions

Action mapping is defined by `NAV_TRAFF_AVOID`:

- `0`: Disabled, `1`: Warn only, `2`: Return, `3`: Land, `4`: Hold, `5`: Terminate

### F3442 Mode

This mode is used by firmware built with `CONFIG_NAVIGATOR_ADSB_F3442`.

This mode uses selected alert concepts and default separation values associated with ASTM F3442/F3442M-23.

- **NMAC** is defined as coming within 100 ft (30 m) vertically and 500 ft (153 m) horizontally from other aircraft.
- **Loss of Well Clear (LoWC)** is defined more conservatively at 250 ft (76 m) vertically and 2000 ft (610 m) horizontally from other aircraft.

PX4 places one protective zone around the ownship and an identical zone around each detected aircraft, so the radius and height parameters below are defined _per aircraft_.
A conflict is raised when the two zones touch, which means the separation that triggers it is the **sum of both zones**.
For two identical aircraft that is simply twice the configured value, which is why the critical radius defaults to roughly half of the NMAC horizontal distance ([DAA_LVL_CRIT_RAD](../advanced_config/parameter_reference.md#DAA_LVL_CRIT_RAD) = `77 m`, giving `2 × 77 m ≈ 153 m`).

#### Conflict Levels

F3442 is evaluated in four severity levels and returns the first breached level in this order: `CRITICAL`, `HIGH`, `MEDIUM`, `LOW`.

| Conflict level | Meaning in DAA                                   | Zone                                                                                                                                                                                                                                                                                         | Дія                                                                                                                                           |
| -------------- | ------------------------------------------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------- |
| `CRITICAL`     | Near Mid-Air Collision (NMAC) | [DAA_LVL_CRIT_RAD](../advanced_config/parameter_reference.md#DAA_LVL_CRIT_RAD), [DAA_LVL_CRIT_HGT](../advanced_config/parameter_reference.md#DAA_LVL_CRIT_HGT) | [DAA_LVL_CRIT_ACT](../advanced_config/parameter_reference.md#DAA_LVL_CRIT_ACT) |
| `HIGH`         | Loss of Well Clear (LoWC)     | [DAA_LVL_HIGH_RAD](../advanced_config/parameter_reference.md#DAA_LVL_HIGH_RAD), [DAA_LVL_HIGH_HGT](../advanced_config/parameter_reference.md#DAA_LVL_HIGH_HGT) | [DAA_LVL_HIGH_ACT](../advanced_config/parameter_reference.md#DAA_LVL_HIGH_ACT) |
| `MEDIUM`       | Augmented NMAC                                   | NMAC bounds expanded using [DAA_LVL_MED_TIME](../advanced_config/parameter_reference.md#DAA_LVL_MED_TIME)                                                                                                                     | [DAA_LVL_MED_ACT](../advanced_config/parameter_reference.md#DAA_LVL_MED_ACT)   |
| `LOW`          | Augmented WC                                     | WC bounds expanded using [DAA_LVL_LOW_TIME](../advanced_config/parameter_reference.md#DAA_LVL_LOW_TIME)                                                                                                                       | [DAA_LVL_LOW_ACT](../advanced_config/parameter_reference.md#DAA_LVL_LOW_ACT)   |

The augmented `MEDIUM` and `LOW` tests enlarge each aircraft's bounds in proportion to its horizontal and vertical speed (scaled by [DAA_LVL_MED_TIME](../advanced_config/parameter_reference.md#DAA_LVL_MED_TIME) and [DAA_LVL_LOW_TIME](../advanced_config/parameter_reference.md#DAA_LVL_LOW_TIME)).
The four tests are evaluated in the priority order shown above.
Validated parameter ordering guarantees that `CRITICAL` also breaches `HIGH`, `MEDIUM`, and `LOW`; `HIGH` also breaches `LOW`; and `MEDIUM` also breaches `LOW`.
`HIGH` and `MEDIUM` are not ordered relative to each other.

:::details
Click here to view the F3442 zone computations

PX4 models F3442 using cylindrical horizontal and vertical bounds.
A level is breached only when both the horizontal and vertical inequalities for that level are satisfied.

Use the following notation:

- $d_{h}$, $d_{v}$: absolute horizontal and vertical separation between the aircraft.
- $R_{nmac}$, $H_{nmac}$: per-aircraft NMAC radius and height.
- $R_{wc}$, $H_{wc}$: per-aircraft WC radius and height.
- $t_{med}$, $t_{low}$: augmentation times for the `MEDIUM` and `LOW` levels.
- $v_{own,h}$, $v_{own,v}$: ownship absolute horizontal and vertical speed magnitudes.
- $v_{traf,h}$, $v_{traf,v}$: traffic absolute horizontal and vertical speed magnitudes.

Parameter mapping:

- [DAA_LVL_CRIT_RAD](../advanced_config/parameter_reference.md#DAA_LVL_CRIT_RAD) = $R_{nmac}$
- [DAA_LVL_CRIT_HGT](../advanced_config/parameter_reference.md#DAA_LVL_CRIT_HGT) = $H_{nmac}$
- [DAA_LVL_HIGH_RAD](../advanced_config/parameter_reference.md#DAA_LVL_HIGH_RAD) = $R_{wc}$
- [DAA_LVL_HIGH_HGT](../advanced_config/parameter_reference.md#DAA_LVL_HIGH_HGT) = $H_{wc}$
- [DAA_LVL_MED_TIME](../advanced_config/parameter_reference.md#DAA_LVL_MED_TIME) = $t_{med}$
- [DAA_LVL_LOW_TIME](../advanced_config/parameter_reference.md#DAA_LVL_LOW_TIME) = $t_{low}$

**CRITICAL: NMAC zone**

Per-aircraft volume:

$$
R = R_{nmac},\qquad H = H_{nmac}
$$

Combined breach condition:

$$
d_h \leq 2R_{nmac},\qquad d_v \leq 2H_{nmac}
$$

**HIGH: Loss of Well Clear (LoWC) zone**

Per-aircraft volume:

$$
R = R_{wc},\qquad H = H_{wc}
$$

Combined breach condition:

$$
d_h \leq 2R_{wc},\qquad d_v \leq 2H_{wc}
$$

**MEDIUM: Augmented NMAC**

Each aircraft starts with the NMAC cylinder and expands it using $t_{med}$:

$$
R_{own} = R_{nmac} + v_{own,h} \cdot t_{med}
$$

$$
H_{own} = H_{nmac} + v_{own,v} \cdot t_{med}
$$

$$
R_{traf} = R_{nmac} + v_{traf,h} \cdot t_{med}
$$

$$
H_{traf} = H_{nmac} + v_{traf,v} \cdot t_{med}
$$

Combined breach condition:

$$
d_h \leq R_{own} + R_{traf} = 2R_{nmac} + (v_{own,h} + v_{traf,h}) \cdot t_{med}
$$

$$
d_v \leq H_{own} + H_{traf} = 2H_{nmac} + (v_{own,v} + v_{traf,v}) \cdot t_{med}
$$

**LOW: Augmented WC**

Each aircraft starts with the WC cylinder and expands it using $t_{low}$:

$$
R_{own} = R_{wc} + v_{own,h} \cdot t_{low}
$$

$$
H_{own} = H_{wc} + v_{own,v} \cdot t_{low}
$$

$$
R_{traf} = R_{wc} + v_{traf,h} \cdot t_{low}
$$

$$
H_{traf} = H_{wc} + v_{traf,v} \cdot t_{low}
$$

Combined breach condition:

$$
d_h \leq R_{own} + R_{traf} = 2R_{wc} + (v_{own,h} + v_{traf,h}) \cdot t_{low}
$$

$$
d_v \leq H_{own} + H_{traf} = 2H_{wc} + (v_{own,v} + v_{traf,v}) \cdot t_{low}
$$

PX4 validates that WC base bounds are not smaller than NMAC base bounds, both augmentation times are non-negative, and the `LOW` augmentation time is not shorter than the `MEDIUM` time.
The conflict evaluation order remains `CRITICAL`, `HIGH`, `MEDIUM`, then `LOW`.

:::

#### Data Requirements

- **Mandatory:** Valid coordinates and altitude.
- **Optional:** Traffic velocity.
  - If missing, F3442 is evaluated assuming zero traffic horizontal/vertical speed.
  - If [DAA_EN_DFLT_VEL](../advanced_config/parameter_reference.md#DAA_EN_DFLT_VEL) is enabled, the traffic vertical speed is replaced by [DAA_DFLT_VEL](../advanced_config/parameter_reference.md#DAA_DFLT_VEL) (preserving the reported sign if known, otherwise assuming positive climb).
- **Not required:** Heading is not used. Each aircraft is wrapped in a symmetric zone, which is conservative and covers unexpected course changes.

#### Actions

Each level uses the action parameter listed in the table above.
If that action is `Disabled`, the policy checks only levels whose bounds are guaranteed to have been breached:

- `CRITICAL`: `CRITICAL -> HIGH -> MEDIUM -> LOW`
- `HIGH`: `HIGH -> LOW`
- `MEDIUM`: `MEDIUM -> LOW`
- `LOW`: `LOW`

The first action that is not `Disabled` is used.

## Automated Actions

The action parameter depends on the DAA model selected at build time: `NAV_TRAFF_AVOID` for Crosstrack builds and `DAA_LVL_*_ACT` for F3442 builds.

The DAA action parameters use these values:

- `0`: Disabled, `1`: Warn only, `2`: Return, `3`: Land, `4`: Hold, `5`: Terminate

:::warning
The parameter values are not in severity order.
Internally, automatic-action severity is strictly ordered as:

$$
\texttt{Disabled} < \texttt{Warn only} < \texttt{Hold} < \texttt{Return} < \texttt{Land} < \texttt{Terminate}
$$

:::

::: info

Changing an action parameter does not immediately re-evaluate buffered conflicts.
The updated setting is considered on a later change of the overall most-urgent conflict level, and automatic mode changes are only requested when that level increases.

:::

#### Conflict-to-Action Mapping

- `Crosstrack` only publishes `HIGH` conflicts, so its requested action comes directly from [NAV_TRAFF_AVOID](../advanced_config/parameter_reference.md#NAV_TRAFF_AVOID).
- `F3442` maps each conflict level through its corresponding `DAA_LVL_*_ACT` parameter.
- In `F3442`, a disabled action falls back only to levels guaranteed to have been breached: `CRITICAL -> HIGH -> MEDIUM -> LOW`, `HIGH -> LOW`, or `MEDIUM -> LOW`.

#### Navigator-State Equivalence

Before sending a vehicle command, PX4 maps the current navigator state into the `DAA_ACTION` severity ladder and only triggers an automatic action when the requested action is to change to "a more severe mode" than the current navigator state:

:::details
Click here to view the full navigator-state to DAA-action mapping

| Current navigator state                                                                                                   | Equivalent DAA action | Practical effect                                   |
| ------------------------------------------------------------------------------------------------------------------------- | --------------------- | -------------------------------------------------- |
| `AUTO_MISSION`, `AUTO_TAKEOFF`, `AUTO_FOLLOW_TARGET`, `AUTO_VTOL_TAKEOFF`, `GUIDED_COURSE`                                | `DISABLED`            | Any automatic DAA action may still escalate        |
| `ORBIT`, `AUTO_LOITER`                                                                                                    | `POSITION_HOLD_MODE`  | Only `Return`, `Land`, or `Terminate` can escalate |
| `AUTO_RTL`                                                                                                                | `RETURN_MODE`         | Only `Land` or `Terminate` can escalate            |
| `AUTO_LAND`, `DESCEND`, `AUTO_PRECLAND`, `MANUAL`, `ALTCTL`, `ALTITUDE_CRUISE`, `POSCTL`, `POSITION_SLOW`, `ACRO`, `STAB` | `LAND_MODE`           | Only `Terminate` can escalate                      |
| `TERMINATION`                                                                                                             | `TERMINATE`           | No stronger DAA action exists                      |
| `OFFBOARD`, `EXTERNAL1` through `EXTERNAL8`, and unknown states                                                           | `MAX_ACTION_VALUE`    | PX4 will not inject an automatic DAA mode change   |

Manual modes are intentionally treated as `LAND_MODE`.
That means DAA will not automatically switch a manually flown vehicle into Hold, Return, or Land; only `Terminate` is considered a stronger action than those modes.

:::

**Action behavior:**

- DAA only triggers an automatic action when the requested action is to change to "a more severe mode" than the current navigator state.
- Warnings are evaluated separately from automatic mode changes.
  A conflict configured as `Warn only` or stronger can still emit operator warnings and status messages even when DAA does not request a mode change because the current navigator state is already equivalent or more severe.
- DAA continues to publish status updates even when no automatic action is sent.
- Automatic actions are evaluated from changes in the overall most-urgent conflict level.
  DAA does not retry a command while that level remains unchanged, including when a different aircraft becomes most urgent at the same level.
- Conflict de-escalation is conservative.
  For example, DAA does not automatically resume the mission after a hold conflict is cleared.
- `OFFBOARD` is handled conservatively: DAA does not inject an automatic mode change there, so arming and operating procedures need to account for that.
- If the vehicle is landed, DAA warns instead of changing mode.

## Arming, Preflight, and Ground Behavior

The commander health check reads [`detect_and_avoid_most_urgent`](../msg_docs/DetectAndAvoidMostUrgent.md) and applies the result only while the vehicle is disarmed.

This section covers conflict-based DAA behavior when traffic is already present.
The separate traffic-system-presence arming check is configured by [COM_ARM_TRAFF](../advanced_config/parameter_reference.md#COM_ARM_TRAFF) and described in [ADS-B/FLARM/UTM Receivers > Arming Check](../peripherals/adsb_flarm.md#arming-check).

Preflight behavior:

- If DAA is enabled and the most urgent conflict requires an automatic action, arming is rejected until the conflict clears.
- If the most urgent conflict is configured as `Warn only`, arming is not blocked.
- Once the vehicle is armed, commander stops emitting the prearm DAA conflict warning.
  In-flight conflict handling continues through the navigator action logic and DAA status messages.

Ground behavior:

- If the vehicle is landed and a conflict requires an action, DAA reports a ground warning instead of changing flight mode.
- If the vehicle is disarmed, the operator is warned not to arm.
- If the vehicle is already armed on the ground, the operator is warned not to take off until the conflict is resolved.

This is especially important for:

- Ground operations with nearby traffic already inside an action-triggering volume.
- `OFFBOARD` use cases where arming protection may be the primary safeguard.

## Traffic Inputs and Identification

DAA consumes the [`transponder_report`](../msg_docs/TransponderReport.md) topic, which can be populated by ADS-B, FLARM, UTM, or another integration.
PX4 maps incoming MAVLink [`ADSB_VEHICLE`](https://mavlink.io/en/messages/common.html#ADSB_VEHICLE) messages to this topic.
Other integrations must provide their own compatible `transponder_report` publisher.

:::info
PX4 does not currently map incoming MAVLink [`UTM_GLOBAL_POSITION`](https://mavlink.io/en/messages/common.html#UTM_GLOBAL_POSITION) messages to `transponder_report`.
The similarly named MAVLink stream in PX4 sends ownship UTM state; it is not a traffic-input adapter.
:::

Reports without a usable identifier are ignored before they enter the active-conflict buffer.

DAA chooses one identifier per traffic report in this order:

1. ICAO address
2. ADS-B callsign
3. UAS ID

The selected identifier is published on both `detect_and_avoid` and `detect_and_avoid_most_urgent` as:

- `unique_id`: the encoded 64-bit identifier value
- `unique_id_encoding`: `ICAO`, `ADSB_CALLSIGN`, or `UAS_ID`

The encoding is part of the traffic key.
That means an ICAO address and a callsign can encode to the same integer and still remain separate traffic entries.
To decode a logged `unique_id` back into an ICAO address, callsign, or shortened UAS ID, see [Identifying the aircraft](#identifying-the-aircraft), which includes a Python helper.

:::details
Click here to view how DAA accepts and encodes each identifier source

| Identifier source | Accepted if                                                                           | Stored value                                                                                                                                                                                                                                                                                                                                                                                                                               |
| ----------------- | ------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| ICAO address      | `0 < icao_address <= 0xFFFFFF`                                                        | Stored directly in the low bits of `unique_id` with `UNIQUE_ID_ENCODING_ICAO`                                                                                                                                                                                                                                                                                                                                                              |
| ADS-B callsign    | `PX4_ADSB_FLAGS_VALID_CALLSIGN` is set and the `callsign[9]` field is null-terminated | Packs up to 8 callsign characters into a `uint64`. Byte `0` is the first character. Bytes `0-3` correspond to [ADSB_CALLSIGN_1](../advanced_config/parameter_reference.md#ADSB_CALLSIGN_1) and bytes `4-7` correspond to [ADSB_CALLSIGN_2](../advanced_config/parameter_reference.md#ADSB_CALLSIGN_2). |
| UAS ID            | At least one of the final 8 bytes in `uas_id[18]` is non-zero                         | Packs those final 8 bytes into a `uint64` with `UNIQUE_ID_ENCODING_UAS_ID`                                                                                                                                                                                                                                                                                                                                                                 |

:::

Important implications of that priority order:

- DAA stops at the first usable identifier.
  It does not compare a later callsign or UAS ID once a non-zero ICAO address has been accepted.
- Callsign self-filtering only works if the incoming report either has no ICAO address or the ICAO address is also configured as ownship.
- UAS ID self-filtering is the last fallback.
  On boards that expose a PX4 UUID, DAA compares the reduced UAS ID against the reduced tail bytes of the board UUID.

Self-filtering uses:

- [ADSB_ICAO_ID](../advanced_config/parameter_reference.md#ADSB_ICAO_ID): primary ownship ICAO address.
- [ADSB_ICAO_ID_2](../advanced_config/parameter_reference.md#ADSB_ICAO_ID_2): optional second ownship ICAO address checked independently for self-filtering.
- [ADSB_CALLSIGN_1](../advanced_config/parameter_reference.md#ADSB_CALLSIGN_1): first 4 characters of the ownship ADS-B callsign.
- [ADSB_CALLSIGN_2](../advanced_config/parameter_reference.md#ADSB_CALLSIGN_2): last 4 characters of the same ownship ADS-B callsign.

`ADSB_CALLSIGN_1` and `ADSB_CALLSIGN_2` together define one 8-character callsign, and both halves must match for callsign-based self-filtering. By contrast, `ADSB_ICAO_ID` and `ADSB_ICAO_ID_2` are two separate ICAO addresses that DAA checks independently.

::: warning

`ADSB_ICAO_ID` now defaults to `-1`, which leaves the primary self-filter unset and disables PX4-managed ADS-B Out.
Older PX4 releases defaulted this parameter to `1194684`; a vehicle that relied on that default rather than saving an explicitly assigned ICAO address will therefore stop PX4-managed ADS-B transmission after upgrading.
Configure the aircraft's assigned ICAO address and reboot before relying on ADS-B Out.

:::

The callsign parameters use the documented character order within each 32-bit word.
For example, `ADSB_CALLSIGN_1 = 0x50583420` (`"PX4 "`) and `ADSB_CALLSIGN_2 = 0x54455354` (`"TEST"`) identify the callsign `"PX4 TEST"`; DAA converts those words to its internal packed-ID byte order before comparing reports.

DAA also removes conflicts that stop receiving updates after [DAA_TRAFF_TOUT](../advanced_config/parameter_reference.md#DAA_TRAFF_TOUT).

## Key Parameters

For the operator-facing setup flow, see [ADS-B/FLARM/UTM Receivers > Configure Traffic Avoidance](../peripherals/adsb_flarm.md#configure-traffic-avoidance).
On this page, parameters are described where their behavior matters: `NAV_TRAFF_*` in [Crosstrack Mode](#crosstrack-mode), `DAA_LVL_*`, `DAA_EN_DFLT_VEL`, and `DAA_DFLT_VEL` in [F3442 Mode](#f3442-mode), [DAA_NOTIF_STATE](../advanced_config/parameter_reference.md#DAA_NOTIF_STATE) in [Operator Messages](#operator-messages), [DAA_TRAFF_TOUT](../advanced_config/parameter_reference.md#DAA_TRAFF_TOUT) in [Traffic Inputs and Identification](#traffic-inputs-and-identification), and ownship ADS-B identification parameters in the same traffic-input section.
F3442-specific parameters are only present in firmware built with `CONFIG_NAVIGATOR_ADSB_F3442`.

## Telemetry and Logging

The main DAA topics are:

- [`detect_and_avoid`](../msg_docs/DetectAndAvoid.md): per-traffic assessments for reports that create, update, or clear a tracked conflict.
  First-seen reports evaluated at conflict level `NONE` are omitted.
- [`detect_and_avoid_most_urgent`](../msg_docs/DetectAndAvoidMostUrgent.md): the current most urgent active conflict.
- [`transponder_report`](../msg_docs/TransponderReport.md): raw traffic input to DAA.

`detect_and_avoid` and `detect_and_avoid_most_urgent` are logged by default.

These topics are useful for:

- Flight review
- Integration testing
- Preflight debugging
- Verifying conflict escalation and action selection

## Operator Messages

DAA talks to the operator on two channels at the same time:

- **MAVLink STATUSTEXT** is the short, human-readable line such as `DAA Main: 6E9F7B lvl UP 3. 199 m.`.
  PX4 emits these as [`mavlink_log`](../msg_docs/MavlinkLog.md) records, and the MAVLink module forwards them on its `STATUSTEXT` stream as MAVLink [`STATUSTEXT`](https://mavlink.io/en/messages/common.html#STATUSTEXT) messages to connected ground stations.
  The identifier is already decoded into a readable string here, so this is the easiest channel to read while flying.
- **PX4 Events** carry the same information through the [Events Interface](../concept/events_interface.md), but as structured data: a fixed event ID plus numeric arguments (identifier, conflict level, distance, cause, notification kind).

For levels configured as `Warn only` or stronger, messages are emitted after the queue-processing cycle when a conflict level changes.
The most urgent conflict is also reported periodically (the period is [DAA_NOTIF_STATE](../advanced_config/parameter_reference.md#DAA_NOTIF_STATE)).

### Reading the message fields

Every conflict message refers to one traffic aircraft and reuses the same handful of fields.

:::details
Click here to view the fields used in events

#### `<ID>`: the traffic identifier

DAA tracks each aircraft by the identifier selected in [Traffic Inputs and Identification](#traffic-inputs-and-identification).
The two channels show that identifier differently:

- In **STATUSTEXT** it is already decoded to a readable string: a 6-character ICAO hex such as `6E9F7B`, an up-to-8-character callsign such as `LX00777A`, or a shortened UAS-ID hex such as `eaebecedee`.
- In **events** it is the raw 64-bit integer (`unique_id`) that DAA stores internally.
  The same integer, together with its `unique_id_encoding`, is also published on the [`detect_and_avoid`](../msg_docs/DetectAndAvoid.md) and [`detect_and_avoid_most_urgent`](../msg_docs/DetectAndAvoidMostUrgent.md) topics.

To turn that integer back into a readable identifier (for example an ICAO address) you need the encoding and the matching decode rule.
A worked example with code is in [Identifying the aircraft](#identifying-the-aircraft).

#### `<N>`: the conflict level

The level is the same integer used on the uORB topics:

| `<N>` | Рівень     | F3442 meaning          |
| ----- | ---------- | ---------------------- |
| `0`   | `NONE`     | no conflict or cleared |
| `1`   | `LOW`      | augmented Well Clear   |
| `2`   | `MEDIUM`   | augmented NMAC         |
| `3`   | `HIGH`     | Loss of Well Clear     |
| `4`   | `CRITICAL` | NMAC                   |

Crosstrack builds only ever publish `HIGH` (`3`).
See [Conflict Levels](#conflict-levels) for the full F3442 definitions.
In the main and secondary messages, the words `UP` and `DOWN` tell you whether the level just increased or decreased.

#### `<dist>`: distance to the traffic

The straight-line 3D range to that aircraft in meters, computed from the horizontal and vertical separation ($\sqrt{d_{hor}^2 + d_{vert}^2}$).

#### Notification kind (events only)

The `navigator_traffic_conflict_update` event carries a _notification kind_ argument that says which conflict the update is about.
It matches the prefix used in the STATUSTEXT line:

| Notification kind | STATUSTEXT prefix   | Значення                                                                                  |
| ----------------- | ------------------- | ----------------------------------------------------------------------------------------- |
| `0`               | `DAA Main:`         | update for the conflict that is already the most urgent                                   |
| `1`               | `DAA New and Main:` | a conflict that appeared this cycle and is also now the most urgent                       |
| `2`               | `DAA SEC:`          | a secondary (non-primary) tracked conflict that is not the most urgent |

#### Action code (events only)

The `navigator_traffic_action` event reports which automatic response was triggered, using the same numbering as the `NAV_TRAFF_AVOID` / `DAA_LVL_*_ACT` parameters:

| Action code | Дія         |
| ----------- | ----------- |
| `2`         | `Return`    |
| `3`         | `Land`      |
| `4`         | `Hold`      |
| `5`         | `Terminate` |

`0` (`Disabled`) and `1` (`Warn only`) do not generate an action event because no automatic action is requested.
See [Automated Actions](#automated-actions) for the full action parameter convention.

:::

:::details
Click here to view the message families and notification timing

- **New conflict:** `DAA New <ID> lvl <N>. <dist> m.`: emitted after the current `check_traffic()` queue drain completes when a new warning-level conflict remains in the active buffer but is not the current most urgent conflict.
- **Main conflict:**
  - `DAA Main: <ID> lvl UP <N>. <dist> m.`: emitted immediately when the most urgent conflict level increases.
  - `DAA Main: <ID> lvl DOWN <N>. <dist> m.`: emitted immediately when the most urgent conflict level decreases but remains active.
  - `DAA Main: <ID> lvl <N>. <dist> m.`: periodic message for the current most urgent conflict when its level stays unchanged.
    The message period is [DAA_NOTIF_STATE](../advanced_config/parameter_reference.md#DAA_NOTIF_STATE); setting it to `0` disables this periodic message.
- **New and Main conflict:** `DAA New and Main: <ID> lvl <N>. <dist> m.`, `DAA New and Main: <ID> lvl UP <N>. <dist> m.`, `DAA New and Main: <ID> lvl DOWN <N>. <dist> m.`: emitted after the current `check_traffic()` queue drain completes when a new warning-level conflict is also the current most urgent conflict.
- **Secondary conflicts:** `DAA SEC: <ID> lvl UP <N>. <dist> m.`, `DAA SEC: <ID> lvl DOWN <N>. <dist> m.`, `DAA SEC: <ID> solved. <dist> m.`: emitted immediately for non-primary traffic whose level changes while another conflict remains most urgent.
- **Buffer handling:**
  - `DAA <ID> out (<cause>) lvl <N> (<age>s).`: emitted immediately when a previously buffered conflict is removed because it became stale or because it was evicted to make room for a more urgent conflict.
    New conflicts that are inserted and then displaced within the same queue drain before any `DAA New` notification are intentionally silent.
    The cause is mapped to:
    - STALE_CONFLICT = 0,
    - BUFFER_FULL = 1
  - `DAA <ID> ignored (<cause>) lvl <N>.`: emitted when traffic could not be inserted or updated.
    This warning is rate-limited to once every `2 s`.
    The cause can be:
    - BUFFER_FULL = 0
- **Actions:** `DAA <ID>: Hold!`, `Return!`, `Land!`, `Terminate!`, and the on-ground `DAA do not arm until air conflict solved!` / `DAA do not takeoff until air conflict solved!` warnings only appear when a conflict level is configured with an automatic action stronger than `Warn only`. The default DAA parameters are warn-only so by default these action messages are not emitted.
- **No more conflicts:** `DAA all conflicts solved.`: emitted immediately when the most urgent conflict clears and no warning-level conflicts remain.

The shared fields `<ID>`, `<N>`, and `<dist>`, plus the cause numbers above, are explained in [Reading the message fields](#reading-the-message-fields).
To turn a logged or event identifier back into an ICAO address, callsign, or UAS ID, see [Analysing a Conflict Using the Logs](#analysing-a-conflict-using-the-logs).

:::

## Analysing a Conflict Using the Logs

After a flight you can reconstruct exactly what DAA saw and did from the default log.
This is the recommended way to investigate a conflict, because the logged topics carry the full identifier, encoding, geometry, and timing that the short operator messages only summarize.

### What to look at

- [`detect_and_avoid_most_urgent`](../msg_docs/DetectAndAvoidMostUrgent.md): the single conflict that drove the DAA status and any automatic action or prearm block.
  Start here to find the worst moment of the encounter (highest `conflict_level`, smallest `aircraft_dist`) and whether `has_action` was set.
- [`detect_and_avoid`](../msg_docs/DetectAndAvoid.md): samples for traffic that creates, updates, or clears a tracked conflict, with the per-aircraft `conflict_level` and the horizontal, vertical, and time geometry.
  Use it to follow one specific aircraft over time.
- [`transponder_report`](../msg_docs/TransponderReport.md): the raw input.
  Match it by timestamp to see the traffic's reported position, altitude, velocity, heading, and validity flags, plus the original `icao_address`, `callsign`, and `uas_id` fields before encoding.
- Navigator state and commander messages: confirm whether a requested action actually changed the flight mode.

You can open the log in [Flight Review](../log/flight_review.md) or plot these topics directly.
Live, the same topics can be inspected from the shell with `listener detect_and_avoid` and `listener detect_and_avoid_most_urgent`.

### Identifying the aircraft

Both DAA topics publish `unique_id` (the 64-bit integer) and `unique_id_encoding` (the type of ID).
Decode the integer according to the encoding:

| `unique_id_encoding`                     | Source              | How to decode                                                                                 |
| ---------------------------------------- | ------------------- | --------------------------------------------------------------------------------------------- |
| `0` (`ICAO`)          | ICAO 24-bit address | low 24 bits formatted as 6 uppercase hex digits                                               |
| `1` (`ADSB_CALLSIGN`) | ADS-B callsign      | bytes little-endian, byte `0` is the first character, stop at first `\0`                     |
| `2` (`UAS_ID`)        | shortened UAS ID    | low bytes as hex; the displayed string uses the low 5 bytes (10 hex chars) |

The selection and storage rules behind this table are described in [Traffic Inputs and Identification](#traffic-inputs-and-identification).

:::details
Click here to view a Python script example on how to decode `unique_id` (the 64-bit integer)
The following helper applies the same conversion DAA uses to build the STATUSTEXT string, so its output matches what you saw live:

```python
import argparse

def decode_daa_id(unique_id: int, encoding: int) -> str:
    if encoding == 0:  # ICAO
        return f"{unique_id & 0xFFFFFF:06X}"
    if encoding == 1:  # ADS-B callsign
        out = bytearray()
        for i in range(8):
            byte = (unique_id >> (8 * i)) & 0xFF
            if byte == 0:
                break
            out.append(byte)
        return out.decode("ascii", "replace")
    if encoding == 2:  # UAS ID (shortened, low 5 bytes shown)
        return "".join(f"{(unique_id >> (8 * i)) & 0xFF:02x}" for i in range(5))
    return "unknown"

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Decode a DAA ID based on its encoding type.")

    parser.add_argument("unique_id", type=int, help="The 64-bit unique ID")
    parser.add_argument(
        "encoding",
        type=int,
        choices=[0, 1, 2],
        help="Encoding type: 0 (ICAO), 1 (ADS-B), or 2 (UAS ID)",
    )

    args = parser.parse_args()

    print(decode_daa_id(args.unique_id, args.encoding))
```

Example 1: Decoding an ICAO address (encoding `0`): the integer `7249787` decodes to the 6-character hex ICAO address `6E9F7B`.

```bash
python3 decode_daa.py 7249787 0
```

Example 2: Decoding an ADS-B callsign (encoding `1`): the integer `827736405` is the packed ASCII representation of the callsign `UAV1`.

```bash
python3 decode_daa.py 827736405 1
```

Example 3: Decoding a UAS ID (encoding `2`): the integer `737869762868` is unpacked into the shortened 10-character hex string `341177ccab`.

```bash
python3 decode_daa.py 737869762868 2
```

:::

### Turning an ICAO address into a real aircraft

When the encoding is `ICAO`, the decoded 6-digit hex (for example `6E9F7B`) is the 24-bit address reported over ADS-B or Mode S.
When the address has been assigned and transmitted correctly, an aircraft registry may map it to a tail number, type, or operator.

This is how you confirm whether the intruder was, for instance, a helicopter, a general-aviation aircraft, or even your own second transponder.
If it turns out to be ownship, update the self-filtering parameters described in [Traffic Inputs and Identification](#traffic-inputs-and-identification).

### Опис

A typical investigation:

1. In `detect_and_avoid_most_urgent`, find when `conflict_level` peaked and `aircraft_dist` was smallest, and note `unique_id` and `unique_id_encoding`.
2. Decode the identifier; if it is an ICAO address, look up the aircraft.
3. In `detect_and_avoid`, follow that identifier to see how the horizontal, vertical, and time separation evolved and how the level escalated.
4. Cross-check `transponder_report` at the same timestamps for the raw geometry and validity flags.
   Missing velocity or heading changes how the model evaluates the conflict (see the data requirements for [F3442 Mode](#f3442-mode) or [Crosstrack Mode](#crosstrack-mode)).
5. Confirm in the navigator and commander messages whether the configured action ran and whether the resulting mode change matched expectations.

## Testing and Simulation

Synthetic traffic generation is available when the firmware is built with `CONFIG_NAVIGATOR_ADSB_FAKE_TRAFFIC`.
This option is intended for simulation and dedicated test images; it should not be enabled in production flight firmware.

Use the navigator shell command:

```sh
navigator fake_traffic
```

Available modes:

- `unique_ids`, `escalation`, `spam_same`, `spam_new`, `flags`, `queue_fill`

Example:

```sh
navigator fake_traffic escalation
```

Use `navigator fake_traffic help` for the short mode list and `navigator fake_traffic stop` to cancel a pending scripted run.

:::warning
`navigator fake_traffic` injects traffic into the normal DAA pipeline.
If the configured action for the triggered conflict level is `Hold`, `Return`, `Land`, or `Terminate`, PX4 can request that action exactly as it would for real traffic.
:::

Run these scenarios with a valid vehicle position, because the synthetic reports are generated around the current ownship latitude, longitude, and altitude.

The command schedules a deterministic synthetic traffic sequence and returns immediately.
DAA then publishes the scripted `transponder_report` samples from its normal update loop around the current ownship latitude, longitude, and altitude.

:::details
Click here for more details on each fake traffic scenario

Scenario guide:

- `unique_ids`: Publishes three isolated four-step approach sequences: about `1500 m`, `800 m`, `200 m`, then `5000 m` to clear the conflict.
  The first traffic uses a valid ICAO address, the second removes ICAO so DAA must fall back to the ADS-B callsign, and the third removes ICAO plus a valid callsign so DAA must fall back to UAS ID.
  Use this to verify identifier priority without overlap between the three identifier cases.
- `escalation`: Publishes one aircraft every `2 s` while its range decreases from about `3000 m` to `100 m`.
  Use this to watch conflict levels rise, confirm when the most urgent conflict level changes, and verify that the configured action thresholds trigger at the expected ranges.
- `spam_same`: Publishes the same aircraft `40` times at `10 Hz` while keeping it at about `200 m`.
  Use this to confirm that repeated updates from one target are not generating excessive notifications (expect a period of [DAA_NOTIF_STATE](../advanced_config/parameter_reference.md#DAA_NOTIF_STATE)).
- `spam_new`: Publishes `10` different ICAO identities, one per second, with fixed distances chosen to fill the `5`-entry conflict buffer, evict the least urgent items deterministically, and exercise ignored-traffic throttling once the buffer is full.
- `flags`: Publishes one aircraft with valid position, altitude, heading, and callsign, but with the velocity-valid flag intentionally cleared.
  In F3442 builds the report is still accepted because only coordinates and altitude are mandatory; the missing traffic velocity is treated as zero, and if [DAA_EN_DFLT_VEL](../advanced_config/parameter_reference.md#DAA_EN_DFLT_VEL) is enabled the vertical speed is replaced by [DAA_DFLT_VEL](../advanced_config/parameter_reference.md#DAA_DFLT_VEL).
  In Crosstrack builds the same report is rejected because velocity is required there.
- `queue_fill`: Runs three consecutive batches of `ORB_QUEUE_LENGTH` reports.
  Batch 1 repeats the same close critical aircraft for the full queue length.
  Batch 2 publishes new lower-priority conflicts that get progressively closer so each one is accepted while the original aircraft remains the most urgent.
  Batch 3 starts even closer and then gets farther away so only the first `4` entries replace the existing lower-priority traffic and the rest are ignored.
  Use this to confirm DAA drains one full queued `transponder_report` burst per `check_traffic()` call and preserves the correct final priority order after each batch.

:::

What to watch while running a scenario:

- [`detect_and_avoid`](../msg_docs/DetectAndAvoid.md): per-traffic output for tracker-accepted reports that create, update, or clear a conflict.
- [`detect_and_avoid_most_urgent`](../msg_docs/DetectAndAvoidMostUrgent.md): the single conflict that currently drives the main DAA status and any automatic action decision.
- [`mavlink_log`](../msg_docs/MavlinkLog.md), MAVLink `STATUSTEXT`, and PX4 Events: new conflict, escalation, reduction, ignored traffic, and stale or evicted traffic messages.

Useful shell commands:

```sh
listener detect_and_avoid
listener detect_and_avoid_most_urgent
```

### Expected Behavior

Exact distances and message text depend on the selected parameters and vehicle state. With the default warn-only F3442 actions:

| Scenario     | Expected behavior                                                                                                                    |
| ------------ | ------------------------------------------------------------------------------------------------------------------------------------ |
| `unique_ids` | Three otherwise similar encounters are tracked by ICAO address, callsign, and UAS ID respectively.                   |
| `escalation` | One aircraft progresses through increasing conflict levels, then becomes stale and is removed.                       |
| `spam_same`  | Repeated reports refresh one tracked conflict without producing a message for every sample.                          |
| `spam_new`   | Multiple aircraft fill the conflict buffer; less urgent reports are evicted or ignored according to buffer priority. |
| `flags`      | F3442 accepts missing traffic velocity and applies its configured defaults; Crosstrack rejects the report.           |
| `queue_fill` | DAA drains a full queued burst and retains the highest-priority conflicts.                                           |

Use the uORB topics and operator messages above to verify the result. Automated-action testing should use a controlled simulation environment.

The sequences below assume the default F3442 configuration.

:::details
Click here to view default configuration

- Firmware built with `CONFIG_NAVIGATOR_ADSB_F3442`
- [DAA_LVL_LOW_ACT](../advanced_config/parameter_reference.md#DAA_LVL_LOW_ACT) = `Warn only`
- [DAA_LVL_MED_ACT](../advanced_config/parameter_reference.md#DAA_LVL_MED_ACT) = `Warn only`
- [DAA_LVL_HIGH_ACT](../advanced_config/parameter_reference.md#DAA_LVL_HIGH_ACT) = `Warn only`
- [DAA_LVL_CRIT_ACT](../advanced_config/parameter_reference.md#DAA_LVL_CRIT_ACT) = `Warn only`
- [DAA_EN_DFLT_VEL](../advanced_config/parameter_reference.md#DAA_EN_DFLT_VEL) = `1`
- [DAA_DFLT_VEL](../advanced_config/parameter_reference.md#DAA_DFLT_VEL) = `10 m/s`
- Ownship horizontal and vertical speed = `0`
- Ownship and traffic altitude are equal

:::

Under those assumptions, the expected operator-visible messages for each scenario are:

**`unique_ids`**: four-step process repeated for the three unique identifier cases.

:::details
Click here to view the expected results

- ICAO case `6E9F7B`:
  - Step 1: `DAA New and Main: 6E9F7B lvl UP 2. 1499 m.`
  - Step 2: `6E9F7B` at 800 m stays at the same conflict level (2), so it does not emit another `DAA Main` line before [DAA_NOTIF_STATE](../advanced_config/parameter_reference.md#DAA_NOTIF_STATE) expires.
  - Step 3: `DAA Main: 6E9F7B lvl UP 3. 199 m.`
  - Step 4: move conflict 5000m away and expect: `DAA all conflicts solved.`
- Callsign case `LX00777A`: the same level sequence as above, but with `LX00777A` as the identifier.
- UAS ID fallback case: the same level sequence as above, but with the shortened board-dependent UAS ID string as the identifier e.g., `eaebecedee`.

:::

**`escalation`**: thirty-step conflict with a single incoming aircraft flying towards the UAV.

:::details
Click here to view the expected results

- Example with default warn-only parameters:
  - No DAA warning is expected from `3000 m` down to about `2300 m`.
  - At about `2200 m`: `DAA New and Main: 9F3FA3 lvl UP 1. 2200 m.`
  - At about `1800 m`: `DAA Main: 9F3FA3 lvl UP 2. 1800 m.`
  - At about `600 m`: `DAA Main: 9F3FA3 lvl UP 3. 600 m.`
  - At about `100 m`: `DAA Main: 9F3FA3 lvl UP 4. 100 m.`
  - After approximately [DAA_TRAFF_TOUT](../advanced_config/parameter_reference.md#DAA_TRAFF_TOUT): `DAA 9F3FA3 out (0) lvl 4 (<age>s).` followed by `DAA all conflicts solved.`
  - No `DAA SEC` or automatic-action messages are expected with the default warn-only parameters.
- Example with actions defined as Hold, Return, Land, and Terminate: `DAA: Actions lvl: low: 4, med: 2, high: 3, crit: 5`:
  - `DAA New and Main: 9F3FA3 lvl UP 1. 2366 m.`
  - Hold triggered on augmented WC breach: `DAA 9F3FA3: Hold! lvl 1. 2366 m.`
  - `DAA Main: 9F3FA3 lvl UP 2. 1773 m.`
  - Return triggered on augmented NMAC breach: `DAA 9F3FA3: Return! lvl 2. 1773 m.`
    - `[commander] Returning to launch`
    - `[navigator] RTL: start return at 519 m (30 m above destination)`
  - `DAA Main: 9F3FA3 lvl 2. 849 m.` (the periodic most-urgent status, emitted because the level held at 2 for [DAA_NOTIF_STATE](../advanced_config/parameter_reference.md#DAA_NOTIF_STATE) since the previous `9F3FA3` message)
  - `DAA Main: 9F3FA3 lvl UP 3. 527 m.`
  - Land triggered on WC breach: `DAA 9F3FA3: Land! lvl 3. 527 m.`
    - `[commander] Landing at current position`
  - `DAA Main: 9F3FA3 lvl UP 4. 124 m.`
  - Terminate triggered on NMAC breach: `DAA 9F3FA3: Terminate! lvl 4. 124 m.`
    - `[failsafe] Failsafe activated`

:::

**`spam_same`**: forty-step conflict with an aircraft spamming transponder reports from the same location.

:::details
Click here to view the expected results

- First sample only: `DAA New and Main: 61F77C lvl UP 3. 199 m.`
- The remaining `39` updates refresh the same conflict entry but do not emit extra status lines because the level does not change and the whole scenario finishes before [DAA_NOTIF_STATE](../advanced_config/parameter_reference.md#DAA_NOTIF_STATE) elapses. Eventually the traffic becomes stale at approximately [DAA_TRAFF_TOUT](../advanced_config/parameter_reference.md#DAA_TRAFF_TOUT), followed by `DAA all conflicts solved.`.

:::

**`queue_fill`**: three-cycle queue-drain burst.

:::details
Click here to view the expected results

- **Step 1:** the first queue is filled with the same aircraft, expect a single message: `DAA New and Main: DDF0A1 lvl UP 4. 100 m.`
- **Step 2:** the second queue is filled with different aircraft getting closer and closer. DAA drains the full queue before it emits `DAA New` lines, so only the four entries that survive the batch are reported:
  - `DAA New 51000D lvl 2. 680 m.`, `DAA New 51000E lvl 2. 670 m.`, `DAA New 51000F lvl 2. 660 m.`, `DAA New 510010 lvl 2. 650 m.`
  - Earlier step-2 candidates `510001` through `51000C` are accepted provisionally and then replaced within the same drain, so they do not emit `out` or `ignored` operator messages.
- **Step 3:** the third queue is filled with different aircraft getting farther and farther. The first four entries are still more important than the surviving step-2 traffic, so expect:
  - `DAA New 520001 lvl 3. 569 m.`, `DAA New 520002 lvl 3. 589 m.`, `DAA New 520003 lvl 3. 609 m.`, `DAA New 520004 lvl 2. 629 m.`
  - Because those four replace already-known step-2 conflicts from the previous cycle, expect removals: `DAA 51000D out (1) lvl 2 (2s).`, `DAA 51000E out (1) lvl 2 (2s).`, `DAA 51000F out (1) lvl 2 (2s).`, `DAA 510010 out (1) lvl 2 (2s).`
  - The remaining step-3 traffic is less important than the buffer contents, so it is ignored. Because ignored notifications are throttled, the first operator-visible message is typically `DAA 520005 ignored (0) lvl 2.`

:::

**`spam_new`**: ten-step multi-aircraft conflict that fills the buffer.

:::details
Click here to view the expected results

| Step | ICAO     | Nominal range | Expected result                                                                                                                                                                                                                                                  |
| ---- | -------- | ------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 1    | `380E97` | `1200 m`      | `DAA New and Main: 380E97 lvl UP 1. 1200 m.`                                                                                                                                                                                                                     |
| 2    | `67F621` | `900 m`       | `DAA New and Main: 67F621 lvl 1. 899 m.`                                                                                                                                                                                                                         |
| 3    | `ACDE77` | `700 m`       | `DAA New and Main: ACDE77 lvl UP 2. 700 m.`                                                                                                                                                                                                                      |
| 4    | `45F276` | `500 m`       | `DAA New and Main: 45F276 lvl UP 3. 500 m.`                                                                                                                                                                                                                      |
| 5    | `34990F` | `100 m`       | `DAA New and Main: 34990F lvl UP 4. 100 m.` The buffer is now full.                                                                                                                                                                              |
| 6    | `EB61D8` | `130 m`       | New entry on a full buffer, least urgent conflict is removed first for cause: BUFFER_FULL: `DAA 380E97 out (1) lvl 1 (5s).` and then we add the new traffic: `DAA New EB61D8 lvl 4. 130 m.` |
| 7    | `A1EAB2` | `550 m`       | New entry on a full buffer, least urgent conflict is removed first for cause: BUFFER_FULL: `DAA 67F621 out (1) lvl 1 (5s).` then `DAA New A1EAB2 lvl 3. 550 m.`                                             |
| 8    | `C5ED22` | `1100 m`      | `DAA C5ED22 ignored (0) lvl 1.` ignored for cause BUFFER_FULL and all other conflicts are more important.                                                                                                                   |
| 9    | `B2F854` | `750 m`       | No new operator message is expected, because the traffic is ignored again while ignored-traffic warnings are still inside the `2 s` rate limit                                                                                                                   |
| 10   | `E16498` | `140 m`       | New entry on a full buffer, least urgent conflict is removed first for cause: BUFFER_FULL: `DAA ACDE77 out (1) lvl 2 (7s).` then `DAA New E16498 lvl 4. 140 m.` and `DAA Main: 34990F lvl 4. 100 m.`        |

Notes:

- The main conflict `DAA Main: 34990F lvl 4. 100 m.` appears periodically during the scenario at the rate of [DAA_NOTIF_STATE](../advanced_config/parameter_reference.md#DAA_NOTIF_STATE).
- After the last step, all conflicts are eventually marked as stale and resolved:
  - `DAA 34990F out (0) lvl 4 (20s).`
  - `DAA 45F276 out (0) lvl 3 (21s).`
  - `DAA A1EAB2 out (0) lvl 3 (20s).`
  - `DAA EB61D8 out (0) lvl 4 (21s).`
  - `DAA E16498 out (0) lvl 4 (21s).`
  - `DAA all conflicts solved.`

:::

**`flags`**: one aircraft with the velocity-valid flag intentionally cleared.

:::details
Click here to view the expected results

- In `F3442`: `DAA New and Main: L07NOVEL lvl UP 4. 100 m.` followed by the periodic notification: `DAA Main: L07NOVEL lvl 4. 100 m.` and finally: `DAA L07NOVEL out (0) lvl 4 (20s).` and `DAA all conflicts solved.`.
- In `Crosstrack`: no DAA conflict message is expected, because the same report is rejected when the velocity-valid flag is missing.

:::

## Current Limitations

- DAA handles cooperative traffic only.
- DAA does not generate lateral or vertical avoidance trajectories.
- DAA selects high-level navigator actions; it does not solve conflict geometry beyond the selected conflict model.
- Conflict handling is bounded by a fixed-size active-conflict buffer.
- Action selection and the commander arming check are driven by the single geometrically most urgent conflict.
  A lower-priority buffered conflict does not independently drive an action or arming block, even if its configured action is stronger.
- DAA compares ownship altitude directly with `transponder_report.altitude` and does not use `altitude_type` to normalize pressure and geometric altitude references.
  A datum mismatch can therefore bias vertical separation.
- DAA quality depends on the quality and freshness of both traffic data and vehicle state.

## Implementation Structure

The implementation is split between a shared PX4 library and the navigator integration:

- `src/lib/adsb`: the DAA core, with no navigator dependency; uORB message structs and enums are only used for data types.
  It contains the conflict standards (`DaaCrosstrack`, `DaaF3442`, wrapped by `AdsbConflict`), the traffic identifier encoding and self-detection (`DaaEncodedId`), the active-conflict buffer (`ConflictTracker`), and the action policy (`DaaActionPolicy`).
  `ConflictTracker` owns the fixed-size conflict buffer and the priority rules: it inserts, updates, and evicts conflicts, caches the most urgent one, and reports every change that may require a user-facing message as a change record (`conflict_tracker_change_s`), without sending any notification itself.
  `DaaActionPolicy` is a stateless decision function: given the most-urgent level transition, the navigation state, the landed/armed state, and the action parameter values, it returns which vehicle command to request and which action or ground message to send. It implements the level-to-action mapping (including the F3442 per-level action fallback described in [Automated Actions](#automated-actions)) and the navigator-state escalation gate.
- `src/modules/navigator/DetectAndAvoid`: the navigator integration.
  It validates and identifies incoming `transponder_report` data, runs the built standard, feeds the result to the tracker, publishes the DAA topics, and executes the policy's action decisions (vehicle commands and action messages).
  All operator messaging is concentrated in one class, `ConflictNotifier`: it turns the change records collected over one cycle into the messages described in [Operator Messages](#operator-messages) in a single per-cycle call (`report_cycle()`), and also owns the message formatting, severity mapping, and rate-limit timers used by the action and ground warnings.

## Adding a New Standard

At the library level, a DAA standard is a class that consumes one ownship state plus one traffic state and returns both a conflict level and reporting metrics.

:::details
Click here to view an in depth guide on how to add a new standard

```cpp
class DaaNewStandard : public ModuleParams
{
public:
  DaaNewStandard();

  uint8_t calculate_daa_stats(const aircraft_state_s &uav_state,
                              const aircraft_state_s &traffic_state,
                              daa_stats_s &daa_stats);

  bool try_setting_params();
};
```

The shared input and output are:

- `aircraft_state_s` input fields:
  - `lat_lon`: ownship or traffic latitude/longitude in degrees.
  - `altitude`: ownship or traffic altitude in meters.
  - `velocity_ned`: velocity in meters per second in north-east-down frame.
    In the wrapper path, traffic without a finite heading is converted to a due-north horizontal vector `(hor_speed, 0, -ver_speed_up)` so heading-agnostic standards can still use speed magnitude.
    A new standard that depends on the actual traffic track direction must reject missing heading explicitly.
  - `heading`: course-over-ground or track heading in radians.
    A standard may ignore this field, but a standard that builds projected-track geometry must require it to be finite.
- `daa_stats_s` output fields:
  - `aircraft_dist_hor`: the standard's reported horizontal-separation metric in meters.
    This may be direct range, signed crosstrack distance, or another horizontal metric defined by the standard.
  - `aircraft_dist_vert`: the standard's reported vertical-separation metric in meters.
  - `expected_min_dist_time_sec`: the standard's reported conservative time metric in seconds.
    This may be a worst-case closure-time estimate or another minimum-distance timing metric defined by the standard.
- Return value:
  - one of `detect_and_avoid_s::DAA_CONFLICT_LVL_NONE`, `LOW`, `MEDIUM`, `HIGH`, or `CRITICAL`

To add a new standard end-to-end:

1. Implement a new class with `calculate_daa_stats()` and define its runtime parameter validation.
2. Decide which `aircraft_state_s` fields are mandatory for that standard and enforce those requirements in the wrapper path (`AdsbConflict::calculate_daa_output()`) and inside the standard itself if it can be called directly.
   In particular, decide whether the due-north fallback for `velocity_ned` without traffic heading is acceptable, or whether the standard requires the true traffic track direction.
3. Add a build-time config and CMake selection for the standard, including any standard-specific parameter metadata only when that config is enabled.
4. Add the class to `AdsbConflict` and update the compile-time branches that validate inputs, run `calculate_daa_stats()`, and refresh parameters.
5. Add unit tests for both the standard math and the outer `AdsbConflict` validation path, then update this documentation with the standard-specific data requirements, parameters, and action semantics.

:::
