# External Setpoints

_External Setpoints_ allow an onboard computer to contribute a temporary velocity/acceleration deviation to [Position mode](../flight_modes_mc/position.md) and [Mission mode](../flight_modes_mc/mission.md), without taking control of the vehicle the way [Offboard mode](../flight_modes_mc/offboard.md) does.

While fresh setpoints arrive the vehicle follows them. When the stream stops, the underlying Position or Mission setpoint resumes automatically.

PX4 retains ownership of the flight mode, the mission state, and the failsafes throughout. The external source is responsible for deciding *what* the deviation should be; PX4 only accepts and blends the result.

::: warning
This feature grants an external source authority over the vehicle's velocity while in an otherwise PX4-controlled mode.
It is disabled by default and should only be enabled with a source you trust.
:::

## Overview

PX4 offers two levels of external control, and historically nothing in between:

- **No authority.** In Position and Mission mode, a companion computer cannot influence the trajectory at all.
- **Total authority.** In Offboard mode, the companion commands the vehicle entirely — but the vehicle is no longer flying a mission, and returning to one requires a mode switch and re-engagement.

External Setpoints fill the gap. They are intended for behaviours that need to *adjust* an autonomous flight rather than replace it, such as steering around an obstacle mid-mission and then continuing.

The mechanism itself is application-agnostic. Obstacle avoidance is the motivating use case, but terrain following, target tracking, or any other onboard behaviour can use the same interface.

::: info
This is distinct from [Collision Prevention](../computer_vision/collision_prevention.md), which slows and stops the vehicle in the horizontal plane using range data, and only applies to Position mode.
External Setpoints do not perform any obstacle detection or planning of their own.
:::

## Enabling

Set [EXT_SP_EN](../advanced_config/parameter_reference.md#EXT_SP_EN) to `Enabled`.

Fusion is applied only in Position and Mission mode. In every other mode — including Offboard, Return, Takeoff, and Land — the setpoint is passed through untouched.

## Sending Setpoints

Stream [SET_POSITION_TARGET_LOCAL_NED](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED) with the desired velocity, and optionally acceleration, in the NED frame. Set the position axes to ignored via the type mask.

Setpoints must be streamed continuously. If no new setpoint arrives within [EXT_SP_TIMEOUT](../advanced_config/parameter_reference.md#EXT_SP_TIMEOUT) (default 0.5 s), fusion stops and the vehicle resumes its underlying setpoint.

Any axis set to `NaN` is treated as uncommanded and is left under PX4's control, so a deviation can be applied on one axis while PX4 continues to manage the others.

### Behaviour in Mission mode

When an axis is overridden in velocity, PX4 also releases the position setpoint on that axis. This is necessary because the position controller would otherwise continue to steer the vehicle toward the mission waypoint, fighting the commanded deviation.

The mission itself is not modified. Once the external stream stops, the vehicle resumes navigating to the active waypoint from wherever the deviation left it.

## Parameters

| Parameter                                                                              | Description                                                                                 |
| -------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------- |
| <a id="EXT_SP_EN"></a>[EXT_SP_EN](../advanced_config/parameter_reference.md#EXT_SP_EN)              | Enable external setpoint fusion. Default: Disabled.                                          |
| <a id="EXT_SP_TIMEOUT"></a>[EXT_SP_TIMEOUT](../advanced_config/parameter_reference.md#EXT_SP_TIMEOUT) | Maximum age of an external setpoint before it is ignored, in seconds. Default: 0.5. |

## Interface Specification

This section is normative. The key words "MUST", "MUST NOT", "SHOULD" and "MAY" are to be interpreted as described in [RFC 2119](https://www.ietf.org/rfc/rfc2119.txt).

It defines what an external source may send and what PX4 guarantees in response, so that independent implementations behave the same way. It deliberately says nothing about *how* a sender decides what to send.

### Mode-dependent interpretation

`SET_POSITION_TARGET_LOCAL_NED` is interpreted differently depending on the active flight mode:

| Mode | Interpretation |
| --- | --- |
| Offboard | The setpoint commands the vehicle directly. Unchanged pre-existing behaviour. |
| Position, Mission | The velocity/acceleration content is fused as a temporary deviation, subject to this specification. |
| All other modes | Ignored. |

Senders MUST NOT assume a given stream produces the same vehicle response in every mode. This overloading is a known limitation of reusing an existing message rather than introducing a new one.

### Transport

- The sender MUST use `SET_POSITION_TARGET_LOCAL_NED`.
- The `coordinate_frame` MUST be `MAV_FRAME_LOCAL_NED` or `MAV_FRAME_BODY_NED`. Any other frame is rejected and reported to the operator.
- In `MAV_FRAME_BODY_NED` the velocity is rotated into NED by vehicle yaw before use.
- The `x`, `y`, `z` position fields are never consumed by this path, in any mode covered by this specification.

### Preconditions

PX4 applies the setpoint only when all of the following hold. If any fails, the underlying setpoint is passed through unmodified.

- `EXT_SP_EN` is enabled.
- The navigation state is Position (`POSCTL`) or Mission (`AUTO_MISSION`).
- The most recent setpoint is marked valid, which requires at least one finite velocity or acceleration axis.
- The most recent setpoint is not stale (see [Timing](#timing)).

### Field semantics

- An axis set to `NaN`, or masked off via `type_mask`, is uncommanded and MUST be left under PX4 control.
- For each axis with a finite velocity, PX4 overrides that velocity axis **and** releases the corresponding position axis. This is required so the position controller does not steer back toward the waypoint and fight the deviation.
- For each axis with a finite acceleration, PX4 overrides that acceleration axis. Position is not released.
- Finite `yaw` and `yaw_rate` override their respective setpoints.
- In `MAV_FRAME_LOCAL_NED`, velocity axes MAY be masked individually. In `MAV_FRAME_BODY_NED`, masking any velocity axis discards all three; senders needing per-axis control MUST use `MAV_FRAME_LOCAL_NED`.

### Timing

- The sender MUST publish more frequently than `EXT_SP_TIMEOUT` (default 0.5 s).
- A setpoint older than `EXT_SP_TIMEOUT` MUST be ignored. PX4 does not extrapolate, hold, or decay a stale setpoint.

### Resumption

- When fusion stops for any reason — timeout, `EXT_SP_EN` cleared, or a mode change — PX4 resumes the underlying Position or Mission setpoint on the next iteration.
- PX4 retains no state across the deviation. The mission is not modified and the active waypoint does not change; the vehicle resumes navigating to it from wherever the deviation ended.

### Non-guarantees

Implementers MUST NOT rely on any of the following, none of which PX4 provides:

- **No validation.** PX4 holds no obstacle representation on this path and does not check the setpoint against the environment. The sender is solely responsible for the safety of what it commands.
- **No deviation bound.** The excursion is limited only by the active mode's own velocity and acceleration limits. There is no maximum displacement from the mission track.
- **No arbitration with Collision Prevention.** Where both are active in Position mode, this is applied after Collision Prevention and takes precedence over its output.
- **No delivery guarantee.** The transport is unacknowledged. A sender that stops transmitting is indistinguishable from one that never started.

### Conformance

Each requirement above is enforced by a test in `src/lib/external_setpoint/ExternalSetpointTest.cpp`, so the specification cannot drift from the implementation:

| Requirement | Test |
| --- | --- |
| Disabled by default | `disabledByDefaultIsNoOp` |
| Applied only in Position and Mission | `unsupportedModeIsNoOp` |
| Stale setpoints ignored | `staleSetpointIsNoOp` |
| Invalid setpoints ignored | `invalidSetpointIsNoOp` |
| Velocity override releases the position axis | `fusesVelocityAndReleasesPositionInMission` |
| Applied in Position mode | `fusesInPositionMode` |
| Uncommanded axes left untouched | `uncommandedAxesAreLeftUntouched` |
| Baseline resumes when fusion stops | `disablingMidStreamRestoresBaseline` |

## Implementation

The fusion is implemented in `ExternalSetpoint` (`src/lib/external_setpoint/`) and applied in `FlightModeManager::generateTrajectorySetpoint()`, the single point where the manual and autonomous flight tasks converge, immediately before the trajectory setpoint is published.

Incoming MAVLink setpoints are mirrored onto the `external_setpoint` uORB topic by the MAVLink receiver. The mirror is always published; whether it is acted upon is gated by `EXT_SP_EN` on the consumer side, so normal Offboard use of `SET_POSITION_TARGET_LOCAL_NED` is unaffected.
