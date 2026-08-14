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

## Implementation

The fusion is implemented in `ExternalSetpoint` (`src/lib/external_setpoint/`) and applied in `FlightModeManager::generateTrajectorySetpoint()`, the single point where the manual and autonomous flight tasks converge, immediately before the trajectory setpoint is published.

Incoming MAVLink setpoints are mirrored onto the `external_setpoint` uORB topic by the MAVLink receiver. The mirror is always published; whether it is acted upon is gated by `EXT_SP_EN` on the consumer side, so normal Offboard use of `SET_POSITION_TARGET_LOCAL_NED` is unaffected.
