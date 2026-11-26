# Mission Mode (VTOL)

<img src="../../assets/site/position_fixed.svg" title="Global position fix required (e.g. GPS)" width="30px" />

_Mission mode_ causes the vehicle to execute a predefined autonomous [mission](../flying/missions.md) (flight plan) that has been uploaded to the flight controller.
The mission is typically created and uploaded with a Ground Control Station (GCS) application like [QGroundControl](https://docs.qgroundcontrol.com/master/en/) (QGC).

VTOL vehicles follow the behavior and parameters of fixed-wing when in FW mode, and of multicopter when in MC mode.
For more information see the specific docs for each mode:

- [Mission Mode (MC)](../flight_modes_mc/mission.md)
- [Mission Mode (FW)](../flight_modes_fw/mission.md)

The following sections outline mission mode behaviour that is VTOL specific.

## Mission Commands

The following VTOL-specific commands are as defined in the MAVLink specification.

- [MAV_CMD_NAV_VTOL_TAKEOFF](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_VTOL_TAKEOFF)
  - `MAV_CMD_NAV_VTOL_TAKEOFF.param2` (transition heading) is ignored.
    Instead the heading to the next waypoint is used for the transition heading. <!-- at LEAST until PX4 v1.13: https://github.com/PX4/PX4-Autopilot/issues/12660 -->
- [MAV_CMD_NAV_VTOL_LAND](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_VTOL_LAND)
- [MAV_CMD_DO_VTOL_TRANSITION](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_VTOL_TRANSITION)

Otherwise PX4 generally "accepts" the mission commands for fixed wing or multicopter when in the respective modes.

In fixed-wing mode there are the following exceptions:

- [MAV_CMD_NAV_LAND](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_LAND) is transformed into [MAV_CMD_NAV_VTOL_LAND](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_VTOL_LAND) unless [NAV_FORCE_VT](../advanced_config/parameter_reference.md#NAV_FORCE_VT) is set to `0` (disabled).
- [MAV_CMD_NAV_TAKEOFF](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_TAKEOFF) is not supported.

## Mission Transitions

Before a [MAV_CMD_DO_VTOL_TRANSITION](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_VTOL_TRANSITION) item that transitions to fixed-wing mode, a vehicle in multicopter mode turns towards the next waypoint.
It keeps flying towards that waypoint while it turns, climbing to the waypoint's altitude if that is higher, but never descending.
The transition starts where the vehicle is, once its heading is within [MIS_YAW_ERR](../advanced_config/parameter_reference.md#MIS_YAW_ERR) of the bearing to the waypoint.
If the heading is not reached within the [waypoint heading timeout](#waypoint-heading-timeout), the mission fails; until then the vehicle keeps flying towards the waypoint.
With [MPC_YAW_MODE](../advanced_config/parameter_reference.md#MPC_YAW_MODE) set to `towards waypoint (yaw first)` the vehicle still turns before it moves on.

If a waypoint directly follows the transition item, the item completes once the front transition starts, and the vehicle follows the mission from where the transition started.
It can turn during the transition at a waypoint it passes: waypoints are then reached as in fixed-wing flight, without their heading or hold time.
The mission only goes on from waypoint to waypoint during the transition.
Any other item, such as a command, a loiter or a landing, starts in fixed-wing mode, and the vehicle holds its heading from the last waypoint before that item until then.
A waypoint passed outside its fixed-wing altitude acceptance radius is not reached until the vehicle is within that radius: it holds its heading meanwhile.
If the mission is paused during the transition while it is still on the leg after the transition item, it resumes at that item, which starts the transition again if it was stopped.
If the transition is stopped while the mission stays active, the mission continues in multicopter mode from the waypoint it flies to.
A transition item that no waypoint follows still waits for fixed-wing mode, as before.
A front transition commanded from the ground station or RC during a mission follows the leg to the current waypoint in the same way, if that leg lies within 90° of the vehicle heading.
Otherwise it holds the transition heading, as in other modes.
In other modes, and for tailsitters, which fly the transition attitude, the vehicle holds the transition heading until it is in fixed-wing mode.
<Badge type="tip" text="main (PX4 v2.0)" />

## Mission Command Timeouts

Some mission commands/items can take time to complete, such as a gripper opening and closing, a winch extending or retracting, or a gimbal moving to point at a region of interest.

Where provided PX4 may use sensor feedback from the hardware to determine when the action has completed and then move to the next mission item.
If not provided, or if the feedback is lost, a mission command timeout can be used to ensure that these kinds of actions will progress to the next mission item rather than blocking progression.

The timeout is set using the [MIS_COMMAND_TOUT](../advanced_config/parameter_reference.md#MIS_COMMAND_TOUT) parameter.
This should be set to be a small amount greater than the time required for the longest long-running action in the mission to complete.

## Waypoint Heading Timeout

In multicopter mode, a VTOL vehicle only waits for headings that the mission requires, such as the alignment towards the next waypoint before a forward transition.
Other headings set by mission items are not awaited.

If a required heading is not reached within [MIS_YAW_TMT](../advanced_config/parameter_reference.md#MIS_YAW_TMT) (default: 10 s) after it was commanded, the mission fails and the vehicle switches to [Return mode](../flight_modes_vtol/return.md).
Set `MIS_YAW_TMT` to -1 to always wait for the heading.
<Badge type="tip" text="main (PX4 v2.0)" />

## Mission Takeoff

Plan a VTOL mission takeoff by adding a `VTOL Takeoff` mission item to the map.

During mission execution the vehicle will ascend vertically to the minimum takeoff altitude defined in the [MIS_TAKEOFF_ALT](../advanced_config/parameter_reference.md#MIS_TAKEOFF_ALT) parameter, then transition to fixed-wing mode with the heading defined in the mission item.
The vehicle heads towards the 3D position defined in the mission item, already during the transition.

A VTOL mission requires a `VTOL Takeoff` mission item (`MAV_CMD_NAV_VTOL_TAKEOFF`) to takeoff (or a `MAV_CMD_NAV_TAKEOFF` when the vehicle is in MC mode); if however the vehicle is already flying when the mission is started the takeoff item will be treated as a normal waypoint.

## See Also

- [Mission Mode (MC)](../flight_modes_mc/mission.md)
- [Mission Mode (FW)](../flight_modes_fw/mission.md)
- [Missions](../flying/missions.md)
  - [Package Delivery Mission](../flying/package_delivery_mission.md)
