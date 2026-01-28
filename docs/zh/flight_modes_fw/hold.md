# 保持模式 (Fixed-wing)

<img src="../../assets/site/position_fixed.svg" title="Position fix required (e.g. GPS)" width="30px" />

The _Hold_ flight mode causes the vehicle to loiter around its current GPS position and maintain its current altitude.

The mode supports a [number of distinct loiter modes](#loiter-modes), which are triggered using different QGC controls or MAVLink commands.
These allow loitering with circular and figure 8 flight paths.

:::tip
_Hold mode_ can be used to pause a mission or to help you regain control of a vehicle in an emergency.
It is usually activated with a pre-programmed RC switch.
:::

::: info

- Mode is automatic - no user intervention is _required_ to control the vehicle.
- Mode requires a global 3d position estimate (from GPS or inferred from a [local position](../ros/external_position_estimation.md#enabling-auto-modes-with-a-local-position)).
  - Flying vehicles can't switch to this mode without global position.
  - Flying vehicles will failsafe if they lose the position estimate.
  - Disarmed vehicles can switch to mode without valid position estimate but can't arm.
- Mode requires wind and flight time are within allowed limits (specified via parameters).
- 遥控开关可以在任何无人机上更改飞行模式。
- RC stick movement is ignored.

<!-- https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/ModeUtil/mode_requirements.cpp -->

:::

## Loiter modes

### Default Loiter

The aircraft circles around the position at which the mode was triggered and maintain its current altitude.
The loiter radius is set by the parameter [NAV_LOITER_RAD](#NAV_LOITER_RAD).
Note that if the vehicle altitude is below [NAV_MIN_LTR_ALT](#NAV_MIN_LTR_ALT), it will ascend to that minimum altitude before circling.

The default loiter mode is entered when you switch to Hold mode without explicitly specifying any loiter behaviour.
For example, if you switch to Hold mode using an RC switch, select **Hold** on the QGC flight mode selector, or activate the mode using the MAVLink [MAV_CMD_DO_SET_MODE](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_MODE) command.

### Orbit Loiter Mode

<Badge type="tip" text="PX4 v1.12" />

The aircraft travels towards a _specified_ orbit center position, then circles it with a given direction and radius.

This behaviour can be accessed in QGroundControl by clicking on the map in Fly view, selecting **Orbit at Location**, and configuring the radius.

The behavior can be triggered using the MAVLink [MAV_CMD_DO_ORBIT](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_ORBIT) command.
Note that PX4 respects the specified centre point (`param5`, `param6`, `param7`), and the radius and direction (`param1`).
PX4 ignores `param3` (Yaw behaviour) and `param4` (Orbits).
The value of `param2` (velocity) is also ignored, but the speed can be controlled using the [MAV_CMD_DO_CHANGE_SPEED](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_CHANGE_SPEED) command (constrained between `FW_AIRSPD_MAX` and `FW_AIRSPD_MIN`).
PX4 outputs orbit status using the [ORBIT_EXECUTION_STATUS](https://mavlink.io/en/messages/common.html#ORBIT_EXECUTION_STATUS) message.

### Figure 8 Loiter Mode

<Badge type="tip" text="PX4 v1.15" /> <Badge type="warning" text="Experimental" />

The aircraft flys towards the closest point on a specified figure 8 path and then follows it.
The path is defined by the figure 8 centre position, orientation, and radius of two circles.

The feature is experimental, and is not present in PX4 firmware by default (on most flight controller boards).
It can be included by setting the `CONFIG_FIGURE_OF_EIGHT` key in the [PX4 board configuration](../hardware/porting_guide_config.md#px4-board-configuration-kconfig) for your board and rebuilding.
For example, this is enabled on the [default.px4board](https://github.com/PX4/PX4-Autopilot/blob/main/boards/auterion/fmu-v6s/default.px4board#L46) file for the `auterion/fmu-v6s` board.

The behavior can be triggered using the MAVLink [MAV_CMD_DO_FIGURE_EIGHT](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_FIGURE_EIGHT) command (PX4 respects all the parameters).
PX4 outputs the figure 8 status using the [FIGURE_EIGHT_EXECUTION_STATUS](https://mavlink.io/en/messages/common.html#FIGURE_EIGHT_EXECUTION_STATUS) message.

:::info
Figure 8 loitering is not currently supported by QGC: [QGC#12778: Need Support Figure of eight (8 figure) loitering by QGC](https://github.com/mavlink/qgroundcontrol/issues/12778).
:::

Figure 8 loitering is also available in the simulator.
You can test it in [Gazebo](../sim_gazebo_gz/index.md) using a fixed wing frame:

```sh
make px4_sitl gz_rc_cessna
```

## 参数

Hold mode behaviour can be configured using the parameters below.

| 参数                                                                                                                                                                      | 描述                                                                                                                                               |
| ----------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------ |
| <a id="NAV_LOITER_RAD"></a>[NAV_LOITER_RAD](../advanced_config/parameter_reference.md#NAV_LOITER_RAD)                         | The radius of the loiter circle.                                                                                                 |
| <a id="NAV_MIN_LTR_ALT"></a>[NAV_MIN_LTR_ALT](../advanced_config/parameter_reference.md#NAV_MIN_LTR_ALT) | Minimum height for loiter mode (vehicle will ascend to this altitude if mode is engaged at a lower altitude). |

## MAVLink Commands

The following commands are relevant to this mode:

- [MAV_CMD_DO_ORBIT](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_ORBIT) - Switch to Hold mode and start the specified [Orbit loiter](#orbit-loiter-mode).
  Params 2 (velocity), 3 (yaw), 4 (orbits) are ignored.
  [ORBIT_EXECUTION_STATUS](https://mavlink.io/en/messages/common.html#ORBIT_EXECUTION_STATUS) is emitted.
- [MAV_CMD_DO_FIGURE_EIGHT](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_FIGURE_EIGHT) - Switch to Hold mode and start the specified [Figure 8 loiter](#figure-8-loiter-mode).
  All params are respected.
  [FIGURE_EIGHT_EXECUTION_STATUS](https://mavlink.io/en/messages/common.html#FIGURE_EIGHT_EXECUTION_STATUS) is emitted.

Note, other commands may be supported.

## 另见

- [Hold Mode (MC)](../flight_modes_mc/hold.md)

<!-- this maps to AUTO_LOITER in flight mode state machine -->
