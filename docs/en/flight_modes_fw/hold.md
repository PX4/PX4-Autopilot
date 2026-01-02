# Hold Mode (Fixed-Wing)

<img src="../../assets/site/position_fixed.svg" title="Position fix required (e.g. GPS)" width="30px" />

The _Hold_ flight mode causes the vehicle to loiter (circle) around its current GPS position and maintain its current altitude.

There are also a number of [Loiter modes](#loiter-modes), such as figure 8 loiter, that can be run from within Hold mode.

:::tip
_Hold mode_ can be used to pause a mission or to help you regain control of a vehicle in an emergency.
It is usually activated with a pre-programmed switch.
:::

::: info

- Mode is automatic - no user intervention is _required_ to control the vehicle.
- Mode requires a global 3d position estimate (from GPS or inferred from a [local position](../ros/external_position_estimation.md#enabling-auto-modes-with-a-local-position)).
  - Flying vehicles can't switch to this mode without global position.
  - Flying vehicles will failsafe if they lose the position estimate.
  - Disarmed vehicles can switch to mode without valid position estimate but can't arm.
- Mode requires wind and flight time are within allowed limits (specified via parameters).
- RC control switches can be used to change flight modes on any vehicle.
- RC stick movement is ignored.

<!-- https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/ModeUtil/mode_requirements.cpp -->

:::

## Loiter modes

### Default Loiter

The default loiter mode is entered when you switch to Hold mode without explicitly specifying a loiter behaviour.

The aircraft circles around the GPS hold position at the current altitude and with the radius [NAV_LOITER_RAD](#NAV_LOITER_RAD)
The vehicle will first ascend to [NAV_MIN_LTR_ALT](#NAV_MIN_LTR_ALT) if the mode is engaged below this altitude.

RC stick movement is ignored.

### Orbit Loiter Mode

<Badge type="tip" text="PX4 v1.12)" />

Orbit loiter mode allows you to set a position that the vehicle travels to, and then orbits following a circular path.
Unlike the "default loiter" hold mode, the sticks can be used to control the radius and the velocity.

This behaviour can be accessed in QGroundControl by clicking on the map in Fly mode, and selecting **Orbit at Location**.

The behavior can be triggered using the MAVLink [MAV_CMD_DO_ORBIT](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_ORBIT) command and tracked using the [ORBIT_EXECUTION_STATUS](https://mavlink.io/en/messages/common.html#ORBIT_EXECUTION_STATUS) message.

::: info
Unlike MC vehicle, which have a distinct "Orbit Mode", Fixed-Wing vehicles orbit within the hold mode.
:::

### Figure 8 Loiter Mode

<Badge type="tip" text="PX4 v1.15)" /> <Badge type="warning" text="Experimental" />

Figure 8 loiter mode allows you to loiter the vehicle in a figure 8 path, as defined by the radius of two circles.

The feature is experimental and can be enabled by setting the `CONFIG_FIGURE_OF_EIGHT` key in the [PX4 board configuration](../hardware/porting_guide_config.md#px4-board-configuration-kconfig) for your board and rebuilding.
For example, this is enabled on the [default.px4board](https://github.com/PX4/PX4-Autopilot/blob/main/boards/auterion/fmu-v6s/default.px4board#L46) file for the `auterion/fmu-v6s` board.

The mode can also be triggered using the MAVLink [MAV_CMD_DO_FIGURE_EIGHT](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_FIGURE_EIGHT) command.
PX4 then outputs the [FIGURE_EIGHT_EXECUTION_STATUS](https://mavlink.io/en/messages/common.html#FIGURE_EIGHT_EXECUTION_STATUS) message.

Figure 8 loitering is also available in the simulator.
You can test it in [Gazebo](../sim_gazebo_gz/index.md) using a fixed wing frame:

```sh
make px4_sitl gz_rc_cessna
```

Note that at time of writing Figure8 loitering is not supported in QGC ([QGC##12778 Need Support Figure of eight (8 figure) loitering](https://github.com/mavlink/qgroundcontrol/issues/12778)) so you will need to trigger the operation directly via MAVLink.

::: info
Fixed-Wing vehicles fly figure 8 paths within the hold mode.
There is no distinct "Figure 8" mode.
:::

## Parameters

Hold mode behaviour can be configured using the parameters below.

| Parameter                                                                                                | Description                                                                                                   |
| -------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------- |
| <a id="NAV_LOITER_RAD"></a>[NAV_LOITER_RAD](../advanced_config/parameter_reference.md#NAV_LOITER_RAD)    | The radius of the loiter circle.                                                                              |
| <a id="NAV_MIN_LTR_ALT"></a>[NAV_MIN_LTR_ALT](../advanced_config/parameter_reference.md#NAV_MIN_LTR_ALT) | Minimum height for loiter mode (vehicle will ascend to this altitude if mode is engaged at a lower altitude). |

## See Also

[Hold Mode (MC)](../flight_modes_mc/hold.md)

<!-- this maps to AUTO_LOITER in flight mode state machine -->
