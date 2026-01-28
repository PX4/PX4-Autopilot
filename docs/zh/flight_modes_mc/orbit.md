# 环绕模式 （多旋翼）

<img src="../../assets/site/difficulty_easy.png" title="Easy to fly" width="30px" />&nbsp;<img src="../../assets/site/position_fixed.svg" title="Position fix required (e.g. GPS)" width="30px" />

The _Orbit_ guided flight mode allows you to command a multicopter (or VTOL in multicopter mode) to fly in a circle at a particular location, by [default](https://mavlink.io/en/messages/common.html#ORBIT_YAW_BEHAVIOUR) yawing so that it always faces towards the center.

::: info

- Mode is automatic - no user intervention is _required_ to control the vehicle.
- Mode requires at least a valid local position estimate (does not require a global position).
  - Flying vehicles can't switch to this mode without valid local position.
  - Flying vehicles will failsafe if they lose the position estimate.
- Mode prevents arming (vehicle must be armed when switching to this mode).
- Mode requires wind and flight time are within allowed limits (specified via parameters).
- This mode is currently only supported on multicopter (or VTOL in MC mode).
- RC stick movement can control ascent/descent and orbit speed and direction.
- The mode can be triggered using the [MAV_CMD_DO_ORBIT](https://mavlink.io/en/messages/common.html#MMAV_CMD_DO_ORBIT) MAVLink command.

<!-- https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/ModeUtil/mode_requirements.cpp -->

:::

## 综述

![Orbit Mode - MC](../../assets/flying/orbit.jpg)

_QGroundControl_ (or other compatible GCS or MAVLink API) is _required_ to enable the mode, and to set the center position, initial radius and altitude of the orbit.
Once enabled the vehicle will fly as fast as possible to the closest point on the commanded circle trajectory and do a slow (1m/s) clockwise orbit on the planned circle, facing the center.

Instructions for how to start an orbit can be found here: [FlyView > Orbit Location](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/fly_view/fly_view.html#orbit) (_QGroundControl_ guide).

:::info
The use of an RC control is _optional_.
If no RC control is present the orbit will proceed as described above.
无法使用遥控来启动该模式（如果使用遥控切换该模式，无人机会处于空闲状态）。
:::

遥控可以用于改变绕圈的高度，半径，速度和绕圈方向：

- **Left stick:**
  - _up/down:_ controls speed of ascent/descent, as in [Position mode](../flight_modes_mc/position.md).
    When in center deadzone, altitude is locked.
  - _left/right:_ no effect.
- **Right stick:**
  - _left/right:_ controls acceleration of orbit in clockwise/counter-clockwise directions.
    When centered the current speed is locked.
    - Maximum velocity is [MPC_XY_VEL_MAX](#MPC_XY_VEL_MAX) and further limited to keep the centripetal acceleration below 2m/s^2.
  - _up/down:_ controls orbit radius (smaller/bigger). When centered the current radius is locked.
    - 最小半径是 1 米。
      Maximum radius is [MC_ORBIT_RAD_MAX](#MC_ORBIT_RAD_MAX).

The diagram below shows the mode behaviour visually (for a [mode 2 transmitter](../getting_started/rc_transmitter_receiver.md#transmitter_modes)).

![Orbit Mode - MC](../../assets/flight_modes/orbit_mc.png)

切换到其他飞行模式（使用遥控或 QGC 地面站）可以停止此模式。

## 参数/限制

该模式受以下参数影响：

| 参数                                                                                                                                                                         | 描述                                                                                                                                                               |
| -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="MC_ORBIT_RAD_MAX"></a>[MC_ORBIT_RAD_MAX](../advanced_config/parameter_reference.md#MC_ORBIT_RAD_MAX) | Maximum radius of orbit. Default: 1000m.                                                                         |
| <a id="MC_ORBIT_YAW_MOD"></a>[MC_ORBIT_YAW_MOD](../advanced_config/parameter_reference.md#MC_ORBIT_YAW_MOD) | Yaw behaviour during orbit flight. Default: Front to Circle Center.                                              |
| <a id="MPC_XY_VEL_MAX"></a>[MPC_XY_VEL_MAX](../advanced_config/parameter_reference.md#MPC_XY_VEL_MAX)       | Tangential speed limit. Stick input won't accelerate beyond this limit. Higher commands are accepted but capped. |

下面的限制是写死的：

- Initial/default rotation is 1m/s in a clockwise direction.
- The maximum acceleration is limited to 2m/s^2, with priority on keeping the commanded circle trajectory rather than commanded ground speed (i.e. the vehicle will slow down in order to achieve the correct circle if the acceleration exceeds 2m/s^2).

## MAVLink 消息 （开发者）

环绕模式使用以下 MAVLink 命令：

- [MAV_CMD_DO_ORBIT](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_ORBIT) - Start an orbit with specified center point, radius, direction, altitude, speed and [yaw direction](https://mavlink.io/en/messages/common.html#ORBIT_YAW_BEHAVIOUR).
  The same defaults and limits apply.
  When exceeding limits the command is accepted but velocity and radius capped.
- [ORBIT_EXECUTION_STATUS](https://mavlink.io/en/messages/common.html#ORBIT_EXECUTION_STATUS) - Orbit status emitted during orbit to update GCS of current orbit parameters (these may be changed by the RC controller).
