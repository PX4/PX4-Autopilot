# 位置模式（多旋翼）

<img src="../../assets/site/difficulty_easy.png" title="Easy to fly" width="30px" />&nbsp;<img src="../../assets/site/remote_control.svg" title="Manual/Remote control required" width="30px" />&nbsp;<img src="../../assets/site/position_fixed.svg" title="Position fix required (e.g. GPS)" width="30px" />

_Position_ is an easy-to-fly RC mode in which roll and pitch sticks control acceleration over ground in the vehicle's left-right and forward-back directions (similar to a car's accelerator pedal), and throttle controls speed of ascent-descent.
When the sticks are released/centered the vehicle will actively brake, level, and be locked to a position in 3D space — compensating for wind and other forces.
With full stick deflection the vehicle accelerates initially with [MPC_ACC_HOR_MAX](#MPC_ACC_HOR_MAX) ramping down until it reaches the final velocity [MPC_VEL_MANUAL](#MPC_VEL_MANUAL).

:::tip
Position mode is the safest manual mode for new fliers.
Unlike [Altitude](../flight_modes_mc/altitude.md) and [Stabilized](../flight_modes_mc/manual_stabilized.md) modes the vehicle will stop when the sticks are centered rather than continuing until slowed by wind resistance.
:::

The diagram below shows the mode behaviour visually (for a mode 2 transmitter).

![MC Position Mode](../../assets/flight_modes/position_mc.png)

### 降落

该模式中降落是很容易的：

1. 使用横滚和俯仰杆控制无人机水平位置于降落点上方。
2. 松开横滚和俯仰杆并给予足够的时间使其完全停止。
3. 轻轻下拉油门杆直到机体触碰地面。
4. 将油门杆一直向下拉以促进和加快着陆检测。
5. The vehicle will lower propeller thrust, detect the ground and [automatically disarm](../advanced_config/prearm_arm_disarm.md#auto-disarming) (by default).

:::warning
While very rare on a well calibrated vehicle, sometimes there may be problems with landing.

- 如果机体无法停止水平移动：
  - You can still land under control in [Altitude mode](../flight_modes_mc/altitude.md).
    方法与上述相同，除了您必须使用横滚和俯仰杆手动确保机体保持在降落点上方。
  - 降落后检查 GPS 和磁罗盘方向，并校准。
- If the vehicle does not detect the ground/landing and disarm:
  - After the vehicle is on the ground switch to [Stabilized mode](../flight_modes_mc/manual_stabilized.md) keeping the throttle stick low, and manually disarm using a gesture or other command.
    或者，当机体已经在地面上时，您也可以使用断电开关。

:::

## 技术总结

遥控模式下，横滚、俯仰、油门 (RPT) 杆控制相应轴/方向的运动。
摇杆居中使机体水平并将其保持在固定的高度和位置并抗风。

- Centered roll, pitch, throttle sticks (within RC deadzone [MPC_HOLD_DZ](../advanced_config/parameter_reference.md#MPC_HOLD_DZ)) hold x, y, z position steady against any disturbance like wind.
- 外部中心：
  - Roll/Pitch sticks control horizontal acceleration over ground in the vehicle's left-right and forward-back directions (respectively).
  - Throttle stick controls speed of ascent-descent.
  - 偏航摇杆控制水平面上方的角度旋转速率。
- 起飞：
  - 降落时，如果将油门杆抬高至 62.5%（从油门杆最低开始的整个范围），无人机将起飞。
- Global position estimate is required.
- Manual control input is required (such as RC control, joystick).
  - Roll, Pitch, Throttle: Assistance from autopilot to hold position against wind.
  - Yaw: Assistance from autopilot to stabilize the attitude rate.
    Position of RC stick maps to the rate of rotation of vehicle in that orientation.

### 参数

All the parameters in the [Multicopter Position Control](../advanced_config/parameter_reference.md#multicopter-position-control) group are relevant. 下面列出了一些特别值得注意的参数。

| 参数                                                                                                                                                                                              | 描述                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="MPC_HOLD_DZ"></a>[MPC_HOLD_DZ](../advanced_config/parameter_reference.md#MPC_HOLD_DZ)                                                          | 启用位置保持的摇杆死区。 默认值：0.1（摇杆全行程的 10％）。                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| <a id="MPC_Z_VEL_MAX_UP"></a>[MPC_Z_VEL_MAX_UP](../advanced_config/parameter_reference.md#MPC_Z_VEL_MAX_UP) | 最大垂直上升速度。 默认：3m/s。                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| <a id="MPC_Z_VEL_MAX_DN"></a>[MPC_Z_VEL_MAX_DN](../advanced_config/parameter_reference.md#MPC_Z_VEL_MAX_DN) | 最大垂直下降速度。 默认：1m/s。                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| <a id="MPC_LAND_ALT1"></a>[MPC_LAND_ALT1](../advanced_config/parameter_reference.md#MPC_LAND_ALT1)                                                    | 触发第一阶段降速的高度。 Below this altitude descending velocity gets limited to a value between [MPC_Z_VEL_MAX_DN](#MPC_Z_VEL_MAX_DN) (or `MPC_Z_V_AUTO_DN`) and [MPC_LAND_SPEED](#MPC_LAND_SPEED). Value needs to be higher than [MPC_LAND_ALT2](#MPC_LAND_ALT2). Default 10m. |
| <a id="MPC_LAND_ALT2"></a>[MPC_LAND_ALT2](../advanced_config/parameter_reference.md#MPC_LAND_ALT2)                                                    | 触发第二阶段降速的高度。 Below this altitude descending velocity gets limited to [`MPC_LAND_SPEED`](#MPC_LAND_SPEED). Value needs to be lower than "MPC_LAND_ALT1". Default 5m.                                                                                                                                                                                                                                                   |
| <a id="RCX_DZ"></a>`RCX_DZ`                                                                                                                                                                     | RC dead zone for channel X. The value of X for throttle will depend on the value of [RC_MAP_THROTTLE](../advanced_config/parameter_reference.md#RC_MAP_THROTTLE). For example, if the throttle is channel 4 then [RC4_DZ](../advanced_config/parameter_reference.md#RC4_DZ) specifies the deadzone.                                                                                              |
| <a id="MPC_xxx"></a>`MPC_XXXX`                                                                                                                                                                  | 大多数 MPC_xxx参数会影响此模式下的飞行行为（至少在某种程度上）。 For example, [MPC_THR_HOVER](../advanced_config/parameter_reference.md#MPC_THR_HOVER) defines the thrust at which a vehicle will hover.                                                                                                                                                                                                                                                     |
| <a id="MPC_POS_MODE"></a>[MPC_POS_MODE](../advanced_config/parameter_reference.md#MPC_POS_MODE)                                                       | 从摇杆输入到机体动作的转换策略。 From PX4 v1.12 the default (`Acceleration based`) is that stick position controls acceleration (in a similar way to a car accelerator pedal). 其他选项允许操纵杆偏转直接控制地面速度，有或没有平滑和加速度限制。                                                                                                                                                                                                                                          |
| <a id="MPC_ACC_HOR_MAX"></a>[MPC_ACC_HOR_MAX](../advanced_config/parameter_reference.md#MPC_ACC_HOR_MAX)                         | 最大水平加速度。                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| <a id="MPC_VEL_MANUAL"></a>[MPC_VEL_MANUAL](../advanced_config/parameter_reference.md#MPC_VEL_MANUAL)                                                 | 最大水平速度。                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| <a id="MPC_LAND_SPEED"></a>[MPC_LAND_SPEED](../advanced_config/parameter_reference.md#MPC_LAND_SPEED)                                                 | Landing descend rate. Landing descend rate. Default 0.7 m/s.                                                                                                                                                                                                                                                                                                                                                                                    |

## 附加信息

### 位置丢失/安全

位置模式依赖于一个可接受的位置估计。
If the estimate falls below acceptable levels, for example due to GPS loss, this may trigger a [Position (GPS) Loss Failsafe](../config/safety.md#position-gnss-loss-failsafe).
如果估计值低于可接受的水平，例如由于 GPS 丢失，这可能会触发位置 (GPS) 丢失故障保护 根据配置，是否有遥控器，以及是否有足够的高度估计，PX4 可能会切换到高度模式、手动模式、降落模式或终止。

## See Also

- [Position Slow Mode](../flight_modes_mc/position_slow.md)
