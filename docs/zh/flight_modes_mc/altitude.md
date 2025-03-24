# 定高模式（多旋翼）

<img src="../../assets/site/difficulty_easy.png" title="Easy to fly" width="30px" />&nbsp;<img src="../../assets/site/remote_control.svg" title="Manual/Remote control required" width="30px" />&nbsp;<img src="../../assets/site/altitude_icon.svg" title="Altitude required (e.g. Baro, Rangefinder)" width="30px" />

_Altitude mode_ is a _relatively_ easy-to-fly RC mode in which roll and pitch sticks control vehicle movement in the left-right and forward-back directions (relative to the "front" of the vehicle), yaw stick controls rate of rotation over the horizontal plane, and throttle controls speed of ascent-descent.

When the sticks are released/centered the vehicle will level and maintain the current _altitude_.
如果在水平面上运动，飞机将继持续运动直到任何动量被风阻力消散。
如果刮风，飞机会向风的方向漂移。

:::tip
_Altitude mode_ is the safest non-GPS manual mode for new fliers. It is just like [Stabilized](../flight_modes_mc/manual_stabilized.md) mode but additionally locks the vehicle altitude when the sticks are released.
:::

The diagram below shows the mode behaviour visually (for a [mode 2 transmitter](../getting_started/rc_transmitter_receiver.md#transmitter_modes)).

![Altitude Control MC - Mode2 RC Controller](../../assets/flight_modes/altitude_mc.png)

## 技术总结

RC/manual mode like [Stabilized mode](../flight_modes_mc/manual_stabilized.md) but with _altitude stabilization_ (centred sticks level vehicle and hold it to fixed altitude).
The horizontal position of the vehicle can move due to wind (or pre-existing momentum).

- 回正摇杆（内带死区）：
  - RPY摇杆使飞机水平。
  - 油门（~50%）抗风保持当前姿态。
- 外部中心：
  - 翻滚/俯仰摇杆控制各自方向的倾斜角，导致左右和前后的移动。
  - 油门摇杆以预定的最大速率（和其他轴上的移动速度）控制上升速度。
  - 偏航摇杆控制水平面上方的角度旋转速率。
- 起飞：
  - 降落时，如果将油门杆抬高至 62.5%（从油门杆最低开始的整个范围），无人机将起飞。
- Altitude is normally measured using a barometer, which may become inaccurate in extreme weather conditions.
  带有激光雷达/距离传感器的飞机将能够以更高的可靠性和准确性控制高度。
- Manual control input is required (such as RC control, joystick).
  - Roll, Pitch: Assistance from autopilot to stabilize the attitude.
    Position of RC stick maps to the orientation of vehicle.
  - Throttle: Assistance from autopilot to hold position against wind.
  - Yaw: Assistance from autopilot to stabilize the attitude rate.
    Position of RC stick maps to the rate of rotation of vehicle in that orientation.

## 参数

该模式受以下参数影响：

| 参数                                                                                                                                                                                              | 描述                                                                                                                                                                                                                                                                                                                                                                                                                 |
| ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| <a id="MPC_Z_VEL_MAX_UP"></a>[MPC_Z_VEL_MAX_UP](../advanced_config/parameter_reference.md#MPC_Z_VEL_MAX_UP) | 最大垂直上升速度。 默认：3m/s。                                                                                                                                                                                                                                                                                                                                                                                                 |
| <a id="MPC_Z_VEL_MAX_DN"></a>[MPC_Z_VEL_MAX_DN](../advanced_config/parameter_reference.md#MPC_Z_VEL_MAX_DN) | 最大垂直下降速度。 默认：1m/s。                                                                                                                                                                                                                                                                                                                                                                                                 |
| <a id="RCX_DZ"></a>`RCX_DZ`                                                                                                                                                                     | RC dead zone for channel X. The value of X for throttle will depend on the value of [RC_MAP_THROTTLE](../advanced_config/parameter_reference.md#RC_MAP_THROTTLE). For example, if the throttle is channel 4 then [RC4_DZ](../advanced_config/parameter_reference.md#RC4_DZ) specifies the deadzone. |
| <a id="MPC_xxx"></a>`MPC_XXXX`                                                                                                                                                                  | 大多数 MPC_xxx参数会影响此模式下的飞行行为（至少在某种程度上）。 For example, [MPC_THR_HOVER](../advanced_config/parameter_reference.md#MPC_THR_HOVER) defines the thrust at which a vehicle will hover.                                                                                                                                                        |
