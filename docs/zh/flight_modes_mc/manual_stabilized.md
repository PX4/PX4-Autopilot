# Stabilized Mode (Multicopter)

<img src="../../assets/site/difficulty_medium.png" title="Medium difficulty to fly" width="30px" />&nbsp;<img src="../../assets/site/remote_control.svg" title="Manual/Remote control required" width="30px" />&nbsp;

The _Stabilized_ manual mode stabilizes and levels the multicopter when the RC control sticks are centred.
To move/fly the vehicle you move the sticks outside of the centre.

:::info
This mode is also enabled if you set the flight mode to _Manual_.
:::

When sticks are outside the centre, the roll and pitch sticks control the _angle_ of the vehicle (attitude) around the respective axes, the yaw stick controls the rate of rotation above the horizontal plane, and the throttle controls altitude/speed.

一旦释放摇杆，它们将会返回中心停顿区。
一旦横滚和俯仰摇杆居中，多旋翼无人机将平稳并停止运动。
The vehicle will then hover in place/maintain altitude - provided it is properly balanced, throttle is set appropriately (see [below](#params)), and no external forces are applied (e.g. wind).
飞行器将朝着任何风的方向漂移，您必须控制油门以保持高度。

![MC Manual Flight](../../assets/flight_modes/stabilized_mc.png)

## 技术描述

RC mode where centered sticks level vehicle.

:::info
[Altitude mode](../flight_modes_mc/altitude.md) additionally stabilizes the vehicle altitude when sticks are centred, and [Position mode](../flight_modes_mc/position.md) stabilizes both altitude and position over ground.
:::

飞手的输入通过横滚和俯仰角度以及偏航角速率指令传递给自驾仪。
Throttle is rescaled (see [below](#params)) and passed directly to control allocation.
自动驾驶仪控制着飞机的姿态角，这意味着当 RC 摇杆居中时自驾仪调整飞机的滚转和俯仰角为零（从而实现飞机姿态的改平）。
自动驾驶仪不能补偿由于风（或其他来源）引起的漂移。

- 回正摇杆（内带死区）：
  - Roll/Pitch sticks level vehicle.
- 外部中心：
  - Roll/Pitch sticks control tilt angle in those orientations, resulting in corresponding left-right and forward-back movement.
  - Throttle stick controls up/down speed (and movement speed in other axes).
  - 偏航摇杆控制水平面上方的角度旋转速率。
- Manual control input is required (such as RC control, joystick).
  - Roll, Pitch: Assistance from autopilot to stabilize the attitude.
    Position of RC stick maps to the orientation of vehicle.
  - Throttle: Manual control via RC sticks. RC input is sent directly to control allocation.
  - Yaw: Assistance from autopilot to stabilize the attitude rate.
    Position of RC stick maps to the rate of rotation of vehicle in that orientation.

<a id="params"></a>

## 参数

| 参数                                                                                                                                           | 描述                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| -------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="MPC_THR_HOVER"></a>[MPC_THR_HOVER](../advanced_config/parameter_reference.md#MPC_THR_HOVER) | Hover throttle that is output when the throttle stick is centered and `MPC_THR_CURVE` is set to default.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| <a id="MPC_THR_CURVE"></a>[MPC_THR_CURVE](../advanced_config/parameter_reference.md#MPC_THR_CURVE) | 定义油门缩放比例。 By default this is set to **Rescale to hover thrust**, which means that when the throttle stick is centered the configured hover throttle is output (`MPC_THR_HOVER`) and the stick input is linearly rescaled below and above that (allowing for a smooth transition between Stabilized and Altitude/Position control). <br>在动力很强的机体上，悬停油门可能非常低（例如低于 20％），因此重新调整会使油门输入变形 - 对应上面举例， 80％ 的推力将仅由摇杆输入的中位以上部分控制，20％ 的推力由中位以下的部分来控制。 If needed `MPC_THR_CURVE` can be set to **No Rescale** so that there is no rescaling (stick input to throttle mapping is independent of `MPC_THR_HOVER`). |
