# Stabilized Mode (Fixed-wing)

<img src="../../assets/site/difficulty_medium.png" title="Medium difficulty to fly" width="30px" />&nbsp;<img src="../../assets/site/remote_control.svg" title="Manual/Remote control required" width="30px" />

_Stabilized mode_ is a manual mode were centering the sticks levels the vehicle attitude (roll and pitch) and maintains the horizontal posture.

:::info
_Stabilized mode_ is similar to [Altitude mode](../flight_modes_fw/altitude.md) in that releasing the sticks levels the vehicle, but unlike altitude mode it does not maintain altitude or heading.
It is much easier to fly than [Manual mode](../flight_modes_fw/manual.md) because you can't roll or flip it, and if needed it is easy to level the vehicle (by centering the control sticks).
:::

The vehicle climbs/descends based on pitch and throttle input and performs a [coordinated turn](https://en.wikipedia.org/wiki/Coordinated_flight) if the roll stick is non-zero.
横滚和俯仰是角度控制的（不能上下滚动或循环）。

如果油门降至 0％（电机停止），飞机将滑行。
为了执行转弯，必须在整个操纵过程中保持命令，因为如果释放横滚摇杆，则飞机将停止转动并自行调平（对于俯仰和偏航命令也是如此）。

The yaw stick can be used to increase/reduce the yaw rate of the vehicle in turns.
If left at center the controller does the turn coordination by itself, meaning that it will apply the necessary yaw rate for the current roll angle to perform a smooth turn.

The diagram below shows the mode behaviour visually (for a [mode 2 transmitter](../getting_started/rc_transmitter_receiver.md#transmitter_modes)).

![FW Manual Flight](../../assets/flight_modes/stabilized_fw.png)

## 技术描述

Manual mode where centered roll/pitch sticks levels vehicle attitude.
The vehicle course and altitude are not maintained, and can drift due to wind.

- Centered Roll/Pitch/Yaw sticks (inside deadband) put vehicle into straight and level flight.
  The vehicle course and altitude are not maintained, and can drift due to wind.
- 横滚摇杆控制横滚角度。
  自动驾驶仪将保持 <a href="https://en.wikipedia.org/wiki/Coordinated_flight">协调飞行</a>。
- Pitch stick controls pitch angle around the defined offset [FW_PSP_OFF](../advanced_config/parameter_reference.md#FW_PSP_OFF)
- Throttle stick controls throttle directly.
- 偏航摇杆操纵会驱动方向舵（指令将被加到自动驾驶仪计算的指令中以维持 <a href="https://en.wikipedia.org/wiki/Coordinated_flight">协调飞行</a>）。
  这和<a href="../flight_modes/stabilized_fw.md">稳定模式</a>一样。
- Manual control input is required (such as RC control, joystick).

## 参数

该模式受以下参数影响：

| 参数                                                                                                                                                                | 描述                                                                                                                                    |
| ----------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="FW_MAN_P_MAX"></a>[FW_MAN_P_MAX](../advanced_config/parameter_reference.md#FW_MAN_P_MAX)    | Max pitch for manual control in attitude stabilized mode. Default: 45 degrees.        |
| <a id="FW_MAN_R_MAX"></a>[FW_MAN_R_MAX](../advanced_config/parameter_reference.md#FW_MAN_R_MAX)    | Max roll for manual control in attitude stabilized mode. Default: 45 degrees.         |
| <a id="FW_MAN_YR_MAX"></a>[FW_MAN_YR_MAX](../advanced_config/parameter_reference.md#FW_MAN_YR_MAX) | Maximum manually added yaw rate . Default: 30 degrees per second.                     |
| <a id="FW_PSP_OFF"></a>[FW_PSP_OFF](../advanced_config/parameter_reference.md#FW_PSP_OFF)                               | Pitch setpoint offset (pitch at level flight). Default: 0 degrees. |

<!-- this document needs to be extended -->
