# Position Mode (Fixed-wing)

<img src="../../assets/site/difficulty_easy.png" title="Easy to fly" width="30px" />&nbsp;<img src="../../assets/site/remote_control.svg" title="Manual/Remote control required" width="30px" />&nbsp;<img src="../../assets/site/position_fixed.svg" title="Position fix required (e.g. GPS)" width="30px" />

_Position mode_ is the easiest and safest manual mode.
It is supported on vehicles that have a position estimate (e.g. GPS).
It makes it easier for pilots to control vehicle altitude, and in particular to reach and maintain a fixed altitude.
The mode will hold the vehicle's course against wind.
Airspeed is actively controlled if an airspeed sensor is installed.

The vehicle performs a [coordinated turn](https://en.wikipedia.org/wiki/Coordinated_flight) if the roll sticks are non-zero, while the pitch stick controls the rate of ascent/descent.
滚动和俯仰是角度控制的（因此不可能实现飞机滚转或环绕）。

When all sticks are released/centered (no roll, pitch, yaw, and ~50% throttle) the aircraft will return to straight, level flight, and keep its current altitude and flight path irrespective of wind.
This makes it easy to recover from any problems when flying.
Roll and pitch are angle-controlled (so it is impossible to roll over or loop the vehicle).

The yaw stick can be used to increase/reduce the yaw rate of the vehicle in turns.
If left at center the controller does the turn coordination by itself, meaning that it will apply the necessary yaw rate for the current roll angle to perform a smooth turn.
The diagram below shows the mode behaviour visually (for a [mode 2 transmitter](../getting_started/rc_transmitter_receiver.md#transmitter_modes)).

![FW Position Mode](../../assets/flight_modes/position_fw.png)

## 技术描述

Position mode is like [Stabilized mode](../flight_modes_fw/altitude.md) but with course stabilization.
Airspeed is also stabilized if an airspeed sensor is present.

- 回中的滚动/俯仰/偏航输入（在死区内）：
  - Autopilot levels vehicle and maintains altitude, airspeed and course over ground.
- 外部中心：
  - 俯仰摇杆控制高度。
  - 如果空速传感器已连接，油门杆控制飞机速度。 Without an airspeed sensor the vehicle will fly level at trim throttle ([FW_THR_TRIM](../advanced_config/parameter_reference.md#FW_THR_TRIM)), increasing or decreasing throttle as needed to climb or descend.
  - 横滚摇杆控制横滚角度。 Autopilot will maintain [coordinated flight](https://en.wikipedia.org/wiki/Coordinated_flight).
  - 偏航摇杆操纵会驱动方向舵（指令将被加到自动驾驶仪计算的指令中以维持 <a href="https://en.wikipedia.org/wiki/Coordinated_flight">协调飞行</a>）。
    这和<a href="../flight_modes/stabilized_fw.md">稳定模式</a>一样。
- Manual control input is required (such as RC control, joystick).
- An altitude measurement source is required (usually barometer or GPS)

## 参数

该模式受以下参数影响：

| 参数                                                                                                                                                                                              | 描述                                                                                              |
| ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------- |
| <a id="FW_AIRSPD_MIN"></a>[FW_AIRSPD_MIN](../advanced_config/parameter_reference.md#FW_AIRSPD_MIN)                                                    | 最小空速/油门。 默认：10 m/s。                                                                             |
| <a id="FW_AIRSPD_MAX"></a>[FW_AIRSPD_MAX](../advanced_config/parameter_reference.md#FW_AIRSPD_MAX)                                                    | 最大空速/油门。 默认：20 m/s。                                                                             |
| <a id="FW_AIRSPD_TRIM"></a>[FW_AIRSPD_TRIM](../advanced_config/parameter_reference.md#FW_AIRSPD_TRIM)                                                 | 巡航速度。 默认：15 m/s。                                                                                |
| <a id="FW_MAN_P_MAX"></a>[FW_MAN_P_MAX](../advanced_config/parameter_reference.md#FW_MAN_P_MAX)                                  | 在高度稳定模式下手动控制的最大俯仰角。 Default: 45 degrees.                        |
| <a id="FW_MAN_R_MAX"></a>[FW_MAN_R_MAX](../advanced_config/parameter_reference.md#FW_MAN_R_MAX)                                  | 在高度稳定模式下手动控制的最大滚转角。 Default: 45 degrees.                        |
| <a id="FW_T_CLMB_R_SP"></a>[FW_T_CLMB_R_SP](../advanced_config/parameter_reference.md#FW_T_CLMB_R_SP)       | Max climb rate setpoint. 默认：3m/s。                                               |
| <a id="FW_T_SINK_R_SP"></a>[FW_T_SINK_R_SP](../advanced_config/parameter_reference.md#FW_T_SINK_R_SP)       | Max sink rate setpoint. Default: 2 m/s.         |
| <a id="FW_PN_R_SLEW_MAX"></a>[FW_PN_R_SLEW_MAX](../advanced_config/parameter_reference.md#FW_PN_R_SLEW_MAX) | Roll setpoint slew rate limit. Default: 90 °/s. |
