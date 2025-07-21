# 飞行终止配置

The _Flight termination_ [failsafe action](../config/safety.md#failsafe-actions) may be triggered by a [safety check](../config/safety.md) (e.g. RC Loss, geofence violation, etc. on any vehicle type or in any flight mode), or by the [Failure Detector](../config/safety.md#failure-detector).

:::info
Flight termination may also be triggered from a ground station or companion computer using the MAVLink [MAV_CMD_DO_FLIGHTTERMINATION](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_FLIGHTTERMINATION) command.
This is sent, for example, when you call the [MAVSDK Action plugin](https://mavsdk.mavlink.io/main/en/cpp/api_reference/classmavsdk_1_1_action.html#classmavsdk_1_1_action_1a47536c4a4bc8367ccd30a92eb09781c5) `terminate()` or `terminate_async()` methods.
:::

When _Flight termination_ is activated, PX4 simultaneously turns off all controllers and sets all PWM outputs to their failsafe values.

根据所连接的设备，PWM 故障保护输出可以用来完成以下动作：

- Deploy a [parachute](../peripherals/parachute.md).
- 伸展可伸缩起落架。
- 将连接了 PWM 的云台移动到安全的方向（或将它回收）以保护摄像机。
- 触发一个可充气的装置，如安全气囊。
- 触发警报。

飞行终止是无法取消的。
在飞行终止触发后，您应尽快拔下电池。
您需要在重启载具或重新上电后才能再次使用。

:::tip
PX4 does not know what safety devices are attached - it just applies a predefined set of PWM values to its outputs.
:::

:::tip
Failsafe values are applied to all outputs on termination.
无法配置电机或特定安全装置为独立的特定时间（或其他方式）触发。
:::

:::info
This is _not_ an independent _Flight Termination System_.
如果断电或者自驾仪完全失控，失效保护设备则不会触发。
:::

## 硬件配置

Any _safety device(s)_ (e.g. a [parachute](../peripherals/parachute.md)) that can be triggered by changing a PWM value can be used, and may be connected to any free PWM port (both MAIN and AUX).

:::info
If you're using Pixhawk-series board you will have to separately power the servo rail (i.e. from a 5V BEC, which is often also available from your ESC).
:::

## 软件配置

The [Safety](../config/safety.md) topic explains how to set the _flight termination_ as the [failsafe action](../config/safety.md#failsafe-actions) to be performed for particular failsafe check.

The [Failure Detector](../config/safety.md#failure-detector) can also (optionally) be configured to trigger flight termination if the vehicle flips (exceeds a certain attitude) or if failure is detected by an external automatic trigger system (ATS):

- Enable the failure detector during flight by setting [CBRK_FLIGHTTERM=0](../advanced_config/parameter_reference.md#CBRK_FLIGHTTERM).
- [Safety > Failure Detector > Attitude Trigger](../config/safety.md#attitude-trigger) explains how to configure the attitude limits that trigger _Flight termination_.
  ::: info
  During _takeoff_ excessive attitutes will trigger _lockdown_ (kill motors, but not launch parachute) rather than flight termination.
  This is always enabled, irrespective of the value of `CBRK_FLIGHTTERM`.

:::
- [Safety > External Automatic Trigger System (ATS)](../config/safety.md#external-automatic-trigger-system-ats) explains how to configure an external trigger system.

对于每个与安全装置相连的 MAIN 输出，其中“n”指 PWM 端口号，设置：

- [PWM_MAIN_DISn](../advanced_config/parameter_reference.md#PWM_MAIN_DIS1) to the device's "OFF" PWM value.
- [PWM_MAIN_FAILn](../advanced_config/parameter_reference.md#PWM_MAIN_FAIL1) to the device's "ON" PWM value.

对于每个与安全装置相连的 AUX 输出，其中“n”指 PWM 端口号，设置：

- [PWM_AUX_DIS1](../advanced_config/parameter_reference.md#PWM_AUX_DIS1) to the device's "OFF" PWM value.
- [PWM_AUX_FAILn](../advanced_config/parameter_reference.md#PWM_AUX_FAIL1) to the device's "ON" PWM value.

Finally, set the `PWM_AUX_FAILn` and `PWM_MAIN_FAILn` PWM values for any motors.

## 逻辑图解

下图显示了飞行终止的逻辑流程。

![Logic diagram](../../assets/config/flight_termination_logic_diagram.png)
