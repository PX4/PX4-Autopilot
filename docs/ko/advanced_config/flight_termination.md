# 비행 중단 설정

The _Flight termination_ [failsafe action](../config/safety.md#failsafe-actions) may be triggered by a [safety check](../config/safety.md) (e.g. RC Loss, geofence violation, etc. on any vehicle type or in any flight mode), or by the [Failure Detector](../config/safety.md#failure-detector).

:::info
Flight termination may also be triggered from a ground station or companion computer using the MAVLink [MAV_CMD_DO_FLIGHTTERMINATION](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_FLIGHTTERMINATION) command.
This is sent, for example, when you call the [MAVSDK Action plugin](https://mavsdk.mavlink.io/main/en/cpp/api_reference/classmavsdk_1_1_action.html#classmavsdk_1_1_action_1a47536c4a4bc8367ccd30a92eb09781c5) `terminate()` or `terminate_async()` methods.
:::

When _Flight termination_ is activated, PX4 simultaneously turns off all controllers and sets all PWM outputs to their failsafe values.

Depending on what devices are connected, the PWM failsafe outputs can be used to:

- Deploy a [parachute](../peripherals/parachute.md).
- 랜딩기어 펼치기.
- 카메라를 보호하기 위해 PWM방식으로 연결된 짐벌을 안전 각도(또는 수납 위치) 로 움직이기.
- 에어백 같은 팽창 장비 가동하기.
- 알람 울리기.

:::tip
PX4는 어떤 안전 장치가 장착되어 있는지 알지 못합니다. 미리 정의된 PWM 값 세트를 출력에 적용하기만 하면 됩니다.
After triggering you should unplug the battery as soon as possible.
You will need to reboot/power cycle the vehicle before it can be used again.

:::tip
PX4 does not know what safety devices are attached - it just applies a predefined set of PWM values to its outputs.
:::

:::tip
Failsafe values are applied to all outputs on termination.
전원이 끊기거나 자동조종장치가 완전히 충돌하면 안전 장치가 작동하지 않습니다.
:::

:::info
This is _not_ an independent _Flight Termination System_.
If power is lost or if the autopilot crashes completely, the failsafe devices will not be triggered.
:::

## 하드웨어 설정

Any _safety device(s)_ (e.g. a [parachute](../peripherals/parachute.md)) that can be triggered by changing a PWM value can be used, and may be connected to any free PWM port (both MAIN and AUX).

:::info
If you're using Pixhawk-series board you will have to separately power the servo rail (i.e. from a 5V BEC, which is often also available from your ESC).
:::

## 소프트웨어 설정

The [Safety](../config/safety.md) topic explains how to set the _flight termination_ as the [failsafe action](../config/safety.md#failsafe-actions) to be performed for particular failsafe check.

The [Failure Detector](../config/safety.md#failure-detector) can also (optionally) be configured to trigger flight termination if the vehicle flips (exceeds a certain attitude) or if failure is detected by an external automatic trigger system (ATS):

- Enable the failure detector during flight by setting [CBRK_FLIGHTTERM=0](../advanced_config/parameter_reference.md#CBRK_FLIGHTTERM).
- [Safety > Failure Detector > Attitude Trigger](../config/safety.md#attitude-trigger) explains how to configure the attitude limits that trigger _Flight termination_.
  ::: info
  During _takeoff_ excessive attitutes will trigger _lockdown_ (kill motors, but not launch parachute) rather than flight termination.
  This is always enabled, irrespective of the value of `CBRK_FLIGHTTERM`.

:::
- [Safety > External Automatic Trigger System (ATS)](../config/safety.md#external-automatic-trigger-system-ats) explains how to configure an external trigger system.

아래의 다이어그램은 비행 중단과 관련된 논리적 흐름을 보여 줍니다.

- [PWM_MAIN_DISn](../advanced_config/parameter_reference.md#PWM_MAIN_DIS1) to the device's "OFF" PWM value.
- [PWM_MAIN_FAILn](../advanced_config/parameter_reference.md#PWM_MAIN_FAIL1) to the device's "ON" PWM value.

마지막으로, 모든 모터에 대해 <code>PWM_AUX_FAILn</code> 및 <code>PWM_MAIN_FAILn</code> PWM 값을 설정합니다.

- [PWM_AUX_DIS1](../advanced_config/parameter_reference.md#PWM_AUX_DIS1) to the device's "OFF" PWM value.
- [PWM_AUX_FAILn](../advanced_config/parameter_reference.md#PWM_AUX_FAIL1) to the device's "ON" PWM value.

Finally, set the `PWM_AUX_FAILn` and `PWM_MAIN_FAILn` PWM values for any motors.

## 로직 다이어그램

The diagram below shows the logical flow around flight termination.

![Logic diagram](../../assets/config/flight_termination_logic_diagram.png)
