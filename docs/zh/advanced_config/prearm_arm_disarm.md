# Arm, Disarm, Prearm Configuration

机体是有可移动的部件的，其中一些在通电后会有潜在的危险性（特别是电机和螺旋桨）！

为了减少发生事故的可能性， PX4 有明确的载具部件供电状态：

- **Disarmed:** There is no power to motors or actuators.
- **Pre-armed:** Motors/propellers are locked but actuators for non-dangerous electronics are powered (e.g. ailerons, flaps etc.).
- **Armed:** Vehicle is fully powered. 电机/螺旋桨可能转动（危险！）

:::info
Ground stations may display _disarmed_ for pre-armed vehicles.
虽然技术上不适合预解锁的载具，但它是“安全的”。
:::

Users can control progression though these states using a [safety switch](../getting_started/px4_basic_concepts.md#safety-switch) on the vehicle (optional) _and_ an [arming switch/button](#arm_disarm_switch), [arming gesture](#arm_disarm_gestures), or _MAVLink command_ on the ground controller:

- A _safety switch_ is a control _on the vehicle_ that must be engaged before the vehicle can be armed, and which may also prevent prearming (depending on the configuration).
  通常，安全开关被集成到 GPS 单元中，但也可能是一个单独的物理组件。

  :::warning
  A vehicle that is armed is potentially dangerous.
  安全开关是防止意外解锁发生的一个附加机制。

:::

- An _arming switch_ is a switch or button _on an RC controller_ that can be used to arm the vehicle and start motors (provided arming is not prevented by a safety switch).

- An _arming gesture_ is a stick movement _on an RC controller_ that can be used as an alternative to an arming switch.

- 也可以通过地面站发送 MAVLink 命令来解锁/锁定载具。

如果在解锁后的一段时间内没有起飞，和如果着陆后没有手动锁定， PX4 会自动锁定载具。
这减少了解锁的载具在地面上的时间。

PX4 allows you to configure how pre-arming, arming and disarming work using parameters (which can be edited in _QGroundControl_ via the [parameter editor](../advanced_config/parameters.md)), as described in the following sections.

:::tip
Arming/disarming parameters can be found in [Parameter Reference > Commander](../advanced_config/parameter_reference.md#commander) (search for `COM_ARM_*` and `COM_DISARM_*`).
:::

## Arming/Disarming Gestures {#arm_disarm_gestures}

默认情况下，载具可以通过移动遥控器的油门/偏航摇杆到特定的位置，并保持 1 秒来解锁和锁定。

- **Arming:** Throttle minimum, yaw maximum
- **Disarming:** Throttle minimum, yaw minimum

RC controllers will use different sticks for throttle and yaw [based on their mode](../getting_started/rc_transmitter_receiver.md#types-of-remote-controllers), and hence different gestures:

- **Mode 2**:
  - _Arm:_ Left stick to bottom right.
  - _Disarm:_ Left stick to the bottom left.
- **Mode 1**:
  - _Arm:_ Left-stick to right, right-stick to bottom.
  - _Disarm:_ Left-stick to left, right-stick to the bottom.

The required hold time can be configured using [COM_RC_ARM_HYST](#COM_RC_ARM_HYST).
Note that by default ([COM_DISARM_MAN](#COM_DISARM_MAN)) you can also disarm in flight using gestures/buttons: you may choose to disable this to avoid accidental disarming.

| 参数                                                                                                                                                                      | 描述                                                                                                                                                                                                      |
| ----------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="MAN_ARM_GESTURE"></a>[MAN_ARM_GESTURE](../advanced_config/parameter_reference.md#MAN_ARM_GESTURE)                      | Enable arm/disarm stick guesture. `0`: Disabled, `1`: Enabled (default).                                             |
| <a id="COM_DISARM_MAN"></a>[COM_DISARM_MAN](../advanced_config/parameter_reference.md#COM_DISARM_MAN)                         | Enable disarming in flight via switch/stick/button in MC manual thrust modes. `0`: Disabled, `1`: Enabled (default). |
| <a id="COM_RC_ARM_HYST"></a>[COM_RC_ARM_HYST](../advanced_config/parameter_reference.md#COM_RC_ARM_HYST) | Time that RC stick must be held in arm/disarm position before arming/disarming occurs (default: `1` second).                                         |

## Arming Button/Switch {#arm_disarm_switch}

An _arming button_ or "momentary switch" can be configured to trigger arm/disarm _instead_ of [gesture-based arming](#arm_disarm_gestures) (setting an arming switch disables arming gestures).
The button should be held down for ([nominally](#COM_RC_ARM_HYST)) one second to arm (when disarmed) or disarm (when armed).

A two-position switch can also be used for arming/disarming, where the respective arm/disarm commands are sent on switch _transitions_.

:::tip
Two-position arming switches are primarily used in/recommended for racing drones.
:::

The switch or button is assigned (and enabled) using [RC_MAP_ARM_SW](#RC_MAP_ARM_SW), and the switch "type" is configured using [COM_ARM_SWISBTN](#COM_ARM_SWISBTN).

| 参数                                                                                                                                                                | 描述                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| ----------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="RC_MAP_ARM_SW"></a>[RC_MAP_ARM_SW](../advanced_config/parameter_reference.md#RC_MAP_ARM_SW) | RC arm switch channel (default: 0 - unassigned). If defined, the specified RC channel (button/switch) is used for arming instead of a stick gesture. <br>**Note:**<br>- This setting _disables the stick gesture_!<br>- This setting applies to RC controllers. It does not apply to Joystick controllers that are connected via _QGroundControl_. |
| <a id="COM_ARM_SWISBTN"></a>[COM_ARM_SWISBTN](../advanced_config/parameter_reference.md#COM_ARM_SWISBTN)                | Arm switch is a momentary button. <br>- `0`: Arm switch is a 2-position switch where arm/disarm commands are sent on switch transitions.<br>-`1`: Arm switch is a button or momentary button where the arm/disarm command ae sent after holding down button for set time ([COM_RC_ARM_HYST](#COM_RC_ARM_HYST)).        |

:::info
The switch can also be set as part of _QGroundControl_ [Flight Mode](../config/flight_mode.md) configuration.
:::

## 自动上锁

By default vehicles will automatically disarm on landing, or if you take too long to take off after arming.
The feature is configured using the following timeouts.

| 参数                                                                                                                                                    | 描述                                                                                                                                                                          |
| ----------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="COM_DISARM_LAND"></a>[COM_DISARM_LAND](../advanced_config/parameter_reference.md#COM_DISARM_LAND)    | 降落后自动锁定超时时间. Default: 2s (-1 to disable).                                                                |
| <a id="COM_DISARM_PRFLT"></a>[COM_DISARM_PRFLT](../advanced_config/parameter_reference.md#COM_DISARM_PRFLT) | Time-out for auto disarm if too slow to takeoff. Default: 10s (<=0 to disable). |

## Pre-Arm Checks

To reduce accidents, vehicles are only allowed to arm certain conditions are met (some of which are configurable).
Arming is prevented if:

- The vehicle is not in a "healthy" state.
  For example it is not calibrated, or is reporting sensor errors.
- The vehicle has a [safety switch](../getting_started/px4_basic_concepts.md#safety-switch) that has not been engaged.
- The vehicle has a [remote ID](../peripherals/remote_id.md) that is unhealthy or otherwise not ready
- A VTOL vehicle is in fixed-wing mode ([by default](../advanced_config/parameter_reference.md#CBRK_VTOLARMING)).
- The current mode requires an adequate global position estimate but the vehicle does not have GPS lock.
- Many more (see [arming/disarming safety settings](../config/safety.md#arming-disarming-settings) for more information).

The current failed checks can be viewed in QGroundControl (v4.2.0 and later) [Arming Check Report](../flying/pre_flight_checks.md#qgc-arming-check-report) (see also [Fly View > Arming and Preflight Checks](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/fly_view/fly_view.md#arm)).

Note that internally PX4 runs arming checks at 10Hz.
A list of the failed checks is kept, and if the list changes PX4 emits the current list using the [Events interface](../concept/events_interface.md).
The list is also sent out when the GCS connects.
Effectively the GCS knows the status of prearm checks immediately, both when disarmed and armed.

:::details
Implementation notes for developers
The client implementation is in [libevents](https://github.com/mavlink/libevents):

- [libevents > Event groups](https://github.com/mavlink/libevents#event-groups)
- [health_and_arming_checks.h](https://github.com/mavlink/libevents/blob/main/libs/cpp/parse/health_and_arming_checks.h)

QGC implementation: [HealthAndArmingCheckReport.cc](https://github.com/mavlink/qgroundcontrol/blob/master/src/MAVLink/LibEvents/HealthAndArmingCheckReport.cc).
:::

PX4 also emits a subset of the arming check information in the [SYS_STATUS](https://mavlink.io/en/messages/common.html#SYS_STATUS) message (see [MAV_SYS_STATUS_SENSOR](https://mavlink.io/en/messages/common.html#MAV_SYS_STATUS_SENSOR)).

## Arming Sequence: Pre Arm Mode & Safety Button

The arming sequence depends on whether or not there is a _safety switch_, and is controlled by the parameters [COM_PREARM_MODE](#COM_PREARM_MODE) (Prearm mode) and [CBRK_IO_SAFETY](#CBRK_IO_SAFETY) (I/O safety circuit breaker).

The [COM_PREARM_MODE](#COM_PREARM_MODE) parameter defines when/if pre-arm mode is enabled ("safe"/non-throttling actuators are able to move):

- _Disabled_: Pre-arm mode disabled (there is no stage where only "safe"/non-throttling actuators are enabled).
- _Safety Switch_ (Default): The pre-arm mode is enabled by the safety switch.
  If there is no safety switch then pre-arm mode will not be enabled.
- _Always_: Prearm mode is enabled from power up.

If there is a safety switch then this will be a precondition for arming.
If there is no safety switch the I/O safety circuit breaker must be engaged ([CBRK_IO_SAFETY](#CBRK_IO_SAFETY)), and arming will depend only on the arm command.

The sections below detail the startup sequences for the different configurations

### 默认：COM_PREARM_MODE=Safety and Safety Switch

The default configuration uses safety switch to prearm.
From prearm you can then arm to engage all motors/actuators.
It corresponds to: [COM_PREARM_MODE=1](#COM_PREARM_MODE) (safety switch) and [CBRK_IO_SAFETY=0](#CBRK_IO_SAFETY) (I/O safety circuit breaker disabled).

The default startup sequence is:

1. Power-up.
  - All actuators locked into disarmed position
  - Not possible to arm.
2. Safety switch is pressed.
  - System now prearmed: non-throttling actuators can move (e.g. ailerons).
  - System safety is off: Arming possible.
3. Arm command is issued.

  - The system is armed.
  - All motors and actuators can move.

### COM_PREARM_MODE=Disabled and Safety Switch

When prearm mode is _Disabled_, engaging the safety switch does not unlock the "safe" actuators, though it does allow you to then arm the vehicle.
This corresponds to [COM_PREARM_MODE=0](#COM_PREARM_MODE) (Disabled) and [CBRK_IO_SAFETY=0](#CBRK_IO_SAFETY) (I/O safety circuit breaker disabled).

The startup sequence is:

1. Power-up.
  - All actuators locked into disarmed position
  - Not possible to arm.
2. Safety switch is pressed.
  - _All actuators stay locked into disarmed position (same as disarmed)._
  - System safety is off: Arming possible.
3. Arm command is issued.

  - The system is armed.
  - All motors and actuators can move.

### COM_PREARM_MODE=Always and Safety Switch

When prearm mode is _Always_, prearm mode is enabled from power up.
To arm, you still need the safety switch.
This corresponds to [COM_PREARM_MODE=2](#COM_PREARM_MODE) (Always) and [CBRK_IO_SAFETY=0](#CBRK_IO_SAFETY) (I/O safety circuit breaker disabled).

The startup sequence is:

1. Power-up.
  - System now prearmed: non-throttling actuators can move (e.g. ailerons).
  - Not possible to arm.
2. Safety switch is pressed.
  - System safety is off: Arming possible.
3. Arm command is issued.
  - The system is armed.
  - All motors and actuators can move.

### COM_PREARM_MODE=Safety or Disabled and No Safety Switch

With no safety switch, when `COM_PREARM_MODE` is set to _Safety_ or _Disabled_ prearm mode cannot be enabled (same as disarmed).
This corresponds to [COM_PREARM_MODE=0 or 1](#COM_PREARM_MODE) (Disabled/Safety Switch) and [CBRK_IO_SAFETY=22027](#CBRK_IO_SAFETY) (I/O safety circuit breaker engaged).

The startup sequence is:

1. Power-up.
  - All actuators locked into disarmed position
  - System safety is off: Arming possible.
2. Arm command is issued.
  - The system is armed.
  - All motors and actuators can move.

### COM_PREARM_MODE=Always and No Safety Switch

When prearm mode is _Always_, prearm mode is enabled from power up.
This corresponds to [COM_PREARM_MODE=2](#COM_PREARM_MODE) (Always) and [CBRK_IO_SAFETY=22027](#CBRK_IO_SAFETY) (I/O safety circuit breaker engaged).

The startup sequence is:

1. Power-up.
  - System now prearmed: non-throttling actuators can move (e.g. ailerons).
  - System safety is off: Arming possible.
2. Arm command is issued.
  - The system is armed.
  - All motors and actuators can move.

### 参数

| 参数                                                                                                                                                 | 描述                                                                                                                                                                                                                                                                                                                                                                                                          |
| -------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="COM_PREARM_MODE"></a>[COM_PREARM_MODE](../advanced_config/parameter_reference.md#COM_PREARM_MODE) | Condition to enter prearmed mode. `0`: Disabled, `1`: Safety switch (prearm mode enabled by safety switch; if no switch present cannot be enabled), `2`: Always (prearm mode enabled from power up). Default: `1` (safety button). |
| <a id="CBRK_IO_SAFETY"></a>[CBRK_IO_SAFETY](../advanced_config/parameter_reference.md#CBRK_IO_SAFETY)    | Circuit breaker for IO safety.                                                                                                                                                                                                                                                                                                                                                              |

<!-- Discussion:
https://github.com/PX4/PX4-Autopilot/pull/12806#discussion_r318337567
https://github.com/PX4/PX4-user_guide/issues/567#issue-486653048
-->
