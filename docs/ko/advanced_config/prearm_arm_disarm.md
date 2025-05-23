# Arm, Disarm, Prearm Configuration

기체에는 움직이는 부품이 있으며, 그 중에는 특히 모터와 프로펠러는 전원 공급시 위험할 수 있습니다.

사고의 위험을 줄이기 위하여, PX4는 명확한 상태에서만 부품들의 전원을 공급합니다.

- **Disarmed:** There is no power to motors or actuators.
- **Pre-armed:** Motors/propellers are locked but actuators for non-dangerous electronics are powered (e.g. ailerons, flaps etc.).
- **Armed:** Vehicle is fully powered. 모터와 프로펠러가 동작할 수 있음(위험!)

:::info
Ground stations may display _disarmed_ for pre-armed vehicles.
시동전의 기체는 기술적으로 타당하지는 않지만, "안전"한 상태입니다.
:::

Users can control progression though these states using a [safety switch](../getting_started/px4_basic_concepts.md#safety-switch) on the vehicle (optional) _and_ an [arming switch/button](#arm_disarm_switch), [arming gesture](#arm_disarm_gestures), or _MAVLink command_ on the ground controller:

- A _safety switch_ is a control _on the vehicle_ that must be engaged before the vehicle can be armed, and which may also prevent prearming (depending on the configuration).
  보통 안전 스위치는 GPS 장치에 붙어있으나, 별도의 부품으로 공급되기도 합니다.

  :::warning
  A vehicle that is armed is potentially dangerous.
  안전 스위치는 의도하지 않게 갑자기시동이 걸리는 것을 방지합니다.

:::

- An _arming switch_ is a switch or button _on an RC controller_ that can be used to arm the vehicle and start motors (provided arming is not prevented by a safety switch).

- An _arming gesture_ is a stick movement _on an RC controller_ that can be used as an alternative to an arming switch.

- MAVLink 명령은 지상국에서 기체의 시동을 걸거나 시동을 해제할 수 있습니다.

PX4는 시동 후 일정 시간 내에 이륙하지 않고, 착륙 후 수동으로 시동 해제하지 않으면, 기체의 시동은 자동으로 해제됩니다.
이것은 시동이 걸린 기체가 지상에서 안전사고를 유발할 수 있는 시간을 줄입니다.

PX4 allows you to configure how pre-arming, arming and disarming work using parameters (which can be edited in _QGroundControl_ via the [parameter editor](../advanced_config/parameters.md)), as described in the following sections.

:::tip
Arming/disarming parameters can be found in [Parameter Reference > Commander](../advanced_config/parameter_reference.md#commander) (search for `COM_ARM_*` and `COM_DISARM_*`).
:::

## Arming/Disarming Gestures {#arm_disarm_gestures}

기본적으로, 기체는 무선조종장치의 추진 제어 스틱과 방위 제어 스틱을 움직인 후  잠깐 동안 또는 1초 동안 상태를 유지하면 시동을 걸거나 시동을 해제할 수 있습니다.

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

| 매개변수                                                                                                                                                                    | 설명                                                                                                                                                                                                      |
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

| 매개변수                                                                                                                                                              | 설명                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| ----------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="RC_MAP_ARM_SW"></a>[RC_MAP_ARM_SW](../advanced_config/parameter_reference.md#RC_MAP_ARM_SW) | RC arm 스위치 채널 (기본값 : 0 - 할당되지 않음). 정의된 경우 지정된 RC 채널(버튼/스위치)이 스틱 제스처 대신 시동용으로 사용됩니다. <br>**Note:**<br>- This setting _disables the stick gesture_!<br>- This setting applies to RC controllers. It does not apply to Joystick controllers that are connected via _QGroundControl_.                                                   |
| <a id="COM_ARM_SWISBTN"></a>[COM_ARM_SWISBTN](../advanced_config/parameter_reference.md#COM_ARM_SWISBTN)                | 시동 스위치는 순간적으로 동작하는 버튼입니다. <br>- `0`: Arm switch is a 2-position switch where arm/disarm commands are sent on switch transitions.<br>-`1`: Arm switch is a button or momentary button where the arm/disarm command ae sent after holding down button for set time ([COM_RC_ARM_HYST](#COM_RC_ARM_HYST)). |

:::info
The switch can also be set as part of _QGroundControl_ [Flight Mode](../config/flight_mode.md) configuration.
:::

## 자동 시동 끄기

기본적으로, 기체는 착륙시 시동 해제 되며, 시동후 이륙 시간이 너무 오래 걸리면 자동으로 시동 해제됩니다.
이 기능은 다음 시간 제한을 사용하여 설정됩니다.

| 매개변수                                                                                                                                                  | 설명                                                                                                                                                           |
| ----------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| <a id="COM_DISARM_LAND"></a>[COM_DISARM_LAND](../advanced_config/parameter_reference.md#COM_DISARM_LAND)    | 착륙후 자동 시동 해제 대기 시간. 기본값: 2s (-1 비활성화).                                                    |
| <a id="COM_DISARM_PRFLT"></a>[COM_DISARM_PRFLT](../advanced_config/parameter_reference.md#COM_DISARM_PRFLT) | 이륙 속도가 너무 느리면 자동 시동 해제 시간이 초과됩니다. Default: 10s (<=0 to disable). |

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
  안전 스위치가 없으면 시동전 모드가 활성화되지 않습니다.
- _Always_: Prearm mode is enabled from power up.

기본 설정에서는 시동전에 안전 스위치를 사용하도록 설정합니다.
If there is no safety switch the I/O safety circuit breaker must be engaged ([CBRK_IO_SAFETY](#CBRK_IO_SAFETY)), and arming will depend only on the arm command.

아래 섹션에서는 여러가지 설정의 시작 순서를 자세히 설명합니다.

### 기본값: COM_PREARM_MODE = Safety and Safety Switch

기본 설정에서는 시동전에 안전 스위치를 사용하도록 설정합니다.
시동전에 이 스위치를 켜면 모든 모터와 액츄에이터를 가동하기 위하여 시동을 걸 수 있습니다.
It corresponds to: [COM_PREARM_MODE=1](#COM_PREARM_MODE) (safety switch) and [CBRK_IO_SAFETY=0](#CBRK_IO_SAFETY) (I/O safety circuit breaker disabled).

시작 절차는 다음과 같습니다:

1. 전원 인가
  - 모든 액츄에이터를 시동 해제 상태로 잠금
  - 시동 걸기 불가능
2. 안전 스위치 누름
  - 시스템이 시동전 상태로 전환: 추진 모터를 제외한 모든 액츄에이터 동작 가능(예: 보조익)
  - 시스템 안전 장치 꺼짐: 시동 가능
3. 시동 명령 인가

  - 시스템에 시동이 걸림
  - 모든 모터와 액츄에이터를 움직일 수 있음

### COM_PREARM_MODE = Disabled and Safety Switch

When prearm mode is _Disabled_, engaging the safety switch does not unlock the "safe" actuators, though it does allow you to then arm the vehicle.
This corresponds to [COM_PREARM_MODE=0](#COM_PREARM_MODE) (Disabled) and [CBRK_IO_SAFETY=0](#CBRK_IO_SAFETY) (I/O safety circuit breaker disabled).

시작 절차는 다음과 같습니다:

1. 전원 인가
  - 모든 액츄에이터를 시동 해제 상태로 잠금
  - 시동 걸기 불가능
2. 안전 스위치 누름
  - _All actuators stay locked into disarmed position (same as disarmed)._
  - 시스템 안전 장치 꺼짐: 시동 가능
3. 시동 명령 인가

  - 시스템에 시동이 걸림
  - 모든 모터와 액츄에이터를 움직일 수 있음

### COM_PREARM_MODE = Always and Safety Switch

When prearm mode is _Always_, prearm mode is enabled from power up.
시동 걸기 위하여 여전히 안전 스위치가 필요합니다.
This corresponds to [COM_PREARM_MODE=2](#COM_PREARM_MODE) (Always) and [CBRK_IO_SAFETY=0](#CBRK_IO_SAFETY) (I/O safety circuit breaker disabled).

시작 절차는 다음과 같습니다:

1. 전원 인가
  - 시스템이 시동전 상태로 전환: 추진 모터를 제외한 모든 액츄에이터 동작 가능(예: 보조익)
  - 시동 걸기 불가능
2. 안전 스위치 누름
  - 시스템 안전 장치 꺼짐: 시동 가능
3. 시동 명령 인가
  - 시스템에 시동이 걸림
  - 모든 모터와 액츄에이터를 움직일 수 있음

### COM_PREARM_MODE = Safety or Disabled and No Safety Switch

With no safety switch, when `COM_PREARM_MODE` is set to _Safety_ or _Disabled_ prearm mode cannot be enabled (same as disarmed).
This corresponds to [COM_PREARM_MODE=0 or 1](#COM_PREARM_MODE) (Disabled/Safety Switch) and [CBRK_IO_SAFETY=22027](#CBRK_IO_SAFETY) (I/O safety circuit breaker engaged).

시작 절차는 다음과 같습니다:

1. 전원 인가
  - 모든 액츄에이터를 시동 해제 상태로 잠금
  - 시스템 안전 장치 꺼짐: 시동 가능
2. 시동 명령 인가
  - 시스템에 시동이 걸림
  - 모든 모터와 액츄에이터를 움직일 수 있음

### COM_PREARM_MODE = Always and No Safety Switch

When prearm mode is _Always_, prearm mode is enabled from power up.
This corresponds to [COM_PREARM_MODE=2](#COM_PREARM_MODE) (Always) and [CBRK_IO_SAFETY=22027](#CBRK_IO_SAFETY) (I/O safety circuit breaker engaged).

시작 절차는 다음과 같습니다:

1. 전원 인가
  - 시스템이 시동전 상태로 전환: 추진 모터를 제외한 모든 액츄에이터 동작 가능(예: 보조익)
  - 시스템 안전 장치 꺼짐: 시동 가능
2. 시동 명령 인가
  - 시스템에 시동이 걸림
  - 모든 모터와 액츄에이터를 움직일 수 있음

### 매개변수

| 매개변수                                                                                                                                               | 설명                                                                                                                                                                                                                                                                                                                                                                                            |
| -------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="COM_PREARM_MODE"></a>[COM_PREARM_MODE](../advanced_config/parameter_reference.md#COM_PREARM_MODE) | 시동전 모드로 진입하는 상태입니다. `0`: Disabled, `1`: Safety switch (prearm mode enabled by safety switch; if no switch present cannot be enabled), `2`: Always (prearm mode enabled from power up). Default: `1` (safety button). |
| <a id="CBRK_IO_SAFETY"></a>[CBRK_IO_SAFETY](../advanced_config/parameter_reference.md#CBRK_IO_SAFETY)    | 입출력 안전을 위한 회로 차단.                                                                                                                                                                                                                                                                                                                                                             |

<!-- Discussion:
https://github.com/PX4/PX4-Autopilot/pull/12806#discussion_r318337567
https://github.com/PX4/PX4-user_guide/issues/567#issue-486653048
-->
