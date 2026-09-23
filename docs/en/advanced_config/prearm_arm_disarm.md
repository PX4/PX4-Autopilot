# Arm, Disarm, Prearm Configuration

Vehicles may have moving parts, some of which are potentially dangerous when powered (in particular motors and propellers)!

To reduce the chance of accidents, PX4 has explicit state(s) for powering the vehicle components:

- **Disarmed:** There is no power to motors or actuators.
- **Pre-armed:** Motors/propellers are locked but actuators for non-dangerous electronics are powered (e.g. ailerons, flaps etc.).
- **Armed:** Vehicle is fully powered. Motors/propellers may be turning (dangerous!)

::: info
Ground stations may display _disarmed_ for pre-armed vehicles.
While not technically correct for pre-armed vehicles, it is "safe".
:::

Users can control progression though these states using a [safety switch](../getting_started/px4_basic_concepts.md#safety-switch) on the vehicle (optional) _and_ an [arming switch/button](#arm_disarm_switch), [arming gesture](#arm_disarm_gestures), or _MAVLink command_ on the ground controller:

- A _safety switch_ is a control _on the vehicle_ that must be engaged before the vehicle can be armed, and which may also prevent pre-arming (depending on the configuration).
  Commonly the safety switch is integrated into a GPS unit, but it may also be a separate physical component.

  :::warning
  A vehicle that is armed is potentially dangerous.
  The safety switch is an additional mechanism that prevents arming from happening by accident.
  :::

- An _arming switch_ is a switch or button _on an RC controller_ that can be used to arm the vehicle and start motors (provided arming is not prevented by a safety switch).
- An _arming gesture_ is a stick movement _on an RC controller_ that can be used as an alternative to an arming switch.
- MAVLink commands can also be sent by a ground control station to arm/disarm, or pre-arm a vehicle.

PX4 will also automatically disarm the vehicle if it does not takeoff within a certain amount of time after arming, and if it is not manually disarmed after landing.
This reduces the amount of time where an armed (and therefore dangerous) vehicle is on the ground.

PX4 allows you to configure how pre-arming, arming and disarming work using parameters (which can be edited in _QGroundControl_ via the [parameter editor](../advanced_config/parameters.md)), as described in the following sections.

:::tip
Arming/disarming parameters can be found in [Parameter Reference > Commander](../advanced_config/parameter_reference.md#commander) (search for `COM_ARM_*` and `COM_DISARM_*`).
:::

## Arming/Disarming Gestures {#arm_disarm_gestures}

By default, the vehicle is armed and disarmed by moving RC throttle/yaw sticks to particular extremes and holding them for 1 second.

- **Arming:** Throttle minimum, yaw maximum
- **Disarming:** Throttle minimum, yaw minimum

RC controllers will use different sticks for throttle and yaw [based on their mode](../getting_started/rc_transmitter_receiver.md#types-of-remote-controllers), and hence different gestures:

- **Mode 2**:
  - _Arm:_ Left stick to bottom right.
  - _Disarm:_ Left stick to the bottom left.
- **Mode 1**:
  - _Arm:_ Left-stick to right, right-stick to bottom.
  - _Disarm:_ Left-stick to left, right-stick to the bottom.

Note that disarming in any altitude controlled mode is only possible after landing was detected.
In manually piloted modes without altitude control, such as Stabilized, Acro, and Manual, it's always possible to disarm using gestures or buttons — even in flight.

| Parameter                                                                                                | Description                                                              |
| -------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------ |
| <a id="MAN_ARM_GESTURE"></a>[MAN_ARM_GESTURE](../advanced_config/parameter_reference.md#MAN_ARM_GESTURE) | Enable arm/disarm stick guesture. `0`: Disabled, `1`: Enabled (default). |

## Arming Button/Switch {#arm_disarm_switch}

An _arming button_ or "momentary switch" can be configured to trigger arm/disarm _instead_ of [gesture-based arming](#arm_disarm_gestures) (setting an arming switch disables arming gestures).
The button should be held down for one second to arm (when disarmed) or disarm (when armed).

A two-position switch can also be used for arming/disarming, where the respective arm/disarm commands are sent on switch _transitions_.

:::tip
Two-position arming switches are primarily used in/recommended for racing drones.
:::

The switch or button is assigned (and enabled) using [RC_MAP_ARM_SW](#RC_MAP_ARM_SW), and the switch "type" is configured using [COM_ARM_SWISBTN](#COM_ARM_SWISBTN).

| Parameter                                                                                                | Description                                                                                                                                                                                                                                                                                                                                        |
| -------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="RC_MAP_ARM_SW"></a>[RC_MAP_ARM_SW](../advanced_config/parameter_reference.md#RC_MAP_ARM_SW)       | RC arm switch channel (default: 0 - unassigned). If defined, the specified RC channel (button/switch) is used for arming instead of a stick gesture. <br>**Note:**<br>- This setting _disables the stick gesture_!<br>- This setting applies to RC controllers. It does not apply to Joystick controllers that are connected via _QGroundControl_. |
| <a id="COM_ARM_SWISBTN"></a>[COM_ARM_SWISBTN](../advanced_config/parameter_reference.md#COM_ARM_SWISBTN) | Arm switch is a momentary button. <br>- `0`: Arm switch is a 2-position switch where arm/disarm commands are sent on switch transitions.<br>-`1`: Arm switch is a momentary button where the arm/disarm command is sent after holding down the button for one second.                                                                              |

::: info
The switch can also be set as part of _QGroundControl_ [Flight Mode](../config/flight_mode.md) configuration.
:::

## Auto-Disarming

By default vehicles will automatically disarm on landing, or if you take too long to take off after arming.
The feature is configured using the following timeouts.

| Parameter                                                                                                   | Description                                                                    |
| ----------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------ |
| <a id="COM_DISARM_LAND"></a>[COM_DISARM_LAND](../advanced_config/parameter_reference.md#COM_DISARM_LAND)    | Time-out for auto disarm after landing. Default: 2s (-1 to disable).           |
| <a id="COM_DISARM_PRFLT"></a>[COM_DISARM_PRFLT](../advanced_config/parameter_reference.md#COM_DISARM_PRFLT) | Time-out for auto disarm if too slow to takeoff. Default: 10s (-1 to disable). |

By default, the vehicle remains in the pre-armed state, unless [COM_FORCE_SAFETY](#COM_FORCE_SAFETY) is set to 1, in which case it goes back to "true" Disarmed state so that the vehicle requires to be pre-armed again before the next arming command.

| Parameter                                                                                                   | Description                                                                    |
| ----------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------ |
| <a id="COM_FORCE_SAFETY"></a>[COM_FORCE_SAFETY](../advanced_config/parameter_reference.md#COM_FORCE_SAFETY) | Force safety when the vehicle disarms. Default: `0` (Disabled).                |

## Auto-Arming on Boot

The vehicle can be configured to arm automatically on boot once all preflight checks pass,
using the `COM_ARM_ON_BOOT` parameter. For safety, PX4 enforces a minimum 5-second delay after boot before attempting to arm.

Once armed this way, the vehicle will not re-arm automatically after a manual disarm.

::: info
The parameter value is read once at boot.
Changing it while the system is running has no effect until the next reboot.
:::

:::warning
Use with caution.
A vehicle that arms automatically can spin up motors and actuators without any operator gesture.
Ensure the vehicle is in a safe state before powering on.
:::

| Parameter                                                                                                | Description                                                                       |
| -------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------- |
| <a id="COM_ARM_ON_BOOT"></a>[COM_ARM_ON_BOOT](../advanced_config/parameter_reference.md#COM_ARM_ON_BOOT) | Arm automatically once preflight checks pass after boot. Default: `0` (Disabled). |

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

The current failed checks can be viewed in QGroundControl (v4.2.0 and later) [Arming Check Report](../flying/pre_flight_checks.md#qgc-arming-check-report) (see also [Fly View > Toolbar > Flight Status](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/fly_view/fly_view_toolbar.html#flight-status)).

Note that internally PX4 runs arming checks at 10Hz.
A list of the failed checks is kept, and if the list changes PX4 emits the current list using the [Events interface](../concept/events_interface.md).
The list is also sent out when the GCS connects.
Effectively the GCS knows the status of pre-arm checks immediately, both when disarmed and armed.

:::details Implementation notes for developers
The client implementation is in [libevents](https://github.com/mavlink/libevents):

- [libevents > Event groups](https://github.com/mavlink/libevents#event-groups)
- [health_and_arming_checks.h](https://github.com/mavlink/libevents/blob/main/libs/cpp/parse/health_and_arming_checks.h)

QGC implementation: [HealthAndArmingCheckReport.cc](https://github.com/mavlink/qgroundcontrol/blob/master/src/MAVLink/LibEvents/HealthAndArmingCheckReport.cc).
:::

PX4 also emits a subset of the arming check information in the [SYS_STATUS](https://mavlink.io/en/messages/common.html#SYS_STATUS) message (see [MAV_SYS_STATUS_SENSOR](https://mavlink.io/en/messages/common.html#MAV_SYS_STATUS_SENSOR)).

## Arming Sequence: Pre-Arm Mode & Safety Button

The arming sequence depends on whether or not there is a _safety switch_, and is controlled by the parameters [COM_PREARM_MODE](#COM_PREARM_MODE) (Pre-arm mode) and [CBRK_IO_SAFETY](#CBRK_IO_SAFETY) (I/O safety circuit breaker).

The [COM_PREARM_MODE](#COM_PREARM_MODE) parameter defines when/if pre-arm mode is enabled ("safe"/non-throttling actuators are able to move):

- _Disabled_: Pre-arm mode disabled (there is no stage where only "safe"/non-throttling actuators are enabled).
- _Safety Switch or Mavlink_ (Default): The pre-arm mode is enabled by either the safety switch, or via the mavlink command [MAV_CMD_DO_SET_SAFETY_SWITCH_STATE](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_SAFETY_SWITCH_STATE).
- _Always_: Pre-arm mode is enabled from power up.
- _Safety Switch only_: The pre-arm mode is enabled by the safety switch. If there is no safety switch then pre-arm mode will not be enabled.
- _Mavlink only_: The pre-arm mode is enabled by [MAV_CMD_DO_SET_SAFETY_SWITCH_STATE](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_SAFETY_SWITCH_STATE). The safety switch is ignored.

The sections below detail the startup sequences for the different configurations

### Default: COM_PREARM_MODE=Safety Switch or Mavlink

The default configuration allows to use either the safety switch or a mavlink command to pre-arm.
From pre-arm you can then arm to engage all motors/actuators.
It corresponds to: [COM_PREARM_MODE=1](#COM_PREARM_MODE) (safety switch or mavlink) and [CBRK_IO_SAFETY=0](#CBRK_IO_SAFETY) (I/O safety circuit breaker disabled).

The default startup sequence is:

1. Power-up.
   - All actuators locked into disarmed position
   - Not possible to arm.
1. Safety switch is pressed or mavlink command is received.
   - System now pre-armed: non-throttling actuators can move (e.g. ailerons).
   - System safety is off: Arming possible.
1. Arm command is issued.
   - The system is armed.
   - All motors and actuators can move.

### COM_PREARM_MODE=Disabled

When pre-arm mode is _Disabled_, engaging the safety switch or sending a mavlink command does not unlock the "safe" actuators, though it does allow you to then arm the vehicle.
This corresponds to [COM_PREARM_MODE=0](#COM_PREARM_MODE) (Disabled) and [CBRK_IO_SAFETY=0](#CBRK_IO_SAFETY) (I/O safety circuit breaker disabled).

The startup sequence is:

1. Power-up.
   - All actuators locked into disarmed position
   - Not possible to arm.
1. Safety switch is pressed or mavlink command is received.
   - _All actuators stay locked into disarmed position (same as disarmed)._
   - System safety is off: Arming possible.
1. Arm command is issued.
   - The system is armed.
   - All motors and actuators can move.

### COM_PREARM_MODE=Always

When pre-arm mode is _Always_, pre-arm mode is enabled from power up.
To arm, you still need the safety switch or mavlink command.
This corresponds to [COM_PREARM_MODE=2](#COM_PREARM_MODE) (Always) and [CBRK_IO_SAFETY=0](#CBRK_IO_SAFETY) (I/O safety circuit breaker disabled).

The startup sequence is:

1. Power-up.
   - System now pre-armed: non-throttling actuators can move (e.g. ailerons).
   - Not possible to arm.
1. Safety switch is pressed or mavlink command is received.
   - System safety is off: Arming possible.
1. Arm command is issued.
   - The system is armed.
   - All motors and actuators can move.

### COM_PREARM_MODE=Safety Switch only

When pre-arm mode is _Safety Switch only_, it strictly requires you to press the safety switch to enter pre-armed state. The mavlink command is ignored.
This corresponds to [COM_PREARM_MODE=3](#COM_PREARM_MODE) (Safety Switch only) and [CBRK_IO_SAFETY=0](#CBRK_IO_SAFETY) (I/O safety circuit breaker disabled).

The startup sequence is:

1. Power-up.
   - All actuators locked into disarmed position
   - Not possible to arm.
1. Safety switch is pressed.
   - System now pre-armed: non-throttling actuators can move (e.g. ailerons).
   - System safety is off: Arming possible.
1. Arm command is issued.
   - The system is armed.
   - All motors and actuators can move.

### COM_PREARM_MODE=Mavlink only

When pre-arm mode is _Mavlink only_, it requires you to a mavlink command to enter pre-armed state. The safety switch is ignored.
This corresponds to [COM_PREARM_MODE=4](#COM_PREARM_MODE) (Mavlink only) and [CBRK_IO_SAFETY=0](#CBRK_IO_SAFETY) (I/O safety circuit breaker disabled).

The startup sequence is:

1. Power-up.
   - All actuators locked into disarmed position
   - Not possible to arm.
1. Mavlink command is received.
   - System now pre-armed: non-throttling actuators can move (e.g. ailerons).
   - System safety is off: Arming possible.
1. Arm command is issued.
   - The system is armed.
   - All motors and actuators can move.

### CBRK_IO_SAFETY=22027

Activating this circuitbreaker turns system safety off directly from power-up, meaning that arming is always possible. Use this circuitbreaker with caution.
The pre-arm logic is unchanged, meaning that non-throttling actuators are still governed by `COM_PREARM_MODE` as outlined above.

The startup sequence is:

1. Power-up.
   - System safety is off: Arming possible.
   - Pre-arm state is still governed by `COM_PREARM_MODE` as outlined above.
1. Arm command is issued.
   - The system is armed.
   - All motors and actuators can move.

### Parameters

| Parameter                                                                                                | Description                                                                                                                                                                                                                        |
| -------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="COM_PREARM_MODE"></a>[COM_PREARM_MODE](../advanced_config/parameter_reference.md#COM_PREARM_MODE) | Condition to enter pre-armed mode. `0`: Disabled, `1`: Safety switch (pre-arm mode enabled by safety switch; if no switch present cannot be enabled), `2`: Always (pre-arm mode enabled from power up). Default: `1` (safety button). |
| <a id="CBRK_IO_SAFETY"></a>[CBRK_IO_SAFETY](../advanced_config/parameter_reference.md#CBRK_IO_SAFETY)    | Circuit breaker for IO safety. |
| <a id="COM_FORCE_SAFETY"></a>[COM_FORCE_SAFETY](../advanced_config/parameter_reference.md#COM_FORCE_SAFETY)    | Force safety when the vehicle disarms. |

<!-- Discussion:
https://github.com/PX4/PX4-Autopilot/pull/12806#discussion_r318337567
https://github.com/PX4/PX4-user_guide/issues/567#issue-486653048
-->
