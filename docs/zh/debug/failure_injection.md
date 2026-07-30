# System Failure Injection

System failure injection allows you to induce different types of sensor and system failures, either programmatically using the [MAVSDK failure plugin](https://mavsdk.mavlink.io/main/en/cpp/api_reference/classmavsdk_1_1_failure.html), or "manually" via a PX4 console like the [MAVLink shell](../debug/mavlink_shell.md#mavlink-shell).
This enables easier testing of [safety failsafe](../config/safety.md) behaviour, and more generally, of how PX4 behaves when systems and sensors stop working correctly.

Failure injection is disabled by default, and can be enabled using the [SYS_FAILURE_EN](../advanced_config/parameter_reference.md#SYS_FAILURE_EN) parameter.

Failures can be injected both in simulation and on real hardware. In simulation the available failures depend on the simulator. On hardware the `off` (stop publishing) and `stuck` (freeze the last value) types are supported for the `gyro`, `accel`, `mag`, `baro`, `distance_sensor` and `gps` components; this requires firmware built with the failure-injection module. In addition, the `battery` component supports `off` (report a depleted pack, triggering the battery failsafe).

:::info
PX4 may accept a command to set a particular failure mode even it that mode is not supported by your simulator.

All [MAV_CMD_INJECT_FAILURE](https://mavlink.io/en/messages/common.html#MAV_CMD_INJECT_FAILURE) commands are handled internally by the failure-injection module, which acknowledges each command and republishes the active failures for the sensor/actuator simulators to apply.
The failure-injection module will NACK the command with [MAV_RESULT_UNSUPPORTED](https://mavlink.io/en/messages/common.html#MAV_RESULT_UNSUPPORTED) for failure combinations that are not implemented by PX4 or any simulator.
However it the module will accept (respond with [MAV_MISSION_ACCEPTED](https://mavlink.io/en/messages/common.html#MAV_MISSION_ACCEPTED)) for any other failure-type, even if it is not supported by your _particular_ simulator.
:::

## Failure System Command

Failures can be injected using the [failure system command](../modules/modules_command.md#failure) from any PX4 [console/shell](../debug/consoles.md) (such as the [QGC MAVLink Console](../debug/mavlink_shell.md#qgroundcontrol-mavlink-console) or SITL _pxh shell_), specifying both the target and type of the failure.

### Syntax

The full syntax of the [failure](../modules/modules_command.md#failure) command is:

```sh
failure <component> <failure_type> [-i <instance_number>] [-m <instance_bitmask>]
```

where:

- _component_:
  - 传感器：
    - `gyro`: Gyroscope
    - `accel`: Accelerometer
    - `mag`: Magnetometer
    - `baro`: Barometer
    - `gps`: Global navigation satellite system
    - `optical_flow`: Optical flow.
    - `vio`: Visual inertial odometry
    - `distance_sensor`: Distance sensor (rangefinder).
    - `airspeed`: Airspeed sensor
  - Systems:
    - `battery`: Battery
    - `motor`: Motor
    - `servo`: Servo
    - `avoidance`: Avoidance
    - `rc_signal`: RC Signal
    - `mavlink_signal`: MAVLink data telemetry connection
- _failure_type_:
  - `ok`: Publish as normal (Disable failure injection)
  - `off`: Stop publishing
  - `stuck`: Constantly report the same value which _can_ happen on a malfunctioning sensor
  - `garbage`: Publish random noise. This looks like reading uninitialized memory
  - `wrong`: Publish invalid values that still look reasonable/aren't "garbage"
  - `slow`: Publish at a reduced rate
  - `delayed`: Publish valid data with a significant delay
  - `intermittent`: Publish intermittently
- _instance number_ (optional): Instance number of affected sensor.
  0 (default) indicates all sensors of specified type.
- _instance bitmask_ (optional): address several instances at once (bit 0 = first instance, bit 1 = second, …; decimal or `0x` hex). Used only when `-i` is omitted. Example: `-m 0x5` targets instances 1 and 3.

:::info
The simulated GPS (SITL) implements only the `off`, `stuck`, and `wrong` failure modes; the other failure types have no effect on it.
:::

## RC Switch Trigger

A failure can also be injected from an RC switch, without a console or telemetry link. This is useful for in-flight hardware testing. It is configured with the following parameters:

- [SYS_FAIL_RC_SRC](../advanced_config/parameter_reference.md#SYS_FAIL_RC_SRC): the auxiliary RC input that triggers the failure — `0` disables it, `1`–`6` select AUX1–AUX6 (mapped via `RC_MAP_AUXn`).
- [SYS_FAIL_RC_UNIT](../advanced_config/parameter_reference.md#SYS_FAIL_RC_UNIT): the affected component (the `FAILURE_UNIT` value; e.g. `101` = motor).
- [SYS_FAIL_RC_MODE](../advanced_config/parameter_reference.md#SYS_FAIL_RC_MODE): the failure type (the `FAILURE_TYPE` value; e.g. `1` = off).
- [SYS_FAIL_RC_INST](../advanced_config/parameter_reference.md#SYS_FAIL_RC_INST): the affected instance (1-based; `0` = all instances).

While the selected aux switch is on the configured failure is injected; switching it back off clears the failure. The injection goes through the same path as the console/MAVLink commands, so for a motor it stops the motor exactly as `failure motor off` does (which also requires [CA_FAILURE_MODE](../advanced_config/parameter_reference.md#CA_FAILURE_MODE)).

## MAVSDK Failure Plugin

The [MAVSDK failure plugin](https://mavsdk.mavlink.io/main/en/cpp/api_reference/classmavsdk_1_1_failure.html) can be used to programmatically inject failures.
It is used in [PX4 Integration Testing](../test_and_ci/integration_testing_mavsdk.md) to simulate failure cases (for example, see [PX4-Autopilot/test/mavsdk_tests/autopilot_tester.cpp](https://github.com/PX4/PX4-Autopilot/blob/main/test/mavsdk_tests/autopilot_tester.cpp)).

The plugin API is a direct mapping of the failure command shown above, with a few additional error signals related to the connection.

## Example: GPS

To test the GPS failsafe by stopping GPS:

1. Enable the [SYS_FAILURE_EN](../advanced_config/parameter_reference.md#SYS_FAILURE_EN) parameter.
2. Enter the following commands on the MAVLink console or SITL _pxh shell_:

   ```sh
   # Stop GPS publishing
   failure gps off

   # Restart GPS publishing
   failure gps ok
   ```

## Example: Motor

To stop a motor mid-flight without the system anticipating it or excluding it from allocation effectiveness:

1. Enable the [SYS_FAILURE_EN](../advanced_config/parameter_reference.md#SYS_FAILURE_EN) parameter.
2. Enable [CA_FAILURE_MODE](../advanced_config/parameter_reference.md#CA_FAILURE_MODE) parameter to allow turning off motors.
3. Enter the following commands on the MAVLink console or SITL _pxh shell_:

   ```sh
   # Turn off first motor
   failure motor off -i 1

   # Turn it back on
   failure motor ok -i 1
   ```

## Example: Battery

To trigger the battery failsafe by reporting a depleted pack:

1. Enable the [SYS_FAILURE_EN](../advanced_config/parameter_reference.md#SYS_FAILURE_EN) parameter.
2. Enter the following commands on the MAVLink console or SITL _pxh shell_:

   ```sh
   # Report the battery as depleted (warning EMERGENCY) -> battery failsafe
   failure battery off

   # Stop injecting the failure
   failure battery ok
   ```
