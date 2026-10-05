# System Failure Injection

System failure injection allows you to induce different types of sensor and system failures, either via MAVLink (using [MAV_CMD_INJECT_FAILURE](https://mavlink.io/en/messages/common.html#MAV_CMD_INJECT_FAILURE) or via the [MAVSDK failure plugin](https://mavsdk.mavlink.io/main/en/cpp/api_reference/classmavsdk_1_1_failure.html)), or "manually" via a PX4 console like the [MAVLink shell](../debug/mavlink_shell.md#mavlink-shell).
This enables easier testing of [safety failsafe](../config/safety.md) behaviour, and more generally, of how PX4 behaves when systems and sensors stop working correctly.

Failure injection is disabled by default, and can be enabled using the [SYS_FAILURE_EN](../advanced_config/parameter_reference.md#SYS_FAILURE_EN) parameter.

Failures can be injected both in simulation and on real hardware; this requires firmware built with the failure-injection module.
The command always goes through the same firmware failure-injection module — whether it arrives over MAVLink or from the console, the accepted combinations are identical.
What differs is whether a _consumer_ applies the failure, and that depends on the environment.

## Supported Failure Types

The table lists the failure types that actually take effect per environment: `off`, `stuck`, `wrong`, `slow` (`ok` is not listed, but clears an active injection on all environments).
A `—` means the module still accepts the command, but no consumer applies it in that environment.

| Component         | [Gazebo] (gz)                   | [SIH]                           | `simulator_mavlink` (Gazebo Classic) | Hardware                        |
| ----------------- | ------------------------------- | ------------------------------- | ------------------------------------ | ------------------------------- |
| `gyro`            | `off`, `stuck`                  | `off`, `stuck`                  | `off`, `stuck`                       | `off`, `stuck`                  |
| `accel`           | `off`, `stuck`                  | `off`, `stuck`                  | `off`, `stuck`                       | `off`, `stuck`                  |
| `mag`             | `off`, `stuck`                  | `off`, `stuck`                  | `off`, `stuck`                       | `off`, `stuck`                  |
| `baro`            | `off`, `stuck`                  | `off`, `stuck`                  | `off`, `stuck`                       | `off`, `stuck`                  |
| `distance_sensor` | `off`, `stuck`                  | `off`, `stuck`                  | `off`, `stuck`                       | `off`, `stuck`                  |
| `gps`             | `off`, `stuck`, `wrong`, `slow` | `off`, `stuck`, `wrong`, `slow` | `off`, `stuck`, `wrong`, `slow`      | `off`, `stuck`, `wrong`, `slow` |
| `airspeed`        | `off`, `stuck`, `wrong`         | —                               | `off`, `wrong`                       | —                               |
| `vio`             | —                               | —                               | `off`                                | —                               |
| `battery`         | `off`, `wrong`                  | `off`, `wrong`                  | `off`, `wrong`                       | `off`, `wrong`                  |
| `traffic`         | `off`                           | `off`                           | `off`                                | `off`                           |
| `motor`           | `off`, `wrong`                  | `off`, `wrong`                  | `off`, `wrong`                       | `off`, `wrong`                  |
| `esc`             | `off`, `wrong`                  | `off`, `wrong`                  | `off`, `wrong`                       | `off`, `wrong`                  |
| `can`             | —                               | —                               | —                                    | `off`                           |

[SIH]: ../sim_sih/index.md
[Gazebo]: ../sim_gazebo_gz/index.md

::: info

- `gps` failures on Gazebo (Gz): applied to the GNSS data from the simulator's own sensor as well as to the simulated-GNSS module ([SIM_GZ_EN_GPS](../advanced_config/parameter_reference.md#SIM_GZ_EN_GPS) `0`).
- `gps` failures apply to the addressed receiver's heading as well (`sensor_gnss_relative`), so `gps off` on a dual-antenna receiver also stops its heading.
  A moving-base rover's heading is dropped while its moving base is silent, whether that receiver failed or was injected `off`.
- `airspeed off | stuck | wrong` on Gazebo (Gz): only injectable when airspeed is provided by the simulated-airspeed module ([SENS_EN_ARSPDSIM](../advanced_config/parameter_reference.md#SENS_EN_ARSPDSIM)); worlds that model an airspeed sensor directly are not injected.
- `battery wrong` reports the remaining charge just below the [SYS_FAIL_BAT_LVL](../advanced_config/parameter_reference.md#SYS_FAIL_BAT_LVL) warning threshold to trigger the battery failsafe; `off` stops publishing the battery status entirely.
- `traffic off` suppresses incoming reports and marks the ADS-B/FLARM link unhealthy.
- `motor off` and `motor wrong` are the two motor failures, and both require [CA_FAILURE_MODE](../advanced_config/parameter_reference.md#CA_FAILURE_MODE). `motor off` is the _detected_ failure: the motor is reported as failed, so the control allocator removes it from the allocation and handles the opposite motor per `CA_FAILURE_MODE`. Only a single failed motor can be recovered from, so an instance mask naming several motors has no effect. `motor wrong` is the _undetected_ failure: the output is stopped without informing the allocator, which models a failure with no feedback path (such as PWM ESCs), and any number of motors can be addressed.
- `esc off` reports the addressed ESC as offline and blanks its telemetry; `esc wrong` keeps it online but reports implausible telemetry (voltage and current at 10% of the real value, RPM 10x). ESCs are addressed by motor instance (the ESC's actuator function), so `-i 1` targets the ESC driving motor 1.
- On hardware, ESC injection is applied only by the UAVCAN (DroneCAN) ESC driver; the other ESC drivers (DShot, Cyphal, VOXL, TAP ESC) publish their telemetry unmodified.
- `can off` takes the addressed CAN bus offline entirely, so every node on it stops responding; the instance selects the bus. Only applied on fmu-v6x-class hardware.

:::

Sensors delivered through the shared driver layer (IMU, magnetometer, barometer, rangefinder via the `PX4*` sensor wrappers) support `off`/`stuck` in every environment that uses that layer — including the Gazebo and SIH sensor simulators, which feed synthesized measurements through the same wrappers.
The remaining gaps are backend-specific: GNSS and airspeed are handled by dedicated simulator code (see the GNSS receiver and airspeed notes in the info box above), SIH does not simulate an injectable airspeed.
Components not listed (`optical_flow`, `servo`, `avoidance`, `rc_signal`, `mavlink_signal`) are rejected everywhere (`MAV_RESULT_UNSUPPORTED`); see the note below on NACK behaviour.

::: info
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
  - Sensors:
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
    - `esc`: ESC telemetry
    - `servo`: Servo
    - `avoidance`: Obstacle/collision avoidance system
    - `traffic`: Traffic avoidance (ADS-B/transponder)
    - `rc_signal`: RC Signal
    - `mavlink_signal`: MAVLink data telemetry connection
    - `can`: CAN bus. The instance selects the bus. Supported on fmu-v6x-class hardware.
- _failure_type_:
  - `ok`: Publish as normal (Disable failure injection)
  - `off`: Stop publishing
  - `stuck`: Constantly report the same value which _can_ happen on a malfunctioning sensor
  - `garbage`: Publish random noise. This looks like reading uninitialized memory
  - `wrong`: Publish invalid values that still look reasonable/aren't "garbage". For an actuator such as `motor` there is nothing to publish, so `wrong` means the motor stops without anything reporting it
  - `slow`: Publish at a reduced rate
  - `delayed`: Publish valid data with a significant delay
  - `intermittent`: Publish intermittently
- _instance number_ (optional): Instance number of affected sensor.
  0 (default) indicates all sensors of specified type.
- _instance bitmask_ (optional): address several instances at once (bit 0 = first instance, bit 1 = second, …; decimal or `0x` hex). Used only when `-i` is omitted.
  Example: `-m 0x5` targets instances 1 and 3.

::: info
GNSS implements the `off`, `stuck`, `wrong` and `slow` failure modes; the other failure types have no effect on it.
`gps wrong` leaves the reported position untouched and reports the fields set by these parameters; `0` (`Unchanged`) keeps the receiver's own value:

| Parameter                   | Reported field                   |
| --------------------------- | -------------------------------- |
| [SYS_FAIL_GPS_WRG]          | Fix type (default 2D)            |
| [SYS_FAIL_GPS_EPH]          | Horizontal position accuracy [m] |
| [SYS_FAIL_GPS_EPV]          | Vertical position accuracy [m]   |
| [SYS_FAIL_GPS_SAC]          | Speed accuracy [m/s]             |
| [SYS_FAIL_GPS_SAT]          | Satellites used                  |
| [SYS_FAIL_GPS_JAM]          | Jamming state                    |
| [SYS_FAIL_GPS_SPF]          | Spoofing state                   |

`gps slow` publishes one sample in [SYS_FAIL_GPS_DIV].

[SYS_FAIL_GPS_WRG]: ../advanced_config/parameter_reference.md#SYS_FAIL_GPS_WRG
[SYS_FAIL_GPS_EPH]: ../advanced_config/parameter_reference.md#SYS_FAIL_GPS_EPH
[SYS_FAIL_GPS_EPV]: ../advanced_config/parameter_reference.md#SYS_FAIL_GPS_EPV
[SYS_FAIL_GPS_SAC]: ../advanced_config/parameter_reference.md#SYS_FAIL_GPS_SAC
[SYS_FAIL_GPS_SAT]: ../advanced_config/parameter_reference.md#SYS_FAIL_GPS_SAT
[SYS_FAIL_GPS_JAM]: ../advanced_config/parameter_reference.md#SYS_FAIL_GPS_JAM
[SYS_FAIL_GPS_SPF]: ../advanced_config/parameter_reference.md#SYS_FAIL_GPS_SPF
[SYS_FAIL_GPS_DIV]: ../advanced_config/parameter_reference.md#SYS_FAIL_GPS_DIV
:::

## RC Switch Trigger

A failure can also be injected from an RC switch, without a console or telemetry link. This is useful for in-flight hardware testing. It is configured with the following parameters:

- [SYS_FAIL_RC_SRC](../advanced_config/parameter_reference.md#SYS_FAIL_RC_SRC): the auxiliary RC input that triggers the failure — `0` disables it, `1`–`6` select AUX1–AUX6 (mapped via `RC_MAP_AUXn`).
- [SYS_FAIL_RC_UNIT](../advanced_config/parameter_reference.md#SYS_FAIL_RC_UNIT): the affected component (the `FAILURE_UNIT` value; e.g. `101` = motor).
- [SYS_FAIL_RC_MODE](../advanced_config/parameter_reference.md#SYS_FAIL_RC_MODE): the failure type (the `FAILURE_TYPE` value; e.g. `1` = off).
- [SYS_FAIL_RC_INST](../advanced_config/parameter_reference.md#SYS_FAIL_RC_INST): the affected instances, as a bitmask (bit 0 = instance 1, e.g. `5` = instances 1 and 3; `0` = all instances).

While the selected aux switch is on the configured failure is injected; switching it back off clears the failure. The injection goes through the same path as the console/MAVLink commands, so for a motor it fails the motor exactly as `failure motor off` (`SYS_FAIL_RC_MODE` = `1`) or `failure motor wrong` (`SYS_FAIL_RC_MODE` = `4`) does, with the same [CA_FAILURE_MODE](../advanced_config/parameter_reference.md#CA_FAILURE_MODE) requirement. A `SYS_FAIL_RC_INST` mask addressing several motors only takes effect for `wrong`, the undetected failure.

## MAVSDK Failure Plugin

The [MAVSDK failure plugin](https://mavsdk.mavlink.io/main/en/cpp/api_reference/classmavsdk_1_1_failure.html) can be used to programmatically inject failures.
It is used in [PX4 Integration Testing](../test_and_ci/integration_testing_mavsdk.md) to simulate failure cases (for example, see [PX4-Autopilot/test/mavsdk_tests/autopilot_tester.cpp](https://github.com/PX4/PX4-Autopilot/blob/main/test/mavsdk_tests/autopilot_tester.cpp)).

The plugin API is a direct mapping of the failure command shown above, with a few additional error signals related to the connection.

## Example: GNSS {#example-gps}

To test the GNSS failsafe by stopping GNSS:

1. Enable the [SYS_FAILURE_EN](../advanced_config/parameter_reference.md#SYS_FAILURE_EN) parameter.
2. Enter the following commands on the MAVLink console or SITL _pxh shell_:

   ```sh
   # Stop GPS publishing
   failure gps off

   # Restart GPS publishing
   failure gps ok
   ```

## Example: GNSS Receiver Failover

With two receivers, failing the selected one tests the switch to the other receiver and the estimator reset that comes with it.
In SIH, set [SIM_GNSS_NUM](../advanced_config/parameter_reference.md#SIM_GNSS_NUM) to `2` and give the receivers different [SIM_GNSSx_BIAS_N](../advanced_config/parameter_reference.md#SIM_GNSS1_BIAS_N), `_E` and `_D`, so that the switch shows as a reset by the difference of the biases.

```sh
# Accuracy above the in-flight limit of the GNSS checks, fix type unchanged
param set SYS_FAIL_GPS_WRG 0
param set SYS_FAIL_GPS_EPH 60
failure gps wrong -i 1

# Update rate below a third: one sample in SYS_FAIL_GPS_DIV
failure gps slow -i 1

failure gps ok -i 1
```

The `[gnss_failover]` cases in [test/mavsdk_tests](https://github.com/PX4/PX4-Autopilot/tree/main/test/mavsdk_tests) run these failures in SIH.

### Companion Computer Tool

`gnss_failover` injects one GNSS failure from a companion computer while a pilot flies, using the same scenario code as the SIH tests.
It never arms or changes the mode, refuses to inject unless [SYS_FAILURE_EN](../advanced_config/parameter_reference.md#SYS_FAILURE_EN) is set, both receivers report a 3D fix on `GPS_RAW_INT` and `GPS2_RAW`, and the vehicle is armed, in the air and in a position-controlled mode (`--ground` allows a bench test).
It sends `ok` when it ends, including on `SIGINT` and `SIGTERM`; an injection stays active in the vehicle until then.
The firmware needs the failure-injection module, which only `px4_sitl` builds by default.

```sh
cmake -S test/mavsdk_tests -B build/gnss_failover && cmake --build build/gnss_failover --target gnss_failover
build/gnss_failover/gnss_failover --url serial:///dev/ttyS1:921600 --instance 1 --type off --duration 20 --record flight.csv
```

The record holds every injection and acknowledgement, both receivers, estimator resets, position validity, mode changes and events, with wall and vehicle time.

### Log Report

[Tools/gnss_failover_report.py](https://github.com/PX4/PX4-Autopilot/blob/main/Tools/gnss_failover_report.py) grades every injected GNSS failure in a ULog (SIH or flight): the switch to the standby, one horizontal reset by the offset between the receivers (and one height reset with GNSS as the height reference), position validity, mode and failsafes, no return to the failed receiver while armed, GNSS yaw, and the switch event, fusion state and receiver inconsistency the sensors module and EKF2 report.
In SIH it also checks against ground truth that the vehicle held its position in Hold.

```sh
Tools/gnss_failover_report.py log.ulg
```

## Example: Motor

To fail a motor mid-flight:

1. Enable the [SYS_FAILURE_EN](../advanced_config/parameter_reference.md#SYS_FAILURE_EN) parameter.
2. Enable [CA_FAILURE_MODE](../advanced_config/parameter_reference.md#CA_FAILURE_MODE) parameter to allow turning off motors.
3. Pick the failure type for the behavior you want to test:
   - `off` — detected: the motor is flagged as failed and removed from control allocation, so the allocator compensates using the remaining motors (per `CA_FAILURE_MODE`).
     Only one failed motor can be recovered from, so an instance mask naming several motors has no effect.
   - `wrong` — undetected: the motor output is stopped without notifying the allocator, so its effectiveness is not excluded.
     This models a failure with no feedback path, such as PWM ESCs.
     Any number of motors can be addressed.
4. Enter the following commands on the MAVLink console or SITL _pxh shell_:

   ```sh
   # Detected failure: motor 1 is reported as failed and removed from the allocation
   failure motor off -i 1

   # Undetected failure: motor 1 stops, the allocator is not informed
   failure motor wrong -i 1

   # Turn it back on
   failure motor ok -i 1
   ```

## Example: Battery

To trigger the battery failsafe by reporting a depleted pack:

1. Enable the [SYS_FAILURE_EN](../advanced_config/parameter_reference.md#SYS_FAILURE_EN) parameter.
2. Optionally select the injected warning level with [SYS_FAIL_BAT_LVL](../advanced_config/parameter_reference.md#SYS_FAIL_BAT_LVL): Warn, Critical or Emergency. The reported remaining charge is set just below the matching threshold ([BAT_LOW_THR](../advanced_config/parameter_reference.md#BAT_LOW_THR), [BAT_CRIT_THR](../advanced_config/parameter_reference.md#BAT_CRIT_THR) or [BAT_EMERGEN_THR](../advanced_config/parameter_reference.md#BAT_EMERGEN_THR)).
3. Enter the following commands on the MAVLink console or SITL _pxh shell_:

   ```sh
   # Report the battery as depleted at the SYS_FAIL_BAT_LVL warning level -> battery failsafe
   failure battery wrong

   # Stop publishing the battery status entirely
   failure battery off

   # Stop injecting the failure
   failure battery ok
   ```
