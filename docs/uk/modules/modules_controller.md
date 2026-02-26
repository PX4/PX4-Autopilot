# Modules Reference: Controller

## airship_att_control

Source: [modules/airship_att_control](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/airship_att_control)

### Опис

Це реалізує регулятор положення аеростата і швидкості. Ideally it would
take attitude setpoints (`vehicle_attitude_setpoint`) or rate setpoints (in acro mode
via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

Currently it is feeding the `manual_control_setpoint` topic directly to the actuators.

### Імплементація

Щоб зменшити затримку керування, модуль безпосередньо опитує тему гіроскопа, опубліковану драйвером IMU.

### Usage {#airship_att_control_usage}

```
airship_att_control <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## control_allocator

Source: [modules/control_allocator](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/control_allocator)

### Опис

This implements control allocation. It takes torque and thrust setpoints
as inputs and outputs actuator setpoint messages.

### Usage {#control_allocator_usage}

```
control_allocator <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## flight_mode_manager

Source: [modules/flight_mode_manager](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/flight_mode_manager)

### Опис

This implements the setpoint generation for all modes. It takes the current mode state of the vehicle as input
and outputs setpoints for controllers.

### Usage {#flight_mode_manager_usage}

```
flight_mode_manager <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## fw_att_control

Source: [modules/fw_att_control](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/fw_att_control)

### Опис

fw_att_control is the fixed wing attitude controller.

### Usage {#fw_att_control_usage}

```
fw_att_control <command> [arguments...]
 Commands:
   start
     [vtol]      VTOL mode

   stop

   status        print status info
```

## fw_lat_lon_control

Source: [modules/fw_lateral_longitudinal_control](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/fw_lateral_longitudinal_control)

### Опис

fw_lat_lon_control computes attitude and throttle setpoints from lateral and longitudinal control setpoints.

### Usage {#fw_lat_lon_control_usage}

```
fw_lat_lon_control <command> [arguments...]
 Commands:
   start
     [vtol]      VTOL mode

   stop

   status        print status info
```

## fw_mode_manager

Source: [modules/fw_mode_manager](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/fw_mode_manager)

### Опис

This implements the setpoint generation for all PX4-internal fixed-wing modes, height-rate control and higher.
It takes the current mode state of the vehicle as input and outputs setpoints consumed by the fixed-wing
lateral-longitudinal controller and and controllers below that (attitude, rate).

### Usage {#fw_mode_manager_usage}

```
fw_mode_manager <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## fw_rate_control

Source: [modules/fw_rate_control](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/fw_rate_control)

### Опис

fw_rate_control is the fixed-wing rate controller.

### Usage {#fw_rate_control_usage}

```
fw_rate_control <command> [arguments...]
 Commands:
   start
     [vtol]      VTOL mode

   stop

   status        print status info
```

## mc_att_control

Source: [modules/mc_att_control](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/mc_att_control)

### Опис

This implements the multicopter attitude controller. It takes attitude
setpoints (`vehicle_attitude_setpoint`) as inputs and outputs a rate setpoint.

The controller has a P loop for angular error

Publication documenting the implemented Quaternion Attitude Control:
Nonlinear Quadrocopter Attitude Control (2013)
by Dario Brescianini, Markus Hehn and Raffaello D'Andrea
Institute for Dynamic Systems and Control (IDSC), ETH Zurich

https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf

### Usage {#mc_att_control_usage}

```
mc_att_control <command> [arguments...]
 Commands:
   start
     [vtol]      VTOL mode

   stop

   status        print status info
```

## mc_nn_control

Source: [modules/mc_nn_control](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/mc_nn_control)

### Опис

Multicopter Neural Network Control module.
This module is an end-to-end neural network control system for multicopters.
It takes in 15 input values and outputs 4 control actions.
Inputs: [pos_err(3), att(6), vel(3), ang_vel(3)]
Outputs: [Actuator motors(4)]

### Usage {#mc_nn_control_usage}

```
mc_nn_control <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## mc_pos_control

Source: [modules/mc_pos_control](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/mc_pos_control)

### Опис

The controller has two loops: a P loop for position error and a PID loop for velocity error.
Output of the velocity controller is thrust vector that is split to thrust direction
(i.e. rotation matrix for multicopter orientation) and thrust scalar (i.e. multicopter thrust itself).

The controller doesn't use Euler angles for its work, they are generated only for more human-friendly control and
logging.

### Usage {#mc_pos_control_usage}

```
mc_pos_control <command> [arguments...]
 Commands:
   start
     [vtol]      VTOL mode

   stop

   status        print status info
```

## mc_rate_control

Source: [modules/mc_rate_control](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/mc_rate_control)

### Опис

This implements the multicopter rate controller. It takes rate setpoints (in acro mode
via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

The controller has a PID loop for angular rate error.

### Usage {#mc_rate_control_usage}

```
mc_rate_control <command> [arguments...]
 Commands:
   start
     [vtol]      VTOL mode

   stop

   status        print status info
```

## navigator

Source: [modules/navigator](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/navigator)

### Опис

Module that is responsible for autonomous flight modes. This includes missions (read from dataman),
takeoff and RTL.
It is also responsible for geofence violation checking.

### Імплементація

The different internal modes are implemented as separate classes that inherit from a common base class `NavigatorMode`.
The member `_navigation_mode` contains the current active mode.

Navigator publishes position setpoint triplets (`position_setpoint_triplet_s`), which are then used by the position
controller.

### Usage {#navigator_usage}

```
navigator <command> [arguments...]
 Commands:
   start

   fencefile     load a geofence file from SD card, stored at etc/geofence.txt

   fake_traffic  publishes 24 fake transponder_report_s uORB messages

   stop

   status        print status info
```

## rover_ackermann

Source: [modules/rover_ackermann](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/rover_ackermann)

### Опис

Rover ackermann module.

### Usage {#rover_ackermann_usage}

```
rover_ackermann <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## rover_differential

Source: [modules/rover_differential](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/rover_differential)

### Опис

Rover differential module.

### Usage {#rover_differential_usage}

```
rover_differential <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## rover_mecanum

Source: [modules/rover_mecanum](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/rover_mecanum)

### Опис

Rover mecanum module.

### Usage {#rover_mecanum_usage}

```
rover_mecanum <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## spacecraft

Source: [modules/spacecraft](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/spacecraft)

```
### Description
This implements control allocation for spacecraft vehicles.
It takes torque and thrust setpoints as inputs and outputs
actuator setpoint messages.
```

### Usage {#spacecraft_usage}

```
spacecraft <command> [arguments...]
 Commands:
   start

   status

   stop

   status        print status info
```

## uuv_att_control

Source: [modules/uuv_att_control](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/uuv_att_control)

### Опис

Controls the attitude of an unmanned underwater vehicle (UUV).

Publishes `vehicle_thrust_setpont` and `vehicle_torque_setpoint` messages at a constant 250Hz.

### Імплементація

Currently, this implementation supports only a few modes:

- Full manual: Roll, pitch, yaw, and throttle controls are passed directly through to the actuators
- Auto mission: The uuv runs missions

### Приклади

CLI usage example:

```
uuv_att_control start
uuv_att_control status
uuv_att_control stop
```

### Usage {#uuv_att_control_usage}

```
uuv_att_control <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## uuv_pos_control

Source: [modules/uuv_pos_control](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/uuv_pos_control)

### Опис

Controls the attitude of an unmanned underwater vehicle (UUV).
Publishes `attitude_setpoint` messages.

### Імплементація

Currently, this implementation supports only a few modes:

- Full manual: Roll, pitch, yaw, and throttle controls are passed directly through to the actuators
- Auto mission: The uuv runs missions

### Приклади

CLI usage example:

```
uuv_pos_control start
uuv_pos_control status
uuv_pos_control stop
```

### Usage {#uuv_pos_control_usage}

```
uuv_pos_control <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```

## vtol_att_control

Source: [modules/vtol_att_control](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/vtol_att_control)

### Опис

fw_att_control is the fixed wing attitude controller.

### Usage {#vtol_att_control_usage}

```
vtol_att_control <command> [arguments...]
 Commands:

   stop

   status        print status info
```
