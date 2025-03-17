# Modules Reference: Controller

## airship_att_control
Source: [modules/airship_att_control](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/airship_att_control)


### Description
This implements the airship attitude and rate controller. Ideally it would
take attitude setpoints (`vehicle_attitude_setpoint`) or rate setpoints (in acro mode
via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

Currently it is feeding the `manual_control_setpoint` topic directly to the actuators.

### Implementation
To reduce control latency, the module directly polls on the gyro topic published by the IMU driver.


<a id="airship_att_control_usage"></a>
### Usage
```
airship_att_control <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```
## control_allocator
Source: [modules/control_allocator](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/control_allocator)


### Description
This implements control allocation. It takes torque and thrust setpoints
as inputs and outputs actuator setpoint messages.

<a id="control_allocator_usage"></a>
### Usage
```
control_allocator <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```
## flight_mode_manager
Source: [modules/flight_mode_manager](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/flight_mode_manager)


### Description
This implements the setpoint generation for all modes. It takes the current mode state of the vehicle as input
and outputs setpoints for controllers.


<a id="flight_mode_manager_usage"></a>
### Usage
```
flight_mode_manager <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```
## fw_att_control
Source: [modules/fw_att_control](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/fw_att_control)


### Description
fw_att_control is the fixed wing attitude controller.


<a id="fw_att_control_usage"></a>
### Usage
```
fw_att_control <command> [arguments...]
 Commands:
   start
     [vtol]      VTOL mode

   stop

   status        print status info
```
## fw_pos_control
Source: [modules/fw_pos_control](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/fw_pos_control)


### Description
fw_pos_control is the fixed-wing position controller.


<a id="fw_pos_control_usage"></a>
### Usage
```
fw_pos_control <command> [arguments...]
 Commands:
   start
     [vtol]      VTOL mode

   stop

   status        print status info
```
## fw_rate_control
Source: [modules/fw_rate_control](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/fw_rate_control)


### Description
fw_rate_control is the fixed-wing rate controller.


<a id="fw_rate_control_usage"></a>
### Usage
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


### Description
This implements the multicopter attitude controller. It takes attitude
setpoints (`vehicle_attitude_setpoint`) as inputs and outputs a rate setpoint.

The controller has a P loop for angular error

Publication documenting the implemented Quaternion Attitude Control:
Nonlinear Quadrocopter Attitude Control (2013)
by Dario Brescianini, Markus Hehn and Raffaello D'Andrea
Institute for Dynamic Systems and Control (IDSC), ETH Zurich

https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf


<a id="mc_att_control_usage"></a>
### Usage
```
mc_att_control <command> [arguments...]
 Commands:
   start
     [vtol]      VTOL mode

   stop

   status        print status info
```
## mc_pos_control
Source: [modules/mc_pos_control](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/mc_pos_control)


### Description
The controller has two loops: a P loop for position error and a PID loop for velocity error.
Output of the velocity controller is thrust vector that is split to thrust direction
(i.e. rotation matrix for multicopter orientation) and thrust scalar (i.e. multicopter thrust itself).

The controller doesn't use Euler angles for its work, they are generated only for more human-friendly control and
logging.

<a id="mc_pos_control_usage"></a>
### Usage
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


### Description
This implements the multicopter rate controller. It takes rate setpoints (in acro mode
via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

The controller has a PID loop for angular rate error.


<a id="mc_rate_control_usage"></a>
### Usage
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


### Description
Module that is responsible for autonomous flight modes. This includes missions (read from dataman),
takeoff and RTL.
It is also responsible for geofence violation checking.

### Implementation
The different internal modes are implemented as separate classes that inherit from a common base class `NavigatorMode`.
The member `_navigation_mode` contains the current active mode.

Navigator publishes position setpoint triplets (`position_setpoint_triplet_s`), which are then used by the position
controller.


<a id="navigator_usage"></a>
### Usage
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


### Description
Rover ackermann module.

<a id="rover_ackermann_usage"></a>
### Usage
```
rover_ackermann <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```
## rover_differential
Source: [modules/rover_differential](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/rover_differential)


### Description
Rover differential module.

<a id="rover_differential_usage"></a>
### Usage
```
rover_differential <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```
## rover_mecanum
Source: [modules/rover_mecanum](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/rover_mecanum)


### Description
Rover mecanum module.

<a id="rover_mecanum_usage"></a>
### Usage
```
rover_mecanum <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```
## rover_pos_control
Source: [modules/rover_pos_control](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/rover_pos_control)


### Description
Controls the position of a ground rover using an L1 controller.

Publishes `vehicle_thrust_setpoint (only in x) and vehicle_torque_setpoint (only yaw)` messages at IMU_GYRO_RATEMAX.

### Implementation
Currently, this implementation supports only a few modes:

 * Full manual: Throttle and yaw controls are passed directly through to the actuators
 * Auto mission: The rover runs missions
 * Loiter: The rover will navigate to within the loiter radius, then stop the motors

### Examples
CLI usage example:
```
rover_pos_control start
rover_pos_control status
rover_pos_control stop
```


<a id="rover_pos_control_usage"></a>
### Usage
```
rover_pos_control <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```
## spacecraft
Source: [modules/spacecraft](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/spacecraft)


	### Description
	This implements control allocation for spacecraft vehicles.
	It takes torque and thrust setpoints as inputs and outputs
	actuator setpoint messages.
	
<a id="spacecraft_usage"></a>
### Usage
```
spacecraft <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```
## uuv_att_control
Source: [modules/uuv_att_control](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/uuv_att_control)


### Description
Controls the attitude of an unmanned underwater vehicle (UUV).

Publishes `vehicle_thrust_setpont` and `vehicle_torque_setpoint` messages at a constant 250Hz.

### Implementation
Currently, this implementation supports only a few modes:

 * Full manual: Roll, pitch, yaw, and throttle controls are passed directly through to the actuators
 * Auto mission: The uuv runs missions

### Examples
CLI usage example:
```
uuv_att_control start
uuv_att_control status
uuv_att_control stop
```


<a id="uuv_att_control_usage"></a>
### Usage
```
uuv_att_control <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```
## uuv_pos_control
Source: [modules/uuv_pos_control](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/uuv_pos_control)


### Description
Controls the attitude of an unmanned underwater vehicle (UUV).
Publishes `attitude_setpoint` messages.
### Implementation
Currently, this implementation supports only a few modes:
 * Full manual: Roll, pitch, yaw, and throttle controls are passed directly through to the actuators
 * Auto mission: The uuv runs missions
### Examples
CLI usage example:
```
uuv_pos_control start
uuv_pos_control status
uuv_pos_control stop
```

<a id="uuv_pos_control_usage"></a>
### Usage
```
uuv_pos_control <command> [arguments...]
 Commands:
   start

   stop

   status        print status info
```
## vtol_att_control
Source: [modules/vtol_att_control](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/vtol_att_control)


### Description
fw_att_control is the fixed wing attitude controller.

<a id="vtol_att_control_usage"></a>
### Usage
```
vtol_att_control <command> [arguments...]
 Commands:

   stop

   status        print status info
```
