# Mode Requirements

Mode requirements define the set of conditions that must be met in order to arm in a particular flight mode, or to switch to the mode if it is already armed.

Requirements are defined for internal modes in [/src/modules/commander/ModeUtil/mode_requirements.cpp](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/ModeUtil/mode_requirements.cpp), and for ROS 2 external modes in [requirement_flags.hpp](https://github.com/Auterion/px4-ros2-interface-lib/blob/main/px4_ros2_cpp/include/px4_ros2/common/requirement_flags.hpp) (Github `Auterion/px4-ros2-interface-lib` repository).
The mode requirements are the same in both cases.

The following sections provide an overview of the requirements and what modes they are used in.

## Requirement definitions

| Requirement                                | Example                                                                                                                                                                                                                                                                                                                                                                                                                               |
| ------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `mode_req_angular_velocity`                | Angular velocity (gyroscope) <br><br>                                                                                                                                                                                                                                                                                                                                                                                                 |
| `mode_req_attitude`                        | Attitude/pose (IMU, or theoretically a motion capture system) <br><br>                                                                                                                                                                                                                                                                                                                                                                |
| `mode_req_local_position`                  | Position relative to EKF2 origin ('0') point (GNSS, VIO, mocap) <br><br>                                                                                                                                                                                                                                                                                                                                                              |
| `mode_req_local_position_relaxed`          | Position relative to EKF2 origin ('0') point but accepts poor accuracy (Optical flow) <br><br>Typically optical flow. You set zero when you take off and just integrate up the flow. So the absolute position can be very inaccurate but better than nothing. Useful to not drift away quickly and roughly know where you are.                                                                                                        |
| `mode_req_global_position`                 | Position measurement updates in a global coordinate frame (GNSS, or local position and global reference to EKF 0) <br><br>                                                                                                                                                                                                                                                                                                            |
| `mode_req_global_position_relaxed`         | Position measurement updates in a global coordinate frame but accepts poor accuracy <br><br>Local position and global reference to EKF2 origin                                                                                                                                                                                                                                                                                        |
| `mode_req_local_alt`                       | Local altitude relative to EKF2 origin ('0') position (Barometer corrected by GNSS altitude over time, distance sensor) <br><br>Usually not the distance sensor because if the ground shifts then the coordinate frame would shift with it (range aid problem).                                                                                                                                                                       |
| `mode_req_mission`                         | Valid mission in autopilot's storage (Mission mode only requirement) <br><br>can be from last time no need to upload fresh                                                                                                                                                                                                                                                                                                            |
| `mode_req_offboard_signal`                 | Offboard heartbeat <br><br>MAVLink messages SET_ATTITUDE_TARGET or SET_POSITION_TARGET_LOCAL_NED or SET_POSITION_TARGET_GLOBAL_INT not timing out. Offboard mode specific requirement.                                                                                                                                                                                                                                                |
| `mode_req_home_position`                   | Global home reference must be set <br><br>Specific requirement for Return mode                                                                                                                                                                                                                                                                                                                                                        |
| `mode_req_wind_and_flight_time_compliance` | Safety compliance limits on wind and flight time. <br><br>Autonomous flight prevented in high winds (20m/s?) or if exceeds planned flight time, except for failsafe flight. See COM_FLT_TIME_MAX or COM_WIND_MAX for limits (also see COM_WIND_WARN, COM_WIND_MAX_ACT).                                                                                                                                                               |
| `mode_req_prevent_arming`                  | Mode prevents arming (vehicle must be armed to switch to this mode) <br><br>Set for modes like Land, Orbit, Return that you can't take off in                                                                                                                                                                                                                                                                                         |
| `mode_req_manual_control`                  | Requires stick input <br><br>This can come from multiple channels: RC driver -> channels -> RC mapping/calibration -> [`manual_control_input` or `manual_control_setpoint` topic](../msg_docs/ManualControlSetpoint.md) -> Selector using [COM_RC_IN_MODE](../advanced_config/parameter_reference.md#COM_RC_IN_MODE) -> Joystick (MAVLink [MANUAL_CONTROL](https://mavlink.io/en/messages/common.html#MANUAL_CONTROL) message) -> ... |
| `mode_req_other`                           | Others requirement. <br><br>This is used by external modes. It is intended to specify additional requirements not covered by the existing flags                                                                                                                                                                                                                                                                                       |

### Naming Conventions

In general requirement flag names are abstracted from specific sensors.
This is done because particular requirements can often be met by several sensors.
For example, GNSS is the most common source of global position, but it isn't the only one.

The requirements include frame and accuracy information hints in their names:

- `global` means an absolute world frame, such as that provided by GNSS.
- `local` means a frame that relative to an initialization point, such as the position of an IMU on boot.
- `relaxed` means that the mode does not require or rely on accurate data: as long as sensors are providing some data the state is considered valid.
  Relaxed conditions are used for modes where some sensor data is considered more important than none at all, such as when calculating position via optical flow velocity measurements.
  By contrast, a position mode that is not relaxed requires reliable sensor data, and will block arming if inaccuracy is detected.

Note that a global position requirement can be met if you have a valid _local position_, by mapping the local frame to a global position.
This can be done by setting the global position of the local origin using the MAVLink message [SET_GPS_GLOBAL_ORIGIN](https://mavlink.io/en/messages/common.html#SET_GPS_GLOBAL_ORIGIN), either directly or via a GCS (see [External Position Estimate > Enabling Auto Modes with a Local Position](../ros/external_position_estimation#enabling-auto-modes-with-a-local-position)).
Similarly, if the vehicle has `mode_req_local_position_relaxed`, then you can map to a global position in order to meet the `global_position_relaxed` requirement.
This allows PX4 automatic flight modes that require a global position to be used locally, such as Mission and Return.

<!-- 
## Requirement definitions


### mode_req_angular_velocity

Angular velocity (gyroscope)  



### mode_req_attitude

Attitude/pose (IMU, or theoretically a motion capture system)  



### mode_req_local_position

Position relative to EKF2 origin ('0') point (GNSS, VIO, mocap)  



### mode_req_local_position_relaxed

Position relative to EKF2 origin ('0') point but accepts poor accuracy (Optical flow)  

Typically optical flow. You set zero when you take off and just integrate up the flow. So the absolute position can be very inaccurate but better than nothing. Useful to not drift away quickly and roughly know where you are.

### mode_req_global_position

Position measurement updates in a global coordinate frame (GNSS, or local position and global reference to EKF 0)  



### mode_req_global_position_relaxed

Position measurement updates in a global coordinate frame but accepts poor accuracy 

Local position and global reference to EKF2 origin

### mode_req_local_alt

Local altitude relative to EKF2 origin ('0') position (Barometer corrected by GNSS altitude over time, distance sensor)  

Usually not the distance sensor because if the ground shifts then the coordinate frame would shift with it (range aid problem).

### mode_req_mission

Valid mission in autopilot's storage (Mission mode only requirement)  

can be from last time no need to upload fresh

### mode_req_offboard_signal

Offboard heartbeat 

MAVLink messages SET_ATTITUDE_TARGET or SET_POSITION_TARGET_LOCAL_NED or SET_POSITION_TARGET_GLOBAL_INT not timing out. Offboard mode specific requirement.

### mode_req_home_position

Global home reference must be set 

Specific requirement for Return mode

### mode_req_wind_and_flight_time_compliance

Safety compliance limits on wind and flight time. 

Autonomous flight prevented in high winds (20m/s?) or if exceeds planned flight time, except for failsafe flight. See COM_FLT_TIME_MAX or COM_WIND_MAX for limits (also see COM_WIND_WARN, COM_WIND_MAX_ACT).

### mode_req_prevent_arming

Mode prevents arming (vehicle must be armed to switch to this mode) 

Set for modes like Land, Orbit, Return that you can't take off in

### mode_req_manual_control

Requires stick input 

This can come from multiple channels: RC driver -> channels -> RC mapping/calibration -> [`manual_control_input` or `manual_control_setpoint` topic](../msg_docs/ManualControlSetpoint.md) -> Selector using [COM_RC_IN_MODE](../advanced_config/parameter_reference.md#COM_RC_IN_MODE) -> Joystick (MAVLink [MANUAL_CONTROL](https://mavlink.io/en/messages/common.html#MANUAL_CONTROL) message) -> ...

### mode_req_other

Others requirement. 

This is used by external modes. It is intended to specify additional requirements not covered by the existing flags

-->

## Multicopter (VEHICLE_TYPE_ROTARY_WING)

### [Mission Mode](../flight_modes_mc/mission.md) (NAVIGATION_STATE_AUTO_MISSION)

- mode_req_angular_velocity
- mode_req_attitude
- mode_req_global_position
- mode_req_local_alt
- mode_req_local_position
- mode_req_mission
- mode_req_wind_and_flight_time_compliance

### [Hold Mode](../flight_modes_mc/hold.md) (NAVIGATION_STATE_AUTO_LOITER)

- mode_req_angular_velocity
- mode_req_attitude
- mode_req_global_position
- mode_req_local_alt
- mode_req_local_position
- mode_req_wind_and_flight_time_compliance

### [Return Mode](../flight_modes_mc/return.md) (NAVIGATION_STATE_AUTO_RTL)

- mode_req_angular_velocity
- mode_req_attitude
- mode_req_global_position
- mode_req_home_position
- mode_req_local_alt
- mode_req_local_position
- mode_req_prevent_arming

### [Takeoff Mode](../flight_modes_mc/takeoff.md) (NAVIGATION_STATE_AUTO_TAKEOFF)

- mode_req_angular_velocity
- mode_req_attitude
- mode_req_local_alt
- mode_req_local_position

### NAVIGATION_STATE_AUTO_LAND

- mode_req_angular_velocity
- mode_req_attitude
- mode_req_local_alt
- mode_req_local_position_relaxed
- mode_req_prevent_arming

### NAVIGATION_STATE_AUTO_FOLLOW_TARGET

- mode_req_angular_velocity
- mode_req_attitude
- mode_req_local_alt
- mode_req_local_position
- mode_req_prevent_arming
- mode_req_wind_and_flight_time_compliance

### NAVIGATION_STATE_AUTO_PRECLAND

- mode_req_angular_velocity
- mode_req_attitude
- mode_req_local_alt
- mode_req_local_position
- mode_req_prevent_arming

### NAVIGATION_STATE_ORBIT

- mode_req_angular_velocity
- mode_req_attitude
- mode_req_local_alt
- mode_req_local_position
- mode_req_prevent_arming
- mode_req_wind_and_flight_time_compliance

### NAVIGATION_STATE_AUTO_VTOL_TAKEOFF

- mode_req_angular_velocity
- mode_req_attitude
- mode_req_local_alt
- mode_req_local_position_relaxed

### NAVIGATION_STATE_MANUAL

- mode_req_manual_control

### NAVIGATION_STATE_ALTCTL

- mode_req_angular_velocity
- mode_req_attitude
- mode_req_local_alt
- mode_req_manual_control

### NAVIGATION_STATE_ALTITUDE_CRUISE

- mode_req_angular_velocity
- mode_req_attitude
- mode_req_local_alt
- mode_req_manual_control

### NAVIGATION_STATE_POSCTL

- mode_req_angular_velocity
- mode_req_attitude
- mode_req_local_alt
- mode_req_local_position_relaxed
- mode_req_manual_control

### NAVIGATION_STATE_POSITION_SLOW

- mode_req_angular_velocity
- mode_req_attitude
- mode_req_local_alt
- mode_req_local_position_relaxed
- mode_req_manual_control

### NAVIGATION_STATE_ACRO

- mode_req_angular_velocity
- mode_req_manual_control

### NAVIGATION_STATE_DESCEND

- mode_req_angular_velocity
- mode_req_attitude
- mode_req_prevent_arming

### NAVIGATION_STATE_TERMINATION

- mode_req_prevent_arming

### NAVIGATION_STATE_OFFBOARD

- mode_req_angular_velocity
- mode_req_attitude
- mode_req_offboard_signal

### NAVIGATION_STATE_STAB

- mode_req_angular_velocity
- mode_req_attitude
- mode_req_manual_control

## Fixed-wing (VEHICLE_TYPE_FIXED_WING)

### [Mission Mode](../flight_modes_fw/mission.md) (NAVIGATION_STATE_AUTO_MISSION)

- mode_req_angular_velocity
- mode_req_attitude
- mode_req_global_position_relaxed
- mode_req_local_alt
- mode_req_local_position_relaxed
- mode_req_mission
- mode_req_wind_and_flight_time_compliance

### [Hold Mode](../flight_modes_fw/hold.md) (NAVIGATION_STATE_AUTO_LOITER)

- mode_req_angular_velocity
- mode_req_attitude
- mode_req_global_position_relaxed
- mode_req_local_alt
- mode_req_local_position_relaxed
- mode_req_wind_and_flight_time_compliance

### [Return Mode](../flight_modes_fw/return.md) (NAVIGATION_STATE_AUTO_RTL)

- mode_req_angular_velocity
- mode_req_attitude
- mode_req_global_position_relaxed
- mode_req_home_position
- mode_req_local_alt
- mode_req_local_position_relaxed
- mode_req_prevent_arming

### NAVIGATION_STATE_AUTO_VTOL_TAKEOFF

- mode_req_local_position

### NAVIGATION_STATE_MANUAL

- mode_req_manual_control

### NAVIGATION_STATE_ALTCTL

- mode_req_angular_velocity
- mode_req_attitude
- mode_req_local_alt
- mode_req_manual_control

### NAVIGATION_STATE_ALTITUDE_CRUISE

- mode_req_angular_velocity
- mode_req_attitude
- mode_req_local_alt
- mode_req_manual_control

### NAVIGATION_STATE_POSCTL

- mode_req_angular_velocity
- mode_req_attitude
- mode_req_local_alt
- mode_req_local_position_relaxed
- mode_req_manual_control

### NAVIGATION_STATE_POSITION_SLOW

- mode_req_angular_velocity
- mode_req_attitude
- mode_req_local_alt
- mode_req_local_position_relaxed
- mode_req_manual_control

### NAVIGATION_STATE_ACRO

- mode_req_angular_velocity
- mode_req_manual_control

### NAVIGATION_STATE_DESCEND

- mode_req_angular_velocity
- mode_req_attitude
- mode_req_prevent_arming

### NAVIGATION_STATE_TERMINATION

- mode_req_prevent_arming

### NAVIGATION_STATE_OFFBOARD

- mode_req_angular_velocity
- mode_req_attitude
- mode_req_offboard_signal

### NAVIGATION_STATE_STAB

- mode_req_angular_velocity
- mode_req_attitude
- mode_req_manual_control

### [Takeoff Mode](../flight_modes_fw/takeoff.md) (NAVIGATION_STATE_AUTO_TAKEOFF)

- mode_req_angular_velocity
- mode_req_attitude
- mode_req_local_alt
