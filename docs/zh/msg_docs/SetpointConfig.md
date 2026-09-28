---
pageClass: is-wide-page
---

# SetpointConfig (UORB message)

Setpoint configuration message.

Published by external modes and PX4 will respond with SetpointConfigReply.

**TOPICS:** setpoint_config

## Fields

| 参数名                                                            | 类型       | Unit [Frame] | Range/Enum    | 描述                                                                                                                                                                                                                                                                                                                       |
| -------------------------------------------------------------- | -------- | ---------------------------------------------------------------- | ------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| <a id="fld_timestamp"></a>timestamp                            | `uint64` | us                                                               |               | Time since system start                                                                                                                                                                                                                                                                                                  |
| <a id="fld_type"></a>type                                      | `uint16` |                                                                  | [TYPE](#TYPE) | setpoint type (corresponding to one or more setpoint messages)                                                                                                                                                                                                                                        |
| <a id="fld_source_id"></a>source_id       | `uint8`  |                                                                  |               | nav_state of the mode                                                                                                                                                                                                                                                                               |
| <a id="fld_should_apply"></a>should_apply | `bool`   |                                                                  |               | if true: apply as current setpoint configuration (mode should be active). If false: setpoint configuration is not changed (can be used to check if a setpoint can be used with the current vehicle configuration). |
| <a id="fld_timeout_ms"></a>timeout_ms     | `uint16` |                                                                  |               | Configure setpoint timeout. If no setpoint received for this time, PX4 triggers a failsafe. 0 disables the timeout (unresponsive modes still trigger a timeout through arming checks).                                                                |

## Enums

### TYPE {#TYPE}

Used in field(s): [type](#fld_type)

| 参数名                                                                                                                                                  | 类型       | 值  | 描述                                                         |
| ---------------------------------------------------------------------------------------------------------------------------------------------------- | -------- | -- | ---------------------------------------------------------- |
| <a id="#TYPE_INVALID"></a> TYPE_INVALID                                                                                         | `uint16` | 0  |                                                            |
| <a id="#TYPE_DIRECT_ACTUATORS"></a> TYPE_DIRECT_ACTUATORS                                                  | `uint16` | 1  | ActuatorMotors and ActuatorServos                          |
| <a id="#TYPE_MULTICOPTER_GOTO"></a> TYPE_MULTICOPTER_GOTO                                                  | `uint16` | 2  | GotoSetpoint                                               |
| <a id="#TYPE_FIXEDWING_LATERAL_LONGITUDINAL"></a> TYPE_FIXEDWING_LATERAL_LONGITUDINAL | `uint16` | 3  | FixedWingLateralSetpoint and FixedWingLongitudinalSetpoint |
| <a id="#TYPE_TRAJECTORY"></a> TYPE_TRAJECTORY                                                                                   | `uint16` | 4  | TrajectorySetpoint                                         |
| <a id="#TYPE_RATES"></a> TYPE_RATES                                                                                             | `uint16` | 5  | VehicleRatesSetpoint                                       |
| <a id="#TYPE_ATTITUDE"></a> TYPE_ATTITUDE                                                                                       | `uint16` | 6  | VehicleAttitudeSetpoint                                    |
| <a id="#TYPE_ROVER_POSITION"></a> TYPE_ROVER_POSITION                                                      | `uint16` | 7  | RoverPositionSetpoint                                      |
| <a id="#TYPE_ROVER_SPEED_ATTITUDE"></a> TYPE_ROVER_SPEED_ATTITUDE                     | `uint16` | 8  | RoverSpeedSetpoint and RoverAttitudeSetpoint               |
| <a id="#TYPE_ROVER_SPEED_RATE"></a> TYPE_ROVER_SPEED_RATE                             | `uint16` | 9  | RoverSpeedSetpoint and RoverRateSetpoint                   |
| <a id="#TYPE_ROVER_SPEED_STEERING"></a> TYPE_ROVER_SPEED_STEERING                     | `uint16` | 10 | RoverSpeedSetpoint and RoverSteeringSetpoint               |
| <a id="#TYPE_ROVER_THROTTLE_ATTITUDE"></a> TYPE_ROVER_THROTTLE_ATTITUDE               | `uint16` | 11 | RoverThrottleSetpoint and RoverAttitudeSetpoint            |
| <a id="#TYPE_ROVER_THROTTLE_RATE"></a> TYPE_ROVER_THROTTLE_RATE                       | `uint16` | 12 | RoverThrottleSetpoint and RoverRateSetpoint                |
| <a id="#TYPE_ROVER_THROTTLE_STEERING"></a> TYPE_ROVER_THROTTLE_STEERING               | `uint16` | 13 | RoverThrottleSetpoint and RoverSteeringSetpoint            |
| <a id="#TYPE_TRAJECTORY_6DOF"></a> TYPE_TRAJECTORY_6DOF                                                    | `uint16` | 14 | TrajectorySetpoint6dof                                     |
| <a id="#TYPE_THRUST_AND_TORQUE"></a> TYPE_THRUST_AND_TORQUE                           | `uint16` | 15 | VehicleThrustSetpoint and VehicleTorqueSetpoint            |
| <a id="#TYPE_POSITION_TRIPLET"></a> TYPE_POSITION_TRIPLET                                                  | `uint16` | 16 | PositionSetpointTriplet                                    |

## Constants

| 参数名                                                                | 类型       | 值 | 描述 |
| ------------------------------------------------------------------ | -------- | - | -- |
| <a id="#MESSAGE_VERSION"></a> MESSAGE_VERSION | `uint32` | 0 |    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/versioned/SetpointConfig.msg)

:::details
Click here to see original file

```c
# Setpoint configuration message
#
# Published by external modes and PX4 will respond with SetpointConfigReply.

uint32 MESSAGE_VERSION = 0

uint64 timestamp # [us] Time since system start

uint16 TYPE_INVALID = 0
uint16 TYPE_DIRECT_ACTUATORS = 1 # ActuatorMotors and ActuatorServos
uint16 TYPE_MULTICOPTER_GOTO = 2 # GotoSetpoint
uint16 TYPE_FIXEDWING_LATERAL_LONGITUDINAL = 3 # FixedWingLateralSetpoint and FixedWingLongitudinalSetpoint
uint16 TYPE_TRAJECTORY = 4 # TrajectorySetpoint
uint16 TYPE_RATES = 5 # VehicleRatesSetpoint
uint16 TYPE_ATTITUDE = 6 # VehicleAttitudeSetpoint
uint16 TYPE_ROVER_POSITION = 7 # RoverPositionSetpoint
uint16 TYPE_ROVER_SPEED_ATTITUDE = 8 # RoverSpeedSetpoint and RoverAttitudeSetpoint
uint16 TYPE_ROVER_SPEED_RATE = 9 # RoverSpeedSetpoint and RoverRateSetpoint
uint16 TYPE_ROVER_SPEED_STEERING = 10 # RoverSpeedSetpoint and RoverSteeringSetpoint
uint16 TYPE_ROVER_THROTTLE_ATTITUDE = 11 # RoverThrottleSetpoint and RoverAttitudeSetpoint
uint16 TYPE_ROVER_THROTTLE_RATE = 12 # RoverThrottleSetpoint and RoverRateSetpoint
uint16 TYPE_ROVER_THROTTLE_STEERING = 13 # RoverThrottleSetpoint and RoverSteeringSetpoint
uint16 TYPE_TRAJECTORY_6DOF = 14 # TrajectorySetpoint6dof
uint16 TYPE_THRUST_AND_TORQUE = 15 # VehicleThrustSetpoint and VehicleTorqueSetpoint
uint16 TYPE_POSITION_TRIPLET = 16 # PositionSetpointTriplet

uint16 type # [@enum TYPE] setpoint type (corresponding to one or more setpoint messages)

uint8 source_id # nav_state of the mode

bool should_apply # if true: apply as current setpoint configuration (mode should be active). If false: setpoint configuration is not changed (can be used to check if a setpoint can be used with the current vehicle configuration).

uint16 timeout_ms # Configure setpoint timeout. If no setpoint received for this time, PX4 triggers a failsafe. 0 disables the timeout (unresponsive modes still trigger a timeout through arming checks).
```

:::
