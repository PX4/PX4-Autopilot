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
