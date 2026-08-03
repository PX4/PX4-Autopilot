# Aggregated feedback from the six GL40II cable-driven tilt actuators.
uint64 timestamp                         # [us] Time since system start
uint64 timestamp_sample                  # [us] Newest accepted CAN response

uint8 STATE_DISABLED = 0
uint8 STATE_ENABLED = 1
uint8 STATE_OVERVOLTAGE = 8
uint8 STATE_UNDERVOLTAGE = 9
uint8 STATE_OVERCURRENT = 10
uint8 STATE_DRIVER_OVERTEMP = 11
uint8 STATE_MOTOR_OVERTEMP = 12
uint8 STATE_COMMUNICATION_LOSS = 13
uint8 STATE_OVERLOAD = 14

float32[6] angle                         # [rad] Measured radial-axis joint angles
float32[6] angular_velocity              # [rad/s] Measured joint angular velocities
float32[6] motor_torque                  # [Nm] GL40II motor-side torque estimate
int8[6] driver_temperature               # [degC] Driver MOSFET temperatures
int8[6] motor_temperature                # [degC] Motor winding temperatures
uint8[6] state                           # STATE_* value from feedback high nibble
uint32[6] rx_count                       # Accepted feedback frames since driver start
uint8 active_mask                        # Motors configured for this driver instance
uint8 valid_mask                         # Feedback received within GL40_FB_TMO
uint8 enabled_mask                       # Motors reporting STATE_ENABLED
uint8 fault_mask                         # Motors reporting a STATE_* fault
uint8 stale_mask                         # Configured motors without fresh feedback
