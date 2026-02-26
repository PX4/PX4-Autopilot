# Raptor Status

# Diagnostic messages for the Raptor foundation policy.
# This diagnostic data is useful for debugging (e.g. identifying missing input information).

uint32 MESSAGE_VERSION = 0

uint64 timestamp # [us] Time since system start
uint64 timestamp_sample # [us] Sampling timestamp of the data this control response is based on
bool subscription_update_angular_velocity # [bool] Flag signalling if the vehicle_angular_velocity was updated
bool subscription_update_local_position # [bool] Flag signalling if the vehicle_local_position was updated
bool subscription_update_attitude # [bool] Flag signalling if the vehicle_attitude was updated
bool subscription_update_trajectory_setpoint # [bool] Flag signalling if the trajectory_setpoint was updated
bool subscription_update_vehicle_status # [bool] Flag signalling if the vehicle_status was updated

uint8 exit_reason # [enum] Exit reason identifier. Representing conditions that lead to the Raptor policy not being executed
uint8 EXIT_REASON_NONE = 0 # No exit reason => Raptor control step was executed (actuator_motors should have been published)
uint8 EXIT_REASON_NO_ANGULAR_VELOCITY_UPDATE = 1 # We synchronize the control onto the input observation with the highest update frequency, which is vehicle_angular_velocity. If there was no update, we do not need to execute the policy again
uint8 EXIT_REASON_NOT_ALL_OBSERVATIONS_SET = 2 # We can not execute the policy if not all observations are available
uint8 EXIT_REASON_ANGULAR_VELOCITY_STALE = 3 # If OBSERVATION_TIMEOUT_ANGULAR_VELOCITY is exceeded, we treat the vehicle_angular_velocity as stale and can not run the policy
uint8 EXIT_REASON_LOCAL_POSITION_STALE = 4 # If OBSERVATION_TIMEOUT_LOCAL_POSITION is exceeded, we treat the vehicle_local_position as stale and can not run the policy
uint8 EXIT_REASON_ATTITUDE_STALE = 5 # If OBSERVATION_TIMEOUT_ATTITUDE is exceeded, we treat the vehicle_attitude as stale and can not run the policy
uint8 EXIT_REASON_EXECUTOR_STATUS_SOURCE_NOT_CONTROL = 6 # The executor that runs the policy can run in oversampling mode, where it decides if the policy should be ran based on the timestamp and not based on fixed synchronization onto the vehicle_angular_velocity. In this case the executor can decide to skip running the policy if the interval is too small, in which case this flag is set.

uint32 timestamp_last_vehicle_angular_velocity # [us] Timestamp of the last received vehicle_angular_velocity message
uint32 timestamp_last_vehicle_local_position # [us] Timestamp of the last received vehicle_local_position message
uint32 timestamp_last_vehicle_attitude # [us] Timestamp of the last received vehicle_attitude message
uint32 timestamp_last_trajectory_setpoint # [us] Timestamp of the last received trajectory_setpoint message

bool vehicle_angular_velocity_stale # [bool] True if vehicle_angular_velocity data is considered stale (exceeded timeout)
bool vehicle_local_position_stale # [bool] True if vehicle_local_position data is considered stale (exceeded timeout)
bool vehicle_attitude_stale # [bool] True if vehicle_attitude data is considered stale (exceeded timeout)
bool trajectory_setpoint_stale # [bool] True if trajectory_setpoint data is considered stale (exceeded timeout)

bool active # [bool] True if the Raptor policy is currently active (publishing actuator_motors)
uint8 substep # [-] The policy is trained at a fixed frequency (e.g. 100 Hz) but we might want to use it for control at higher frequencies (e.g. 400 Hz), which leads to a number of intermediate steps before the actual policy state is advanced (in this case 4 = 400 Hz / 100 Hz). This field provides the current substep (e.g. 0-3).
float32 control_interval # [s] Time interval between control updates

float32 trajectory_setpoint_dt_mean # [us] The average trajectory setpoint arrival time interval (since Raptor mode activation within NUM_TRAJECTORY_SETPOINT_DTS received trajectory_setpoint messages)
float32 trajectory_setpoint_dt_max # [us] The max trajectory setpoint arrival time interval (since Raptor mode activation and within NUM_TRAJECTORY_SETPOINT_DTS received trajectory_setpoint messages)
float32 trajectory_setpoint_dt_max_since_activation # [us] The max trajectory setpoint arrival time interval (since Raptor mode activation)

float32[3] internal_reference_position # [m] [@frame FLU] Internal reference position
float32[3] internal_reference_linear_velocity # [m/s] [@frame FLU] Internal reference linear velocity

# TOPICS raptor_status
