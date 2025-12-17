# Raptor Input

# The exact inputs to the Raptor foundation policy.
# Having access to the exact inputs helps with debugging and post-hoc analysis

uint32 MESSAGE_VERSION = 0

uint64 timestamp            # [us] Time since system start
uint64 timestamp_sample     # [us] Sampling timestamp of the data this control response is based on
bool active                 # Signals if the policy is active (aka publishing actuator_motors)
float32[3] position         # Position in [m] in the vehicle_local_position frame but using the FLU convention
float32[4] orientation      # Orientation in the vehicle_attitude frame but using the FLU convention as a unit quaternion (w, x, y, z)
float32[3] linear_velocity  # Linear velocity [m/s] in the vehicle_local_position frame but using the FLU convention
float32[3] angular_velocity # Angular velocity [rad/s] in the body frame (FLU convention)
uint8 ACTION_DIM = 4        # Policy output dimensionality (for quadrotors)
float32[4] previous_action  # Previous action. Motor commands normalized to [-1, 1]

# TOPICS raptor_input
