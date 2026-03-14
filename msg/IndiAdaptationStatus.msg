# INDI Adaptation Status Message
# Tracks changes in G matrices during INDI adaptation

uint64 timestamp        # time since system start (microseconds)
uint64 timestamp_sample # timestamp of the data used for adaptation

# G1 matrix (moment part) - 3x16 matrix flattened
float32[48] g1_matrix   # G1 matrix elements (3 rows * 16 columns)

# G2 matrix (gyroscopic part) - 3x16 matrix flattened
float32[48] g2_matrix   # G2 matrix elements (3 rows * 16 columns)

# Adaptation inputs
float32[16] rotor_speeds        # Current rotor speeds (rad/s)
float32[16] rotor_speed_dots    # Rotor speed derivatives (rad/s^2)
float32[3] angular_accel_delta  # Angular acceleration delta (rad/s^2)

# Adaptation constants
float32[3] g1_adaptive_constants # G1 adaptation constants
float32[3] g2_adaptive_constants # G2 adaptation constants

# Status flags
bool adaptation_enabled          # Whether adaptation is currently enabled
uint8 num_actuators             # Number of actuators being adapted
