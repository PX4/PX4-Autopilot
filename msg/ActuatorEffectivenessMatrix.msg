# Actuator effectiveness matrix for a single matrix
uint64 timestamp

uint8 NUM_AXES = 6
uint8 NUM_ACTUATORS = 16

# Effectiveness matrix (flattened row-major)
# 6 rows, 16 columns
float32[96] effectiveness_matrix_row_major

# Number of actuators used in this matrix
uint8 num_actuators
