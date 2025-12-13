# TOPICS raptor_input
uint64 timestamp
uint64 timestamp_sample
bool active
float32[3] position
float32[4] orientation # w, x, y, z
float32[3] linear_velocity
float32[3] angular_velocity
uint8 ACTION_DIM = 4
float32[4] previous_action
