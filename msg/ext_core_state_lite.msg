# this message describes the format for an external state lite

# state timestamp (microseconds)
uint64 timestamp
uint64 timestamp_sample		

# Position of the imu frame w.r.t. the world frame
float32[3] p_wi				

# Velocity of the imu frame w.r.t. the world frame
float32[3] v_wi		

# Orientation of the imu frame w.r.t. the world frame
float32[4] q_wi		
