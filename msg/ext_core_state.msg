# this message describes the format for an external state

# state timestamp (microseconds)
uint64 timestamp
uint64 timestamp_sample		

# Position of the imu frame w.r.t. the world frame
float32[3] p_wi				

# Velocity of the imu frame w.r.t. the world frame
float32[3] v_wi		

# Orientation of the imu frame w.r.t. the world frame
float32[4] q_wi		

# Accelerometer bias
float32[3] b_a	

# Gyroscope bias
float32[3] b_w	

# Position orientation and velocity upper triangular covariance in Row-Major order
float32[45] cov_urt
