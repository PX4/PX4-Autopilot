# this message describes the format for an external state

# state timestamp (microseconds)
uint64 timestamp		

# Position of the imu frame w.r.t. the world frame
float64 p_wi_x			
float64 p_wi_y			
float64 p_wi_z			

# Velocity of the imu frame w.r.t. the world frame
float64 v_wi_x			
float64 v_wi_y			
float64 v_wi_z			

# Orientation of the imu frame w.r.t. the world frame
float64 q_wi_w			
float64 q_wi_x			
float64 q_wi_y			
float64 q_wi_z			

