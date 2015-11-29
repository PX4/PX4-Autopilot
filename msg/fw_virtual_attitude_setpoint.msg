
uint64 timestamp				# in microseconds since system start, is set whenever the writing thread stores new data

float32 roll_body				# body angle in NED frame
float32 pitch_body				# body angle in NED frame
float32 yaw_body				# body angle in NED frame

float32 yaw_sp_move_rate		# rad/s (commanded by user)

float32[9] R_body				# Rotation matrix describing the setpoint as rotation from the current body frame
bool R_valid					# Set to true if rotation matrix is valid

# For quaternion-based attitude control
float32[4] q_d					# Desired quaternion for quaternion control
bool q_d_valid					# Set to true if quaternion vector is valid
float32[4] q_e					# Attitude error in quaternion
bool q_e_valid					# Set to true if quaternion error vector is valid

float32 thrust					# Thrust in Newton the power system should generate

bool roll_reset_integral			# Reset roll integral part (navigation logic change)
bool pitch_reset_integral			# Reset pitch integral part (navigation logic change)
bool yaw_reset_integral				# Reset yaw integral part (navigation logic change)
