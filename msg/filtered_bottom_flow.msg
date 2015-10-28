# Filtered bottom flow in bodyframe.
uint64 timestamp		# time of this estimate, in microseconds since system start

float32 sumx				# Integrated bodyframe x flow in meters
float32 sumy				# Integrated bodyframe y flow in meters

float32 vx 				# Flow bodyframe x speed, m/s
float32 vy				# Flow bodyframe y Speed, m/s

float32 gyro_rad_s[3] 			# Flow module rotational speed, rad/s (filtered)
float32 gyro_bias[3]			# Flow module gyroscope biases
