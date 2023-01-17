# IRLOCK_REPORT message data

uint64 timestamp		# time since system start (microseconds)

uint16 signature

# When looking along the optical axis of the camera, x points right, y points down, and z points along the optical axis.
float32 sensor_ray_n # Rotated tan(theta) in NED frame, where theta is the angle between the target and the camera center of projection in camera x-axis
float32 sensor_ray_e # Rotated tan(theta) in NED frame, where theta is the angle between the target and the camera center of projection in camera y-axis
float32 sensor_ray_d # sensory in Down direction
