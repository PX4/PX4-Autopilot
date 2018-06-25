#
# IMU integrated measurements in SI-unit form.
#

uint16 delta_angle_dt		# gyroscope measurement sampling period in us
float32[3] delta_angle		# integration of the gyroscope measurements in the XYZ body frame in radians

uint16 delta_velocity_dt	# accelerometer measurement sampling period in us
float32[3] delta_velocity	# integration of the accelerometer measurements in the XYZ body frame in m/s
