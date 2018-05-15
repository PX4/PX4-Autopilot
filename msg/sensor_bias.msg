#
# Sensor readings and in-run biases in SI-unit form. Sensor readings are compensated for static offsets,
# scale errors, in-run bias and thermal drift (if thermal compensation is enabled and available).
#

float32 gyro_x		# Bias corrected angular velocity about X body axis (roll) (in rad/s)
float32 gyro_y		# Bias corrected angular velocity about Y body axis (pitch) (in rad/s)
float32 gyro_z		# Bias corrected angular velocity about Z body axis (yaw) (in rad/s)

float32 accel_x		# Bias corrected acceleration in body X axis (in rad/s)
float32 accel_y		# Bias corrected acceleration in body Y axis (in rad/s)
float32 accel_z		# Bias corrected acceleration in body Z axis (in rad/s)

float32 mag_x		# Bias corrected magnetometer reading in body X axis (in Gauss)
float32 mag_y		# Bias corrected magnetometer reading in body Y axis (in Gauss)
float32 mag_z		# Bias corrected magnetometer reading in body Z axis (in Gauss)

# In-run bias estimates (subtract from uncorrected data)

float32 gyro_x_bias		# X gyroscope in-run bias in body frame (rad/s, x forward)
float32 gyro_y_bias		# Y gyroscope in-run bias in body frame (rad/s, y right)
float32 gyro_z_bias		# Z gyroscope in-run bias in body frame (rad/s, z down)

float32 accel_x_bias	# X accelerometer in-run bias in body frame (m/s^2, x forward)
float32 accel_y_bias	# Y accelerometer in-run bias in body frame (m/s^2, y right)
float32 accel_z_bias	# Z accelerometer in-run bias in body frame (m/s^2, z down)

float32 mag_x_bias		# X magnetometer in-run bias in body frame (Gauss, x forward)
float32 mag_y_bias		# Y magnetometer in-run bias in body frame (Gauss, y right)
float32 mag_z_bias		# Z magnetometer in-run bias in body frame (Gauss, z down)