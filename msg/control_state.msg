# This is similar to the mavlink message CONTROL_SYSTEM_STATE, but for onboard use */

float32 x_acc			# X acceleration in body frame
float32 y_acc			# Y acceleration in body frame
float32 z_acc			# Z acceleration in body frame

float32 horz_acc_mag		# low pass filtered magnitude of the horizontal acceleration

float32 x_vel			# X velocity in body frame
float32 y_vel			# Y velocity in body frame
float32 z_vel			# Z velocity in body frame
