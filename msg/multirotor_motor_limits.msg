uint64 timestamp		# time since system start (microseconds)
uint16 saturation_status	# Integer bit mask indicating which axes in the control mixer are saturated

# 0 - True if the saturation status is valid
# 1 - True if any motor is saturated at the upper limit
# 2 - True if any motor is saturated at the lower limit
# 3 - True if a positive roll increment will increase motor saturation
# 4 - True if negative roll increment will increase motor saturation
# 5 - True if positive pitch increment will increase motor saturation
# 6 - True if negative pitch increment will increase motor saturation
# 7 - True if positive yaw increment will increase motor saturation
# 8 - True if negative yaw increment will increase motor saturation
# 9 - True if positive thrust increment will increase motor saturation
# 10 - True if negative thrust increment will increase motor saturation
