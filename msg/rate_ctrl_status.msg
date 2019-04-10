uint64 timestamp		# time since system start (microseconds)

# rates used by the controller
float32 rollspeed		# Bias corrected angular velocity about X body axis in rad/s
float32 pitchspeed		# Bias corrected angular velocity about Y body axis in rad/s
float32 yawspeed		# Bias corrected angular velocity about Z body axis in rad/s

# rate controller integrator status
float32 rollspeed_integ
float32 pitchspeed_integ
float32 yawspeed_integ
float32 additional_integ1	# FW: wheel rate integrator (optional)
