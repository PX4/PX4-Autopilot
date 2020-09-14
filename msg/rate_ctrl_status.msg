uint64 timestamp		# time since system start (microseconds)

# rate controller integrator status
float32 rollspeed_integ
float32 pitchspeed_integ
float32 yawspeed_integ
float32 additional_integ1	# FW: wheel rate integrator (optional)

float32[3] error                # rate error [radians/s]
float32[3] error_integrated     # rate error integrated [radians]
