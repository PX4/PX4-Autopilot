# Auxiliary control fields for fixed-wing runway takeoff/landing

# Passes information from the FixedWingModeManager to the FixedWingAttitudeController

uint64 timestamp # [us] time since system start

bool wheel_steering_enabled		# Flag that enables the wheel steering.
float32 wheel_steering_nudging_rate	# [norm] [@range -1, 1] [FRD] Manual wheel nudging, added to controller output. NAN is interpreted as 0.
