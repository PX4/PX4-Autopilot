# Auxiliary control fields for fixed-wing runway takeoff/landing

# Passes information from the FixedWingModeManager to the FixedWingAttitudeController (wheel control) and FixedWingLandDetector (takeoff state)

uint64 timestamp # [us] time since system start

uint8 STATE_THROTTLE_RAMP = 0		# ramping up throttle
uint8 STATE_CLAMPED_TO_RUNWAY = 1	# clamped to runway, controlling yaw directly (wheel or rudder)
uint8 STATE_CLIMBOUT = 2		# climbout to safe height before navigation
uint8 STATE_FLYING = 3			# navigate freely

uint8 runway_takeoff_state		# Current state of runway takeoff state machine

bool wheel_steering_enabled		# Flag that enables the wheel steering.
float32 wheel_steering_nudging_rate	# [norm] [@range -1, 1] [FRD] Manual wheel nudging, added to controller output. NAN is interpreted as 0.
