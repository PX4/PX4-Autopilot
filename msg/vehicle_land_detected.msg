uint64 timestamp	# time since system start (microseconds)
bool landed		# true if vehicle is currently landed on the ground
bool freefall		# true if vehicle is currently in free-fall
bool ground_contact	# true if vehicle has ground contact but is not landed
bool maybe_landed	# true if the vehicle might have landed
float32 alt_max 	# maximum altitude in [m] that can be reached
bool in_ground_effect # indicates if from the perspective of the landing detector the vehicle might be in ground effect (baro). This flag will become true if the vehicle is not moving horizontally and is descending (crude assumption that user is landing).
