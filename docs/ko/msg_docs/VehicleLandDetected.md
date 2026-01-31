# VehicleLandDetected (UORB message)

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/versioned/VehicleLandDetected.msg)

```c
uint32 MESSAGE_VERSION = 0

uint64 timestamp	# time since system start (microseconds)

bool freefall		# true if vehicle is currently in free-fall
bool ground_contact	# true if vehicle has ground contact but is not landed (1. stage)
bool maybe_landed	# true if the vehicle might have landed (2. stage)
bool landed		# true if vehicle is currently landed on the ground (3. stage)

bool in_ground_effect # indicates if from the perspective of the landing detector the vehicle might be in ground effect (baro). This flag will become true if the vehicle is not moving horizontally and is descending (crude assumption that user is landing).
bool in_descend

bool has_low_throttle

bool vertical_movement
bool horizontal_movement
bool rotational_movement

bool close_to_ground_or_skipped_check

bool at_rest

```
