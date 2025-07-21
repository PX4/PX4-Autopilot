# HomePosition (UORB message)

GPS home position in WGS84 coordinates.

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/versioned/HomePosition.msg)

```c
# GPS home position in WGS84 coordinates.

uint32 MESSAGE_VERSION = 0

uint64 timestamp			# time since system start (microseconds)

float64 lat				# Latitude in degrees
float64 lon				# Longitude in degrees
float32 alt				# Altitude in meters (AMSL)

float32 x				# X coordinate in meters
float32 y				# Y coordinate in meters
float32 z				# Z coordinate in meters

float32 yaw				# Yaw angle in radians

bool valid_alt		# true when the altitude has been set
bool valid_hpos		# true when the latitude and longitude have been set
bool valid_lpos		# true when the local position (xyz) has been set

bool manual_home	# true when home position was set manually

uint32 update_count 	# update counter of the home position

```
