# LANDING_TARGET msg data
# The location of a landing area captured from a camera

uint64 timestamp_boot		# time of this estimate, in microseconds since system start
uint64 timestamp_computer	# timestamp provided by the companion computer, in us

uint8 target_num 	# The ID of the target if multiple targets are present

uint8 frame		# MAV_FRAME enum specifying the whether the following feilds are earth-frame, body-frame, etc.
uint8 MAV_FRAME_GLOBAL = 0			# Global coordinate frame, WGS84 coordinate system.
uint8 MAV_FRAME_LOCAL_NED = 1			# Local coordinate frame, Z-up (x: north, y: east, z: down).
uint8 MAV_FRAME_MISSION = 2			# NOT a coordinate frame, indicates a mission command.
uint8 MAV_FRAME_GLOBAL_RELATIVE_ALT = 3	# Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home position.
uint8 MAV_FRAME_LOCAL_ENU = 4			# Local coordinate frame, Z-down (x: east, y: north, z: up)
uint8 MAV_FRAME_GLOBAL_INT = 5			# Global coordinate frame, WGS84 coordinate system.
uint8 MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6	# Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home position.
uint8 MAV_FRAME_LOCAL_OFFSET_NED = 7		# Offset to the current local frame. Anything expressed in this frame should be added to the current local frame position.
uint8 MAV_FRAME_BODY_NED = 8			# Setpoint in body NED frame. This makes sense if all position control is externalized - e.g. useful to command 2 m/s^2 acceleration to the right.
uint8 MAV_FRAME_BODY_OFFSET_NED = 9		# Offset in body NED frame. This makes sense if adding setpoints to the current flight path, to avoid an obstacle - e.g. useful to command 2 m/s^2 acceleration to the east.
uint8 MAV_FRAME_GLOBAL_TERRAIN_ALT = 10	# Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude over terrain with respect to the waypoint coordinate.
uint8 MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11	# Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude over terrain with respect to the waypoint coordinate.

float32 angle_x	# X-axis angular offset - phi in spherical coordinates - , in radians, of the target from the center of the image
float32 angle_y	# Y-axis angular offset - theta in spherical coordinates - , in radians, of the target from the center of the image

float32 distance 	# Distance to the target from the vehicle in meters

float32 size_x		# Size in radians of target along x-axis
float32 size_y		# Size in radians of target along y-axis
