# this file is only used in the position_setpoint triple as a dependency

uint64 timestamp		# time since system start (microseconds)

uint8 SETPOINT_TYPE_POSITION=0	# position setpoint
uint8 SETPOINT_TYPE_VELOCITY=1	# velocity setpoint
uint8 SETPOINT_TYPE_LOITER=2	# loiter setpoint
uint8 SETPOINT_TYPE_TAKEOFF=3	# takeoff setpoint
uint8 SETPOINT_TYPE_LAND=4	# land setpoint, altitude must be ignored, descend until landing
uint8 SETPOINT_TYPE_IDLE=5	# do nothing, switch off motors or keep at idle speed (MC)

uint8 LOITER_TYPE_ORBIT=0 	# Circular pattern
uint8 LOITER_TYPE_FIGUREEIGHT=1 # Pattern resembling an 8

bool valid			# true if setpoint is valid
uint8 type			# setpoint type to adjust behavior of position controller

float32 vx			# local velocity setpoint in m/s in NED
float32 vy			# local velocity setpoint in m/s in NED
float32 vz			# local velocity setpoint in m/s in NED

float64 lat			# latitude, in deg
float64 lon			# longitude, in deg
float32 alt			# altitude AMSL, in m
float32 yaw			# yaw (only in hover), in rad [-PI..PI), NaN = leave to flight task

float32 loiter_radius		# [m] [@range 0, INF] loiter major axis radius
float32 loiter_minor_radius	# [m] [@range 0, INF] loiter minor axis radius (used for non-circular loiter shapes)
bool loiter_direction_counter_clockwise # loiter direction is clockwise by default and can be changed using this field
float32 loiter_orientation 	# [rad] [@range -pi, pi] orientation of the major axis with respect to true north
uint8 	loiter_pattern		# loitern pattern to follow

float32 acceptance_radius   # horizontal acceptance_radius (meters)
float32 alt_acceptance_radius # vertical acceptance radius, only used for fixed wing guidance, NAN = let guidance choose (meters)

float32 cruising_speed		# the generally desired cruising speed (not a hard constraint)
bool gliding_enabled		# commands the vehicle to glide if the capability is available (fixed wing only)
float32 cruising_throttle	# the generally desired cruising throttle (not a hard constraint), only has an effect for rover
