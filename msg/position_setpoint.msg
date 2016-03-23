# this file is only used in the position_setpoint triple as a dependency

uint8 SETPOINT_TYPE_POSITION=0	# position setpoint
uint8 SETPOINT_TYPE_VELOCITY=1	# velocity setpoint
uint8 SETPOINT_TYPE_LOITER=2	# loiter setpoint
uint8 SETPOINT_TYPE_TAKEOFF=3	# takeoff setpoint
uint8 SETPOINT_TYPE_LAND=4	# land setpoint, altitude must be ignored, descend until landing
uint8 SETPOINT_TYPE_IDLE=5	# do nothing, switch off motors or keep at idle speed (MC)
uint8 SETPOINT_TYPE_OFFBOARD=6 	# setpoint in NED frame (x, y, z, vx, vy, vz) set by offboard

bool valid			# true if setpoint is valid
uint8 type			# setpoint type to adjust behavior of position controller
float32 x			# local position setpoint in m in NED
float32 y			# local position setpoint in m in NED
float32 z			# local position setpoint in m in NED
bool position_valid		# true if local position setpoint valid
float32 vx			# local velocity setpoint in m/s in NED
float32 vy			# local velocity setpoint in m/s in NED
float32 vz			# local velocity setpoint in m/s in NED
bool velocity_valid		# true if local velocity setpoint valid
float64 lat			# latitude, in deg
float64 lon			# longitude, in deg
float32 alt			# altitude AMSL, in m
float32 yaw			# yaw (only for multirotors), in rad [-PI..PI), NaN = hold current yaw
bool yaw_valid			# true if yaw setpoint valid
bool disable_mc_yaw_control		# control yaw for mc (used for vtol weather-vane mode)
float32 yawspeed		# yawspeed (only for multirotors, in rad/s)
bool yawspeed_valid		# true if yawspeed setpoint valid
float32 loiter_radius		# loiter radius (only for fixed wing), in m
int8 loiter_direction		# loiter direction: 1 = CW, -1 = CCW
float32 pitch_min		# minimal pitch angle for fixed wing takeoff waypoints
float32 a_x			# acceleration x setpoint
float32 a_y			# acceleration y setpoint
float32 a_z			# acceleration z setpoint
bool acceleration_valid		# true if acceleration setpoint is valid/should be used
bool acceleration_is_force	# interprete acceleration as force
float32 acceptance_radius   # navigation acceptance_radius if we're doing waypoint navigation
float32 cruising_speed		# the generally desired cruising speed (not a hard constraint)
