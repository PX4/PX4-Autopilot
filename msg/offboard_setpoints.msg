# offboard message

uint8 TYPE_TAKEOFF = 0	# auto takeoff is requested
uint8 TYPE_LAND = 1 # auto land is requested
uint8 TYPE_LOITER = 2 # auto loiter is requested
uint8 TYPE_IDLE = 3 # idle is requested
uint8 TYPE_STREAM = 4 # continuous offboard setpoint stream
uint8 type

uint8 velocity_frame
px4/vehicle_local_position_setpoint setpoint

# TOPICS offboard_setpoints