# Waypoints Trajectory description. See also Mavlink TRAJECTORY msg

uint8 POINT_0 = 0
uint8 POINT_1 = 1
uint8 POINT_2 = 2
uint8 POINT_3 = 3
uint8 POINT_4 = 4

uint8 X = 0
uint8 Y = 1
uint8 Z = 2
uint8 VX = 3
uint8 VY = 4
uint8 VZ = 5
uint8 AX = 6
uint8 AY = 7
uint8 AZ = 8
uint8 YAW = 9
uint8 YAW_SPEED = 10

uint8 POINT_SIZE = 11
uint8 NUMBER_POINTS = 5

uint8 MAV_TRAJECTORY_REPRESENTATION_WAYPOINTS = 0

uint8 type # Type from MAV_TRAJECTORY_REPRESENTATION enum.

float32[11] point_0    # MAV_TRAJECTORY_REPRESENTATION_WAYPOINTS, index 0-2 position sp, index 3-5 velocity sp, index 6-8 acceleration sp, index 9 yaw, index 10 yaw speed
float32[11] point_1    # MAV_TRAJECTORY_REPRESENTATION_WAYPOINTS, index 0-2 position sp, index 3-5 velocity sp, index 6-8 acceleration sp, index 9 yaw, index 10 yaw speed
float32[11] point_2    # MAV_TRAJECTORY_REPRESENTATION_WAYPOINTS, index 0-2 position sp, index 3-5 velocity sp, index 6-8 acceleration sp, index 9 yaw, index 10 yaw speed
float32[11] point_3    # MAV_TRAJECTORY_REPRESENTATION_WAYPOINTS, index 0-2 position sp, index 3-5 velocity sp, index 6-8 acceleration sp, index 9 yaw, index 10 yaw speed
float32[11] point_4    # MAV_TRAJECTORY_REPRESENTATION_WAYPOINTS, index 0-2 position sp, index 3-5 velocity sp, index 6-8 acceleration sp, index 9 yaw, index 10 yaw speed

bool[5] point_valid   # States if respective point is valid

# TOPICS trajectory_waypoint trajectory_waypoint_desired
