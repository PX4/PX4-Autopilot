# Bezier Trajectory description. See also Mavlink TRAJECTORY msg

uint8 POINT_0 = 0
uint8 POINT_1 = 1
uint8 POINT_2 = 2
uint8 POINT_3 = 3
uint8 POINT_4 = 4

uint8 X = 0
uint8 Y = 1
uint8 Z = 2
uint8 TIME_HORIZON = 3
uint8 YAW = 4
uint8 YAW_SPEED = 5

uint8 MAV_TRAJECTORY_REPRESENTATION_BEZIER = 1

uint8 type # Type from MAV_TRAJECTORY_REPRESENTATION enum.

float32[6] point_0    # MAV_TRAJECTORY_REPRESENTATION_BEZIER, index 0-2 position wp, index 3 time horizon, index 4 yaw, index 5 yaw speed
float32[6] point_1    # MAV_TRAJECTORY_REPRESENTATION_BEZIER, index 0-2 position wp, index 3 time horizon, index 4 yaw, index 5 yaw speed
float32[6] point_2    # MAV_TRAJECTORY_REPRESENTATION_BEZIER, index 0-2 position wp, index 3 time horizon, index 4 yaw, index 5 yaw speed
float32[6] point_3    # MAV_TRAJECTORY_REPRESENTATION_BEZIER, index 0-2 position wp, index 3 time horizon, index 4 yaw, index 5 yaw speed
float32[6] point_4    # MAV_TRAJECTORY_REPRESENTATION_BEZIER, index 0-2 position wp, index 3 time horizon, index 4 yaw, index 5 yaw speed
bool[5] point_valid   # States if respective point is valid

# TOPICS trajectory_bezier trajectory_bezier_desired
