# Vehicle Waypoints Trajectory description. See also MAVLink MAV_TRAJECTORY_REPRESENTATION msg
# The topic vehicle_trajectory_waypoint_desired is used to send the user desired waypoints from the position controller to the companion computer / avoidance module.
# The topic vehicle_trajectory_waypoint is used to send the adjusted waypoints from the companion computer / avoidance module to the position controller.

uint64 timestamp		# time since system start (microseconds)

uint8 MAV_TRAJECTORY_REPRESENTATION_WAYPOINTS = 0

uint8 type # Type from MAV_TRAJECTORY_REPRESENTATION enum.

uint8 POINT_0 = 0
uint8 POINT_1 = 1
uint8 POINT_2 = 2
uint8 POINT_3 = 3
uint8 POINT_4 = 4

uint8 NUMBER_POINTS = 5

px4/TrajectoryWaypoint[5] waypoints

# TOPICS vehicle_trajectory_waypoint vehicle_trajectory_waypoint_desired
