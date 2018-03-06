# Output of a external avoidance. Obstacle avoidance message with trajectory points, PCA and FOV. See also Mavlink OBSTACLE_AVOIDANCE msg

uint8 MAV_TRAJECTORY_REPRESENTATION_WAYPOINTS = 0
uint8 MAV_TRAJECTORY_REPRESENTATION_BEZIER = 1

uint8 type # Type from MAV_TRAJECTORY_REPRESENTATION enum.

float32[11] point_1    # Depending on the type (see MAV_TRAJECTORY_REPRESENTATION)
float32[11] point_2    # Depending on the type (see MAV_TRAJECTORY_REPRESENTATION)
float32[11] point_3    # Depending on the type (see MAV_TRAJECTORY_REPRESENTATION)
float32[11] point_4    # Depending on the type (see MAV_TRAJECTORY_REPRESENTATION)
float32[11] point_5    # Depending on the type (see MAV_TRAJECTORY_REPRESENTATION)

bool[5] point_valid   # States if respective point is valid

int16[4] field_of_view    # Field of View of the sensor/s being used. The first two parameters are used to indicate the direction (Azimuth: 0 - 360 degrees and elevation: -90 - 90 degrees) and the last two indicate the range (Azimuth and elevation). The range is applied symmetrically in the direction. Set first element to -1 to mark as unknown.
