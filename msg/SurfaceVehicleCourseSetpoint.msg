# Surface Vehicle Course Setpoint
#
# Course setpoint for surface vehicles, on land or on water (rovers, boats).
# Consumed by the position controllers of these vehicles (currently AckermannPosControl,
# DifferentialPosControl and MecanumPosControl), which turn it into speed and attitude
# setpoints.
# Published by the Guided Course mode of rover_ackermann, rover_differential and rover_mecanum,
# and can be sent from ROS 2 over uXRCE-DDS (/fmu/in/surface_vehicle_course_setpoint).
# The vehicle tries to track the course over ground, as with the fixed-wing course hold: vehicles
# that steer by heading correct their heading by the estimated drift angle, mecanum rovers drive
# along the course setpoint directly. There is no desired track, so a sideways offset is not
# corrected. To follow a desired track, publish rover_position_setpoint instead.
# The position controller follows whichever of the two was published most recently.

uint64 timestamp # [us] Time since system start
float32 course # [rad] [@range -pi, pi] [@frame NED] [@invalid NaN No course; the position controller stays idle] Course over ground to hold
float32 speed # [m/s] [@range -inf, inf] [@invalid NaN Defaults to maximum speed] Cruising speed; a negative value drives the vehicle in reverse (rear first) along the course
float32 yaw # [rad] [@range -pi, pi] [@frame NED] [@invalid NaN Not considered] Currently unused. Only for vehicles that can move sideways (e.g. mecanum rovers): vehicle yaw while holding the course.
