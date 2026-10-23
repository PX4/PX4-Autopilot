# UGV/USV Course Setpoint
#
# Course setpoint for unmanned ground and surface vehicles (UGVs such as rovers, USVs such as boats).
# Consumed by the position controllers of these vehicles (currently AckermannPosControl,
# DifferentialPosControl and MecanumPosControl), which turn it into speed and attitude
# setpoints.
# Published by the manual (position) drive modes of rover_ackermann, rover_differential
# and rover_mecanum, and can be sent from ROS 2 over uXRCE-DDS
# (/fmu/in/ugv_usv_course_setpoint).
# Holds a course over ground along the line through start_ned, correcting cross-track
# error back onto that line. Alternative to ugv_usv_position_setpoint: the controller
# follows whichever of the two was published most recently.

uint64 timestamp # [us] Time since system start
float32 course # [rad] [@range -pi, pi] [@frame NED] [@invalid NaN No course; the position controller stays idle] Course over ground to hold
float32[2] start_ned # [m] [@range -inf, inf] [@frame NED] [@invalid NaN Defaults to the vehicle position when a new course is received; kept while the same course is published] Point the course line passes through
float32 cruising_speed # [m/s] [@range -inf, inf] [@invalid NaN Defaults to maximum speed] Cruising speed; a negative value drives the vehicle in reverse (rear first) along the course
float32 yaw # [rad] [@range -pi, pi] [@frame NED] [@invalid NaN Defaults to vehicle yaw] Currently unused. Only for vehicles that can move sideways (e.g. mecanum rovers): vehicle yaw while holding the course
