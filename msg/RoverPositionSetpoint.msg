# Rover Position Setpoint
#
# Position setpoint consumed by the rover position controllers (AckermannPosControl,
# DifferentialPosControl, MecanumPosControl), which turn it into rover_speed_setpoint
# and rover_attitude_setpoint.
# Published by the manual (position), auto and offboard drive modes of rover_ackermann,
# rover_differential and rover_mecanum, and can be sent from ROS 2 over uXRCE-DDS
# (/fmu/in/rover_position_setpoint).
# Either tracks a line from start_ned to target_ned, or holds a course along a line
# through start_ned when course is set.

uint64 timestamp # [us] Time since system start
float32[2] target_ned # [m] [@range -inf, inf] [@frame NED] [@invalid NaN No target; the position controller stays idle unless a course is set] Target position, ignored while a course is set
float32[2] start_ned # [m] [@range -inf, inf] [@frame NED] [@invalid NaN Defaults to the vehicle position when the setpoint is received] Start position of the line the rover tracks, towards target_ned or along course
float32 cruising_speed # [m/s] [@range -inf, inf] [@invalid NaN Defaults to maximum speed] Cruising speed; a negative value drives the rover in reverse (rear first) towards target_ned or along course
float32 arrival_speed # [m/s] [@range 0, inf] [@invalid NaN Defaults to 0] Speed magnitude to arrive at the target with, independent of the direction of travel; ignored while a course is set
float32 yaw # [rad] [@range -pi, pi] [@frame NED] [@invalid NaN Defaults to vehicle yaw] Only for vehicles that can move sideways (e.g. mecanum): vehicle yaw during travel
float32 course # [rad] [@range -pi, pi] [@frame NED] [@invalid NaN No course, track target_ned instead] Course over ground to hold along the line through start_ned; takes precedence over target_ned. A course of 0 (north) is valid, so publishers must set NaN when not holding a course.
