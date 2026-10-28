# Rover Position Setpoint
#
# Position setpoint consumed by the rover position controllers (AckermannPosControl,
# DifferentialPosControl, MecanumPosControl), which turn it into rover_speed_setpoint
# and rover_attitude_setpoint.
# Published by the manual (position), auto and offboard drive modes of rover_ackermann,
# rover_differential and rover_mecanum, and can be sent from ROS 2 over uXRCE-DDS
# (/fmu/in/rover_position_setpoint).
# The rover follows the desired track from start_ned to target_ned. To control the course over ground
# instead, publish surface_vehicle_course_setpoint; the position controller follows whichever
# of the two was published most recently.

uint64 timestamp # [us] Time since system start
float32[2] target_ned # [m] [@range -inf, inf] [@frame NED] [@invalid NaN No target; the position controller stays idle] Target position
float32[2] start_ned # [m] [@range -inf, inf] [@frame NED] [@invalid NaN Defaults to the vehicle position when the setpoint is received] Start position of the desired track towards target_ned
float32 cruising_speed # [m/s] [@range -inf, inf] [@invalid NaN Defaults to maximum speed] Cruising speed; a negative value drives the rover in reverse (rear first) towards target_ned
float32 arrival_speed # [m/s] [@range 0, inf] [@invalid NaN Defaults to 0] Speed magnitude to arrive at the target with, independent of the direction of travel
float32 yaw # [rad] [@range -pi, pi] [@frame NED] [@invalid NaN Not considered] Currently unused. Only for vehicles that can move sideways (e.g. mecanum): vehicle yaw during travel.
