# RoverAckermannGuidanceStatus (UORB message)

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/RoverAckermannGuidanceStatus.msg)

```c
uint64 timestamp # time since system start (microseconds)

float32 lookahead_distance 	# [m] Lookahead distance of pure the pursuit controller
float32 heading_error 		# [deg] Heading error of the pure pursuit controller

# TOPICS rover_ackermann_guidance_status

```
