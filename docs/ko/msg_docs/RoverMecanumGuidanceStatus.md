# RoverMecanumGuidanceStatus (UORB message)

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/RoverMecanumGuidanceStatus.msg)

```c
uint64 timestamp # time since system start (microseconds)

float32 lookahead_distance # [m] Lookahead distance of pure the pursuit controller
float32 heading_error      # [rad] Heading error of the pure pursuit controller
float32 desired_speed      # [m/s] Desired velocity magnitude (speed)

# TOPICS rover_mecanum_guidance_status

```
