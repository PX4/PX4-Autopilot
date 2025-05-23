# DifferentialDriveSetpoint (UORB message)

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/DifferentialDriveSetpoint.msg)

```c
uint64 timestamp # time since system start (microseconds)

float32 speed # [m/s] collective roll-off speed in body x-axis
bool closed_loop_speed_control # true if speed is controlled using estimator feedback, false if direct feed-forward
float32 yaw_rate # [rad/s] yaw rate
bool closed_loop_yaw_rate_control # true if yaw rate is controlled using gyroscope feedback, false if direct feed-forward

# TOPICS differential_drive_setpoint differential_drive_control_output

```
