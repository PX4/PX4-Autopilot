# RoverRateStatus (UORB message)

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/RoverRateStatus.msg)

```c
uint64 timestamp # time since system start (microseconds)

float32 measured_yaw_rate          # [rad/s] Measured yaw rate
float32 adjusted_yaw_rate_setpoint # [rad/s] Yaw rate setpoint that is being tracked (Applied slew rates)
float32 pid_yaw_rate_integral      # Integral of the PID for the closed loop yaw rate controller

# TOPICS rover_rate_status

```
