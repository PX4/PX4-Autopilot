# RoverDifferentialStatus (UORB message)

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/RoverDifferentialStatus.msg)

```c
uint64 timestamp # time since system start (microseconds)

float32 measured_forward_speed	        # [m/s] Measured speed in body x direction. Forwards: positiv, Backwards: negativ
float32 adjusted_forward_speed_setpoint # [m/s] Speed setpoint after applying slew rate
float32 measured_yaw                    # [rad] Measured yaw
float32 adjusted_yaw_setpoint           # [rad] Yaw setpoint after applying slew rate
float32 clyaw_yaw_rate_setpoint         # [rad/s] Yaw rate setpoint output by the closed loop yaw controller
float32 measured_yaw_rate               # [rad/s] Measured yaw rate
float32 adjusted_yaw_rate_setpoint      # [rad/s] Yaw rate setpoint from the closed loop yaw controller
float32 pid_yaw_integral                # Integral of the PID for the closed loop yaw controller
float32 pid_yaw_rate_integral           # Integral of the PID for the closed loop yaw rate controller
float32 pid_throttle_integral           # Integral of the PID for the closed loop speed controller

# TOPICS rover_differential_status

```
