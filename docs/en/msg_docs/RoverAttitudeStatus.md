# RoverAttitudeStatus (UORB message)



[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/RoverAttitudeStatus.msg)

```c
uint64 timestamp # time since system start (microseconds)

float32 measured_yaw          # [rad/s] Measured yaw rate
float32 adjusted_yaw_setpoint # [rad/s] Yaw setpoint that is being tracked (Applied slew rates)

# TOPICS rover_attitude_status

```
