# VelocityLimits (повідомлення UORB)

Обмеження швидкості та кутової швидкості для мультикоптера лише в режимі повільного переміщення позиції

[вихідний файл](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VelocityLimits.msg)

```c
# Velocity and yaw rate limits for a multicopter position slow mode only

uint64 timestamp # time since system start (microseconds)

# absolute speeds, NAN means use default limit
float32 horizontal_velocity # [m/s]
float32 vertical_velocity # [m/s]
float32 yaw_rate # [rad/s]

```
