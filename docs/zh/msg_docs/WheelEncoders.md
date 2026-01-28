# 滚轮编码器 (UORB 消息)

[源文件](https://github.com/PX4/PX4-Autopilot/blob/main/msg/WheelEncoders.msg)

```c
uint64 timestamp			#系统启动后的时间（微秒）

# Two wheels: 0 right, 1 left
float32[2] wheel_speed # [rad/s]
float32[2] wheel_angle # [rad]
```
