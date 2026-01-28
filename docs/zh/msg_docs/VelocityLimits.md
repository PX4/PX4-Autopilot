# 速度限制 (UORB 消息)

仅适用于多旋翼飞行器位置慢速模式的速度和偏航率限制

[源文件](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VelocityLimits.msg)

```c
# 多片段位置的速度和yaw 率限制仅限

uint64 时间戳 # 系统启动后的时间 (微秒)

# 绝对速度， NAN 表示使用默认限制
float32 水平速度 # [m/]
float32 vertical_速度 # [m/]
float32 yaw_rate # [rad/]
```
