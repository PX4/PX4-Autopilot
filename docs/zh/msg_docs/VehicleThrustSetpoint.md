# 载具推力设定点（UORB 消息）

[源文件](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VehicleThrustSetpoint.msg)

```c

uint64 timeestamp # 从系统启动起的时间(微秒)
uint64 timestamp_samp #此消息所基于的数据样本的时间戳（微秒）

float32[3] xyz # 沿机体 X、Y、Z 轴的推力设定点 [ - 1, 1] 

# TOPICS vehicle_thent_setpoint
# TOPICS vehicle_thentust_setpoint_virtual_fw vehicle_thent_setpoint_virtual_mc

```
