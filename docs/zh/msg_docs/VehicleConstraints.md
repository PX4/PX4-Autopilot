# 载具限制 (UORB 消息)

Local setpoint constraints in NED frame
setting something to NaN means that no limit is provided

[源文件](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VehicleConstraints.msg)

```c
# 本地设定点在东北天（NED）坐标系中的约束条件
# 将某个值设为 NaN 意味着未设置限制

uint64 timestamp # 自系统启动以来的时间（微秒）

float32 speed_up # 上升速度（米 / 秒）
float32 speed_down # 下降速度（米 / 秒）

bool want_takeoff # 告知控制器在怠速时启动起飞（飞行过程中此指令被忽略）
```
