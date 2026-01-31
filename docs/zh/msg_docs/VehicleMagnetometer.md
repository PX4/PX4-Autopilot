# 载具磁力计器 (UORB 消息)

[源文件](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VehicleMagnetometer.msg)

```c

uint64 timestamp            # 原始数据的时间戳（微秒）

uint64 timestamp_sample     # 所选磁力计的唯一设备 ID

uint32 device_id            # 所选磁力计的唯一设备 ID

float32[3] magnetometer_ga  # 机体框架 FRD 下，XYZ 轴方向的磁场强度（单位：高斯）

uint8 calibration_count     # 校准变化计数器。每当校准发生变化时，该计数器单调递增。
```
