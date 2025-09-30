# 加速度计硬件与设置

PX4使用加速计数据进行速度估计。

你无需将加速度计作为独立的外部设备进行连接。

- 大多数飞行控制器，例如[Pixhawk系列](../flight_controller/pixhawk_series.md)中的飞行控制器，都将加速度计作为飞行控制器[惯性测量单元（IMU）](https://en.wikipedia.org/wiki/Inertial_measurement_unit)的一部分。
- Gyroscopes are present as part of an [external INS, AHRS or INS-enhanced GNSS system](../sensor/inertial_navigation_systems.md).

在首次使用载具之前必须校准加速计：

- [加速度计校准](../config/accelerometer.md)
