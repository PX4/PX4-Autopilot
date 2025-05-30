# 加速度计硬件与设置

PX4使用加速计数据进行速度估计。

你无需将加速度计作为独立的外部设备进行连接。

- 大多数飞行控制器，例如[Pixhawk系列](../flight_controller/pixhawk_series.md)中的飞行控制器，都将加速度计作为飞行控制器[惯性测量单元（IMU）](https://en.wikipedia.org/wiki/Inertial_measurement_unit)的一部分。
- 陀螺仪作为[外部惯性导航系统、姿态与航向参考系统或惯性导航增强型全球导航卫星系统](../sensor/inertial_navigation_systems.md)的一部分而存在。

在首次使用载具之前必须校准加速计：

- [加速度计校准](../config/accelerometer.md)
