# 陀螺仪硬件和设置

PX4使用一个陀螺仪来估计载具的姿态（方向）。

你无需将陀螺仪作为独立的外部设备进行连接：

- 大多数飞行控制器，如 [Pixhawk Series](../flight_controller/pixhawk_series.md) 都将陀螺仪作为飞行控制器 [惯性测量单元（IMU）](https://en.wikipedia.org/wiki/Inertial_measurement_unit) 的一部分。
- 陀螺仪作为[外部惯性导航系统、姿态与航向参考系统或惯性导航增强型全球导航卫星系统](../sensor/inertial_navigation_systems.md)的一部分而存在。

在首次使用载具之前必须校准陀螺仪：

- [陀螺仪校准](../config/gyroscope.md)
