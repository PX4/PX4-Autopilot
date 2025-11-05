# Motion Capture (MoCap)

Motion Capture (MoCap) is a [computer vision](https://en.wikipedia.org/wiki/Computer_vision) technique for estimating the 3D _pose_ (position and orientation) of a vehicle using a positioning mechanism that is _external_ to the vehicle.
It is commonly used to navigate a vehicle in situations where GPS is absent (e.g. indoors), and provides position relative to a _local_ coordinate system.

_MoCap_ systems most commonly detect motion using infrared cameras, but other types of cameras, Lidar, or Ultra Wideband (UWB) may also be used.

:::info
_MoCap_ is conceptually similar to [Visual Inertial Odometry (VIO)](../computer_vision/visual_inertial_odometry.md).
主要区别在于，在 VIO 中，视觉系统运行在无人机上。 额外还需要利用无人机上 IMU 提供速度信息。
:::

## MoCap 资源

有关 MoCap 的信息，请参阅：

- [Using Vision or Motion Capture Systems for Position Estimation](../ros/external_position_estimation.md). <!-- bring across info into user guide? -->
- [Flying with Motion Capture (VICON, NOKOV, Optitrack)](../tutorials/motion-capture.md). <!-- bring across info into user guide? -->
- [Using PX4's Navigation Filter (EKF2) > External Vision System](../advanced_config/tuning_the_ecl_ekf.md#external-vision-system)
