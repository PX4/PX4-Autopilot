# 计算机视觉 (光流，MoCap， VIO，避障)

[Computer vision](https://en.wikipedia.org/wiki/Computer_vision) techniques enable computers to use visual data to make sense of their environment.

PX4 uses computer vision systems (primarily running on [Companion Computers](../companion_computer/index.md)) in order to support the following features:

- [Optical Flow](#optical-flow) provides 2D velocity estimation (using a downward facing camera and a downward facing distance sensor).
- [Motion Capture](#motion-capture) provides 3D pose estimation using a vision system that is _external_ to the vehicle.
  它主要用于室内导航。
- [Visual Inertial Odometry](#visual-inertial-odometry-vio) provides 3D pose and velocity estimation using an onboard vision system and IMU.
  用于在 GNSS 位置信息不存在或不可靠时的导航。
- [Collision Prevention](../computer_vision/collision_prevention.md) is used to stop vehicles before they can crash into an obstacle (primarily when flying in manual modes).

:::tip
The [PX4 Vision Autonomy Development Kit](../complete_vehicles_mc/px4_vision_kit.md) (Holybro) is a robust and inexpensive kit for developers working with computer vision on PX4.
:::

## 运动捕捉

Motion Capture (MoCap) is a technique for estimating the 3D _pose_ (position and orientation) of a vehicle using a positioning mechanism that is _external_ to the vehicle.
MoCap 系统最常使用红外相机检测运动，但也可以使用其他类型的相机，激光雷达或者超宽带 （UWB）。

:::info
MoCap is commonly used to navigate a vehicle in situations where GPS is absent (e.g. indoors), and provides position relative to a _local_ coordinate system.
:::

有关 MoCap 的信息，请参阅：

- [External Position Estimation](../ros/external_position_estimation.md)
- [Flying with Motion Capture (VICON, NOKOV, Optitrack)](../tutorials/motion-capture.md)
- [Using PX4's Navigation Filter (EKF2) > External Vision System](../advanced_config/tuning_the_ecl_ekf.md#external-vision-system)

## Visual Inertial Odometry (VIO)

Visual Inertial Odometry (VIO) is used for estimating the 3D _pose_ (position and orientation) and _velocity_ of a moving vehicle relative to a _local_ starting position.
它通常用于在 GPS 不存在（例如室内）或不可靠的情况下（例如在桥下飞行时）给载具导航。

VIO uses [Visual Odometry](https://en.wikipedia.org/wiki/Visual_odometry) to estimate vehicle _pose_ from visual information, combined with inertial measurements from an IMU (to correct for errors associated with rapid vehicle movement resulting in poor image capture).

:::info
One difference between VIO and [MoCap](#motion-capture) is that VIO cameras/IMU are vehicle-based, and additionally provide velocity information.
:::

关于在 PX4 上配置 VIO 的信息，请参阅：

- [Using PX4's Navigation Filter (EKF2) > External Vision System](../advanced_config/tuning_the_ecl_ekf.md#external-vision-system)
- [T265 Setup guide](../peripherals/camera_t265_vio.md)

## 光流

[Optical Flow](../sensor/optical_flow.md) provides 2D velocity estimation (using a downward facing camera and a downward facing distance sensor).

有关光流的信息，请参阅：

- [Optical Flow](../sensor/optical_flow.md)
- [Using PX4's Navigation Filter (EKF2) > Optical Flow](../advanced_config/tuning_the_ecl_ekf.md#optical-flow)

## 比较

### 本地位置估计 光学流 对 VIO

这两种技术都使用照相机并测量帧之间的差异。
光学流使用向下照相机，而VIO则使用立体照相机或45度跟踪照相机。
假定两者的校准都很好，哪个对本地地位置估计更好？

The consensus [appears to be](https://discuss.px4.io/t/vio-vs-optical-flow/34680):

Optical flow:

- 向下光学流使得你能够通过陀螺仪的角速度来校正角平面速度。
- 需要准确的地面距离并假定地面为平面。
  在这种情况下，它可能与VIO一样准确可靠(例如室内飞行)
- 它比VIO更健壮，因为它的状态较少。
- 更便宜和更容易设置，因为它只需要一个流传感器，一个范围探测器。 并设置几个参数（可以连接到飞行控制器）。

VIO

- 购买更加昂贵，设置更加困难。
  它需要一台单独的配套计算机、校准、软件、配置等等。
- 如果没有可跟踪的点特征（实际上现实世界一般有点特征），效果将会减弱。
- 较为灵活，可以增加诸如避免障碍和制图等其他功能。

组合(两者兼用)可能是最可靠的，但在大多数现实世界的情景中并不必要。
通常您将选择适合您的运行环境、所需功能和成本限制的系统：

- 如果您打算在没有GPS的情况下在室外飞行（或室外和室内飞行），请使用 VIO 或者如果您需要支持避障碍和其他计算机视觉特性。
- 如果您只计划在室内飞行（不使用 GPS），且成本是一个重要的考虑因素，使用Optical Flow。

## 外部资源

- [XTDrone](https://github.com/robin-shaun/XTDrone/blob/master/README.en.md) - ROS + PX4 simulation environment for computer vision.
  The [XTDrone Manual](https://www.yuque.com/xtdrone/manual_en) has everything you need to get started!
