# Computer Vision (Optical Flow, MoCap, VIO, Avoidance)

[Computer vision](https://en.wikipedia.org/wiki/Computer_vision) techniques enable computers to use visual data to make sense of their environment.

PX4 uses computer vision systems (primarily running on [Companion Computers](../companion_computer/index.md)) in order to support the following features:

- [Optical Flow](#optical-flow) provides 2D velocity estimation (using a downward facing camera and a downward facing distance sensor).
- [Motion Capture](#motion-capture) provides 3D pose estimation using a vision system that is _external_ to the vehicle.
  It is primarily used for indoor navigation.
- [Visual Inertial Odometry](#visual-inertial-odometry-vio) provides 3D pose and velocity estimation using an onboard vision system and IMU.
  It is used for navigation when GNSS position information is absent or unreliable.
- [Collision Prevention](../computer_vision/collision_prevention.md) is used to stop vehicles before they can crash into an obstacle (primarily when flying in manual modes).

:::tip
The [PX4 Vision Autonomy Development Kit](../complete_vehicles_mc/px4_vision_kit.md) (Holybro) is a robust and inexpensive kit for developers working with computer vision on PX4.
:::

## Motion Capture

Motion Capture (MoCap) is a technique for estimating the 3D _pose_ (position and orientation) of a vehicle using a positioning mechanism that is _external_ to the vehicle.
MoCap systems most commonly detect motion using infrared cameras, but other types of cameras, Lidar, or Ultra Wideband (UWB) may also be used.

::: info
MoCap is commonly used to navigate a vehicle in situations where GPS is absent (e.g. indoors), and provides position relative to a _local_ coordinate system.
:::

For information about MoCap see:

- [External Position Estimation](../ros/external_position_estimation.md)
- [Flying with Motion Capture (VICON, NOKOV, Optitrack)](../tutorials/motion-capture.md)
- [Using PX4's Navigation Filter (EKF2) > External Vision System](../advanced_config/tuning_the_ecl_ekf.md#external-vision-system)

## Visual Inertial Odometry (VIO)

Visual Inertial Odometry (VIO) is used for estimating the 3D _pose_ (position and orientation) and _velocity_ of a moving vehicle relative to a _local_ starting position.
It is commonly used to navigate a vehicle in situations where GPS is absent (e.g. indoors) or unreliable (e.g. when flying under a bridge).

VIO uses [Visual Odometry](https://en.wikipedia.org/wiki/Visual_odometry) to estimate vehicle _pose_ from visual information, combined with inertial measurements from an IMU (to correct for errors associated with rapid vehicle movement resulting in poor image capture).

::: info
One difference between VIO and [MoCap](#motion-capture) is that VIO cameras/IMU are vehicle-based, and additionally provide velocity information.
:::

For information about configuring VIO on PX4 see:

- [Using PX4's Navigation Filter (EKF2) > External Vision System](../advanced_config/tuning_the_ecl_ekf.md#external-vision-system)
- [T265 Setup guide](../peripherals/camera_t265_vio.md)

## Optical Flow

[Optical Flow](../sensor/optical_flow.md) provides 2D velocity estimation (using a downward facing camera and a downward facing distance sensor).

For information about optical flow see:

- [Optical Flow](../sensor/optical_flow.md)
- [Using PX4's Navigation Filter (EKF2) > Optical Flow](../advanced_config/tuning_the_ecl_ekf.md#optical-flow)

## Comparisons

### Optical Flow vs VIO for Local Position Estimation

Both these techniques use cameras and measure differences between frames.
Optical flow uses a downward facing camera, while VIO uses a stereo camera or a 45 degree tracking camera.
Assuming both are well calibrated, which is better for local position estimation?

The consensus [appears to be](https://discuss.px4.io/t/vio-vs-optical-flow/34680):

Optical flow:

- Downward facing optical flow gives you a planar velocity thats corrected for angular velocity with the gyro.
- Requires an accurate distance to the ground and assumes a planar surface.
  Given those conditions it can be just as accurate/reliable as VIO (such as indoor flight)
- Is more robust than VIO as it has fewer states.
- Is significantly cheaper and easier to set up as it only requires a flow sensor, a rangefinder, and setting up a few parameters (which can be connected to the flight controller).

VIO:

- Is more expensive to purchase and harder to set up.
  It requires a separate companion computer, calibration, software, configuration and so on.
- Will be less effective if there are no point features to track (in practice the real world generally has point features).
- Is more flexible, allowing additional features such as obstacle avoidance and mapping.

A combination (fusing both) is probably the most reliable, though not necessary in most real-world scenarios.
Normally you will select the system that suits your operating environment, required features, and cost constraints:

- Use VIO if you plan on flying outdoors without GPS (or outdoors and indoors), or if you need to support obstacle avoidance and other computer vision features.
- Use Optical Flow if you plan on only flying indoors (without GPS) and cost is an important consideration.

## External Resources

- [XTDrone](https://github.com/robin-shaun/XTDrone/blob/master/README.en.md) - ROS + PX4 simulation environment for computer vision.
  The [XTDrone Manual](https://www.yuque.com/xtdrone/manual_en) has everything you need to get started!
