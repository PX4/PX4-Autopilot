# Computer Vision (Optical Flow, MoCap, VIO, Avoidance)

[Computer vision](https://en.wikipedia.org/wiki/Computer_vision) techniques enable computers to use visual data to make sense of their environment.

PX4 uses computer vision systems (primarily running on [Companion Computers](../companion_computer/index.md)) in order to support the following features:

- Pose/Velocity Estimation:
  - [Optical Flow](../sensor/optical_flow.md) provides 2D velocity estimation (using a downward facing camera and a downward facing distance sensor).
  - [Motion Capture](../computer_vision/motion_capture.md) provides 3D pose estimation using a vision system that is _external_ to the vehicle.
    It is primarily used for indoor navigation.
  - [Visual Inertial Odometry (VIO)](../computer_vision/visual_inertial_odometry.md) provides 3D pose and velocity estimation using an onboard vision system and IMU.
    It is used for navigation when global position information is absent or unreliable.
- Avoidance/Path Planning:
  - [Collision Prevention](../computer_vision/collision_prevention.md) is used to stop MC vehicles before they can crash into an obstacle (primarily when flying in manual modes).

:::tip
The [PX4 Vision Autonomy Development Kit](../complete_vehicles_mc/px4_vision_kit.md) (Holybro) is a robust and inexpensive kit for developers working with computer vision on PX4.
:::

## External Resources

- [XTDrone](https://github.com/robin-shaun/XTDrone/blob/master/README.en.md) - ROS + PX4 v1.9 simulation environment for computer vision.
  The [XTDrone Manual](https://www.yuque.com/xtdrone/manual_en) has everything you need to get started!
