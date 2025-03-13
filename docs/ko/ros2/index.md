# ROS 2

[ROS 2](https://docs.ros.org/en/humble/#) is a powerful general purpose robotics library that can be used with the PX4 Autopilot to create powerful drone applications.

:::warning
Tip
The PX4 development team highly recommend that you use/migrate to this version of ROS!

This is the newest version of [ROS](http://www.ros.org/) (Robot Operating System).
It significantly improves on ROS "1", and in particular allows a much deeper and lower-latency integration with PX4.
:::

ROS는 일반적인 로봇 공학 문제를 해결하고, Linux용으로 작성된 소프트웨어 라이브러리에 대한 액세스를 해결하는 개발자의 활발한 생태계 시스템의 이점이 있습니다.
It can be used, for example, for [computer vision](../computer_vision/index.md) solutions.

ROS 2 enables a very deep integration with PX4, to the extent that you can create flight modes in ROS 2 that are indistinguisable from internal PX4 modes, and directly read from and write to internal uORB topics at high rate.
It is recommended (in particular) for control and communication from a companion computer where low latency is important, when leveraging existing libraries from Linux, or when writing new high level flight modes.

Communication between ROS 2 and PX4 uses middleware that implements the [XRCE-DDS protocol](../middleware/uxrce_dds.md).
This middleware exposes PX4 [uORB messages](../msg_docs/index.md) as ROS 2 messages and types, effectively allowing direct access to PX4 from ROS 2 workflows and nodes.
The middleware uses uORB message definitions to generate code to serialise and deserialise the messages heading in and out of PX4.
These same message definitions are used in ROS 2 applications to allow the messages to be interpreted.

:::info
ROS 2 can also connect with PX4 using [MAVROS](https://github.com/mavlink/mavros/tree/ros2/mavros) instead of XRCE-DDS.
This option is supported by the MAVROS project (it is not documented here).
:::

To use the [ROS 2](../ros2/user_guide.md) over XRCE-DDS effectively, you must (at time of writing) have a reasonable understanding of the PX4 internal architecture and conventions, which differ from those used by ROS.
In the near term future we plan to provide ROS 2 APIs to abstract PX4 conventions, along with examples demonstrating their use.

## Topics

이 섹션의 주요 주제는 다음과 같습니다.

- [ROS 2 User Guide](../ros2/user_guide.md): A PX4-centric overview of ROS 2, covering installation, setup, and how to build ROS 2 applications that communicate with PX4.
- [ROS 2 Offboard Control Example](../ros2/offboard_control.md): A C++ tutorial examples showing how to do position control in [offboard mode](../flight_modes/offboard.md) from a ROS 2 node.
- [ROS 2 Multi Vehicle Simulation](../ros2/multi_vehicle.md): Instructions for connecting to multipole PX4 simulations via single ROS 2 agent.
- [PX4 ROS 2 Interface Library](../ros2/px4_ros2_interface_lib.md): A C++ library that simplies interacting with PX4 from ROS 2.
  Can be used to create and register flight modes wrtten using ROS2 and send position estimates from ROS2 applications such as a VIO system.
- [ROS 2 Message Translation Node](../ros2/px4_ros2_msg_translation_node.md): A ROS 2 message translation node that enables communcation between PX4 and ROS 2 applications that were compiled with different sets of messages versions.

## 추가 정보

- [XRCE-DDS (PX4-ROS 2/DDS Bridge)](../middleware/uxrce_dds.md): PX4 middleware for connecting to ROS 2.
