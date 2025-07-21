# PX4 架构

The sections below provide high-level overview of the PX4 hardware and software stack for two "typical" PX4 systems; one that has just a flight controller, and another that has a flight controller and a companion computer (also known as a "mission computer").

:::info
The [PX4 Architectural Overview](../concept/architecture.md) provides information about the flight stack and middleware.
Offboard APIs are covered in [ROS](../ros/index.md) and [MAVSDK](https://mavsdk.mavlink.io/main/en/).
:::

## Flight Controller (only)

The diagram below provides a high level overview of a typical "simple" PX4 system based around a flight controller.

![PX4 architecture - FC only system](../../assets/diagrams/px4_arch_fc.svg)

<!-- Source for drawing: https://docs.google.com/drawings/d/1_2n43WrbkWTs1kz0w0avVEeebJbfTj5SSqvCmvSOBdU/edit -->

The hardware consists of

- [Flight controller](../flight_controller/index.md) (running the PX4 flight stack). This often includes internal IMUs, compass and barometer.
- [Motor ESCs](../peripherals/esc_motors.md) connected to [PWM outputs](../peripherals/pwm_escs_and_servo.md), [DroneCAN](../dronecan/escs.md) (DroneCAN allows two-way communication, not single direction as shown) or some other bus.
- Sensors ([GPS](../gps_compass/index.md), [compass](../gps_compass/index.md), distance sensors, barometers, optical flow, barometers, ADSB transponders, etc.) connected via I2C, SPI, CAN, UART etc.
- [Camera](../camera/index.md) or other payload. Cameras can be connected to PWM outputs or via MAVLink.
- [Telemetry radios](../telemetry/index.md) for connecting to a ground station computer/software.
- [RC Control System](../getting_started/rc_transmitter_receiver.md) for manual control

The left hand side of the diagram shows the software stack, which is horizontally aligned (approximately) with the hardware parts of the diagram.

- The ground station computer typically runs [QGroundControl](../getting_started/px4_basic_concepts.md#qgc) (or some other ground station software).
  It may also run robotics software like [MAVSDK](https://mavsdk.mavlink.io/) or [ROS](../ros/index.md).
- The PX4 flight stack running on the flight controller includes [drivers](../modules/modules_driver.md), [comms modules](../modules/modules_communication.md), [controllers](../modules/modules_controller.md), [estimators](../modules/modules_controller.md) and other [middleware and system modules](../modules/modules_main.md).

## FC and Companion Computer

The diagram below shows a PX4 system that includes both a flight controller and a companion computer (here referred to as a "mission computer").

![PX4 architecture - FC + Companion Computer](../../assets/diagrams/px4_arch_fc_companion.svg)

<!-- source for drawing: https://docs.google.com/drawings/d/1zFtvA_B-BmfmxFmAd-XIvAZ-jRqOydj0aBtqSolBcqI/edit -->

The flight controller runs the normal PX4 flight stack, while a companion computer provides advanced features that utilise [computer vision](../computer_vision/index.md).
The two systems are connected using a fast serial or IP link, and typically communicate using the [MAVLink protocol](https://mavlink.io/en/).
Communications with the ground stations and the cloud are usually routed via the companion computer (e.g. using the [MAVLink Router](https://github.com/mavlink-router/mavlink-router) (from Intel)).

PX4 systems typically run a Linux OS on the companion computer.
Linux is a much better platform for "general" software development than NuttX; there are many more Linux developers and a lot of useful software has already been written (e.g. for computer vision, communications, cloud integrations, hardware drivers).
Companion computers sometimes run Android for the same reason.

:::info
The diagram shows a cloud or ground station connection via LTE, an approach that has been used a number of PX4-based systems.
PX4 does not deliver software specifically for LTE and/or cloud integration (this requires custom development).
:::
