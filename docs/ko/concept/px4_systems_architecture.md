# PX4 아키텍쳐

아래 섹션에서는 "일반적인" PX4 시스템의 하드웨어와 소프트웨어 개요를 제공합니다. 하나는 비행 콘트롤러만 있는 것이고 다른 하나는 비행 컨트롤러와 보조 컴퓨터("미션 컴퓨터"라고도 함)가 있습니다.

:::info
The [PX4 Architectural Overview](../concept/architecture.md) provides information about the flight stack and middleware.
Offboard APIs are covered in [ROS](../ros/index.md) and [MAVSDK](https://mavsdk.mavlink.io/main/en/).
:::

## 비행 콘트롤러

아래 다이어그램은 비행 콘트롤러 기반 PX4 시스템 개요입니다.

![PX4 architecture - FC only system](../../assets/diagrams/px4_arch_fc.svg)

<!-- Source for drawing: https://docs.google.com/drawings/d/1_2n43WrbkWTs1kz0w0avVEeebJbfTj5SSqvCmvSOBdU/edit -->

하드웨어 구성

- [Flight controller](../flight_controller/index.md) (running the PX4 flight stack). 콘트롤러에는 대부분 내부 IMU, 나침반 및 기압계가 포함되어 있습니다.
- [Motor ESCs](../peripherals/esc_motors.md) connected to [PWM outputs](../peripherals/pwm_escs_and_servo.md), [DroneCAN](../dronecan/escs.md) (DroneCAN allows two-way communication, not single direction as shown) or some other bus.
- Sensors ([GPS](../gps_compass/index.md), [compass](../gps_compass/index.md), distance sensors, barometers, optical flow, barometers, ADSB transponders, etc.) connected via I2C, SPI, CAN, UART etc.
- [Camera](../camera/index.md) or other payload. 카메라는 PWM 출력에 연결하거나 MAVLink로 연결할 수 있습니다.
- [Telemetry radios](../telemetry/index.md) for connecting to a ground station computer/software.
- [RC Control System](../getting_started/rc_transmitter_receiver.md) for manual control

The left hand side of the diagram shows the software stack, which is horizontally aligned (approximately) with the hardware parts of the diagram.

- The ground station computer typically runs [QGroundControl](../getting_started/px4_basic_concepts.md#qgc) (or some other ground station software).
  It may also run robotics software like [MAVSDK](https://mavsdk.mavlink.io/) or [ROS](../ros/index.md).
- The PX4 flight stack running on the flight controller includes [drivers](../modules/modules_driver.md), [comms modules](../modules/modules_communication.md), [controllers](../modules/modules_controller.md), [estimators](../modules/modules_controller.md) and other [middleware and system modules](../modules/modules_main.md).

## 비행 콘트롤러와 보조 컴퓨터

아래 다이어그램은 비행 콘트롤러와 보조 컴퓨터(여기서는 "임무 컴퓨터"라고 함)를 포함하는 PX4 시스템을 나타냅니다.

![PX4 architecture - FC + Companion Computer](../../assets/diagrams/px4_arch_fc_companion.svg)

<!-- source for drawing: https://docs.google.com/drawings/d/1zFtvA_B-BmfmxFmAd-XIvAZ-jRqOydj0aBtqSolBcqI/edit -->

The flight controller runs the normal PX4 flight stack, while a companion computer provides advanced features that utilise [computer vision](../computer_vision/index.md).
The two systems are connected using a fast serial or IP link, and typically communicate using the [MAVLink protocol](https://mavlink.io/en/).
Communications with the ground stations and the cloud are usually routed via the companion computer (e.g. using the [MAVLink Router](https://github.com/mavlink-router/mavlink-router) (from Intel)).

PX4 systems typically run a Linux OS on the companion computer.
Linux는 NuttX보다 "일반" 소프트웨어 개발을 위한 플랫폼입니다. 많은 Linux 개발자와 유용한 소프트웨어가 이미 개발되어 있습니다(예: 컴퓨터 비전, 통신, 클라우드 통합, 하드웨어 드라이버용).
보조 컴퓨터는 때때로 같은 이유로 Android를 사용합니다.

:::info
The diagram shows a cloud or ground station connection via LTE, an approach that has been used a number of PX4-based systems.
PX4는 특별히 LTE와 클라우드 통합을 위한 소프트웨어를 제공하지 않습니다(사용자 맞춤형 개발이 필요함).
:::
