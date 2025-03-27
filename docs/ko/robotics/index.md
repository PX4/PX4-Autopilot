# Drone Apps & APIs

Drone APIs let you write code to control and integrate with PX4-powered vehicles, without having to understand intimate details of the vehicle and flight stack, or having to think about safety-critical behaviour.

PX4는 <a href="https://mavsdk.mavlink.io/">MAVSDK</a> 및 <a href="../ros/README.md">ROS</a>를 포함하는 로봇 공학 API와 함께 사용할 수 있습니다.
Drone APIs allow you to do this using high level instructions in your programming language of choice, and the code can then run on-vehicle in a [companion computer](../companion_computer/index.md), or from a ground station.
Under the the hood the APIs communicate with PX4 using [MAVLink](../middleware/mavlink.md) or [uXRCE-DDS](../middleware/uxrce_dds.md).

PX4 supports the following SDKs/Robotics tools:

- [MAVSDK](../robotics/mavsdk.md)
- [ROS 2](../ros/index.md)
- [ROS 1](../ros/index.md)

## What API should I use?

We recommend using MAVSDK where possible, primarily because it is more intuitive and easier to learn, and can run on more operating systems and less performant-hardware.

You may prefer ROS if you already know how to use it, or if you want to leverage pre-existing integrations (for example computer vision tasks).
More generally, ROS is likely to be a better choice for tasks that require very low latency or a deeper integration with PX4 than is provided by MAVLink.

The main difference are:

- **MAVSDK:**
  - Intuitive and optimised for drones, with a small learning curve and easy setup.
  - You can write apps in C++, Python, Swift, Java, Go, and more.
  - Runs on resource-constrained hardware
  - Runs on broad range of OSs, including Android, Linux, Windows.
  - Communicates over MAVLink.
    - Stable and widely supported.
    - Limited to MAVLink services - needed information may not be exposed.
    - Latency may be too high for some use cases.
- **ROS 2**
  - General-purpose robotics API that has been extended to support drone integration:
    - Conceptually not as well optimised for drones as MAVSDK
    - Significant learning curve
  - Many pre-existing libraries: useful for code-reuse.
  - Supports C++ and Python libraries
  - Runs on Linux
  - ROS 2 is the latest version, which connects via XRCE-DDS.
    - DDS interface layer allows deep integration into any aspect of PX4 that is exposed as a UORB topic (almost everything).
    - Can provide much lower latency.
    - Still under development:
      - At time of writing requires a deeper understanding of PX4 than ROS 1.
      - Message interface with PX4 not stable/maintained across ROS and PX4 releases.

## Deprecated APIs

### ROS 1

While not strictly deprecated, ROS 1 is at its final LTS Release version "Noetic Ninjemys", which reaches end-of-life in May 2025.
That means no new features or bug fixes will be provided, and even security updates will cease in 2025.

ROS 1 still "works" on PX4 because it uses MAVROS, a MAVLINK-ROS abstraction as an integration layer.
This means that ROS 1 has all the limitations of MAVLink, such as a higher latency and a small API surface (and also the benefits, such as a stable interface).

All PX4 investment in ROS is going towards a deep integration with ROS 2.
This will essentially allow ROS 2 applications to be close to indistinguishable from code running in PX4 itself.

:::tip
Use ROS 2 for new projects.
Upgrade to ROS 2 for existing projects as soon as possible.
:::

### DroneKit PX4 통신

DroneKit-Python is a MAVLink API written in Python.
It is not optimised for use with PX4, and has not be maintained for some years.
Legacy docs for using PX4 and DroneKit can be found here: [PX4 v1.12 > DroneKit](https://docs.px4.io/v1.12/en/robotics/dronekit.html).

:::tip
[MAVSDK](https://mavsdk.mavlink.io/) is the recommended MAVLink API for use with PX4.
It is better in almost every way: features, speed, programming language support, maintenance, and so on.
:::
