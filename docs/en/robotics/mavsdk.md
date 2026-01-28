# MAVSDK

[MAVSDK](https://mavsdk.mavlink.io/main/en/index.html) is a MAVLink SDK that allows you to communicate with MAVLink systems such as drones, cameras or ground systems.

The SDK provides interface libraries for various programming languages, with a simple API for managing one or more vehicles.
It provides programmatic access to vehicle information and telemetry, and control over missions, movement and other operations.
It can also be used to communicate with MAVLink components, such as cameras, gimbals, and other hardware.

MAVSDK libraries can be used onboard a drone on a companion computer, or on a ground station or mobile device, and can run on Linux, macOS, Windows, Android, and iOS.

::: info
MAVSDK is easier to learn than [ROS 2](../ros2/index.md) and has a more stable API.
It is recommended for communicating from ground stations, for relatively low bandwidth command/control from a companion computer, and for integrating with onboard components that do not have high bandwidth requirements, such as cameras and gimbals.

ROS is recommended for high-bandwidth onboard communication, to provide features such as obstacle avoidance that require higher rate messages and that can leverage existing ROS and computer vision libraries..
:::

The SDK can also be used to write MAVLink "server" code: the side of a MAVLink communication that is normally implemented by an autopilot, MAVLink camera, or other component.
This can be used to create a MAVLink API for components running on a companion computer, such as a camera API interface for a native camera connected to the computer.
