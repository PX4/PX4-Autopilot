# MAVLink Microservices (Protocols)

MAVLink "microservices" are a protocols that use multiple messages exchanged between components to communicate more complicated information.
For example, the [Command Protocol](https://mavlink.io/en/services/command.html) provides an efficient mechanism for packaging a command in a (particular) message and receiving acknowledgement of the command in another message.

MAVLink microservices are documented the [MAVLink Guide](https://mavlink.io/en/services/) (this is not exhaustive: not all messages are grouped into protocols and not all protocols are documented).

This section lists the services known to be supported/not supported by PX4 in this version.

## Supported Microservices

These services are known to be supported in some form:

- [Battery Protocol](https://mavlink.io/en/services/battery.html)
  - [BATTERY_STATUS](https://mavlink.io/en/messages/common.html#BATTERY_STATUS) and [BATTERY_INFO](https://mavlink.io/en/messages/common.html#BATTERY_STATUS) are streamed.
- Camera Protocols
  - [Camera Protocol v2](https://mavlink.io/en/services/camera.html)
    - [Camera Definition](https://mavlink.io/en/services/camera_def.html)
- [Command Protocol](https://mavlink.io/en/services/command.html)
- [Component Metadata Protocol](https://mavlink.io/en/services/component_information.html)
- [Events Interface](https://mavlink.io/en/services/events.html)
- [File Transfer Protocol (FTP)](https://mavlink.io/en/services/ftp.html)
- Gimbal Protocols
  - [Gimbal Protocol v2](https://mavlink.io/en/services/gimbal_v2.html)
    - Can be enabled by [Gimbal Configuration](../advanced/gimbal_control.md#mavlink-gimbal-mnt-mode-out-mavlink)
    - PX4 an act as a MAVLink Gimbal for one FC-connected Gimbal
- [Heartbeat/Connection Protocol](https://mavlink.io/en/services/heartbeat.html)
- [High Latency Protocol](https://mavlink.io/en/services/high_latency.html) — PX4 streams [HIGH_LATENCY2](https://mavlink.io/en/messages/common.html#HIGH_LATENCY2)
- [Image Transmission Protocol](https://mavlink.io/en/services/image_transmission.html)
- [Landing Target Protocol](https://mavlink.io/en/services/landing_target.html)
- [Manual Control (Joystick) Protocol](https://mavlink.io/en/services/manual_control.html)
- [MAVLink Id Assignment (sysid, compid)](https://mavlink.io/en/services/mavlink_id_assignment.html)
- [Mission Protocol](https://mavlink.io/en/services/mission.html)
- [Offboard Control Protocol](https://mavlink.io/en/services/offboard_control.html)
- [Remote ID](../peripherals/remote_id.md) ([Open Drone ID Protocol](https://mavlink.io/en/services/opendroneid.html))
- [Parameter Protocol](https://mavlink.io/en/services/parameter.html)
- [Parameter Protocol Extended](https://mavlink.io/en/services/parameter_ext.html) — Allows setting string parameters. Used for setting string parameters set in camera definition files.
- [Payload Protocol](https://mavlink.io/en/services/payload.html)
- [Ping Protocol](https://mavlink.io/en/services/ping.html)
- [Standard Modes Protocol](../mavlink/standard_modes.md)
- [Terrain Protocol](https://mavlink.io/en/services/terrain.html)
- [Time Synchronization](https://mavlink.io/en/services/timesync.html)
- [Traffic Management (UTM/ADS-B)](https://mavlink.io/en/services/traffic_management.html)
- [Arm Authorization Protocol](https://mavlink.io/en/services/arm_authorization.html)

## Unsupported

These services are not supported/used by PX4:

- [Illuminator Protocol](https://mavlink.io/en/services/illuminator.html)
- [Tunnel Protocol](https://mavlink.io/en/services/tunnel.html)
