# MAVLink Cameras (Camera Protocol v2)

本节说明了如何使用 PX4 的 MAVLink [相机](../camera/index.md) 实现了 [MAVLink Camera Protocol v2](https://mavlink.io/en/services/camera.html) 的 PX4 和地面站。

:::tip
这是与 PX4 集成相机的推荐方式！
:::

## 综述

The [MAVLink Camera Protocol v2](https://mavlink.io/en/services/camera.html) allows querying of what features are supported by a camera, and provides commands to control image and video capture, stream video, set zoom and focus, select between infrared and visible light feeds, set where captured data is saved, and so on.

A camera may implement the protocol natively, but most MAVLink camera setups involve PX4 communicating with a [camera manager](#camera-managers) running on a companion computer, which then interfaces between MAVLink and the camera's native protocol.

Generally speaking PX4's "integration" with a camera is to re-emit camera commands found in missions using the command protocol.
Otherwise it may act as a bridge, forwarding commands between a ground station and the camera if there is no direct MAVLink channel.

:::info
PX4 does not support using [MAVLink Camera Protocol v2](https://mavlink.io/en/services/camera.html) commands to control cameras that are attached to flight controller outputs.
While this is technically possible, it would require PX4 to implement a camera manager interface.
:::

## 控制相机

### MAVLink 命令和消息

Cameras are discovered using the MAVLink [connection protocol](https://mavlink.io/en/services/heartbeat.html), based on their [HEARTBEAT.type](https://mavlink.io/en/messages/common.html#HEARTBEAT) being set to [MAV_TYPE_CAMERA](https://mavlink.io/en/messages/common.html#MAV_TYPE_CAMERA).

:::tip
Cameras should also use a component ID in the recommended range, such as [MAV_COMP_ID_CAMERA](https://mavlink.io/en/messages/common.html#MAV_COMP_ID_CAMERA), but generally this cannot be relied on to verify that a MAVLink component is a camera.
:::

Once a camera is discovered its properties and capabilities can be queried by using [MAV_CMD_REQUEST_MESSAGE](https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_MESSAGE) to request the [CAMERA_INFORMATION](https://mavlink.io/en/messages/common.html#CAMERA_INFORMATION) message, and then inspecting the `flags` field to determine what standard features in [CAMERA_CAP_FLAGS](https://mavlink.io/en/messages/common.html#CAMERA_CAP_FLAGS) are supported.

Based on the flags, you can determine what other commands and messages are supported by the camera.
The full set of messages, commands, and enums are [summarised here](https://mavlink.io/en/services/camera.html#messagecommandenum-summary).

Additional parameters of a camera _may_ be exposed in a [camera definition file](https://mavlink.io/en/services/camera_def.html) that is linked from `CAMERA_INFORMATION.cam_definition_uri`.
A GCS or SDK can expose these settings though a generic UI, without having to understand any context.
These parameters cannot directly be set in missions and have no specific setter commands.

[MAVLink Camera Protocol v2](https://mavlink.io/en/services/camera.html) describes all the interactions in more detail.

### 地面站和 MAVLink SDK

地面站和 MAVLink SDK 会像前一节所述那样发现相机及其功能。

A ground station can use any feature exposed by the camera.
PX4 has no role in this interaction other than forwarding MAVLink traffic between the camera and ground station or SDK, if needed.

### Camera Commands in Missions

PX4 allows the following subset of [Camera Protocol v2](https://mavlink.io/en/services/camera.html) commands in missions:

- [MAV_CMD_IMAGE_START_CAPTURE](https://mavlink.io/en/messages/common.html#MAV_CMD_IMAGE_START_CAPTURE)
- [MAV_CMD_IMAGE_STOP_CAPTURE](https://mavlink.io/en/messages/common.html#MMAV_CMD_IMAGE_STOP_CAPTURE)
- [MAV_CMD_VIDEO_START_CAPTURE](https://mavlink.io/en/messages/common.html#MAV_CMD_VIDEO_START_CAPTURE)
- [MAV_CMD_VIDEO_STOP_CAPTURE](https://mavlink.io/en/messages/common.html#MAV_CMD_VIDEO_STOP_CAPTURE)
- [MAV_CMD_SET_CAMERA_MODE](https://mavlink.io/en/messages/common.html#MAV_CMD_SET_CAMERA_MODE)
- [MAV_CMD_SET_CAMERA_ZOOM](https://mavlink.io/en/messages/common.html#MAV_CMD_SET_CAMERA_ZOOM)
- [MAV_CMD_SET_CAMERA_FOCUS](https://mavlink.io/en/messages/common.html#MAV_CMD_SET_CAMERA_FOCUS)

PX4 re-emits the camera commands found in missions as MAVLink commands.
The system id of the emitted commands is the same as the ID of the autopilot.
The component id of the commands can vary.
The first four commands are addressed to [MAV_COMP_ID_CAMERA (100)](https://mavlink.io/en/messages/common.html#MAV_COMP_ID_CAMERA) (if a camera has this component ID, it will execute the indicated command).
The camera mode, zoom, and focus, commands are sent to a component with id of [MAV_COMP_ID_ALL](https://mavlink.io/en/messages/common.html#MAV_COMP_ID_ALL).

:::info
PX4 currently ignores the target camera `id` in [MAV_CMD_IMAGE_START_CAPTURE](https://mavlink.io/en/messages/common.html#MAV_CMD_IMAGE_START_CAPTURE) and other camera messages.
See [PX4-Autopilot#23083](https://github.com/PX4/PX4-Autopilot/issues/23083).
:::

<!--
List of all supported commands in missions in:
format_mavlink_mission_item() => https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mavlink/mavlink_mission.cpp#L1672-L1693

Mission items are executed when set active.
void Mission::setActiveMissionItems() => https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/navigator/mission.cpp#L187-L281
  At end the current non-waypoint command is "issued":
  note at end => issue_command(_mission_item);

Issuing command:
MissionBlock::issue_command(const mission_item_s &item) =>  https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/navigator/mission_block.cpp#L543-L562
  At end this publishes the current vehicle command
  _navigator.publish_vehicle_command(vehicle_command);

Publishing command:
void Navigator::publish_vehicle_command(vehicle_command_s &vehicle_command)  => https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/navigator/navigator_main.cpp#L1395
  For camera commands set to vehicle_command.target_component = 100; // MAV_COMP_ID_CAMERA
  All others just get published as-is
-->

### 手动控制器

Joystick buttons can be configured to trigger image capture or toggle video capture.

PX4 emits [MAVLink Camera Protocol v2](https://mavlink.io/en/services/camera.html) commands such as `MAV_CMD_IMAGE_START_CAPTURE` when the associated Joystick button is pressed.
This feature only works for this kind of camera and joystick - there is no support for RC Controllers.

## PX4 配置

### MAVLink 端口和转发

You will need to provide a MAVLink channel to any connected cameras so that PX4 can emit any camera commands found in missions.
If your MAVLink network is such that PX4 is "between" your camera and your ground station, you will also need to forward communications so that they can communicate.

First attach the camera to an unused serial port on your flight controller, such as `TELEM2` (you might also use an Ethernet port if present on both your flight controller and the camera).
Then configure the selected port as a [MAVLink Peripheral](../peripherals/mavlink_peripherals.md).

The linked document explains how, but in summary:

1. Modify an unused `MAV_n_CONFIG` parameter, such as [MAV_2_CONFIG](../advanced_config/parameter_reference.md#MAV_2_CONFIG), so that it is assigned to port to which you connected the camera/companion computer.
2. 将对应的 [MAV_2_MODE](../advanced_config/parameter_reference.md#MAV_2_MODE) 设置为 `2` (板载)。
  This ensures that the right set of MAVLink messages are emitted for a companion computer (or camera).
3. Set [MAV_2_FORWARD](../advanced_config/parameter_reference.md#MAV_2_FORWARD) to enable forwarding of communications from the port to other ports, such as the one that is connected to the ground station.
4. You may need to set some of the other parameters, depending on your connection type and any particular requirements of the camera on the expected baud rate, and so on.

### Manual Control

Joystick buttons can be mapped to capture images, and to toggle video capture on and off.

- [Joystick](../config/joystick.md#enabling-px4-joystick-support) explains how to enable Joysticks on PX4.
- [QGroundControl > Joystick Setup](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/joystick.html) explains how to map buttons to flight stack functions

<!-- Cameras cannot be controlled from an RC controller as far as I can tell -->

## 相机管理器

If you want to use a camera that does not natively support the MAVLink camera protocol you will need a MAVLink camera manager.
The camera manager runs on a companion computer and bridges between the MAVLink camera protocol interface and the camera's native interface.

There are "extensible" camera managers that can be used with many different cameras, camera managers designed to work with a specific camera, and you can also write your own (for example, using MAVSDK server plugins).

Generic/extensible camera managers:

- [MAVLink Camera Manager](https://github.com/mavlink/mavlink-camera-manager) - Extensible cross-platform MAVLink Camera Server built on top of GStreamer and Rust-MAVLink.
- [Dronecode Camera Manager](https://camera-manager.dronecode.org/en/) - Adds Camera Protocol interface for cameras connected to Linux computer.

Camera-specfic camera managers:

- [SIYI A8 mini camera manager](https://github.com/julianoes/siyi-a8-mini-camera-manager) - MAVSDK-plugin based camera manager for the [SIYI A8 mini](https://shop.siyi.biz/products/siyi-a8-mini) (includes tutorial).

  ::: tip
  This is a good example of how MAVSDK can be used to create a MAVLink camera protocol interface for a particular camera.

:::

When using a camera manager you connect the companion computer to the flight controller (rather than directly to the camera), and you'll need additional software on the computer to route MAVLink traffic to the camera manager on the companion computer, such as [mavlink-router](https://github.com/mavlink-router/mavlink-router).

More information about camera manager and companion computer setups can be found in:

- [SIYI A8 mini camera manager](https://github.com/julianoes/siyi-a8-mini-camera-manager) - Tutorial for integrating with the [SIYI A8 mini](https://shop.siyi.biz/products/siyi-a8-mini) using a MAVSDK-based camera manager running on a Raspberry Pi companion computer.
- [Using a Companion Computer with Pixhawk Controllers](../companion_computer/pixhawk_companion.md)
- [Companion Computers > Companion Computer Software](../companion_computer/index.md#companion-computer-software): In particular note [MAVLink-Router](https://github.com/mavlink-router/mavlink-router), which you can setup to route MAVLink traffic between a serial port and an IP link (or other camera manager interface).
