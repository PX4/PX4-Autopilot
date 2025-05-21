# PX4 Camera Architecture/Integration

This topic provides a brief overview of how PX4 camera support is **implemented**.

:::info
See [Camera](../camera/index.md) for information about _using_ cameras.
:::

## 综述

PX4 integrates with three types of cameras:

- [MAVLink cameras](../camera/mavlink_v2_camera.md) that support the [Camera Protocol v2](https://mavlink.io/en/services/camera.html) (**RECOMMENDED**).
- [Simple MAVLink cameras](../camera/mavlink_v1_camera.md) that support the older [Camera Protocol v1](https://mavlink.io/en/services/camera.html).
- [Cameras attached to flight controller outputs](../camera/fc_connected_camera.md), which are controlled using the [Camera Protocol v1](https://mavlink.io/en/services/camera.html).

All of these cameras need to respond to MAVLink commands received over MAVLink or found in missions (the specific protocol depends on the camera).

The broad architecture used is described below.

## MAVLink Cameras (Camera Protocol v2)

PX4 does not have specific handling for [MAVLink cameras](../camera/mavlink_v2_camera.md) that support the [Camera Protocol v2](https://mavlink.io/en/services/camera.html), other than [re-emitting camera items in missions as commands](#camera-commands-in-missions)

Ground stations are expected to communicate with these cameras directly in order to send commands.
PX4 must be configured to route MAVLink traffic between the camera and ground stations if needed.

:::info
The `camera_trigger`, `camera_capture` and `camera_feedback` modules are not used with this camera.
:::

## FC-connected Cameras

[Cameras attached to flight controller outputs](../camera/fc_connected_camera.md) need PX4 to activate the outputs to trigger the camera, and may need PX4 to detect when a [camera capture pin](../camera/fc_connected_camera.md#camera-capture-configuration) has been triggered by the camera hotshoe (in order to improve the logged camera-capture time).

This work is handled by three PX4 components: [`camera_trigger` driver](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/camera_trigger), [`camera_capture` driver](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/camera_capture), [`camera-feedback` module](../modules/modules_system.md#camera-feedback).

`camera_trigger` subscribes to the [VehicleCommand](../msg_docs/VehicleCommand.md) topic and monitors for updates to its [supported commands](../camera/fc_connected_camera.md#mavlink-command-interface).
Thes updates occur when either a command is received via MAVLink or when a [camera item is reached in a mission](#camera-commands-in-missions).

The commands enable and disable triggering, and configure triggering at time and distance intervals.
The driver tracks these intervals, and when needed triggers the outputs.
The driver publishes a [CameraTrigger](../msg_docs/CameraTrigger.md) topic (with `feedback` field set to `false`) that causes a [CAMERA_TRIGGER](https://mavlink.io/en/messages/common.html#CAMERA_TRIGGER) MAVLink message to be emitted.

The `camera_capture` driver, if enabled, monitors the camera capture pin and on triggering publishes a [CameraTrigger](../msg_docs/CameraTrigger.md) topic (with `feedback` field set to `true`) which also causes a [CAMERA_TRIGGER](https://mavlink.io/en/messages/common.html#CAMERA_TRIGGER) MAVLink message to be emitted.

The `camera_feedback` module monitors for updates to the [CameraTrigger](../msg_docs/CameraTrigger.md) topic, and publishes a [CameraCapture](../msg_docs/CameraCapture.md) topic for `CameraTrigger` updates from _either_ `camera_trigger` or `camera_capture`.
The information that is used depends on whether the camera capture pin is enabled and the value is of the `CameraTrigger.feedback` field.
This `CameraCapture` topic is logged, and can be used to get the time of the capture.

## MAVLink Cameras (Camera Protocol v1)

[MAVLink cameras that support the older Camera Protocol v1](../camera/mavlink_v1_camera.md) are integrated in much the same way as [FC-connected cameras](#fc-connected-cameras).

`camera_trigger` subscribes to the [VehicleCommand](../msg_docs/VehicleCommand.md) topic and monitors for updates in the [commands it supports](../camera/fc_connected_camera.md#mavlink-command-interface).
This happens when either a command is received via MAVLink or when a [camera item is found in a missions](#camera-commands-in-missions).

The commands enable and disable triggering, and configure triggering at time and distance intervals.
The driver tracks these intervals, but with the "MAVLink backend" does not need to actually trigger any outputs (since the commands are forwarded to the camera).
When the camera would trigger the driver publishes a [CameraTrigger](../msg_docs/CameraTrigger.md) topic (with `feedback` field set to `false`) that causes a [CAMERA_TRIGGER](https://mavlink.io/en/messages/common.html#CAMERA_TRIGGER) MAVLink message to be emitted.
The `camera_feedback` module should then log a corresponding `CameraCapture` topic.

## Camera Commands in Missions

PX4 re-emits camera items found in missions as MAVLink commands for all supported [Camera Protocol v2](https://mavlink.io/en/services/camera.html) and [Camera Protocol v1](https://mavlink.io/en/services/camera.html) commands.
The system id of the emitted commands is the same as the ID of the autopilot.
The component id of the commands can vary, but these are usually sent to either [MAV_COMP_ID_CAMERA (100)](https://mavlink.io/en/messages/common.html#MAV_COMP_ID_CAMERA) or [MAV_COMP_ID_ALL](https://mavlink.io/en/messages/common.html#MAV_COMP_ID_ALL) (see the individual camera documents for what ID is used in each case).

The commands are emitted irrespective of whether or not there is a connected camera of any type, provided there is a MAVLink channel to emit to.

:::info
More generally PX4 re-emits all mission commands that may be consumed by external MAVLink components, such as gimbals.
Commands for waypoints and conditional behaviour are not emitted.
:::

The sections below highlight interesting parts of the codebase

### Commands supported in missions

Commands supported in missions, including camera commands, are shown in these methods:

- [`bool FeasibilityChecker::checkMissionItemValidity(mission_item_s &mission_item, const int current_index)`](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/navigator/MissionFeasibility/FeasibilityChecker.cpp#L257-L306)
- [`format_mavlink_mission_item()`](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mavlink/mavlink_mission.cpp#L1672-L1693)

### Flow for re-emitting camera commands found in missions

- [`void Mission::setActiveMissionItems()`](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/navigator/mission.cpp#L187-L281)
  - Mission items are executed when set active.
  - `issue_command(_mission_item)` is called at the end of this to send the current non-waypoint command
    - [`MissionBlock::issue_command(const mission_item_s &item)`](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/navigator/mission_block.cpp#L543-L562)
      - Creates a vehicle command for the mission item then calls `publish_vehicle_command` to publish it (`_navigator->publish_vehicle_command(vehicle_command);`)
        - [`void Navigator::publish_vehicle_command(vehicle_command_s &vehicle_command)`](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/navigator/navigator_main.cpp#L1395)
          - For some camera commands it sets the component ID to the camera component id (`vehicle_command.target_component = 100; // MAV_COMP_ID_CAMERA`)
          - All others just get published to default component ID.
          - The `VehicleCommand` UORB topic is published.

The MAVLink streaming code monitors for changes to the `VehicleCommand` topic and publishes them over MAVLink.
The MAVLink command is sent irrespective of whether the camera is a MAVLink camera, or connected to the flight controller.

The `camera_trigger` driver, if enabled, also monitors for changes to the `VehicleCommand`.
If it is configured with a backend for a camera connected to the flight controller outputs, it will trigger those outputs appropriately.

## 日志

`CameraCapture` topics are logged when there is a `CameraTrigger` update.
The logged topic will depend on whether or not the camera capture pin is enabled.

Note that camera capture events are not logged when using the [MAVLink cameras that support Camera Protocol v2](../camera/mavlink_v2_camera.md), because the corresponding trigger events are not generated within PX4.

## See Also

- Camera trigger driver: [source code](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/camera_trigger) <!-- no module doc -->
- Camera capture driver: [source code](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/camera_capture) <!-- no module doc -->
