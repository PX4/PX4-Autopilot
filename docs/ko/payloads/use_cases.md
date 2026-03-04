# Payload Use Cases

This topic lists a few common drone "payload use cases", and how they are supported by PX4.

## Mapping Drones

Mapping drones use cameras to capture images at time or distance intervals during surveys.

PX4 supports cameras that are connected via either [MAVLink](../camera/mavlink_v2_camera.md) or to [Flight Controller Outputs](../camera/fc_connected_camera.md).
Both types of cameras support the mapping use case, but via a different set of MAVLink commands/mission items.

## Cargo Drones (Package Delivery)

Cargo drones commonly use grippers, winches, and other mechanisms to release packages at their destinations.

PX4 supports _package delivery in missions_ using a [gripper](../peripherals/gripper.md).
Grippers can also be triggering using the [MAV_CMD_DO_GRIPPER](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_GRIPPER) MAVLink command, or manually via a Joystick button.

For setup and usage information see:

- [Gripper](../peripherals/gripper.md)
- [Flying > Package Delivery Mission Planning](../flying/package_delivery_mission.md)

:::info
Support for winches and other release mechanisms is also intended.

If you need to perform cargo delivery using hardware that is not yet integrated, you can use [Generic Actuator Control](../payloads/generic_actuator_control.md).
:::

## Surveillance, Search & Rescue

Surveillance and Search & Rescue drones have similar requirements to mapping drones.
The main differences are that, in addition to flying a planned survey area, they typically need good standalone control over the camera for image and video capture, and they may need to be able to work during both day and night

Use a [MAVLink](../camera/mavlink_v2_camera.md) that supports the [MAVLink Camera Protocol](https://mavlink.io/en/services/camera.html), as this enables image and video capture, zooming, storage management, multiple cameras on the same vehicle and switching between them, etc.
These cameras can be controlled either manually from QGroundControl or via MAVSDK (for both [standalone camera operations](https://mavsdk.mavlink.io/main/en/cpp/api_reference/classmavsdk_1_1_camera.html) and in [missions](https://mavsdk.mavlink.io/main/en/cpp/api_reference/structmavsdk_1_1_mission_1_1_mission_item.html#structmavsdk_1_1_mission_1_1_mission_item_1a0299fbbe7c7b03bc43eb116f96b48df4)).
See [MAVLink Camera](../camera/mavlink_v2_camera.md) for information on how to configure your camera to work with MAVLink.

:::info
Cameras connected directly to the flight controller _only_ support camera triggering, and are unlikely to be suitable for most surveillance/search work.
:::

A search and rescue drone may also need to carry cargo, for example, emergency supplies for a stranded hiker.
See [Cargo Drones](#cargo-drones-package-delivery) above for information about payload delivery.

## Agricultural Drones/Crop Spraying

Agricultural drones are commonly used for mapping crop health and pest detection and animal management (herding, tracking,and so on).
These cases are similar to the [mapping](#mapping-drones) and [surveillance, search & rescue](#surveillance-search-rescue) cases above.
While specific crops/animals may need specialist cameras, the integration with PX4 is the same.

Agricultural drone may also be used for crop spraying.
In this case the sprayer must be controlled as a [generic actuator](../payloads/generic_actuator_control.md):

- [Generic Actuator Control](../payloads/generic_actuator_control.md#generic-actuator-control-with-mavlink) explains how you can connect flight controller outputs to your sprayer so that they can be controlled using MAVLink.
  Most sprayers provide controls to activate/deactivate a pump; some also allow control over the rate of flow or the spray field (i.e. by controlling the nozzle shape, or using a spinner to distribute the payload).
- You can define the area to spray using a [Survey pattern](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/plan_view/pattern_survey.html), or you can define the grid to fly using waypoints.
  In either case, it is important to ensure that the vehicle flight path and altitude provide adequate coverage for your particular spray being used.
- You should add a ["Set actuator" mission item](../payloads/generic_actuator_control.md#generic-actuator-control-in-missions) to your mission before and after the survey pattern in order to enable and disable the sprayer.
