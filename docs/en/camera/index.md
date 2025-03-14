# Cameras

Cameras are important for many [payload use cases](../payloads/use_cases.md), including mapping and surveying, surveillance, search & rescue, crop health and pest detection, and so on.
They are commonly mounted on a [gimbal](../advanced/gimbal_control.md) that can provide camera stabilisation, point tracking, and movement independent of the hosting vehicle.

## Camera Types

PX4 integrates with three types of cameras:

- [MAVLink cameras](../camera/mavlink_v2_camera.md) that support the [Camera Protocol v2](https://mavlink.io/en/services/camera.html) (**RECOMMENDED**).
- [Simple MAVLink cameras](../camera/mavlink_v1_camera.md) that support the older [Camera Protocol v1](https://mavlink.io/en/services/camera.html).
- [Cameras attached to flight controller outputs](../camera/fc_connected_camera.md), which are controlled using the [Camera Protocol v1](https://mavlink.io/en/services/camera.html).

[MAVLink cameras](../camera/mavlink_v2_camera.md) are recommended because they provide the broadest access to camera features using a simple and consistent command/message set.
If a camera does not support this prototol, a [camera manager](../camera/mavlink_v2_camera.md#camera-managers) running on a companion computer can be used to interface between MAVLink and the camera's native protocol.

## See Also

- [Gimbal (Camera Mount)](../advanced/gimbal_control.md)
- [Camera Integration/Architecture](../camera/camera_architecture.md) (PX4 Developers)