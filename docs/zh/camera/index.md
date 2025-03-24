# Cameras

相机对许多[有效载荷使用](../payloads/use_cases.md)很重要，包括绘图和勘测、监视、搜索和救援、作物健康和虫害检测等等。
它们通常安装在一个 [云台](../advanced/gimbal_control.md)上，它能够提供相机稳定性、点跟踪和独立运动。

## 相机类型

PX4 integrates with three types of cameras:

- [MAVLink cameras](../camera/mavlink_v2_camera.md) that support the [Camera Protocol v2](https://mavlink.io/en/services/camera.html) (**RECOMMENDED**).
- [Simple MAVLink cameras](../camera/mavlink_v1_camera.md) that support the older [Camera Protocol v1](https://mavlink.io/en/services/camera.html).
- [Cameras attached to flight controller outputs](../camera/fc_connected_camera.md), which are controlled using the [Camera Protocol v1](https://mavlink.io/en/services/camera.html).

推荐[MAVLink 摄像头](../camera/mavlink_v2_camera.md)，因为它们使用简单一致的命令/消息集提供了最广泛的相机功能访问。
如果相机不支持该协议，则可以在一台机载计算机上运行[摄像机管理器](../camera/mavlink_v2_camera.md#camera-managers)以在 MAVLink 和相机的本机协议之间进行接口交互。

## See Also

- [云台(相机支架)](../advanced/gimbal_control.md)
- [相机集成/架构](../camera/camera_architecture.md) ( PX4 开发者)