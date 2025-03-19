# 简单的 MAVLink 摄像头(Camera Protcol v1)

本节说明了如何使用 PX4 的 MAVLink [相机](../camera/index.md), 实现了 [Camera Protocol v1 (简单触发协议)](https://mavlink.io/en/services/camera_v1.html) 的 PX4 和地面站。

:::warning
[MAVLink 相机](../camera/mavlink_v2_camera.md) 在可能的情况下应该使用 [MAVLink Camera Protocol v2](https://mavlink.io/en/services/camera.html) 代替！
此方法保留用于旧的 MAVLink 相机。
:::

## 综述

[相机协议v1](https://mavlink.io/zh/services/camera_v1.html) 定义了一小组命令，允许用于触发相机进行拍摄：

- 仍然根据时间或距离以频率捕获图像
- 视频捕获
- 有限的相机配置

PX4 支持此命令集以触发通过原生支持协议的相机（如本节所述），以及连接到飞控输出的相机。

地面站和 MAVLink SDK 通常将相机命令发送给自动驾驶仪，然后转发给连接的类型为 '板载' 的 MAVLink 通道。
PX4 还会将其在任务中遇到的任何相机任务项重新发出为相机命令：未被接受的命令将被记录。
在所有情况下，命令都是使用自动驾驶仪的系统 ID 和组件 ID 为0（即发送给所有组件，包括摄像头）。

每次触发图像捕获时 PX4 也会发出一个 [CAMERA_TRIGGER](https://mavlink.io/en/messages/common.html#CAMERA_TRIGGER) (相机本身也可能在触发时发出此消息)。

## 控制相机

### MAVLink 命令和消息

[相机协议v1（简单触发协议）](https://mavlink.io/en/services/camera_v1.html)定义了以下命令：

- [MAV_CMD_DO_TRIGGER_CONTROL](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_TRIGGER_CONTROL)
- [MAV_CMD_NAV_CMD_DO_DIGICAM_CONTROL](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_CMD_DO_DIGICAM_CONTROL)
- [MAV_CMD_DO_SET_CAM_TRIGG_DIST](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_CAM_TRIGG_DIST)
- [MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL)
- [MAV_CMD_OBLIQUE_SURVEY](https://mavlink.io/en/messages/common.html#MAV_CMD_OBLIQUE_SURVEY)
- [MAV_CMD_DO_CONTROL_VIDEO](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_CONTROL_VIDEO)

MAVLink 摄像头将支持这些命令的一些子集。
由于协议没有发现过程的功能，唯一的方法是检查响应中返回的[COMMAND_ACK](https://mavlink.io/en/messages/common.html#COMMAND_ACK)。

相机在每次拍摄图像时也应发出[CAMERA_TRIGGER](https://mavlink.io/en/messages/common.html#CAMERA_TRIGGER)。

[Camera Protocol v1](https://mavlink.io/zh/services/camera_v1.html) 更详细地描述了协议。

### 地面站

地面站可以使用 [Camera Protocol v1（简单触发协议）](https://mavlink.io/en/services/camera_v1.html) 中的任何命令，并且应该将这些命令发送给自驾仪组件 id。
如果相机不支持命令，它将返回带有错误结果的[COMMAND_ACK](https://mavlink.io/en/messages/common.html#COMMAND_ACK)。

通常命令是针对自驾仪的，因为这样无论相机是通过 MAVLink 连接还是直接连接飞控都能工作。
如果发送给自驾仪 PX4，每次拍摄图像时 PX4 都会发出[CAMERA_TRIGGER](https://mavlink.io/en/messages/common.html#CAMERA_TRIGGER)，并可能记录相机拍摄事件。

<!-- "May" because the camera feedback module is "supposed"  to log just camera capture from a capture pin connected to camera hotshoe, but currently logs all camera trigger events from the camera trigger driver https://github.com/PX4/PX4-Autopilot/pull/23103 -->

理论上，您也可以直接向相机发送命令。

### Camera Commands in Missions

以下[Camera Protocol v1 (简单触发协议)](https://mavlink.io/en/services/camera_v1.html)命令可在任务中使用(这与上面的命令列表相同)。

- [MAV_CMD_DO_TRIGGER_CONTROL](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_TRIGGER_CONTROL)
- [MAV_CMD_NAV_CMD_DO_DIGICAM_CONTROL](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_CMD_DO_DIGICAM_CONTROL)
- [MAV_CMD_DO_SET_CAM_TRIGG_DIST](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_CAM_TRIGG_DIST)
- [MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL)
- [MAV_CMD_OBLIQUE_SURVEY](https://mavlink.io/en/messages/common.html#MAV_CMD_OBLIQUE_SURVEY)
- [MAV_CMD_DO_CONTROL_VIDEO](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_CONTROL_VIDEO)

PX4 重新使用与自驾仪相同的系统 ID 和组件 ID [MAV_COMP_ID_ALL](https://mavlink.io/en/messages/common.html#MAV_COMP_ID_ALL) 重新发送它们：

<!-- See camera_architecture.md topic for detail on how this is implemented -->

### Manual Control

这些相机不支持手动触发（无论是摇杆还是遥控器）。

## PX4 配置

<!-- set up the mode and triggering -->

### MAVLink 端口和转发配置

将 PX4 连接到您的 MAVLink 相机上，将它连接到您飞控上一个未使用的串口，如`TELEM2`。
然后，您可以将端口配置为[MAVLink 外设](../peripherals/mavlink_peripherals.md).
本文件解释了如何做，总的来说：

1. 修改一个未使用的 `MAV_n_CONFIG` 参数，例如[MAV_2_CONFIG](../advanced_config/parameter_reference.md#MAV_2_CONFIG)，使其分配给相机连接的端口。
2. 将对应的 [MAV_2_MODE](../advanced_config/parameter_reference.md#MAV_2_MODE) 设置为 `2` (板载)。
  这确保正确的 MAVLink 消息集被发出和转发。
3. 您可能需要设置一些其他参数，取决于您的连接 - 例如波特率。

然后按照其用户指南中的建议连接和配置相机。

<!-- Removed this because I am pretty sure forwarding happens automatically for this set. Keeping it simple.
1. Set [MAV_2_FORWARD](../advanced_config/parameter_reference.md#MAV_2_FORWARD) if you want to enable forwarding of MAVLink messages to other ports, such as the one that is connected to the ground station.
-->

### 相机模式和触发

配置 PX4 相机驱动器以启用 MAVLink 相机后端，并设置触发模式为在勘测任务中按命令捕获。

使用 _QGroundControl_:

- 打开 [Vehicle Setup > Camera](https://docs.qgroundcontrol.com/master/zh/qgc-user-guide/setup_view/camera.html#px4-camera-setup)。
- 按照所示设置数值:

  ![相机设置界面 - 触发模式和 MAVLink 接口](../../assets/camera/mavlink_camera_settings.png)

:::info
You can also [set the parameters directly](../advanced_config/parameters.md):

- [TRIG_MODE](../advanced_config/parameter_reference.md#TRIG_MODE) — `4`: 基于距离，按命令触发 (勘测模式)
- [TRIG_INTERFACE](../advanced_config/parameter_reference.md#TRIG_INTERFACE) — `3`: MAVLink

:::
