# 包裹投递任务

<Badge type="tip" text="PX4 v1.14" />

包裹投递任务是航点任务的拓展，用户可以计划将包裹作为航点进行投递。

本节解释了包裹投递功能的架构。
它的目的是为从事扩展架构的开发者提供支持，例如支持新的有效载荷投递机制。

:::info
Currently only [Grippers](../peripherals/gripper.md) can be used for package delivery.
绞盘尚未支持。
:::

:::info
The detailed documentation on how to setup a package delivery mission plan can be found [here](../flying/package_delivery_mission.md).
Setup for the `payload_deliverer` module is covered in the documentation for the delivery mechanism, such as [Gripper](../peripherals/gripper.md#px4-configuration).
:::

## 包裹投递架构图

![Package delivery architecture overview](../../assets/advanced_config/payload_delivery_mission_architecture.png)

Package Delivery feature is centered around the [VehicleCommand](../msg_docs/VehicleCommand.md) & [VehicleCommandAck](../msg_docs/VehicleCommandAck.md) messages.

The central idea lies in having an entity that handles the `DO_GRIPPER` or `DO_WINCH` vehicle command, executes it and sends back an acknowledgement when the successful delivery is confirmed.

Because PX4 automatically broadcasts the `VehicleCommand` uORB message to a UART port configured to communicate in MAVLink as a [`COMMAND_LONG`](https://mavlink.io/en/messages/common.html#COMMAND_LONG) message, an external payload can receive the command and execute it.

Likewise, since PX4 automatically translates the [`COMMAND_ACK`](https://mavlink.io/en/messages/common.html#COMMAND_ACK) message coming in from an external source through a UART port configured for MAVLink into a `vehicle_command_ack` uORB message, an external payload's acknowledgement for a successful package deployment can be received by PX4's `navigator` module.

下面解释了包裹投递架构中涉及的每个实体。

## 导航器

导航器处理接收车辆命令 ACK (下文所述)。
在收到成功部署确认消息后，它会设置任务块级别中的标志，以表示有效载荷部署已成功。

这允许任务安全地继续到下一个项目(例如路径点)，因为我们确信部署已成功得到确认。

## 车辆命令 ACK

We are waiting for the ACK coming from either internally (via `payload_deliverer` module), or externally (external entity sending the MAVLink message `COMMAND_ACK`) to determine if the package delivery action has been successful (either `DO_GRIPPER` or `DO_WINCH`).

## Mission

The Gripper / Winch command is placed as a `Mission Item`.
This is possible since all the Mission item has the `MAV_CMD` to execute (e.g. Land, Takeoff, Waypoint, etc) which can get set to either `DO_GRIPPER` or `DO_WINCH`.

In the Mission logic (green box above) if either Gripper/Winch mission item is reached, it implements brake_for_hold functionality (which sets the `valid` flag of the next mission item waypoint to `false`) for rotary wings (e.g. Multicopter) so that the vehicle would hold it's position while the deployment is getting executed.

固定翼飞机和其他车辆不考虑特殊制动条件。
所以如果你有一个固定翼的悬停任务，飞机在悬停的同时投递包裹，飞机不会停止 (因为这是不可能的)。

## 任务块

`MissionBlock` is the parent class of `Mission` that handles the part "Is Mission completed?".

This all performed in the `is_mission_item_reached_or_completed` function, to handle the time delay / mission item advancement.

Also it implements the actual issue_command function, which will issue a vehicle command corresponding to the mission item's `MAV_CMD`, which will then be received by an external payload or the `payload_deliverer` module internally.

## Payload Deliverer

This is a dedicated module that handles gripper / winch support, which is used for the standard [package delivery mission plan](../flying/package_delivery_mission.md).

Setup for the `payload_deliverer` module is covered within setting up an actual package release mechanism setup documentation like [Gripper](../peripherals/gripper.md#px4-configuration).
