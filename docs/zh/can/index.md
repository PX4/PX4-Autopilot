# CAN

[控制器局域网（CAN）](https://en.wikipedia.org/wiki/CAN_bus)是一种可靠的有线网络，它能让诸如飞行控制器、电调、传感器及其他外设等无人机组件相互通信。
它被设计为分布式架构，使用差分信号，即使在较长的电缆 (大型车辆上) 上也非常强大，避免单点故障。
CAN 还允许来自外设的状态反馈，并通过总线方便的进行固件升级。

PX4 支持与 CAN 设备通信的两个软件协议：

- [DroneCAN](../dronecan/index.md): PX4 推荐大多数常见的设置。
  它得到了 PX4 的很好支持，是一个成熟的产品，具有广泛的外围支持，并经过多年的测试。
- [Cyphal](https://opencyphal.org)：PX4 支持是一个“在进行中的工作”。
  Cyphal 是一种更新的协议，允许更多的灵活性和配置，尤其是对于较大和较复杂的载具。
  它还没有被广泛应用。

:::info
DroneCAN 和 Cyphal 都是早先一个叫做UAVCAN的项目。
在2022年，该项目分为两个部分：原始版本的 UAVCAN (UAVCAN v0) 更名为 DroneCAN，较新的 UAVCAN v1 更名为 Cyphal。
这两项协议之间的差异在[Cyphal vs. DroneCAN](https://forum.opencyphal.org/t/cyphal-vs-dronecan/1814)中作了概述。
:::

:::warning
PX4不支持KDECAN等无人驾驶飞机的其他CAN软件协议(撰写时)。
:::

## 布线

CAN 网络的接线对于 DroneCAN 和 Cyphal/CAN 是一样 (实际上对所有的 CAN 网络都一样)。

设备以任意顺序连接成链。
在链的任一端，应该在两个数据线之间连接一个 120Ω 的终端电阻。
飞控和一些 GNSS 模块为了方便使用内置了终端电阻， 因此应该放在链的终端。
否则，你可以使用终端电阻，比如 [Zubax Robotics 的这款](https://shop.zubax.com/products/uavcan-micro-termination-plug?variant=6007985111069)。如果你有JST - GH压接工具，也可以自己焊接一个。

下图显示了一个 CAN 总线连接飞控到 4 个 CAN 电调和一个 GNSS 的示例。

![CAN 布线](../../assets/can/uavcan_wiring.svg)

图中未显示任何电源接线。
参考制造商的说明，确认组件是否需要单独供电，还是可以通过 CAN 总线供电。

For more information, see [Cyphal/CAN device interconnection](https://wiki.zubax.com/public/cyphal/CyphalCAN-device-interconnection?pageId=2195476) (kb.zubax.com).
虽然本文是以 Cyphal 协议为基础编写的，但同样适用于 DroneCAN 硬件和任何其他 CAN 设置。
如需了解更复杂的场景，请参考 [论CAN总线拓扑结构与终端匹配](https://forum.opencyphal.org/t/on-can-bus-topology-and-termination/1685)。

### 连接器

Pixhawk标准兼容的 CAN 设备使用 4 引脚的 JST-GH 连接器为 CAN。
在连线接线时，有两个连接器用于输入和输出（飞行控制器除外和一些内置终止的全球导航卫星系统（GNSS）设备除外）； 它仅有一个JST-GH连接器)。

其他(非Pixhawk兼容的)设备可能使用不同的连接器。
然而，只要设备固件支持DroneCAN 或Cyphal，它就可以使用。

### 冗余

DroneCAN 和 Cyphal/CAN支持使用第二个(冗余) CAN 接口。
这是完全可选的，但会增加连接的强度。
所有Pixhawk飞行控制器都带有两个CAN接口； 如果您的外围设备也支持 2 CAN 接口，建议您同时进行电线连接以提高安全。

## 固件

CAN 外围设备可以运行专有或开源固件(请检查制造商指南以确认所需的设置)。

PX4 可以构建在支持的 CAN 硬件上作为开源的 DroneCAN 固件运行。
更多信息请参考 [PX4 DroneCAN 固件](../dronecan/px4_cannode_fw.md)。

## 支持和配置

[DroneCAN 设置和配置](../dronecan/index.md)

[PX4 DroneCAN 固件](../dronecan/px4_cannode_fw.md)

## 视频

### DroneCAN

关于 DroneCAN (UAVCANv0) 的介绍和在 QGroundControl 中设置的实用示例：

<lite-youtube videoid="IZMTq9fTiOM" title="Intro to DroneCAN (UAVCANv0) and practical example with setup in QGroundControl"/>

### Cyphal

无人机的 UAVCAN v1 (Cyphal) - PX4 开发者峰会虚拟2020

<lite-youtube videoid="6Bvtn_g8liU" title="UAVCAN v1 for drones — PX4 Developer Summit Virtual 2020"/>

---

在NXP UAVCAN板上使用UAVCAN v1和PX4入门——2020年PX4开发者峰会线上会议

<lite-youtube videoid="MwdHwjaXYKs" title="Getting started using UAVCAN v1 with PX4 on the NXP UAVCAN Board"/>

---

UAVCAN：一个高度可靠的发布-订阅协议，用于硬实时车辆内网络 — PX4 开发者虚拟峰会 2019

<lite-youtube videoid="MBtROivYPik" title="UAVCAN: a highly dependable publish-subscribe protocol for hard ..."/>
