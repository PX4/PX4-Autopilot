# DroneCAN 电调

PX4 支持符合 DroneCAN 标准的电调。

## Supported ESC

:::info
_电调与电机_ 中的[支持的电调](../peripherals/esc_motors.md#supported-esc)可能还包括下文未列出的其他设备。
:::

以下文章提供具体的硬件和固件信息：

- [PX4 Sapog 电调固件](sapog.md)
  - [Holybro Kotleta 20](holybro_kotleta.md)
- [Zubax Telega](zubax_telega.md)
- [Vertiq](../peripherals/vertiq.md)（较大型号的模块）
- [VESC 项目](../peripherals/vesc.md)
- [RaccoonLab Cyphal 和 DroneCAN PWM 节点](raccoonlab_nodes.md)

## 硬件配置

通用的 DroneCAN 硬件配置请参阅 [DroneCAN > 硬件设置](../dronecan/index.md#hardware-setup)。

DroneCAN 电调应使用独立的专用 CAN 接口，因为电调消息可能占满总线，使其他节点无法获得足够的带宽。

## PX4 配置

请按照 [DroneCAN](../dronecan/index.md) 中概述的步骤配置 DroneCAN 外设。

除通用设置外，还需执行以下操作：

- 将 [UAVCAN_ENABLE](../advanced_config/parameter_reference.md#UAVCAN_ENABLE) 设为 `3`。

::: tip
提示 <Badge type="tip" text="main (PX4 v2.0)" />
  无需启用 [UAVCAN_PUB_ARM](../advanced_config/parameter_reference.md#UAVCAN_PUB_ARM)。

  在早期版本中，需要设置 `UAVCAN_PUB_ARM=1` 才能发布 [ArmingStatus](https://dronecan.github.io/Specification/7._List_of_standard_data_types/#armingstatus)（某些电调需要此消息，才能在飞行和执行器页面测试期间正确响应）。现在，只要 `UAVCAN_ENABLE=3`，就会自动发布该状态。

:::

- 使用 [UAVCAN_ESC_IFACE](../advanced_config/parameter_reference.md#UAVCAN_ESC_IFACE) 参数选择用于电调数据输出的 CAN 接口（默认选择所有接口）。

- 配置[电机顺序和舵机输出](../config/actuators.md)。

## 已知兼容性问题 {#known-quirks}

某些 DroneCAN 设备的行为偏离 DSDL 规范，需要在 PX4 端进行兼容处理。这些兼容处理默认禁用，可通过 [UAVCAN_QUIRKS](../advanced_config/parameter_reference.md#UAVCAN_QUIRKS) 位掩码参数启用。

- **Hobbywing 电调：** 某些 Hobbywing DroneCAN 电调上报的 `uavcan.equipment.esc.Status.esc_index` 从 1 开始表示第一个电调，而不是规范要求的 0，但仍要求使用 `RawCommand.cmd[0]` 控制第一个电调。这会导致电调状态反馈（例如转速、电压、电流）被匹配到错误的电机。设置 `UAVCAN_QUIRKS` 的第 0 位，可修正这种索引偏移一位的问题。

## 可反转电机 {#reversible-motors}

<Badge type="tip" text="main (PX4 v2.0)" />

如果电机硬件支持反转、电调固件已配置为 3D/双向运行，并且在执行器配置中将电机设为[双向](../config/actuators.md#bidirectional-motors)，则电机可以在运行过程中切换转向（双向）。

配置为双向后，PX4 按照 DroneCAN 电调消息协议的定义，使用 _有符号_ 的 `uavcan.equipment.esc.RawCommand` 值控制电机：

- **零：** 电机停止 / 中立状态
- **正值：** 正向推力
- **负值：** 反向推力

未配置为双向的电机仍只接收正值。

:::tip
验证运行情况
检查电调数传上报的转速正负号，确认电调正确解析负值指令，并实际驱动电机反向旋转。
:::

可反转电机也可用于[电机故障恢复](../config/motor_failure_recovery.md)，使六旋翼机体在单个电机故障后仍保持可控。这种情况下仍需将电调配置为支持反转。但无需在执行器配置中将电机设为双向，因为 PX4 会在处理故障时使参与恢复的电机能够反转。
