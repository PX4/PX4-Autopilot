# Agam FloRange

<Badge type="tip" text="main (PX4 v2.0)" />

:::warning
PX4 不制造此类（或任何）自动驾驶仪组件。
Contact the [manufacturer](https://www.agamrobotics.com/) for hardware support or compliance issues.
:::

Agam FloRange 是 [Agam Robotics](https://www.agamrobotics.com/) 推出的 [DroneCAN](index.md) [光流](../sensor/optical_flow.md)、[距离传感器](../sensor/rangefinders.md)及 IMU 模块。其主要用途是在没有 GNSS 模块等全局定位设备的情况下提供局部定位，并提高稳定性。

![Agam FloRange](../../assets/hardware/sensors/optical_flow/agam_florange.png)

## 购买渠道

Order this module from:

- [Agam Robotics](https://www.agamrobotics.com/product-page/agam-florange-sensor)

## Hardware Specifications

- 传感器
  - PixArt PAA3905E1 光流传感器
    - 工作距离范围从 80mm 到无穷远，无需校准
    - 支持多芯片同步运行和自动模式切换
  - Broadcom AFBR-S50LV85D 飞行时间距离传感器（[数据手册](https://docs.broadcom.com/doc/AFBR-S50LV85D-DS)）
    - 集成 850nm 激光光源，符合 1 类激光人眼安全等级
    - 接收器视场角（FoV）为 12.4° x 5.4°，共 32 个像素，每个像素为 1.55° x 1.55°
    - 发射器（激光）光束角为 2° x 2°，可照亮 1 至 3 个像素
    - Typical distance range up to 30m
    - 可在高达 200k lux 的环境光照下工作
    - 传感器供电：5V，典型电流 33mA
  - InvenSense ICM-42688-P 六轴 IMU
  - 配备红外 LED，用于低照度环境下工作
- STM32F412CE MCU
- 单个 Pixhawk 标准 CAN 连接器（4 针 JST GH）
- 内置 CAN 终端电阻，可通过软件启用或关闭
- Pixhawk Standard Debug Connector (6 Pin JST SH)
- 通过 CAN 连接器提供 5V 电源
- 小尺寸：44.95mm x 29.4mm x 15.59mm
- 重量：9.38g（含外壳）
- 在印度设计和制造

![Agam FloRange 框图](../../assets/hardware/sensors/optical_flow/agam_florange_block_diagram.svg)

## 硬件安装

### 布线

使用随附线缆将 Agam FloRange 连接到飞行控制器的 CAN 总线。
For more information, refer to the [CAN Wiring](../can/index.md#wiring) instructions.

:::info
Agam FloRange 只有一个 CAN 连接器，因此通常位于总线末端。将 [CANNODE_TERM](#CANNODE_TERM) 设为 `1`，以启用其内置终端电阻。如果通过 CAN 分线器连接了多个模块，仅在距离飞行控制器最远的模块上启用终端电阻。
:::

| 针脚 | 信号                           | 线材    |
| -- | ---------------------------- | ----- |
| 1  | VCC (+5V) | Red   |
| 2  | CAN_H   | Black |
| 3  | CAN_L   | Black |
| 4  | GND                          | Black |

按[线缆接线基础 > UAVCAN 线缆](../assembly/cable_wiring.md#uavcan-cables)所述，将 VCC/GND 和 CAN_H/CAN_L 导线分别绞成双绞线对，以减少电磁干扰和串扰。

### Mounting

安装模块时，使传感器朝下，并使外壳上的 X 箭头指向机体前方。
This corresponds to the default value (`0`) of the parameter [SENS_FLOW_ROT](../advanced_config/parameter_reference.md#SENS_FLOW_ROT).
Change the parameter appropriately if using a different orientation.

## Firmware Setup

Agam FloRange 运行 [PX4 DroneCAN 固件](px4_cannode_fw.md)。它支持通过 CAN 总线更新固件及[动态节点分配](index.md#node-id-allocation)。也可以通过 [Pixhawk Debug Mini](../debug/swd_debug.md#pixhawk-debug-mini) 连接器使用 SWD 刷写固件，该连接器还提供[系统控制台](../debug/system_console.md)。
Agam FloRange 出厂时已预装较新的固件；如需自行构建并刷写最新固件，请参阅 [PX4 DroneCAN 固件 > 构建固件](px4_cannode_fw.md#building-the-firmware)。

- 固件构建目标：`agam_florange_default`
- 引导加载程序构建目标：`agam_florange_canbootloader`

## PX4 配置

:::info
如果上电时飞行控制器未安装 SD 卡，Agam FloRange 将无法启动。
:::

### Enable DroneCAN

步骤如下：

- 在 _QGroundControl_ 中，将参数 [UAVCAN_ENABLE](../advanced_config/parameter_reference.md#UAVCAN_ENABLE) 设为 `2` 以启用动态节点分配（若同时使用 [DroneCAN 电调](../dronecan/escs.md)，则设为 `3`），然后重启（参见[查找/更新参数](../advanced_config/parameters.md)）。
- 将 Agam FloRange 连接到飞行控制器的 CAN 总线（参见[接线](#wiring)）。

启用 DroneCAN 后，模块会在启动时被检测到。

有关 PX4 中 DroneCAN 配置的更多说明，请参阅 [DroneCAN > 启用 DroneCAN](index.md#enabling-dronecan)。

### 光流与测距仪参数

根据所使用的模块功能，在 _QGroundControl_ 中设置以下参数：

- 通过 [EKF2_OF_CTRL](../advanced_config/parameter_reference.md#EKF2_OF_CTRL) 启用光流融合（默认启用）。
- To optionally disable GPS aiding, set [EKF2_GPS_CTRL](../advanced_config/parameter_reference.md#EKF2_GPS_CTRL) to `0`.
- 启用 [UAVCAN_SUB_FLOW](../advanced_config/parameter_reference.md#UAVCAN_SUB_FLOW)。
- 启用 [UAVCAN_SUB_RNG](../advanced_config/parameter_reference.md#UAVCAN_SUB_RNG)。
- 将 [UAVCAN_RNG_MIN](../advanced_config/parameter_reference.md#UAVCAN_RNG_MIN) 设为 `0.08`，即光流传感器的最小工作距离；将 [UAVCAN_RNG_MAX](../advanced_config/parameter_reference.md#UAVCAN_RNG_MAX) 设为 `30`，即距离传感器的最大量程。
- 如果使用距离传感器进行高度辅助，请启用 [EKF2_RNG_CTRL](../advanced_config/parameter_reference.md#EKF2_RNG_CTRL)，并将 [EKF2_RNG_A_HMAX](../advanced_config/parameter_reference.md#EKF2_RNG_A_HMAX) 设为 `10`，将 [EKF2_RNG_QLTY_T](../advanced_config/parameter_reference.md#EKF2_RNG_QLTY_T) 设为 `0.2`。
- 将 [SENS_FLOW_MINHGT](../advanced_config/parameter_reference.md#SENS_FLOW_MINHGT) 设为 `0.08`，即光流传感器的最小工作高度；将 [SENS_FLOW_MAXHGT](../advanced_config/parameter_reference.md#SENS_FLOW_MAXHGT) 设为 `30`，即距离传感器的最大量程。
- 将 [SENS_FLOW_MAXR](../advanced_config/parameter_reference.md#SENS_FLOW_MAXR) 设为 `7.4`，以匹配 PAA3905 的最大光流角速率。
- 可通过设置 [EKF2_OF_POS_X](../advanced_config/parameter_reference.md#EKF2_OF_POS_X)、[EKF2_OF_POS_Y](../advanced_config/parameter_reference.md#EKF2_OF_POS_Y)、[EKF2_OF_POS_Z](../advanced_config/parameter_reference.md#EKF2_OF_POS_Z) 和 [EKF2_RNG_POS_X](../advanced_config/parameter_reference.md#EKF2_RNG_POS_X)、[EKF2_RNG_POS_Y](../advanced_config/parameter_reference.md#EKF2_RNG_POS_Y)、[EKF2_RNG_POS_Z](../advanced_config/parameter_reference.md#EKF2_RNG_POS_Z) 参数，补偿模块相对于机体重心的偏移。

当光流是水平位置和速度的唯一来源时，请降低水平位置增益 [MPC_XY_P](../advanced_config/parameter_reference.md#MPC_XY_P)（例如设为 `0.5`），以减小振荡。

## Agam FloRange 配置

您可能需要在模块上配置以下参数：

| Parameter                                                                                                                                          | 描述                                                           |
| -------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------ |
| <a id="CANNODE_NODE_ID"></a>[CANNODE_NODE_ID](../advanced_config/parameter_reference.md#CANNODE_NODE_ID) | CAN 节点 ID。设为 `0`（默认值）可使用动态节点分配，或设为 `1`-`125` 以使用静态节点 ID。     |
| <a id="CANNODE_TERM"></a>[CANNODE_TERM](../advanced_config/parameter_reference.md#CANNODE_TERM)                               | 内置 CAN 总线终端电阻。如果模块是 CAN 总线上的最后一个节点，请设为 `1`（参见[接线](#wiring)）。 |

## LED含义

- 蓝灯常亮表示运行正常。
- 蓝灯和红灯常亮（两灯同时亮）表示正在更新固件。
- 红灯常亮表示发生错误，或模块正在等待分配节点 ID（每次启动时会短暂出现）。

如果红色 LED 持续亮起，则表示存在错误，应检查以下项目：

- Make sure the flight controller has an SD card installed.
- 请确保在刷写 `agam_florange_default` 之前，Agam FloRange 已安装 `agam_florange_canbootloader`。
- 删除 SD 卡根目录和 `ufw` 目录中的二进制文件，然后尝试重新构建和刷写。

## 另见

- [DroneCAN > 节点 ID 分配](index.md#node-id-allocation)
- [PX4 DroneCAN 固件](px4_cannode_fw.md)
