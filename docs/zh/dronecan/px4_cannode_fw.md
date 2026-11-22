# PX4 DroneCAN 固件

PX4 可以作为固件运行在多种 DroneCAN 外设上。这样做有多项优势：

- PX4 内置了[多种](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers)传感器和外设组件的驱动程序。
- PX4 具有稳健的 DroneCAN 驱动实现，已经过多年的现场测试。
- PX4 一直在持续开发。您可以持续获得最新改进。
- PX4 的状态估计和控制代码使创建集成 AHRS 模块等“智能”CAN 节点变得容易。
- 固件完全开源（PX4 采用 BSD 许可证）。

## 构建固件

按照 [PX4 构建文档](../dev_setup/building_px4.md)操作，方法与为飞行控制器构建固件相同。设备构建配置存放在[此处](https://github.com/PX4/PX4-Autopilot/tree/main/boards)。安装 [PX4 工具链](../dev_setup/dev_env.md)后，克隆源代码并构建。例如，为 [Ark Flow](ark_flow.md) 目标构建：

```sh
git clone --recursive https://github.com/PX4/PX4-Autopilot
cd PX4-Autopilot
make ark_can-flow_default
```

这将在 **build/ark_can-flow_default** 中生成名为 **XX-X.X.XXXXXXXX.uavcan.bin** 的输出文件。按照 [DroneCAN 固件更新](index.md#firmware-update)中的说明刷写固件。

## 预编译固件

PX4 发布版本为每个 CAN 节点附带两个镜像：

- `<target>.uavcan.bin`：应用程序，通过 DroneCAN 刷写
- `<target>_canbootloader.bin`：引导加载程序，通过 SWD 刷写（STM32：`0x08000000`）

这些目标不使用 `.px4` 封装格式。

## 配置

### 静态节点 ID

默认情况下，DroneCAN 设备使用[动态节点分配（DNA）](index.md#node-id-allocation)，从飞行控制器自动获取唯一的节点 ID。不过，您也可以使用 [CANNODE_NODE_ID](../advanced_config/parameter_reference.md#CANNODE_NODE_ID) 参数配置静态节点 ID。

配置静态节点 ID 的步骤如下：

1. 使用 [QGroundControl](index.md#qgc-cannode-parameter-configuration) 将 [CANNODE_NODE_ID](../advanced_config/parameter_reference.md#CANNODE_NODE_ID) 设为 1-125 之间的值
2. 重启设备

要恢复动态分配，请将 `CANNODE_NODE_ID` 重新设为 0。请注意，切换回动态分配后，飞行控制器通常仍会分配先前使用的节点 ID（这是 DNA 的正常行为）。

:::warning
使用静态节点 ID 时，必须确保 CAN 总线上的每个设备都具有唯一的节点 ID。为两个设备配置相同的 ID 会导致通信冲突。
:::

## 开发人员信息

本节面向希望为 PX4 自动驾驶仪添加新 DroneCAN 硬件支持的开发者。

### 安装 DroneCAN 引导加载程序

:::warning
DroneCAN 设备通常在出厂时已预装引导加载程序。除非您正在开发 DroneCAN 设备，
或已（意外）损坏或擦除引导加载程序，否则请勿执行本节中的操作。
:::

PX4 项目包含适用于 STM32 设备的标准 DroneCAN 引导加载程序。

引导加载程序占用 Flash 起始位置的 8-16 KB 空间，是上电后首先执行的代码。通常，引导加载程序会执行设备底层初始化，自动确定 CAN 总线波特率，作为 [DroneCAN 动态节点 ID 客户端](index.md#node-id-allocation)获取唯一的节点 ID，并等待飞行控制器确认后再启动应用程序。

这一过程确保 DroneCAN 设备能够在应用程序固件无效或损坏时无需用户干预即可恢复，同时也支持自动固件更新。

构建引导加载程序固件时，指定相同的外设目标，并使用 `canbootloader` 构建配置替代 `default` 配置。

例如，为 [Ark Flow](ark_flow.md) 目标构建：

```sh
git clone --recursive https://github.com/PX4/PX4-Autopilot
cd PX4-Autopilot
make ark_can-flow_canbootloader
```

随后可使用您熟悉的 SWD/JTAG 调试器将二进制文件刷写到微控制器，例如 [Black Magic Probe](https://black-magic.org/index.html)、[ST-Link](https://www.st.com/en/development-tools/st-link-v2.html) 或 [Segger JLink](https://www.segger.com/products/debug-probes/j-link/)。

在 STM32 板上，`canbootloader` 构建还会将二进制文件写入 `boards/<vendor>/<board>/extras/<vendor>_<board>_bootloader.bin`。如果该文件存在，`default` 构建还会生成 `<vendor>_<board>_default_with_bootloader.bin`：将引导加载程序和应用程序合并到一个镜像中，并刷写到 `0x08000000`（`st-flash write <file> 0x8000000`）。该镜像覆盖参数扇区，因此刷写它会重置节点参数。

配置了 `CONFIG_SYSTEMCMDS_BL_UPDATE=y` 的开发板会将该引导加载程序嵌入应用程序固件，并在设置 [SYS_BL_UPDATE](../advanced_config/parameter_reference.md#SYS_BL_UPDATE) 后约 2 秒内重写引导加载程序，无需重启。在飞行控制器的 [MAVLink Shell](../debug/mavlink_shell.md) 中执行：

```sh
uavcan param set <node_id> SYS_BL_UPDATE 1
```

节点会报告 `bootloader updated`、`bootloader unchanged` 或 `bootloader update failed: <reason>`，飞行控制器将其记录为 `uavcan:<node_id>:uavcannode`。更新期间请勿关闭节点电源。

### 固件内部结构

外设固件的工作方式与飞行控制器固件构建在大多数方面相同。不过，大多数模块均被禁用，只有传感器驱动、DroneCAN 驱动和内部基础设施（uORB 等）处于启用状态。

DroneCAN 通信由 [uavcannode](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/uavcannode) 模块处理。该驱动负责数据生产端的通信：从 uORB 获取传感器或执行器数据，使用 DroneCAN 库进行序列化，然后通过 CAN 发布。将来，该模块可能会与 [uavcan](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/uavcan) 模块合并。后者负责飞行控制器端（数据消费端）的驱动，从 CAN 总线接收并反序列化数据，然后通过 uORB 发布。

除标准的原始二进制、ELF 和 `.px4` 固件文件外，构建系统还会生成专用固件二进制文件，可使用 [PX4's DroneCAN flashing support]（PX4 的 DroneCAN 刷写支持）或 DroneCAN GUI，通过 DroneCAN 引导加载程序进行刷写。
