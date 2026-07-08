# 飞行控制器移植指南

本主题主要针对希望将 PX4 移植到 _新_ 飞控硬件平台上的开发人员。

## PX4 架构

PX4 由两个主要层组成： 基于主机操作系统（NuttX、Linux 或任何其他 POSIX 平台如 Mac OS）的[板级支持与中间件层](../middleware/index.md)，以及应用程序（位于[src/modules](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules)目录下的飞行栈）。
更多信息请参阅[PX4架构概述](../concept/architecture.md)。

本指南仅关注主机操作系统和中间件，因为 应用层/飞行控制栈 可以在任何目标平台上运行。

## 飞行控制器配置文件分布位置

板卡启动和配置文件位于每个板卡厂商专属目录下的 [/boards](https://github.com/PX4/PX4-Autopilot/tree/main/boards/) 路径中（即 **boards/_VENDOR_/_MODEL_/**）。

例如，对于 FMUv5 飞控硬件平台：

- (所有) 板卡专用文件：[/boards/px4/fmu-v5](https://github.com/PX4/PX4-Autopilot/tree/main/boards/px4/fmu-v5)。<!-- 需指定 px4_version -->
- 构建配置：[/boards/px4/fmu-v5/default.px4board](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v5/default.px4board)。<!-- 需要 px4_version -->
- 板卡专用初始化文件：[/boards/px4/fmu-v5/init/rc.board_defaults](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v5/init/rc.board_defaults) <!-- 需指定 px4_version -->
  - 如果在主板目录下找到位于**init/rc.board**的文件，则该主板专用的初始化文件会自动包含在启动脚本中。
  - 该文件用于启动仅存在于特定主板上的传感器 (和其他东西)。
    它也可用于设置电路板的默认参数、UART映射以及任何其他特殊情况。
  - 对于FMUv5，您可以看到所有Pixhawk 4传感器均已启动，同时它还设置了更大的LOGGER_BUF缓冲区。

## 主机操作系统配置

本节介绍了移植每个受支持的主机操作系统到新的飞控板硬件平台上需要用到的配置文件的用途和所处位置。

### NuttX

参见[NuttX 板移植指南](porting_guide_nuttx.md)。

### Linux

基于 Linux 的飞控板不包含任何 操作系统和内核的配置。
这些功能已由该开发板可用的Linux镜像提供（该镜像需开箱即支持惯性传感器）。

- [boards/px4/raspberrypi/default.px4board](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/raspberrypi/default.px4board) - RPi 交叉编译。 <!-- NEED px4_version -->

## 中间件组件和配置

本节介绍各类中间件组件，以及将它们移植到新的飞行控制器硬件所需更新的配置文件。

### QuRT / Hexagon

- 启动脚本位于 [posix-configs/](https://github.com/PX4/PX4-Autopilot/tree/main/posix-configs)。 <!-- NEED px4_version -->
- 操作系统配置是默认 Linux 镜像的一部分（TODO: 需要提供 LINUX 镜像文件位置和程序烧写指南）。
- PX4 中间件配置位于[src/boards](https://github.com/PX4/PX4-Autopilot/tree/main/boards)。 <!-- NEED px4_version --> TODO: 添加BUS CONFIG

## RC UART 接线建议

通常建议通过独立的RX和TX引脚将RC连接至微控制器。但若RX和TX引脚连接在一起，则必须将UART置于单线模式以避免竞争冲突。此操作需通过板级配置文件和清单文件实现。示例可参考<a href="https://github.com/PX4/Firmware/blob/master/src/drivers/boards/px4fmu-v5/manifest.c">px4fmu-v5</a>。
如果 RX 和 TX 连在了一起，那么 UART 需要设置为单线模式以防止出现争用。
这可以用过对飞控板的配置文件和 manifest 文件进行更改来实现。
一个例子是 [px4fmu-v5](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v5/src/manifest.c)。 <!-- NEED px4_version -->

## Getting Your Board Supported

This page covers the _technical_ work of porting PX4 to new hardware.
The _process_ for getting that port reviewed, merged, and listed on the PX4 website, including board IDs, USB VID/PID, flight-test evidence, and maintenance expectations, is documented separately:

- [Manufacturer's PX4 Board Support Guide](../hardware/board_support_guide.md)

In short: build your own firmware target based on PX4, demonstrate stable flight on the current release, and open a pull request with your board support code, documentation, and flight logs.
The board support guide explains each step.

The PX4 project supports and maintains the [FMU standard reference hardware](../hardware/reference_design.md) and any boards compatible with the standard, including the [Pixhawk series](../flight_controller/pixhawk_series.md) (see the [full list of supported hardware](../flight_controller/index.md)). Boards merged into PX4 benefit from a port in the repository, firmware builds accessible from _QGroundControl_, compatibility with the rest of the ecosystem, and automated CI checks.

:::tip
The cost of maintaining a port is proportional to how far it diverges from the standard.
Consider that cost before deviating: staying close to the reference design lets you benefit from day-to-day PX4 development with minimal maintenance burden.
:::

Manufacturers are responsible for keeping their port up to date and working across PX4 releases.
The PX4 project reserves the right to refuse or remove ports that do not meet the project's requirements, and all contributors are expected to follow the [Code of Conduct](https://github.com/PX4/PX4-Autopilot/blob/main/CODE_OF_CONDUCT.md).

## 相关信息

- [设备驱动程序](../middleware/drivers.md) - 如何支持新的外围硬件（设备驱动程序）
- [构建代码](../dev_setup/building_px4.md) - 如何构建源代码并上传固件
- 受支持的飞行控制器：
  - [自动驾驶仪硬件](../flight_controller/index.md)
  - [支持的主板列表](https://github.com/PX4/PX4-Autopilot/#supported-hardware) (Github) - PX4-Autopilot 拥有专用代码的开发板
- [支持的外设](../peripherals/index.md)
