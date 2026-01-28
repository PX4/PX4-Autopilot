# 飞行控制器移植指南

本主题主要针对希望将 PX4 移植到 _新_ 飞控硬件平台上的开发人员。

## PX4 架构

PX4 由两个主要层组成： 基于主机操作系统（NuttX、Linux 或任何其他 POSIX 平台如 Mac OS）的[板级支持与中间件层](../middleware/index.md)，以及应用程序（位于[src/modules](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules)目录下的飞行栈）。 更多信息请参阅[PX4架构概述](../concept/architecture.md)。

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

## 官方支持的硬件

PX4项目支持并维护[FMU标准参考硬件](../hardware/reference_design.md)以及任何符合该标准的开发板。
这包括[Pixhawk系列](../flight_controller/pixhawk_series.md)（详见用户指南中的[官方支持硬件完整列表](../flight_controller/index.md)）。

每个受官方支持的飞控板平台都将受益于：

- PX4 项目仓库中可用的 PX4 移植
- 可通过_QGroundControl_访问的自动固件构建
- 与生态系统其余部分的兼容性
- 可通过 CI 进行自动检查 — 安全仍是这个社区的重中之重
- [飞行测试](../test_and_ci/test_flights.md)

我们鼓励电路板制造商致力于实现与[FMU规范](https://pixhawk.org/)的完全兼容。
我们鼓励电路板制造商致力于实现与<a href="https://pixhawk.org/">FMU规范</a>的完全兼容。通过完全兼容，您既能受益于PX4持续的日常开发成果，又无需承担因支持偏离规范而产生的维护成本。

:::tip
制造商在偏离规格时应仔细考虑维护成本（制造商的成本与偏离程度成正比）。
:::

我们欢迎任何个人或公司提交其移植版本，将其纳入我们支持的硬件范围。前提是他们愿意遵守我们的[行为准则](https://github.com/PX4/PX4-Autopilot/blob/main/CODE_OF_CONDUCT.md)，并与开发团队协作，为用户提供安全且令人满意的PX4体验。

值得注意的是，PX4开发团队有责任发布安全的软件。因此，我们要求所有主板制造商投入必要资源，确保其移植版本保持最新且处于可运行状态。

如果你想让你的飞控板被 PX4 项目正式支持：

- 你的硬件必须在市场上可用（例如它可以被任何开发人员不受限制地购买到） 。
- 必须向PX4开发团队提供硬件设备，以便他们验证移植工作（有关硬件寄送地址的指导，请联系[lorenz@px4.io](mailto:lorenz@px4.io)）。
- 该板必须通过完整的[测试套件](../test_and_ci/index.md)和[飞行测试](../test_and_ci/test_flights.md)。

**PX4项目保留拒绝接受不符合项目要求的新端口（或移除现有端口）的权利。**

您可通过[官方支持渠道](../contribute/support.md)联系核心开发团队及社区。

## 相关信息

- [设备驱动程序](../middleware/drivers.md) - 如何支持新的外围硬件（设备驱动程序）
- [构建代码](../dev_setup/building_px4.md) - 如何构建源代码并上传固件
- 受支持的飞行控制器：
  - [自动驾驶仪硬件](../flight_controller/index.md)
  - [支持的主板列表](https://github.com/PX4/PX4-Autopilot/#supported-hardware) (Github) - PX4-Autopilot 拥有专用代码的开发板
- [支持的外设](../peripherals/index.md)
