# 飞行控制器移植指南

This topic is for developers who want to port PX4 to work with _new_ flight controller hardware.

## PX4 架构

PX4 consists of two main layers: The [board support and middleware layer](../middleware/index.md) on top of the host OS (NuttX, Linux or any other POSIX platform like Mac OS), and the applications (Flight Stack in [src/modules](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules)\). Please reference the [PX4 Architectural Overview](../concept/architecture.md) for more information.

本指南仅关注主机操作系统和中间件，因为 应用层/飞行控制栈 可以在任何目标平台上运行。

## 飞行控制器配置文件分布位置

Board startup and configuration files are located under [/boards](https://github.com/PX4/PX4-Autopilot/tree/main/boards/) in each board's vendor-specific directory (i.e. **boards/_VENDOR_/_MODEL_/**).

例如，对于 FMUv5 飞控硬件平台：

- (All) Board-specific files: [/boards/px4/fmu-v5](https://github.com/PX4/PX4-Autopilot/tree/main/boards/px4/fmu-v5).<!-- NEED px4_version -->
- Build configuration: [/boards/px4/fmu-v5/default.px4board](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v5/default.px4board).<!-- NEED px4_version -->
- Board-specific initialisation file: [/boards/px4/fmu-v5/init/rc.board_defaults](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v5/init/rc.board_defaults) <!-- NEED px4_version -->
  - A board-specific initialisation file is automatically included in startup scripts if found under the boards directory at **init/rc.board**.
  - 该文件用于启动仅存在于特定主板上的传感器 (和其他东西)。
    It may also be used to set a board's default parameters, UART mappings, and any other special cases.
  - For FMUv5 you can see all the Pixhawk 4 sensors being started, and it also sets a larger LOGGER_BUF.

## 主机操作系统配置

本节介绍了移植每个受支持的主机操作系统到新的飞控板硬件平台上需要用到的配置文件的用途和所处位置。

### NuttX

See [NuttX Board Porting Guide](porting_guide_nuttx.md).

### Linux

基于 Linux 的飞控板不包含任何 操作系统和内核的配置。
Linux boards do not include the OS and kernel configuration. These are already provided by the Linux image available for the board (which needs to support the inertial sensors out of the box).

- [boards/px4/raspberrypi/default.px4board](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/raspberrypi/default.px4board) - RPi cross-compilation. <!-- NEED px4_version -->

## 中间件组件和配置

本节介绍各类中间件组件，以及将它们移植到新的飞行控制器硬件所需更新的配置文件。

### QuRT / Hexagon

- The start script is located in [posix-configs/](https://github.com/PX4/PX4-Autopilot/tree/main/posix-configs). <!-- NEED px4_version -->
- 操作系统配置是默认 Linux 镜像的一部分（TODO: 需要提供 LINUX 镜像文件位置和程序烧写指南）。
- The PX4 middleware configuration is located in [src/boards](https://github.com/PX4/PX4-Autopilot/tree/main/boards). <!-- NEED px4_version --> TODO: ADD BUS CONFIG

## RC UART 接线建议

It is generally recommended to connect RC via separate RX and TX pins to the microcontroller. If however RX and TX are connected together, the UART has to be put into singlewire mode to prevent any contention. This is done via board config and manifest files. One example is <a href="https://github.com/PX4/Firmware/blob/master/src/drivers/boards/px4fmu-v5/manifest.c">px4fmu-v5</a>.
如果 RX 和 TX 连在了一起，那么 UART 需要设置为单线模式以防止出现争用。
这可以用过对飞控板的配置文件和 manifest 文件进行更改来实现。
One example is [px4fmu-v5](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v5/src/manifest.c). <!-- NEED px4_version -->

## 官方支持的硬件

The PX4 project supports and maintains the [FMU standard reference hardware](../hardware/reference_design.md) and any boards that are compatible with the standard.
This includes the [Pixhawk-series](../flight_controller/pixhawk_series.md) (see the user guide for a [full list of officially supported hardware](../flight_controller/index.md)).

每个受官方支持的飞控板平台都将受益于：

- PX4 项目仓库中可用的 PX4 移植
- Automatic firmware builds that are accessible from _QGroundControl_
- 与生态系统其余部分的兼容性
- 可通过 CI 进行自动检查 — 安全仍是这个社区的重中之重
- [Flight testing](../test_and_ci/test_flights.md)

We encourage board manufacturers to aim for full compatibility with the [FMU spec](https://pixhawk.org/).
We encourage board manufacturers to aim for full compatibility with the <a href="https://pixhawk.org/">FMU spec</a>. With full compatibility you benefit from the ongoing day-to-day development of PX4, but have none of the maintenance costs that come from supporting deviations from the specification.

:::tip
Manufacturers should carefully consider the cost of maintenance before deviating from the specification (the cost to the manufacturer is proportional to the level of divergence).
:::

We welcome any individual or company to submit their port for inclusion in our supported hardware, provided they are willing to follow our [Code of Conduct](https://github.com/PX4/PX4-Autopilot/blob/main/CODE_OF_CONDUCT.md) and work with the Dev Team to provide a safe and fulfilling PX4 experience to their customers.

如果你想让你的飞控板被 PX4 项目正式支持：

If you want to have your board officially supported in PX4:

- 你的硬件必须在市场上可用（例如它可以被任何开发人员不受限制地购买到） 。
- Hardware must be made available to the PX4 Dev Team so that they can validate the port (contact [lorenz@px4.io](mailto:lorenz@px4.io) for guidance on where to ship hardware for testing).
- The board must pass full [test suite](../test_and_ci/index.md) and [flight testing](../test_and_ci/test_flights.md).

**The PX4 project reserves the right to refuse acceptance of new ports (or remove current ports) for failure to meet the requirements set by the project.**

You can reach out to the core developer team and community on the [official support channels](../contribute/support.md).

## 相关信息

- [Device Drivers](../middleware/drivers.md) - How to support new peripheral hardware (device drivers)
- [Building the Code](../dev_setup/building_px4.md) - How to build source and upload firmware
- 受支持的飞行控制器：
  - [Autopilot Hardware](../flight_controller/index.md)
  - [Supported boards list](https://github.com/PX4/PX4-Autopilot/#supported-hardware) (Github) - Boards for which PX4-Autopilot has specific code
- [Supported Peripherals](../peripherals/index.md)
