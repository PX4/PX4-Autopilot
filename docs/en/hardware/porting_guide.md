# Flight Controller Porting Guide

This topic is for developers who want to port PX4 to work with _new_ flight controller hardware.

## PX4 Architecture

PX4 consists of two main layers: The [board support and middleware layer](../middleware/index.md) on top of the host OS (NuttX, Linux or any other POSIX platform like Mac OS), and the applications (Flight Stack in [src/modules](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules)\). Please reference the [PX4 Architectural Overview](../concept/architecture.md) for more information.

This guide is focused only on the host OS and middleware as the applications/flight stack will run on any board target.

## Flight Controller Configuration File Layout

Board startup and configuration files are located under [/boards](https://github.com/PX4/PX4-Autopilot/tree/main/boards/) in each board's vendor-specific directory (i.e. **boards/_VENDOR_/_MODEL_/**).

For example, for FMUv5:

- (All) Board-specific files: [/boards/px4/fmu-v5](https://github.com/PX4/PX4-Autopilot/tree/main/boards/px4/fmu-v5).<!-- NEED px4_version -->
- Build configuration: [/boards/px4/fmu-v5/default.px4board](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v5/default.px4board).<!-- NEED px4_version -->
- Board-specific initialisation file: [/boards/px4/fmu-v5/init/rc.board_defaults](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v5/init/rc.board_defaults) <!-- NEED px4_version -->
  - A board-specific initialisation file is automatically included in startup scripts if found under the boards directory at **init/rc.board**.
  - The file is used to start sensors (and other things) that only exist on a particular board.
    It may also be used to set a board's default parameters, UART mappings, and any other special cases.
  - For FMUv5 you can see all the Pixhawk 4 sensors being started, and it also sets a larger LOGGER_BUF.

## Host Operating System Configuration

This section describes the purpose and location of the configuration files required for each supported host operating system to port them to new flight controller hardware.

### NuttX

See [NuttX Board Porting Guide](porting_guide_nuttx.md).

### Linux

Linux boards do not include the OS and kernel configuration.
These are already provided by the Linux image available for the board (which needs to support the inertial sensors out of the box).

- [boards/px4/raspberrypi/default.px4board](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/raspberrypi/default.px4board) - RPi cross-compilation. <!-- NEED px4_version -->

## Middleware Components and Configuration

This section describes the various middleware components, and the configuration file updates needed to port them to new flight controller hardware.

### QuRT / Hexagon

- The start script is located in [posix-configs/](https://github.com/PX4/PX4-Autopilot/tree/main/posix-configs). <!-- NEED px4_version -->
- The OS configuration is part of the default Linux image (TODO: Provide location of LINUX IMAGE and flash instructions).
- The PX4 middleware configuration is located in [src/boards](https://github.com/PX4/PX4-Autopilot/tree/main/boards). <!-- NEED px4_version --> TODO: ADD BUS CONFIG

## RC UART Wiring Recommendations

It is generally recommended to connect RC via separate RX and TX pins to the microcontroller.
If however RX and TX are connected together, the UART has to be put into singlewire mode to prevent any contention.
This is done via board config and manifest files.
One example is [px4fmu-v5](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v5/src/manifest.c). <!-- NEED px4_version -->

## Officially Supported Hardware

The PX4 project supports and maintains the [FMU standard reference hardware](../hardware/reference_design.md) and any boards that are compatible with the standard.
This includes the [Pixhawk-series](../flight_controller/pixhawk_series.md) (see the user guide for a [full list of officially supported hardware](../flight_controller/index.md)).

Every officially supported board benefits from:

- PX4 Port available in the PX4 repository
- Automatic firmware builds that are accessible from _QGroundControl_
- Compatibility with the rest of the ecosystem
- Automated checks via CI - safety remains paramount to this community
- [Flight testing](../test_and_ci/test_flights.md)

We encourage board manufacturers to aim for full compatibility with the [FMU spec](https://pixhawk.org/).
With full compatibility you benefit from the ongoing day-to-day development of PX4, but have none of the maintenance costs that come from supporting deviations from the specification.

:::tip
Manufacturers should carefully consider the cost of maintenance before deviating from the specification (the cost to the manufacturer is proportional to the level of divergence).
:::

We welcome any individual or company to submit their port for inclusion in our supported hardware, provided they are willing to follow our [Code of Conduct](https://github.com/PX4/PX4-Autopilot/blob/main/CODE_OF_CONDUCT.md) and work with the Dev Team to provide a safe and fulfilling PX4 experience to their customers.

It's also important to note that the PX4 dev team has a responsibility to release safe software, and as such we require any board manufacturer to commit any resources necessary to keep their port up-to-date, and in a working state.

If you want to have your board officially supported in PX4:

- Your hardware must be available in the market (i.e. it can be purchased by any developer without restriction).
- Hardware must be made available to the PX4 Dev Team so that they can validate the port (contact [lorenz@px4.io](mailto:lorenz@px4.io) for guidance on where to ship hardware for testing).
- The board must pass full [test suite](../test_and_ci/index.md) and [flight testing](../test_and_ci/test_flights.md).

**The PX4 project reserves the right to refuse acceptance of new ports (or remove current ports) for failure to meet the requirements set by the project.**

You can reach out to the core developer team and community on the [official support channels](../contribute/support.md).

## Related Information

- [Device Drivers](../middleware/drivers.md) - How to support new peripheral hardware (device drivers)
- [Building the Code](../dev_setup/building_px4.md) - How to build source and upload firmware
- Supported Flight Controllers:
  - [Autopilot Hardware](../flight_controller/index.md)
  - [Supported boards list](https://github.com/PX4/PX4-Autopilot/#supported-hardware) (Github) - Boards for which PX4-Autopilot has specific code
- [Supported Peripherals](../peripherals/index.md)
