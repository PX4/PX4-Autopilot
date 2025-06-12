# PX4 DroneCAN Firmware

PX4 can run as the firmware on many DroneCAN peripherals. There are multiple benefits to this:

- PX4 has built-in drivers for a [wide range](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers) of sensors and peripherals components.
- PX4 has a robust DroneCAN driver implementation that has undergone multiple years of field testing.
- PX4 is continuously being developed. You routinely get access to the latest improvements.
- PX4's estimation and control code makes it easy to create "smart" cannodes like integrated AHRS modules.
- The firmware is completely open source (PX4 is BSD licensed).

## Building the Firmware

Follow the [PX4 building docs](../dev_setup/building_px4.md) just as you would to build firmware for a flight controller. Device build configurations are stored [here](https://github.com/PX4/PX4-Autopilot/tree/main/boards). After installing the [PX4 toolchain](../dev_setup/dev_env.md), clone the sources and build. For example, to build for the [Ark Flow](ark_flow.md) target:

```sh
git clone --recursive https://github.com/PX4/PX4-Autopilot
cd PX4-Autopilot
make ark_can-flow_default
```

This will create an output in **build/ark_can-flow_default** named **XX-X.X.XXXXXXXX.uavcan.bin**. Follow the instructions at [DroneCAN firmware update](index.md#firmware-update) to flash the firmware.

## Developer Information

This section has information that is relevant to developers who want to add support for new DroneCAN hardware to the PX4 Autopilot.

### DroneCAN Bootloader Installation

:::warning
DroneCAN devices typically ship with a bootloader pre-installed.
Do not follow the instructions in this section unless you are developing DroneCAN devices,
or have (accidentally) corrupted/wiped your bootloader.
:::

The PX4 project includes a standard DroneCAN bootloader for STM32 devices.

The bootloader occupies the first 8-16 KB of flash, and is the first code executed on power-up.
Typically the bootloader performs low-level device initialization, automatically determines the CAN
bus baud rate, acts as a [DroneCAN dynamic node ID client](index.md#node-id-allocation) to obtain a unique node ID, and waits for confirmation from the flight controller before proceeding with application boot.

This process ensures that a DroneCAN device can recover from invalid or corrupted application firmware without user intervention, and also permits automatic firmware updates.

Build the bootloader firmware by specifying the same peripheral target with the `canbootloader` build configuration instead of the `default` configuration.

For example, to build for the [Ark Flow](ark_flow.md) target:

```sh
git clone --recursive https://github.com/PX4/PX4-Autopilot
cd PX4-Autopilot
make ark_can-flow_canbootloader
```

The binary can then be flashed to the microcontroller using your favorite SWD/JTAG debugger, such as the [Black Magic Probe](https://black-magic.org/index.html), [ST-Link](https://www.st.com/en/development-tools/st-link-v2.html), or [Segger JLink](https://www.segger.com/products/debug-probes/j-link/).

### Firmware Internals

For the most part, peripheral firmware works the same way as flight controller firmware builds.
However, most modules are disabled - only the sensor drivers, DroneCAN driver, and internal infrastructure (uORB, etc.) are enabled.

DroneCAN communication is handled by the [uavcannode](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/uavcannode) module.
This driver handles producer-side communication - it takes sensor/actuator data from uORB, serializes it using the DroneCAN libraries, and publishes it over CAN.
In the future, this will likely be merged with the [uavcan](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/uavcan) module which handles flight controller side (consumer side) drivers, which receive/deserialize data from the CAN bus and publish them over uORB.

The build system also produces firmware binaries designed to be flashed through a DroneCAN bootloader via [PX4's DroneCAN flashing support] or the DroneCAN GUI, in addition to the standard raw binary, ELF, and `.px4` firmware files.
