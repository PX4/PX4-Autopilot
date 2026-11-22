# CubePilot Here4 RTK GNSS (DroneCAN)

The [CubePilot Here4](https://docs.cubepilot.org/here-4/here-4-manual) is a dual-band (L1/L5) [RTK GNSS](../gps_compass/rtk_gps.md) module that connects to the flight controller via [DroneCAN](../dronecan/index.md).
It integrates a u-blox NEO-F9P receiver, an RM3100 compass, a barometer, an IMU and RGB LEDs.

:::warning
CubePilot has issued safety service bulletins for the Here4, including a [mandatory bootloader update before further flight](https://docs.cubepilot.org/service-bulletins-and-critical-notices/safety-service-bulletins/sb_0000010-mandatory-here4-bootloader-update-before-further-flight) (SB_0000010).
Check the bulletins before flying (see [Firmware](#firmware)).
:::

## Variants

- [Here4 Base](https://docs.cubepilot.org/here-4/here-4-base): RTK base station with an external antenna connector, for use with _QGroundControl_.
- [Here4 Blue](https://docs.cubepilot.org/here-4/here-4-blue): Here4 with integrated Bluetooth Remote ID (CubeID).

## Де купити

- [CubePilot Resellers](https://cubepilot.org/#/reseller/list)

## Характеристики обладнання

From the [CubePilot Here4 manual](https://docs.cubepilot.org/here-4/here-4-manual):

- GNSS receiver: u-blox NEO-F9P, dual-band (L1/L5)
  - Constellations: GPS, GLONASS, Galileo, BeiDou, QZSS
- Processor: STM32H757, running CubePilot's AP_Periph based firmware
- Compass: RM3100
- IMU: ICM42688
- Барометр: MS5611
- Interface: DroneCAN, two CAN ports (only one supported by the current firmware, see [Wiring](#wiring))
- Working temperature: -40 °C to +85 °C
- Size: 16 x 68 mm
- Weight: 60 g (with cable)

## Налаштування програмного забезпечення

### Підключення

Connect the Here4 to the `CAN1` or `CAN2` port of the flight controller using the supplied 4-pin CAN cable.
For more information, refer to the [CAN Wiring](../can/index.md#wiring) instructions.

:::warning
The Here4 has two CAN ports, but the current firmware only supports one of them: the port whose cable has green, blue, white and grey wires.
If the other port is used, the Here4 does not appear on the bus.
`uavcan status` then reports TX frames and an equally growing number of HW errors, with no RX frames and no online nodes.
:::

### Встановлення

Mount the module as far away from other electronics as possible, with its direction marker pointing towards the front of the vehicle.

## Конфігурація PX4

1. Set [UAVCAN_ENABLE](../advanced_config/parameter_reference.md#UAVCAN_ENABLE) to `2` (_Sensors Automatic Config_), or `3` if you also use [DroneCAN ESCs](../dronecan/escs.md), and reboot.
2. The Here4 is assigned a node ID via dynamic node allocation, and its GNSS and compass are used without further configuration.
3. To also use the Here4 barometer:
   - Enable [UAVCAN_SUB_BARO](../advanced_config/parameter_reference.md#UAVCAN_SUB_BARO) in PX4.
   - Enable the barometer on the Here4 itself, where it is disabled by default.
     Set its `BARO_ENABLE` parameter to `1` and save it to the node, either in the _QGroundControl_ parameters of the Here4 component, or from the [MAVLink Shell](../debug/mavlink_shell.md):

     ```sh
     uavcan param set <node-id> BARO_ENABLE 1
     uavcan param save <node-id>
     ```

     Then power cycle the Here4.

You can check that everything is detected with `uavcan status`.
The Here4 should be listed under _Online nodes_ with mode `OPERAT`, and the `gnss`, `mag` and (if enabled) `baro` sensors should each show a channel bound to its node ID.

DroneCAN configuration in PX4 is explained in more detail in [DroneCAN > PX4 Configuration](../dronecan/index.md#px4-configuration).
For RTK setup see [RTK GNSS](../gps_compass/rtk_gps.md#positioning-setup-configuration).

## Прошивка

CubePilot publishes the Here4 firmware and release notes at [GNSSPeriph-release](https://github.com/CubePilot/GNSSPeriph-release/releases).

PX4 can update the Here4 from the SD card, as described in [DroneCAN > Firmware Update](../dronecan/index.md#firmware-update):

1. Copy `Here4_FW.bin` from the release archive to the SD card.
2. Reboot with the Here4 connected.
   PX4 moves the file to `/fs/microsd/ufw/1043.bin` (1043 is the Here4 board ID) and updates the Here4 if its firmware differs.
   During the update `uavcan status` shows the Here4 with mode `SW_UPD`, and then `OPERAT` again once it has completed.

You can check the installed version with `listener device_information`: `firmware_version` shows the major and minor version followed by the git hash of the build in decimal (DroneCAN does not report the patch version).

If the Here4 stays in `SW_UPD` without completing, remove the file from `/fs/microsd/ufw/` and update the Here4 with CubePilot's tools instead (see the bulletin below).

According to [SB_0000010](https://docs.cubepilot.org/service-bulletins-and-critical-notices/safety-service-bulletins/sb_0000010-mandatory-here4-bootloader-update-before-further-flight), a Here4 with the updated bootloader breathes blue at boot.
If yours does not, follow the bulletin: update the firmware, then set the Here4 parameter `FLASH_BOOTLOADER` to `1` to update the bootloader.

## Дивіться також

- [CubePilot Here4 Manual](https://docs.cubepilot.org/here-4/here-4-manual) (docs.cubepilot.org)
