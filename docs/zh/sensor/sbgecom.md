# SBG Systems INS/AHRS (Pulse, Ellipse, etc.)

[SBG-Systems](https://www.sbg-systems.com/) designs, manufactures, and support an extensive range of state-of-the-art inertial sensors such as Inertial Measurement Units (IMU), Attitude and Heading Reference Systems (AHRS), Inertial Navigation Systems with embedded GNSS (INS/GNSS), and so on.

PX4 supports [all SBG Systems products](https://www.sbg-systems.com/products/) and can use these as an [external INS](../sensor/inertial_navigation_systems.md) (bypassing/replacing the EKF2 estimator), or as a source of raw sensor data provided to the navigation estimator.

![Ellipse](../../assets/hardware/sensors/inertial/ellipse-inertial-navigation-system.png)

## 综述

SBG Systems products provide a range of benefits to PX4 users and can be integrated for:

- Higher accuracy heading, pitch, and roll estimates
- More robust and reliable GNSS positioning
- Improved positioning and attitude performance in GNSS-contested environments
- Performance under challenging dynamic conditions (e.g. catapult launches, VTOL operations, high-g or high angular rate operations)

The sbgECom PX4 driver is streamlined to provide a simple plug-and-play architecture, removing engineering obstacles and allowing the acceleration of the design, development, and launch of platforms to keep pace with the rapid rate of innovation.

The driver supports [all SBG Systems products](https://www.sbg-systems.com/products/).
In particular the following systems are recommended:

- **Pulse:** Recommended for fixed-wing systems without hovering, where static heading is not necessary.
- **Ellipse:** Recommended for multicopter systems where hovering and low dynamics requires the use of static heading.

## 购买渠道

SBG Systems solutions are available directly from [MySBG](https://my.sbg-systems.com) (FR) or through their Global Sales Representatives. For more information on their solutions or for international orders, please contact contact@sbg-systems.com.

## 硬件安装

### 布线

Connect any unused flight controller serial interface, such as a spare `GPS` or `TELEM` port, to the SBG Systems product MAIN port (required by PX4).

### Mounting

The SBG Systems product sensor can be mounted in any orientation, in any position on the vehicle, without regard to center of gravity.
All SBG Systems product sensors default to a coordinate system of x-forward, y-right, and z-down, making the default mounting as connector-back, base down.
This can be changed to any rigid rotation using the sbgECom Reference Frame Rotation register.

If using a GNSS-enabled product, the GNSS antenna must be mounted rigidly with respect to the inertial sensor and with an unobstructed sky view. If using a dual-GNSS-enabled product (Ellipse-D), the secondary antenna must be mounted rigidly with respect to the primary antenna and the inertial sensor with an unobstructed sky view.

For more mounting and configuration requirements and recommendations, see the relevant [SBG SUPPORT CENTER](https://support.sbg-systems.com/sc).

## Firmware Configuration

### PX4 配置

To use the sbgECom driver:

1. Include the module in firmware in the [kconfig board configuration](../hardware/porting_guide_config.md#px4-board-configuration-kconfig) by setting the kconfig variables: `CONFIG_DRIVERS_INS_SBGECOM` or `CONFIG_COMMON_INS`.

2. [Set the parameter](../advanced_config/parameters.md) [SENS_SBG_CFG](../advanced_config/parameter_reference.md#SENS_SBG_CFG) to the hardware port connected to the SBG Systems product (for more information see [Serial Port Configuration](../peripherals/serial_configuration.md)).

   ::: warning
   Disable or change port of other sensors that are using the same one, for example [GPS_1_CONFIG](../advanced_config/parameter_reference.md#GPS_1_CONFIG) if using GPS1 port.

:::

3. Set [SBG_BAUDRATE](../advanced_config/parameter_reference.md#SBG_BAUDRATE) to the desired default baudrate value.

4. Allow the sbgECom driver to initialize by restarting PX4.

5. Configure driver to provide IMU data, GNSS data and INS :

   1. Set [SBG_MODE](../advanced_config/parameter_reference.md#SBG_MODE) to the desired mode.
   2. Make sensor module select sensors by enabling [SENS_IMU_MODE](../advanced_config/parameter_reference.md#SENS_IMU_MODE).
   3. Prioritize SBG Systems sensors using [CAL_GYROn_PRIO](../advanced_config/parameter_reference.md#CAL_GYRO0_PRIO), [CAL_ACCn_PRIO](../advanced_config/parameter_reference.md#CAL_ACC0_PRIO), [CAL_BAROn_PRIO](../advanced_config/parameter_reference.md#CAL_BARO0_PRIO), [CAL_MAGn_PRIO](../advanced_config/parameter_reference.md#CAL_MAG0_PRIO), where _n_ is the instance number of the IMU component (0, 1, etc.).

   ::: tip
   In most cases the external IMU (SBG) is the highest-numbered.
   You can get a list of the IMU components available using [`uorb top -1`](../middleware/uorb.md#uorb-top-command), you can differentiate between them using the [`listener`](../modules/modules_command.md#listener) command and looking through the data, or just the rates.

   Alternatively, you can check [CAL_GYROn_ID](../advanced_config/parameter_reference.md#CAL_GYRO0_ID) to see the device id.
   The priority is 0-255, where 0 is entirely disabled and 255 is highest priority.

:::

   ::: warning
   When configuring both SBG Systems and Pixhawk sensors to have non-zero priority, if the selected sensor is errored (timeout), it can change during operation without being notified.
   In this case, MAVLink messages will be updated with the newly selected sensor.

   If you don't want to have this fallback mechanism, you must disable unwanted sensors.

:::
   4. If using the sbgECom as an INS, disable EKF2 using [EKF2_EN](../advanced_config/parameter_reference.md#EKF2_EN).

6. Restart PX4.

Once enabled, the module will be detected on boot.
IMU data should be published at 200Hz.

## SBG Systems Configuration

All High Performance and Ellipse 3.0 and higher SBG Systems INS can be configured directly from PX4 firmware:

1. Enable [SBG_CONFIGURE_EN](../advanced_config/parameter_reference.md#SBG_CONFIGURE_EN).

2. Provide a JSON file `sbg_settings.json` containing SBG Systems INS settings to be applied in your PX4 board `extras` directory (ex: `boards/px4/fmu-v5/extras`). The settings JSON file will be installed in `/etc/extras/sbg_settings.json` on the board.

   ::: tip
   The settings can be retrieved using [sbgEComAPI](https://github.com/SBG-Systems/sbgECom/tree/main/tools/sbgEComApi) or [sbgInsRestApi](https://developer.sbg-systems.com/sbgInsRestApi/1.3/#tag/Settings) and then modified as a JSON file.

:::

   ::: tip
   The settings file can be provided in the SD card in q`/fs/microsd/etc/extras/sbg_settings.json` to avoid rebuilding a new firmware to change JSON settings file.

:::

3. For testing purpose, it's also possible to modify SBG Systems INS settings on the fly:
   - By passing a JSON file path as argument when starting sbgecom driver (ex: `sbgecom start -f /fs/microsd/new_sbg_settings.json`)
   - By passing a JSON string as argument when starting sbgecom driver: (ex: `sbgecom start -s {"output":{"comA":{"messages":{"airData":"onChange"}}}}`)

For older Ellipse SBG Systems INS or to configure any SBG Systems INS directly, all commands and registers can be found in the [SBG SUPPORT CENTER](https://support.sbg-systems.com/sc).

:::warning
If the baudrate of the serial port on the INS product (used to communicate with PX4) is changed, the parameter [SBG_BAUDRATE](../advanced_config/parameter_reference.md#SBG_BAUDRATE) must be changed to match.
:::

## Published Data

Upon initialization, the driver should print the following information to console (printed using `PX4_INFO`)

- Unit model number
- Unit hardware version
- Unit serial number
- Unit firmware number

This should be accessible using the [`dmesg`](../modules/modules_system.md#dmesg) command.

The sbgECom driver always publishes the unit's data to the following uORB topics:

- [sensor_accel](../msg_docs/SensorAccel.md)
- [sensor_gyro](../msg_docs/SensorGyro.md)
- [sensor_mag](../msg_docs/SensorMag.md)

if configured as a GNSS, publishes:

- [sensor_gps](../msg_docs/SensorGps.md)

and, if configured as an INS, publishes:

- [estimator_status](../msg_docs/EstimatorStatus.md)
- [vehicle_local_position](../msg_docs/VehicleLocalPosition.md)
- [vehicle_global_positon](../msg_docs/VehicleGlobalPosition.md)
- [vehicle_attitude](../msg_docs/VehicleAttitude.md)

:::tip
Published topics can be viewed using the `listener` command.
:::

## Hardware Specifications

- [Product Briefs](https://www.sbg-systems.com/products/)
- [Datasheets](https://www.sbg-systems.com/contact/#products)
