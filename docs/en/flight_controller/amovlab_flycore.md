# Amovlab Flycore

::: warning
PX4 does not manufacture this (or any) autopilot.
Contact the [manufacturer](https://amovlab.com/) for hardware support or compliance issues.
:::

::: info
This flight controller is intended for inclusion in the [manufacturer supported](../flight_controller/autopilot_manufacturer_supported.md) board list.
:::

## Sensor and bus configuration

The default Flycore PX4 port matches the hardware as shipped:

- **Onboard sensors:** BMI088 + ICM42688P + MS5611 (**no magnetometer**).
- **External I2C1 / I2C4:** connectors for an **optional** GPS/compass puck or other I2C peripherals if the user installs them.
- **Default firmware:** `rc.board_sensors` starts only onboard IMUs and barometer; it does **not** `start` magnetometers or other external sensors that are not fitted by default.

`CONFIG_COMMON_MAGNETOMETER` is enabled in the default build so users who add a compatible external compass module can start the appropriate driver from the [system console](../debug/system_console.md) (for example after checking the bus with `i2cdetect`).

Without an external magnetometer, yaw in flight relies on GPS course when available and other estimator settings as configured in QGroundControl. Plan missions accordingly for GPS-denied operation.

## Overview

- **MCU:** STM32H743 (Cortex-M7)
- **Onboard IMU:** Bosch BMI088 (SPI1), InvenSense ICM42688P (SPI2)
- **Onboard barometer:** MS5611 (SPI1)
- **PWM:** 10 FMU outputs (no PX4IO coprocessor)
- **CAN:** dual external CAN (FDCAN1 / FDCAN2)
- **USB:** product string `AMOVLAB FLYCORE`

## Serial port mapping

| Port   | Device        | `default.px4board` |
| ------ | ------------- | ------------------ |
| TELEM1 | `/dev/ttyS0`  | CONFIG_BOARD_SERIAL_TEL1 |
| TELEM2 | `/dev/ttyS1`  | CONFIG_BOARD_SERIAL_TEL2 |
| GPS1   | `/dev/ttyS2`  | CONFIG_BOARD_SERIAL_GPS1 |
| TELEM3 | `/dev/ttyS3`  | CONFIG_BOARD_SERIAL_TEL3 |
| RC     | `/dev/ttyS4`  | CONFIG_BOARD_SERIAL_RC |
| GPS2   | `/dev/ttyS5`  | CONFIG_BOARD_SERIAL_GPS2 |

## Building firmware

```sh
make amovlab_flycore_default
```

Bootloader:

```sh
make amovlab_flycore_bootloader
```

Upload (USB):

```sh
make amovlab_flycore_default upload
```

## Bootloader / board ID

- **board_id:** 106 (`boards/amovlab/flycore/firmware.prototype`)
- Bootloader USB product string: `PX4 BL AMOV FLYCORE`

## Default airframe

For hardware type `FLYCORE0000`, `rc.board_defaults` sets `SYS_AUTOSTART` to **4014** (generic multicopter airframe class).
Adjust the airframe in QGroundControl to match your vehicle.

## Optional external magnetometer

If you install a GPS/compass module on I2C1 or I2C4, probe the bus (`i2cdetect -b 1` / `i2cdetect -b 4`) and start the matching driver manually, for example:

```sh
ist8310 -X -b 1 -R 10 start
```

Refer to the module datasheet and mounting orientation for the correct `-R` rotation argument.

## Maintenance

- **Vendor:** Amovlab
- **PX4 board target:** `amovlab_flycore_default`
