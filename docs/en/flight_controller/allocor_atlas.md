# Allocor Atlas

::: warning
PX4 does not manufacture this (or any) autopilot.
Contact the [manufacturer](https://www.allocor.tech/atlas-product-page) for hardware support or compliance issues.
:::

Atlas is a flight controller manufactured by Allocor, based on the [FMUv6X reference design](https://github.com/pixhawk/Pixhawk-Standards).

It uses an STM32H753II FMU processor paired with a PX4IO v2 coprocessor for PWM output and safety-switch handling.

::: info
This flight controller is [manufacturer supported](../flight_controller/autopilot_manufacturer_supported.md).
:::

## Sensors

- [Bosch BMI088](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi088/) accelerometer/gyroscope (SPI3)
- [InvenSense ICM-42688-P](https://invensense.tdk.com/products/motion-tracking/6-axis/icm-42688-p/) IMU (SPI2)
- [InvenSense IIM-42652](https://invensense.tdk.com/products/motion-tracking/6-axis/iim-42652/) IMU (SPI4)
- [Bosch BMP388](https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/bmp388/) barometer (I2C)
- [Bosch BMM350](https://www.bosch-sensortec.com/products/motion-sensors/magnetometers/bmm350/) magnetometer (I2C, bus 4, address `0x14`)

All three IMUs are on separate SPI buses for sensor redundancy.

## Microprocessor

- STM32H753II FMU (Arm® Cortex®-M7, 480MHz, 2MB flash, 1MB RAM)
- PX4IO v2 coprocessor

## Building Firmware

To [build PX4](../dev_setup/building_px4.md) for this target:

```sh
make atlas_fmu-v6x_default
```

## Serial Port Mapping

| UART   | Device     | Port          |
| ------ | ---------- | ------------- |
| USART1 | /dev/ttyS0 | GPS1          |
| USART2 | /dev/ttyS1 | TELEM3        |
| USART3 | /dev/ttyS2 | Debug Console |
| UART4  | /dev/ttyS3 | TELEM4        |
| UART5  | /dev/ttyS4 | TELEM2        |
| USART6 | /dev/ttyS5 | PX4IO/RC      |
| UART7  | /dev/ttyS6 | TELEM1        |
| UART8  | /dev/ttyS7 | GPS2          |

## RC Input

RC input is wired directly to the FMU, not through PX4IO.

- PPM/S.Bus: dedicated single-wire input on a timer capture pin, decoded by the FMU (`RC_SERIAL_SINGLEWIRE`).
- Spektrum/DSM: dedicated FMU input with a switched 3.3V power rail for the satellite receiver.

PX4IO's serial link (`USART6`/`/dev/ttyS5`) is the FMU↔IO communication bus used for PWM output and safety-switch status, and is unrelated to RC input decoding.

## Power

- 2 digital power bricks (voltage/current sensed over I2C via INA226/INA228/INA238)

## See Also

- [Allocor Atlas Product Page](https://www.allocor.tech/atlas-product-page)
