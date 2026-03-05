# Gear Up AirBrainH743

:::warning
PX4 does not manufacture this (or any) autopilot.
Contact the [manufacturer](https://takeyourgear.com/) for hardware support.
:::

:::info
This flight controller is [manufacturer supported](../flight_controller/autopilot_manufacturer_supported.md).
:::

Purchase from [takeyourgear.com](https://takeyourgear.com/pages/products/airbrain).

For more information and pinout, check the [GitHub documentation](https://github.com/GearUp-Company/AirBrainH743).

## 主要特性

- MCU: STM32H743 32-bit processor running at 480 MHz
- IMU: ICM42688P
- Barometer: DPS310
- Magnetometer: LIS2MDL (internal)
- 128MB NAND Flash for logging (W25N)
- 7x UARTs
- I2C, SPI
- 9x PWM Outputs (8 Motor outputs, 1 LED strip)
- Battery input voltage: 3S-10S
- Battery voltage/current monitoring
- 5V@2A and 10V@2.5A BEC outputs
- USB Type-C (IP68)
- EMC and ESD protection

## Connectors and Pins

:::warning
The pin order is different from the Pixhawk standard (compatible to the Betaflight standard).
:::

### UARTs

Current UART configuration:

| UART   | 设备         | 功能                                      |
| ------ | ---------- | --------------------------------------- |
| USART1 | /dev/ttyS0 | Console/Debug                           |
| USART2 | /dev/ttyS1 | RC Input                                |
| USART3 | /dev/ttyS2 | TEL4 (DJI/MSP)       |
| UART4  | /dev/ttyS3 | TEL1                                    |
| UART5  | /dev/ttyS4 | TEL2                                    |
| UART7  | /dev/ttyS5 | TEL3 (ESC Telemetry) |
| UART8  | /dev/ttyS6 | GPS1                                    |

### Motor/Servo Outputs

| Connector | 针脚 | 功能                           |
| --------- | -- | ---------------------------- |
| ESC       | M1 | Motor 1                      |
| ESC       | M2 | Motor 2                      |
| ESC       | M3 | Motor 3                      |
| ESC       | M4 | Motor 4                      |
| PWM       | M5 | Motor 5                      |
| PWM       | M6 | Motor 6                      |
| PWM       | M7 | Motor 7                      |
| PWM       | M8 | Motor 8                      |
| AUX       | M9 | LED/PWM/etc. |

<a id="bootloader"></a>

## PX4 Bootloader Update

Before PX4 firmware can be installed, the _PX4 bootloader_ must be flashed.
Download the [gearup_airbrainh743_bootloader.bin](https://github.com/PX4/PX4-Autopilot/blob/main/boards/gearup/airbrainh743/extras/gearup_airbrainh743_bootloader.bin) bootloader binary and read [this page](../advanced_config/bootloader_update_from_betaflight.md) for flashing instructions.

## 编译固件

To [build PX4](../dev_setup/building_px4.md) for this target:

```
make gearup_airbrainh743_default
```

## Installing PX4 Firmware

Firmware can be installed in any of the normal ways:

- Build and upload the source:

  ```
  make gearup_airbrainh743_default upload
  ```

- [Load the firmware](../config/firmware.md) using _QGroundControl_.
  You can use either pre-built firmware or your own custom firmware.

### 系统控制台

UART1 (ttyS0) is configured for use as the [System Console](../debug/system_console.md).
