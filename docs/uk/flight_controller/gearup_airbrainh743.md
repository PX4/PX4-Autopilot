# Gear Up AirBrainH743

:::warning
PX4 не розробляє цей (або будь-який інший) автопілот.
Contact the [manufacturer](https://takeyourgear.com/) for hardware support.
:::

:::info
This flight controller is [manufacturer supported](../flight_controller/autopilot_manufacturer_supported.md).
:::

Purchase from [takeyourgear.com](https://takeyourgear.com/pages/products/airbrain).

For more information and pinout, check the [GitHub documentation](https://github.com/GearUp-Company/AirBrainH743).

## Основні характеристики

- MCU: Процесор STM32H743 32-біт, який працює на частоті 480 МГц
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

## Конектори та контакти

:::warning
The pin order is different from the Pixhawk standard (compatible to the Betaflight standard).
:::

### UARTs

Current UART configuration:

| UART   | Пристрій   | Функція                                 |
| ------ | ---------- | --------------------------------------- |
| USART1 | /dev/ttyS0 | Console/Debug                           |
| USART2 | /dev/ttyS1 | RC-вхід                                 |
| USART3 | /dev/ttyS2 | TEL4 (DJI/MSP)       |
| UART4  | /dev/ttyS3 | TEL1                                    |
| UART5  | /dev/ttyS4 | TEL2                                    |
| UART7  | /dev/ttyS5 | TEL3 (ESC Telemetry) |
| UART8  | /dev/ttyS6 | GPS1                                    |

### Motor/Servo Outputs

| З’єднання | Pin | Функція                      |
| --------- | --- | ---------------------------- |
| ESC       | M1  | Двигун 1                     |
| ESC       | M2  | Motor 2                      |
| ESC       | M3  | Motor 3                      |
| ESC       | M4  | Motor 4                      |
| PWM       | M5  | Motor 5                      |
| PWM       | M6  | Motor 6                      |
| PWM       | M7  | Motor 7                      |
| PWM       | M8  | Motor 8                      |
| AUX       | M9  | LED/PWM/etc. |

<a id="bootloader"></a>

## Оновлення завантажувача PX4

Before PX4 firmware can be installed, the _PX4 bootloader_ must be flashed.
Download the [gearup_airbrainh743_bootloader.bin](https://github.com/PX4/PX4-Autopilot/blob/main/boards/gearup/airbrainh743/extras/gearup_airbrainh743_bootloader.bin) bootloader binary and read [this page](../advanced_config/bootloader_update_from_betaflight.md) for flashing instructions.

## Збірка прошивки

To [build PX4](../dev_setup/building_px4.md) for this target:

```
make gearup_airbrainh743_default
```

## Встановлення прошивки PX4

Firmware can be installed in any of the normal ways:

- Збудуйте та завантажте джерело:

  ```
  make gearup_airbrainh743_default upload
  ```

- [Load the firmware](../config/firmware.md) using _QGroundControl_.
  Ви можете використовувати або готове вбудоване програмне забезпечення, або власне користувацьке програмне забезпечення.

### Системна консоль

UART1 (ttyS0) is configured for use as the [System Console](../debug/system_console.md).
