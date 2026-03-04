# Контролер польоту Pixfalcon (знятий з виробництва)

<Badge type="info" text="Discontinued" px4_current="v1.15" year="2024"/>

:::warning
This flight controller has been [discontinued](../flight_controller/autopilot_experimental.md) and is no longer commercially available.
:::

:::warning
PX4 не розробляє цей (або будь-який інший) автопілот.
Contact the [manufacturer](https://holybro.com/) for hardware support or compliance issues.
:::

The Pixfalcon autopilot (designed by [Holybro<sup>&reg;</sup>](https://holybro.com/)) is binary-compatible (FMUv2) derivative of the [Pixhawk 1](../flight_controller/pixhawk.md) design that has been optimized for space-constrained applications such as FPV racers. Він має менше входів-виходів, що дозволяє зменшити розмір.

![Pixfalcon hero image](../../assets/hardware/hardware-pixfalcon.png)

## Короткий опис

- Main System-on-Chip: [STM32F427](https://www.st.com/en/microcontrollers-microprocessors/stm32f427-437.html)
  - CPU: 180 MHz ARM<sup>&reg;</sup> Cortex<sup>&reg;</sup> M4 з одинарною точністю FPU
  - RAM: 256 KB SRAM (L1)
- Failsafe System-on-Chip: STM32F100
  - CPU: 24 MHz ARM Cortex M3
  - RAM: 8 KB SRAM
- GPS: u-blox<sup>&reg;</sup> M8 (в комплекті)

### Підключення

- 1x I2C
- 2x UART (один для телеметрії / OSD, без контролю потоку)
- 8x ШІМ з ручним управлінням
- S.BUS / PPM вхід

## Наявність:

No longer available.

Опціональне обладнання:

- Optical flow: PX4 Flow unit from manufacturer [Holybro](https://holybro.com/products/px4flow)
- Digital Airspeed sensor from manufacturer [Holybro](https://holybro.com/products/digital-air-speed-sensor-ms4525do)
- Екранний дисплей з вбудованою телеметрією:
  - Micro HKPilot Telemetry Radio Module with On Screen Display (OSD) unit - 433MHz. (Discontinued)
- Pure Telemetry опції:
  - [SIK Radios](../telemetry/sik_radio.md)

## Збірка прошивки

:::tip
Most users will not need to build this firmware!
It is pre-built and automatically installed by _QGroundControl_ when appropriate hardware is connected.
:::

To [build PX4](../dev_setup/building_px4.md) for this target:

```
make px4_fmu-v3_default
```

## Відладочний порт

This board does not have a debug port (i.e it does not have a port for accessing the [System Console](../debug/system_console.md) or the [SWD interface](../debug/swd_debug.md) (JTAG).

Розробники повинні будуть припаяти дроти до підключених площин для SWD, а також до STM32F4 (IC) TX і RX, щоб отримати консоль.

## Налаштування послідовного порту

| UART   | Пристрій   | Порт                                            |
| ------ | ---------- | ----------------------------------------------- |
| UART1  | /dev/ttyS0 | IO Debug                                        |
| USART2 | /dev/ttyS1 | TELEM1 (без контролю потоку) |
| UART4  | /dev/ttyS2 | GPS                                             |

<!-- Note: Got ports using https://github.com/PX4/PX4-user_guide/pull/672#issuecomment-598198434 -->
