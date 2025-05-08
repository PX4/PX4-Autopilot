# Контролер польоту Holybro pix32 (знято з виробництва)

<Badge type="info" text="Discontinued" />

:::warning
PX4 не розробляє цей (або будь-який інший) автопілот.
Contact the [manufacturer](https://holybro.com/) for hardware support or compliance issues.
:::

The Holybro<sup>&reg;</sup> [pix32 autopilot](https://holybro.com/collections/autopilot-flight-controllers/products/pix32pixhawk-flight-controller) (also known as "Pixhawk 2", and formerly as HKPilot32) is based on the [Pixhawk<sup>&reg;</sup>-project](https://pixhawk.org/) **FMUv2** open hardware design.
This board is based on hardware version Pixhawk 2.4.6.
It runs the PX4 flight stack on the [NuttX](https://nuttx.apache.org/) OS.

![pix32](../../assets/flight_controller/holybro_pix32/pix32_hero.jpg)

As a CC-BY-SA 3.0 licensed Open Hardware design, schematics and design files should be [available here](https://github.com/PX4/Hardware).

:::tip
The Holybro pix32 is software compatible with the [3DR Pixhawk 1](../flight_controller/pixhawk.md).
It is not connector compatible, but is otherwise physically very similar to the 3DR Pixhawk or mRo Pixhawk.
:::

:::info
This flight controller is [manufacturer supported](../flight_controller/autopilot_manufacturer_supported.md).
:::

## Основні характеристики

- Main System-on-Chip: [STM32F427](http://www.st.com/web/en/catalog/mmc/FM141/SC1169/SS1577/LN1789)
  - Процесор: 32-розрядний ядро STM32F427 Cortex<sup>&reg;</sup> M4 з FPU
  - ОЗП: 168 МГц/256 КБ
  - Flash: 2 МБ
- Failsafe System-on-Chip: STM32F103
- Датчики:
  - ST Micro L3GD20 3-axis 16-бітний гіроскоп
  - ST Micro LSM303D 3-вісний 14-бітний акселерометр / магнітометр
  - Invensense<sup>&reg;</sup> MPU 6000 3-вісний акселерометр/гіроскоп
  - MEAS MS5611 барометр
- Розміри/Вага
  - Розмір: 81x44x15 мм
  - Вага: 33.1гр
- GPS: u-blox<sup>&reg;</sup> супер точний Neo-7M з компасом
- Вхідна напруга: 2~10s (7.4~37V)

### Підключення

- 1x I2C
- 2x CAN
- 3.3 та 6.6V ADC входи
- 5x UART (послідовні порти), один високої потужності, 2x з контролем потоку ГВП
- Вхід, сумісний з приймачами Spektrum DSM / DSM2 / DSM-X® Satellite до DX8 (DX9 та вище не підтримуються)
- Futaba<sup>&reg;</sup> S.BUS сумісний вхід та вихід
- Сигнал суми PPM
- Вхід RSSI (ШІМ або напруга)
- SPI
- Зовнішній порт microUSB
- Раз'єми Molex PicoBlade

## Де купити

[shop.holybro.com](https://holybro.com/collections/autopilot-flight-controllers/products/pix32pixhawk-flight-controller)

### Аксесуари

- [Digital airspeed sensor](https://holybro.com/products/digital-air-speed-sensor)
- [Hobbyking<sup>&reg;</sup> Wifi Telemetry](https://hobbyking.com/en_us/apm-pixhawk-wireless-wifi-radio-module.html)
- [HolyBro SiK Telemetry Radio (EU 433 MHz, US 915 MHz)](../telemetry/holybro_sik_radio.md)

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

See [3DR Pixhawk 1 > Debug Ports](../flight_controller/pixhawk.md#debug-ports).

## Розпіновки та схеми

The board is based on the [Pixhawk project](https://pixhawk.org/) **FMUv2** open hardware design.

- [FMUv2 + IOv2 schematic](https://raw.githubusercontent.com/PX4/Hardware/master/FMUv2/PX4FMUv2.4.5.pdf) -- Schematic and layout

:::info
As a CC-BY-SA 3.0 licensed Open Hardware design, all schematics and design files are [available](https://github.com/PX4/Hardware).
:::

## Налаштування послідовного порту

| UART   | Пристрій   | Порт                                          |
| ------ | ---------- | --------------------------------------------- |
| UART1  | /dev/ttyS0 | IO debug                                      |
| USART2 | /dev/ttyS1 | TELEM1 (керування потоком) |
| USART3 | /dev/ttyS2 | TELEM2 (керування потоком) |
| UART4  |            |                                               |
| UART7  | CONSOLE    |                                               |
| UART8  | SERIAL4    |                                               |

<!-- Note: Got ports using https://github.com/PX4/PX4-user_guide/pull/672#issuecomment-598198434 -->
