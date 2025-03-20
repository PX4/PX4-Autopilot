# mRo-X2.1 Autopilot

:::warning
PX4 не розробляє цей (або будь-який інший) автопілот.
Contact the [manufacturer](https://store.mrobotics.io/) for hardware support or compliance issues.
:::

The [mRo-X2.1 autopilot](http://www.mRobotics.io/) is based on the [Pixhawk<sup>&reg;</sup>-project](https://pixhawk.org/) **FMUv2** open hardware design.
It runs PX4 on the [NuttX](https://nuttx.apache.org/) OS.

![mRo X2.1](../../assets/flight_controller/mro/mro_x2.1.jpg)

:::info
This flight controller is [manufacturer supported](../flight_controller/autopilot_manufacturer_supported.md).
:::

## Короткий опис

- Main System-on-Chip: [STM32F427](http://www.st.com/web/en/catalog/mmc/FM141/SC1169/SS1577/LN1789)
  - CPU: STM32F427VIT6 ARM<sup>&reg;</sup> мікроконтроллер - Revision 3
  - ІО: мікроконтролер STM32F100C8T6 ARM<sup>&reg;</sup>
- Датчики:
  - Invensense<sup>&reg;</sup> MPU9250 9DOF
  - Invensense ICM-20602 6DOF
  - MEAS MS5611 барометр
- Розміри/Вага
  - Size: 36mm x 50mm
    (Can be ordered with vertical, horizontal or no headers installed)
  - Точки кріплення: 30,5 мм х 30,5 мм діаметр 3,2 мм
  - Вага: 10.9g

Діаграма нижче надає порівняльний аналіз з Pixhawk 1. mRo має практично ідентичне апаратне забезпечення й підключення, але має значно менший слід. Основні відмінності - це оновлені датчики та Rev 3 FMU.

![Mro Pixhawk 1 vs X2.1 comparison](../../assets/flight_controller/mro/px1_x21.jpg)

## Підключення

- 2.54 мм заголовки:
- GPS (UART4) з I2C
- CAN шина
- Вхід RC
- PPM вхід
- Вхідний спектр
- RSSI вхід
- вхід SBUS
- sBus вихід
- Вхід живлення
- Вихід зумера
- Вихід світлодіода
- 8 x Виводи сервоприводів
- 6 x Aux outputs
- Позабортовий конектор microUSB
- Kill Pin output _(Currently not supported by firmware)_
- AirSpeed Sensor
- USART2 (Telem 1)
- USART3 (Telem 2)
- UART7 (Console)
- UART8 (OSD)

## Проблема PX4 BootLoader

За замовчуванням mRo X2.1 може бути попередньо налаштований на ArduPilot<sup>&reg;</sup>, а не на PX4. Це можна побачити під час оновлення прошивки, коли плата визнається як FMUv2 замість X2.1.

In this case you must update the BootLoader using [BL_Update_X21.zip](https://github.com/PX4/PX4-user_guide/raw/main/assets/hardware/BL_Update_X21.zip).
Якщо це виправлення не буде зроблено, ваша пеленга буде відображена неправильно і надмірний інерціальний модуль не буде виявлено.

Основні кроки:

1. Download and extract [BL_Update_X21.zip](https://github.com/PX4/PX4-user_guide/raw/main/assets/hardware/BL_Update_X21.zip).
2. Find the folder _BL_Update_X21_. This contains a **bin** file and a subfolder named **/etc** containing an **rc.txt** file
3. Скопіюйте ці файли на кореневий каталог вашої micro SD-карти та вставте її в mRO x2.1
4. Увімкніть mRO x2.1. Зачекайте, доки він завантажиться, а потім перезавантажте 1 раз.

## Доступність

This product can be ordered at the [mRobotics<sup>&reg;</sup> Store](https://store.mrobotics.io/mRo-X2-1-Rev-2-p/m10021a.htm).

## Посібник з підключення

![mRo_X2.1_Wiring](../../assets/flight_controller/mro/mro_x21_wiring.png)

## Збірка прошивки

:::tip
Most users will not need to build this firmware!
It is pre-built and automatically installed by _QGroundControl_ when appropriate hardware is connected.
:::

To [build PX4](../dev_setup/building_px4.md) for this target:

```
make mro_x21_default
```

## Креслення

The board is documented on the mRo hardware repo: [x21_V2_schematic.pdf](https://github.com/mRoboticsIO/Hardware/blob/master/X2.1/Docs/x21_V2_schematic.pdf).

## Налаштування послідовного порту

| UART   | Пристрій   | Порт            |
| ------ | ---------- | --------------- |
| USART1 | /dev/ttyS0 | IO debug        |
| USART2 | /dev/ttyS1 | SERIAL1         |
| USART3 | /dev/ttyS2 | TELEM2          |
| UART4  | /dev/ttyS3 | GPS/I2C         |
| USART6 | /dev/ttyS4 | PX4IO           |
| UART7  | /dev/ttyS5 | SERIAL5 CONSOLE |
| UART8  | /dev/ttyS6 | SERIAL4         |

<!-- Note: Got ports using https://github.com/PX4/PX4-user_guide/pull/672#issuecomment-598198434 -->
