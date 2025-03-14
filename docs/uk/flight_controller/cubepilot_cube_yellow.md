# Контролер польоту CubePilot Cube Yellow

:::warning
PX4 не розробляє цей (або будь-який інший) автопілот.
Contact the [manufacturer](https://cubepilot.org/#/home) for hardware support or compliance issues.
:::

Контролер польоту Cube Yellow - це гнучкий автопілот, призначений в першу чергу для виробників комерційних систем.

![Cube Yellow](../../assets/flight_controller/cube/yellow/cube_yellow_hero.jpg)

Контролер призначений для використання зі специфічною для домену несучою платою, щоб зменшити кількість дротів, підвищити надійність і спростити збірку.
Наприклад, несуча плата для комерційного інспекційного апарату може містити з'єднання для комп'ютера-компаньйона, тоді як несуча плата для пілота може містити ESC для рами транспортного засобу.

Cube має віброізоляцію на двох IMU, з третім фіксованим IMU в якості еталонного/резервного.

:::tip
The manufacturer [Cube Docs](https://docs.cubepilot.org/user-guides/autopilot/the-cube-module-overview) contain detailed information, including an overview of the [Differences between Cube Colours](https://docs.cubepilot.org/user-guides/autopilot/the-cube-module-overview#differences-between-cube-colours).
:::

## Основні характеристики

- 32bit STM32F777VI (32bit [ARM Cortex M7](https://en.wikipedia.org/wiki/ARM_Cortex-M#Cortex-M7), 400 MHz, Flash 2MB, RAM 512 KB).
- 32 bit STM32F103 failsafe co-processor <!-- check -->
- 14 ШІМ / серво виходів (8 з відмовостійкими і ручним керуванням, 6 допоміжних, сумісних з великими потужностями)
- Широкі можливості підключення додаткових периферійних пристроїв (UART, I2C, CAN)
- Інтегрована система резервного копіювання для відновлення в польоті та ручного керування з виділеним процесором та автономним джерелом живлення (для літаків з фіксованим крилом)
- Резервна система інтегрує систему мікшування, забезпечуючи узгоджені режими автопілота та ручного заміщення ( для літаків з фіксованим крилом)
- Резервні входи живлення та автоматичне перемикання на резервне джерело
- Зовнішній запобіжний вимикач
- Головний візуальний індикатор - багатоколірний світлодіод
- Потужний багатотональний п'єзозвуковий індикатор
- Карта microSD для високошвидкісної фіксації даних протягом тривалого періоду часу

<a id="stores"></a>

## Де купити

- [Reseller list](https://www.cubepilot.com/#/reseller/list)

## Збірка

[Cube Wiring Quickstart](../assembly/quick_start_cube.md)

## Характеристики

- **Processor:**
  - STM32F777VI (32bit [ARM Cortex M7](https://en.wikipedia.org/wiki/ARM_Cortex-M#Cortex-M7))
  - 400 МГц
  - 512 KB MB RAM
  - 2 MB Flash
- **Failsafe co-processor:** <!-- inconsistent info on failsafe processor: 32 bit STM32F103 failsafe co-processor http://www.proficnc.com/all-products/191-pixhawk2-suite.html -->
  - STM32F100 (32bit _ARM Cortex-M3_)
  - 24 МГц
  - 8 KB SRAM
- **Sensors:** (all connected via SPI)
  - **Accelerometer:** (3) ICM20948, ICM20649, ICM20602
  - **Gyroscope:** (3) ICM20948, ICM20649, ICM20602
  - **Compass:** (1) ICM20948
  - **Barometric Pressure Sensor:** (2) MS5611
- **Operating Conditions:**
  - **Operating Temp:** -10C to 55C
  - **IP rating/Waterproofing:** Not waterproof
  - **Servo rail input voltage:** 3.3V / 5V
  - **USB port input:**
    - Напруга: 4В - 5.7В
    - Номінальний струм: 250 мА
  - **POWER:**
    - Вхідна напруга: 4.1В - 5.7В
    - Номінальний вхідний струм: 2,5 А
    - Номінальна вхідна/вихідна потужність: 14 Вт
- **Dimensions:**
  - **Cube:** 38.25mm x 38.25mm x 22.3mm
  - **Carrier:** 94.5mm x 44.3mm x 17.3mm
- **Interfaces**
  - Порти вводу-виводу: 14 ШІМ-виходів сервоприводів (8 від IO, 6 від FMU)
  - 5x UART (послідовні порти), один високої потужності, 2x з контролем потоку ГВП
  - 2x CAN (один з внутрішнім 3.3В трансивером, один на конекторі розширювача)
  - **R/C inputs:**
    - Spektrum DSM / DSM2 / DSM-X® Satellite сумісний вхід
    - Futaba S.BUS® сумісний вхід і вивід
    - Вхід сигналу PPM-SUM
  - Вхід RSSI (ШІМ або напруга)
  - I2C
  - SPI
  - 3.3В АЦП вхід
  - Внутрішній порт microUSB і розширення зовнішнього порту microUSB

## Розпіновки та схеми

Board schematics and other documentation can be found here: [The Cube Project](https://github.com/proficnc/The-Cube).

## Порти

### Верхня частина (GPS, TELEM тощо)

![Cube Ports - Top (GPS, TELEM etc) and Main/AUX](../../assets/flight_controller/cube/cube_ports_top_main.jpg)

## Налаштування послідовного порту

| UART   | Пристрій   | Порт                                          |
| ------ | ---------- | --------------------------------------------- |
| USART2 | /dev/ttyS0 | TELEM1 (керування потоком) |
| USART3 | /dev/ttyS1 | TELEM2 (керування потоком) |
| UART4  | /dev/ttyS2 | GPS1                                          |
| USART6 | /dev/ttyS3 | PX4IO                                         |
| UART7  | /dev/ttyS4 | CONSOLE/ADSB-IN                               |
| UART8  | /dev/ttyS5 | GPS2                                          |

<!-- Note: Got ports using https://github.com/PX4/PX4-user_guide/pull/672#issuecomment-598198434 -->

<!-- https://github.com/PX4/PX4-Autopilot/blob/main/boards/hex/cube-orange/default.px4board -->

<!-- https://github.com/PX4/PX4-Autopilot/blob/main/boards/hex/cube-orange/nuttx-config/nsh/defconfig#L194-L200 -->

### Відладочні порти

![Cube Debug Ports](../../assets/flight_controller/cube/cube_ports_debug.jpg)

### Порти USB/SDCard

![Cube USB/SDCard Ports](../../assets/flight_controller/cube/cube_ports_usb_sdcard.jpg)

## Збірка прошивки

:::tip
Most users will not need to build this firmware!
It is pre-built and automatically installed by _QGroundControl_ when appropriate hardware is connected.
:::

To [build PX4](../dev_setup/building_px4.md) for this target:

```
make cubepilot_cubeyellow
```

## Проблеми

Розташування символів CAN1 і CAN2 на кубі перевернуте (CAN1 - це CAN2 і навпаки).

## Додаткова інформація/документація

- [Cube Wiring Quickstart](../assembly/quick_start_cube.md)
- Cube Docs (виробник):
  - [Cube Module Overview](https://docs.cubepilot.org/user-guides/autopilot/the-cube-module-overview)
  - [Cube User Manual](https://docs.cubepilot.org/user-guides/autopilot/the-cube-user-manual)
  - [Mini Carrier Board](https://docs.cubepilot.org/user-guides/carrier-boards/mini-carrier-board)
