# Польотний контролер Hex Cube Black

:::warning
PX4 не розробляє цей (або будь-який інший) автопілот.
Contact the [manufacturer](https://cubepilot.org/#/home) for hardware support or compliance issues.
:::

:::tip
The [Cube Orange](cubepilot_cube_orange.md) is the successor to this product.
We recommend however to consider products built on industry standards, such as the [Pixhawk Standards](autopilot_pixhawk_standard.md).
Цей контролер польоту не дотримується стандарту і використовує патентований роз'єм.
:::

The [Hex Cube Black](https://docs.cubepilot.org/user-guides/autopilot/the-cube) flight controller (previously known as Pixhawk 2.1) is a flexible autopilot intended primarily for manufacturers of commercial systems.
It is based on the [Pixhawk-project](https://pixhawk.org/) **FMUv3** open hardware design and runs PX4 on the [NuttX](https://nuttx.apache.org/) OS.

![Cube Black](../../assets/flight_controller/cube/cube_black_hero.png)

Контролер призначений для використання зі специфічною для домену несучою платою, щоб зменшити кількість дротів, підвищити надійність і спростити збірку.
Наприклад, несуча плата для комерційного інспекційного апарату може містити з'єднання для комп'ютера-компаньйона, в той час як несуча плата для гонщика може включати ESC з рами транспортного засобу.

Cube має віброізоляцію на двох ІВП, з третім фіксованим ІВП в якості еталонного/резервного.

:::info
The manufacturer [Cube User Guide](https://docs.cubepilot.org/user-guides/autopilot/the-cube) contains detailed information, including an overview of the [Differences between Cube Colours](https://docs.cubepilot.org/user-guides/autopilot/the-cube/introduction/specifications).
:::

:::tip
This autopilot is [supported](../flight_controller/autopilot_pixhawk_standard.md) by the PX4 maintenance and test teams.
:::

## Основні характеристики

- 32bit STM32F427 [Cortex-M4F](https://en.wikipedia.org/wiki/ARM_Cortex-M#Cortex-M4)<sup>&reg;</sup> core with FPU
- 168 MHz / 252 MIPS
- 256 KB RAM
- 2 МБ флеш-пам'яті \(повністю доступна\)
- 32 bit STM32F103 відмовостійкий копроцесор
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

[Cube Black](https://www.cubepilot.com/#/reseller/list) (Reseller list)

## Збірка

[Cube Wiring Quickstart](../assembly/quick_start_cube.md)

## Характеристики

### Процесор

- 32bit STM32F427 [Cortex M4](https://en.wikipedia.org/wiki/ARM_Cortex-M#Cortex-M4) core with FPU
- 168 MHz / 252 MIPS
- 256 KB RAM
- 2 MB Flash (повністю доступна)
- 32 bit STM32F103 відмовостійкий копроцесор

### Датчики

- TBA

### Інтерфейси

- 5x UART (послідовні порти), один високої потужності, 2x з контролем потоку ГВП
- 2x CAN (один з внутрішнім 3.3В трансивером, один на конекторі розширювача)
- Spektrum DSM / DSM2 / DSM-X® Satellite сумісний вхід
- Futaba S.BUS® сумісний вхід і вивід
- Вхід сигналу PPM sum
- Вхід RSSI (ШІМ або напруга)
- I2C
- SPI
- 3.3В АЦП вхід
- Внутрішній порт microUSB і розширення зовнішнього порту microUSB

### Система живлення та захист

- Ідеальний діодний контролер з автоматичним перемиканням на резервне живлення
- Сервопривід високої потужності (max. 10В) і сильного струму (10A+)
- Усі периферійні виводи захищені від перевантаження по струму, усі входи захищені від електростатичного розряду

### Номінальна напруга

Pixhawk може мати потрійну резервність у джерелі живлення, якщо подаються три джерела живлення. Три шини: вхід модуля живлення, вхід сервоприводу, вхід USB.

#### Максимальна напруга нормальної роботи

За таких умов всі джерела живлення будуть використовуватися в цьому порядку для живлення системи

- Вхід модуля живлення (4.8В до 5.4В)
- Servo rail input (4.8V to 5.4V) **UP TO 10V FOR MANUAL OVERRIDE, BUT AUTOPILOT PART WILL BE UNPOWERED ABOVE 5.7V IF POWER MODULE INPUT IS NOT PRESENT**
- Вхід живлення USB (4.8В до 5.4В)

#### Абсолютна максимальна напруга

За таких умов система не буде витрачати жодної потужності (не буде працювати), але залишиться неушкодженою.

- Вхід модуля живлення (4.1В до 5.7В, 0В до 20В неушкоджений)
- Вхід сервоприводу (4.1В до 5.7В, 0В до 20В)
- Вхід живлення USB (4.1В до 5.7В, 0В до 6В)

## Розпіновки та схеми

Board schematics and other documentation can be found here: [The Cube Project](https://github.com/proficnc/The-Cube).

## Порти

### Верхня частина (GPS, TELEM тощо)

![Cube Ports - Top (GPS, TELEM etc) and Main/AUX](../../assets/flight_controller/cube/cube_ports_top_main.jpg)

<a id="serial_ports"></a>

### Налаштування послідовного порту

| UART   | Пристрій   | Порт                                          |
| ------ | ---------- | --------------------------------------------- |
| USART1 | /dev/ttyS0 | <!-- IO debug? -->                            |
| USART2 | /dev/ttyS1 | TELEM1 (керування потоком) |
| USART3 | /dev/ttyS2 | TELEM2 (керування потоком) |
| UART4  | /dev/ttyS3 | GPS1                                          |
| USART6 | /dev/ttyS4 | PX4IO                                         |
| UART7  | /dev/ttyS5 | CONSOLE                                       |
| UART8  | /dev/ttyS6 | <!-- unknown -->                              |

<!-- Note: Got ports using https://github.com/PX4/PX4-user_guide/pull/672#issuecomment-598198434 -->

<!-- This originally said " **TEL4:** /dev/ttyS6 (ttyS4 UART):  **Note** `TEL4` is labeled as `GPS2` on Cube." -->

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
make px4_fmu-v3_default
```

## Проблеми

Розташування сілкскрінів CAN1 і CAN2 на Cube Black перевернуте (CAN1 - це CAN2 і навпаки).

## Додаткова інформація/документація

- [Cube Wiring Quickstart](../assembly/quick_start_cube.md)
- Cube Docs (виробник):
  - [Cube User Guide](https://docs.cubepilot.org/user-guides/autopilot/the-cube)
  - [Mini Carrier Board](https://docs.cubepilot.org/user-guides/carrier-boards/mini-carrier-board)
