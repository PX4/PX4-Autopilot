# Автопілот CUAV V5+

:::warning
PX4 не розробляє цей (або будь-який інший) автопілот.
Contact the [manufacturer](https://store.cuav.net/) for hardware support or compliance issues.
:::

_V5+_<sup>&reg;</sup> is an advanced autopilot manufactured by CUAV<sup>&reg;</sup>.
Він був розроблений CUAV<sup>&reg;</sup> у співпраці з командою PX4.

Автопілот рекомендується для комерційної системної інтеграції, але також підходить для академічних досліджень і будь-якого іншого використання.

![V5+ AutoPilot - hero image](../../assets/flight_controller/cuav_v5_plus/v5+_01.png)

Деякі з його основних функцій включають:

- Full compatibility with the [Pixhawk project](https://pixhawk.org/) **FMUv5** design standard and uses the [Pixhawk Connector Standard](https://pixhawk.org/pixhawk-connector-standard/) for all external interfaces.
- Більш продуктивний процесор, оперативна пам'ять та флеш-пам'ять, ніж у FMU v3, разом із більш стабільними та надійними датчиками.
- Прошивка сумісна з PX4.
- Модульна конструкція дозволяє користувачам налаштовувати власну носійну плату.
- Вбудована система поглинання вібрації з високоефективною системою поглинання ударів.
- Множинні резервні датчики та системи живлення для покращення безпеки та стабільності польоту.

:::info
This flight controller is [manufacturer supported](../flight_controller/autopilot_manufacturer_supported.md).
:::

## Короткий опис

- Головний FMU процесор: STM32F765
  - 32 Bit Arm® Cortex®-M7, 216MHz, 2MB memory, 512KB RAM

- IO Processor: STM32F100
  - 32 Bit Arm®️ Cortex®️-M3, 24MHz, 8KB SRAM

- Бортові сенсори:

  - Акселерометр/Гіроскоп: ICM-20689
  - Акселерометр/Гіроскоп: BMI055
  - Магнітометр: IST8310
  - Барометр: MS5611

- Інтерфейси:
  - 8-14 PWM виходів (6 з IO, 8 з FMU)
  - 3 виділених PWM/Capture входи на FMU
  - Виділений R/C вхід для CPPM
  - Виділений R/C вхід для Spektrum / DSM та S.Bus з аналоговим / PWM RSSI входом
  - аналоговий / PWM вхід RSSI
  - Вихід сервоприводу S.Bus
  - 5 загальних послідовних портів
  - 4x I2C порти
  - 4 шини SPI
  - 2 CANBuses з послідовними ESC
  - Аналогові входи для напруги / струму з 2 батарей

- Система живлення:
  - Живлення: 4.3~5.4В
  - Вхід USB: 4.75~5.25В

- Вага та розміри:
  - Вага: 90г
  - Розміри: 85.5\*42\*мм

- Інші характеристики:

  - Робоча температура: -20 ~ 80°c (виміряне значення)

## Де купити

<!-- [CUAV Store](https://store.cuav.net/index.php?id_product=95&id_product_attribute=0&rewrite=cuav-new-pixhack-v5-autopilot-m8n-gps-for-fpv-rc-drone-quadcopter-helicopter-flight-simulator-free-shipping-whole-sale&controller=product&id_lang=1) -->

[CUAV Aliexpress](https://www.aliexpress.com/item/32890380056.html?spm=a2g0o.detail.1000060.1.7a7233e7mLTlVl&gps-id=pcDetailBottomMoreThisSeller&scm=1007.13339.90158.0&scm_id=1007.13339.90158.0&scm-url=1007.13339.90158.0&pvid=d899bfab-a7ca-46e1-adf2-72ad1d649822) (International users)

CUAV Taobao (China Mainland users)

:::info
Autopilot may be purchased with included Neo GPS module
:::

<a id="connection"></a>

## З'єднання (Проводка)

[CUAV V5+ Wiring Quickstart](../assembly/quick_start_cuav_v5_plus.md)

## Схема розташування виводів

Download **V5+** pinouts from [here](http://manual.cuav.net/V5-Plus.pdf).

## Номінальна напруга

_V5+ AutoPilot_ supports redundant power supplies - up to three sources may be used: `Power1`, `Power2` and `USB`.
Ви повинні подати живлення принаймні до одного з цих джерел, інакше контролер польоту буде знеструмлений.

:::info
On FMUv5 based FMUs with PX4IO module (as is the case for the _V5+_), the Servo Power Rail is only monitored by the FMU.
Вона не живиться від FMU і не забезпечує його живленням.
However, the pins marked **+** are all common, and a BEC may be connected to any of the servo pin sets to power the servo power rail.
:::

**Normal Operation Maximum Ratings**

За таких умов всі джерела живлення будуть використовуватися в цьому порядку для живлення системи:

1. `Power1` and `Power2` inputs (4.3V to 5.4V)
2. `USB` input (4.75V to 5.25V)

## Захист від перенапруги

The _V5+_ has over current protection on the 5 Volt Peripheral and 5 Volt high power, which limits the current to 2.5A.
The _V5+_ has short circuit protection.

:::warning
Up to 2.5 A can be delivered to the connectors listed as pin 1 (although these are only rated at 1 A).
:::

## Збірка прошивки

:::tip
Most users will not need to build this firmware!
It is pre-built and automatically installed by _QGroundControl_ when appropriate hardware is connected.
:::

To [build PX4](../dev_setup/building_px4.md) for this target:

```
make px4_fmu-v5_default
```

## Відладочний порт

The [PX4 System Console](../debug/system_console.md) and [SWD interface](../debug/swd_debug.md) operate on the **FMU Debug** port (`DSU7`).
Плата не має інтерфейсу відладки вводу/виводу.

![Debug port (DSU7)](../../assets/flight_controller/cuav_v5_plus/debug_port_dsu7.jpg)

The debug port (`DSU7`) uses a [JST BM06B](https://www.digikey.com.au/product-detail/en/jst-sales-america-inc/BM06B-GHS-TBT-LF-SN-N/455-1582-1-ND/807850) connector and has the following pinout:

| Pin                        | Сигнал                            | Вольтаж               |
| -------------------------- | --------------------------------- | --------------------- |
| 1 (red) | 5V+                               | +5V                   |
| 2 (blk) | DEBUG TX (OUT) | +3.3V |
| 3 (blk) | DEBUG RX (IN)  | +3.3V |
| 4 (blk) | FMU_SWDIO    | +3.3V |
| 5 (blk) | FMU_SWCLK    | +3.3V |
| 6 (blk) | GND                               | GND                   |

The product package includes a convenient debug cable that can be connected to the `DSU7` port.
This splits out an FTDI cable for connecting the [PX4 System Console](../debug/system_console.md) to a computer USB port, and SWD pins used for SWD/JTAG debugging.
The provided debug cable does not connect to the SWD port `Vref` pin (1).

![CUAV Debug cable](../../assets/flight_controller/cuav_v5_plus/cuav_v5_debug_cable.jpg)

:::warning
The SWD Vref pin (1) uses 5V as Vref but the CPU is run at 3.3V!

Деякі JTAG-адаптери (SEGGER J-Link) використовують напругу Vref для встановлення напруги на лініях SWD.
For direct connection to _Segger Jlink_ we recommended you use the 3.3 Volts from pin 4 of the connector marked `DSM`/`SBUS`/`RSSI` to provide `Vtref` to the JTAG (i.e. providing 3.3V and _NOT_ 5V).

For more information see [Using JTAG for hardware debugging](#using-jtag-for-hardware-debugging).
:::

## Налаштування послідовного порту

| UART   | Пристрій   | Порт                                                |
| ------ | ---------- | --------------------------------------------------- |
| UART1  | /dev/ttyS0 | GPS                                                 |
| USART2 | /dev/ttyS1 | TELEM1 (керування потоком)       |
| USART3 | /dev/ttyS2 | TELEM2 (керування потоком)       |
| UART4  | /dev/ttyS3 | TELEM4                                              |
| USART6 | /dev/ttyS4 | TX - RC-вхід з роз'єму SBUS_RC |
| UART7  | /dev/ttyS5 | Debug Console                                       |
| UART8  | /dev/ttyS6 | PX4IO                                               |

<!-- Note: Got ports using https://github.com/PX4/PX4-user_guide/pull/672#issuecomment-598198434 -->

<a id="optional-hardware"></a>

## Периферійні пристрої

- [Цифровий датчик швидкості польоту](https://item.taobao.com/item.htm?spm=a1z10.3-c-s.w4002-16371268452.37.6d9f48afsFgGZI\&id=9512463037)
- [Телеметричні радіо модулі](https://cuav.taobao.com/category-158480951.htm?spm=2013.1.w5002-16371268426.4.410b7a821qYbBq\&search=y\&catName=%CA%FD%B4%AB%B5%E7%CC%A8)
- [Rangefinders/Distance sensors](../sensor/rangefinders.md)

## Підтримувані платформи / Конструкції

Будь-який мультикоптер / літак / наземна платформа / човен, який може керуватися звичайними RC сервоприводами або сервоприводами Futaba S-Bus.
The complete set of supported configurations can be seen in the [Airframes Reference](../airframes/airframe_reference.md).

## Примітки

#### Не підключайте цифровий або аналоговий PM до роз'ємів, сконфігурованих для іншого типу PM

Якщо ви під'єднаєте аналоговий PM до цифрового роз'єму PM, він зупинить всі пристрої I2C на цій шині.
Зокрема, це призведе до зупинки компаса GPS через конфлікт, а також може пошкодити FMU (у довгостроковій перспективі).

Аналогічно, цифровий PM, підключений до аналогового роз'єму, не працюватиме, а також може пошкодити/вивести з ладу модуль живлення (у довгостроковій перспективі).

## Сумісність

CUAV використовує деякі відмінні дизайни і несумісний з деяким обладнанням, про що буде описано нижче.

<a id="compatibility_gps"></a>

#### GPS несумісний з іншими пристроями

The _Neo v2.0 GPS_ recommended for use with _CUAV V5+_ and _CUAV V5 nano_ is not fully compatible with other Pixhawk flight controllers (specifically, the buzzer part is not compatible and there may be issues with the safety switch).

The UAVCAN [NEO V2 PRO GNSS receiver](http://doc.cuav.net/gps/neo-series-gnss/en/neo-v2-pro.html) can also be used, and is compatible with other flight controllers.

<a id="compatibility_jtag"></a>

#### Використання JTAG для апаратного налагодження

`DSU7` FMU Debug Pin 1 is 5 volts - not the 3.3 volts of the CPU.

Деякі JTAG використовують цю напругу для встановлення рівнів вводу-виводу під час обміну даними з ціллю.

For direct connection to _Segger Jlink_ we recommended you use the 3.3 Volts of DSM/SBUS/RSSI pin 4 as Pin 1 on the debug connector (`Vtref`).

## Відомі проблеми

The issues below refer to the _batch number_ in which they first appear.
Номер партії - це чотирицифрова дата виробництва за V01 та відображається на наклейці з боку контролера польоту.
Наприклад, серійний номер партії V011904 ((V01 - це номер V5, 1904 - це дата виробництва, тобто номер партії).

<a id="pin1_unfused"></a>

#### Інтерфейс SBUS / DSM / RSSI Pin1 не захищений від перевантаження

:::warning
This is a safety issue.
:::

Будь ласка, не підключайте інше обладнання (крім RC приймача) до інтерфейсу SBUS / DSM / RSSI - це може призвести до пошкодження обладнання.

- _Found:_ Batches V01190904xxxx
- _Fixed:_ Batches later than V01190904xxxx

## Подальша інформація

- [CUAV V5+ Manual](http://manual.cuav.net/V5-Plus.pdf)
- [CUAV V5+ docs](http://doc.cuav.net/flight-controller/v5-autopilot/en/v5+.html)
- [FMUv5 reference design pinout](https://docs.google.com/spreadsheets/d/1-n0__BYDedQrc_2NHqBenG1DNepAgnHpSGglke-QQwY/edit#gid=912976165)
- [CUAV Github](https://github.com/cuav)
- [Base board design reference](https://github.com/cuav/hardware/tree/master/V5_Autopilot/V5%2B/V5%2BBASE)
- [CUAV V5+ Wiring Quickstart](../assembly/quick_start_cuav_v5_plus.md)
- [Airframe build-log using CUAV v5+ on a DJI FlameWheel450](../frames_multicopter/dji_f450_cuav_5plus.md)
