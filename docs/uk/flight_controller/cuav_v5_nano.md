# Автопілот CUAV V5 nano

:::warning
PX4 не розробляє цей (або будь-який інший) автопілот.
Contact the [manufacturer](https://store.cuav.net/) for hardware support or compliance issues.
:::

**V5 nano**<sup>&reg;</sup> is an autopilot for space-constrained applications, designed by CUAV<sup>&reg;</sup> in collaboration with the PX4 team.

Автопілот досить малий, щоб використовувати в гоночних дронах розміром 220 мм, але залишається достатньо потужним для більшості використань дрона.

![V5 nano - Hero image](../../assets/flight_controller/cuav_v5_nano/v5_nano_01.png)

:::info
The V5 nano is similar to the [CUAV V5+](../flight_controller/cuav_v5_plus.md), but has an all-in-one form factor, fewer PWM ports (can't be used for [airframes](../airframes/airframe_reference.md) that use AUX ports), and does not have internal damping.
:::

Деякі з його основних функцій включають:

- Full compatibility with the [Pixhawk project](https://pixhawk.org/) **FMUv5** design standard and uses the [Pixhawk Connector Standard](https://pixhawk.org/pixhawk-connector-standard/) for all external interfaces.
- Більш продуктивний процесор, оперативна пам'ять та флеш-пам'ять, ніж у FMU v3, разом із більш стабільними та надійними датчиками.
- Прошивка сумісна з PX4.
- Велика відстань між контактами вводу/виводу 2,6 мм, що полегшує використання всіх інтерфейсів.

:::info
This flight controller is [manufacturer supported](../flight_controller/autopilot_manufacturer_supported.md).
:::

### Короткий опис

Основний процесор FMU: STM32F765◦32 Bit Arm® Cortex®-M7, 216MHz, 2MB пам'яті, 512KB RAM

- Бортові сенсори:

  - Акселератор/гіроскоп: ICM-20689
  - Акселератор/гіроскоп: ICM-20602
  - Акселератор/гіроскоп: BMI055
  - Магнітометр: IST8310
  - Барометр: MS5611

- Інтерфейси: 8 виходів PWM

  - 3 виділених PWM/Capture входи на FMU
  - Виділений R/C вхід для CPPM
  - Виділений вхід R/C для Spektrum / DSM і S.Bus
  - Аналоговий / PWM вхід RSSI
  - 4 загальних послідовних порти
  - 3 I2C порти
  - 4 шини SPI
  - 2 CAN шини
  - Аналогові входи для напруги / струму з батареї
  - 2 додаткових аналогових входи
  - Підтримка nARMED

- Система енергопостачання: вхід Power Brick: 4,75~5,5В

- Вхід USB Power: 4.75~5.25V

- Вага та розміри:
  - Розміри: 60\*40\*14mm

- Інші характеристики:
  - Робоча температура: -20 ~ 85°С (виміряне значення)

## Де купити

[CUAV Store](https://store.cuav.net/shop/v5-nano/)

CUAV Aliexpress (international users)

CUAV Taobao (China Mainland users)

:::info
Autopilot may be purchased with included Neo GPS module
:::

<a id="connection"></a>

## З'єднання (Проводка)

[V5 nano Wiring Quickstart](../assembly/quick_start_cuav_v5_nano.md)

## Схема розташування виводів

Download **V5 nano** pinouts from [here](http://manual.cuav.net/V5-Plus.pdf).

## Збірка прошивки

:::tip
Most users will not need to build this firmware!
It is pre-built and automatically installed by _QGroundControl_ when appropriate hardware is connected.
:::

To [build PX4](../dev_setup/building_px4.md) for this target:

```
make px4_fmu-v5_default
```

<a id="debug_port"></a>

## Відладочний порт

The [PX4 System Console](../debug/system_console.md) and [SWD interface](../debug/swd_debug.md) operate on the **FMU Debug** port (`DSU7`).
Плата не має інтерфейсу відладки вводу/виводу.

![Debug port (DSU7)](../../assets/flight_controller/cuav_v5_nano/debug_port_dsu7.jpg)

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

![CUAV Debug cable](../../assets/flight_controller/cuav_v5_nano/cuav_nano_debug_cable.jpg)

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
| UART8  | /dev/ttyS6 | Не підключено (без PX4IO)        |

<!-- Note: Got ports using https://github.com/PX4/PX4-user_guide/pull/672#issuecomment-598198434 -->

## Номінальна напруга

_V5 nano_ must be powered from the `Power` connector during flight, and may also/alternatively be powered from `USB` for bench testing.

:::info
The `PM2` connector cannot not be used for powering the _V5 nano_ (see [this issue](#compatibility_pm2)).
:::

:::info
The Servo Power Rail is neither powered by, nor provides power to the FMU.
However, the pins marked **+** are all common, and a BEC may be connected to any of the servo pin sets to power the servo power rail.
:::

## Захист від перенапруги

The _V5 nano_ has no over current protection.

<a id="Optional-hardware"></a>

## Периферійні пристрої

- [Цифровий датчик швидкості польоту](https://item.taobao.com/item.htm?spm=a1z10.3-c-s.w4002-16371268452.37.6d9f48afsFgGZI\&id=9512463037)
- [Телеметричні радіо модулі](https://cuav.taobao.com/category-158480951.htm?spm=2013.1.w5002-16371268426.4.410b7a821qYbBq\&search=y\&catName=%CA%FD%B4%AB%B5%E7%CC%A8)
- [Rangefinders/Distance sensors](../sensor/rangefinders.md)

## Підтримувані платформи / Конструкції

Будь-який мультикоптер / літак / наземна платформа / човен, який може керуватися звичайними RC сервоприводами або сервоприводами Futaba S-Bus.
The complete set of supported configurations can be seen in the [Airframes Reference](../airframes/airframe_reference.md).

## Сумісність

CUAV використовує деякі відмінні дизайни і несумісний з деяким обладнанням, про що буде описано нижче.

<a id="compatibility_gps"></a>

#### Neo v2.0 GPS несумісний з іншими пристроями

The _Neo v2.0 GPS_ that is recommended for use with _CUAV V5+_ and _CUAV V5 nano_ is not fully compatible with other Pixhawk flight controllers (specifically, the buzzer part is not compatible and there may be issues with the safety switch).

The UAVCAN [NEO V2 PRO GNSS receiver](http://doc.cuav.net/gps/neo-series-gnss/en/neo-v2-pro.html) can also be used, and is compatible with other flight controllers.

<a id="compatibility_jtag"></a>

#### Використання JTAG для апаратного налагодження

`DSU7` FMU Debug Pin 1 is 5 volts - not the 3.3 volts of the CPU.

Деякі JTAG-зонди використовують це напругу для встановлення рівнів введення-виведення при спілкуванні з цільовим об'єктом.

For direct connection to _Segger Jlink_ we recommended you use the 3.3 Volts of DSM/SBUS/RSSI pin 4 as Pin 1 on the debug connector (`Vtref`).

<a id="compatibility_pm2"></a>

#### PM2 не може живити модульний політний контролер

`PM2` can only measure battery voltage and current, but **not** power the flight controller.

:::warning
PX4 does not support this interface.
:::

## Відомі проблеми

The issues below refer to the _batch number_ in which they first appear.
Номер партії - це чотирицифрова дата виробництва за V01 та відображається на наклейці з боку контролера польоту.
Наприклад, серійний номер партії V011904 ((V01 - це номер V5, 1904 - це дата виробництва, тобто номер партії).

<a id="pin1_unfused"></a>

#### Інтерфейс SBUS / DSM / RSSI Pin1 не захищений від перевантаження

:::warning
This is a safety issue.
:::

Будь ласка, не підключайте інше обладнання (крім RC приймача) до інтерфейсу SBUS / DSM / RSSI - це може призвести до пошкодження обладнання!

- _Found:_ Batches V01190904xxxx
- _Fixed:_ Batches later than V01190904xxxx

## Подальша інформація

- [V5 nano manual](http://manual.cuav.net/V5-nano.pdf) (CUAV)
- [FMUv5 reference design pinout](https://docs.google.com/spreadsheets/d/1-n0__BYDedQrc_2NHqBenG1DNepAgnHpSGglke-QQwY/edit#gid=912976165) (CUAV)
- [CUAV Github](https://github.com/cuav) (CUAV)
- [Airframe build-log using CUAV v5 nano on a DJI FlameWheel450](../frames_multicopter/dji_f450_cuav_5nano.md)
