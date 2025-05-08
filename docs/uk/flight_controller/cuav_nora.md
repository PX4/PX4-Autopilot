# Диспетчер польотів CUAV Nora

:::warning
PX4 не розробляє цей (або будь-який інший) автопілот.
Contact the [manufacturer](https://www.cuav.net) for hardware support or compliance issues.
:::

The [Nora](https://doc.cuav.net/flight-controller/x7/en/nora.html)<sup>&reg;</sup> flight controller is a high-performance autopilot.
Це ідеальний вибір для промислових дронів і великомасштабних важких дронів.
В основному постачається комерційним виробникам.

![CUAV x7](../../assets/flight_controller/cuav_nora/nora.png)

Нора - це варіант CUAV X7.
Він використовує інтегровану материнську плату (м'яку і тверду), що зменшує кількість внутрішніх роз'ємів польотного контролера, підвищує надійність і розміщує всі інтерфейси збоку (роблячи проводку більш лаконічною).

:::info
This flight controller is [manufacturer supported](../flight_controller/autopilot_manufacturer_supported.md).
:::

## Функції

- Внутрішнє поглинання ударів
- Інтегрований процес зменшує відмову, спричинену пошкодженням інтерфейсу.
- Підтримка USB_HS, швидше завантаження журналів (PX4 ще не підтримується)
- Підтримка більшої кількості виходів dshot
- Підтримка нагріву IMU, покращення роботи датчика
- Виділений порт для акумулятора CAN
- 3 комплекти датчиків IMU
- Автомобільний компас RM3100
- Високопродуктивний процесор

:::tip
The manufacturer [CUAV Docs](https://doc.cuav.net/flight-controller/x7/en/nora.html) are the canonical reference for Nora.
Вони повинні використовуватися за перевагою, оскільки вони містять найбільш повну та актуальну інформацію.
:::

## Короткий опис

- Головний FMU процесор: STM32H743

- Бортові сенсори:

  - Акселерометр/Гіроскоп: ICM-20689
  - Прискорювач/гіроскоп: ICM-20649
  - Акселерометр/Гіроскоп: BMI088
  - Магнітометр: RM3100
  - Барометр: MS5611\*2

- Інтерфейси:
  - 14 ШІМ-виходів （12 підтримує Dshot）
  - Підтримка декількох входів RC (SBU / CPPM / DSM)
  - Аналоговий / PWM вхід RSSI
  - 2 GPS порти (GPS і UART4 порти)
  - 4 шини i2c (два виділені порти i2c)
  - 2 порти CAN шини
  - 2 порти живлення (Power A - загальний інтерфейс адаптера, Power C - інтерфейс акумулятора DroneCAN)
  - 2 входи АЦП
  - 1 USB порт

- Система живлення:
  - Живлення: 4.3~5.4В
  - Вхід USB: 4.75~5.25В
  - Вхід сервоприводу: 0~36V

- Вага та розміри:
  - Вага: 101 g

- Інші характеристики:
  - Робоча температура: -20 ~ 80°c (виміряне значення)
  - Три імуси
  - Підтримка компенсації температури
  - Внутрішнє поглинання ударів

:::info
When it runs PX4 firmware, only 8 PWM outputs work.
Решта 6 ШІМ-портів все ще адаптуються (тому на момент написання статті вони не сумісні з VOLT).
:::

## Де купити

- [CUAV Store](https://store.cuav.net)<\br>
- [CUAV Aliexpress](https://www.aliexpress.com/item/4001042501927.html?gps-id=8041884&scm=1007.14677.110221.0&scm_id=1007.14677.110221.0&scm-url=1007.14677.110221.0&pvid=3dc0a3ba-fa82-43d2-b0b3-6280e4329cef&spm=a2g0o.store_home.promoteRecommendProducts_7913969.58)

## З'єднання (Проводка)

[CUAV nora Wiring Quickstart](https://doc.cuav.net/flight-controller/x7/en/quick-start/quick-start-nora.html)

## Розмір та роз'єми

![CUAV x7](../../assets/flight_controller/cuav_nora/nora-size.jpg)

![X7 pinouts](../../assets/flight_controller/cuav_nora/nora-pinouts.jpg)

:::warning
The `RCIN` port is limited to powering the rc receiver and cannot be connected to any power/load.
:::

## Номінальна напруга

Nora AutoPilot\* може мати потрійне резервування джерела живлення, якщо до нього підключено три джерела живлення. The two power rails are: **POWERA**, **POWERC** and **USB**.

:::info
The output power rails **PWM OUT** (0V to 36V) do not power the flight controller board (and are not powered by it).
You must supply power to one of **POWERA**, **POWERC** or **USB** or the board will be unpowered.
:::

**Normal Operation Maximum Ratings**

За таких умов всі джерела живлення будуть використовуватися в цьому порядку для живлення системи:

1. **POWERA** and **POWERC** inputs (4.3V to 5.4V)
2. **USB** input (4.75V to 5.25V)

## Збірка прошивки

:::tip
Most users will not need to build this firmware!
It is pre-built and automatically installed by _QGroundControl_ when appropriate hardware is connected.
:::

To [build PX4](../dev_setup/building_px4.md) for this target:

```
make cuav_nora_default
```

## Захист від перенапруги

The _Nora_ has over-current protection on the 5 Volt Peripheral and 5 Volt high power, which limits the current to 2.5A.
The _Nora_ has short circuit protection.

:::warning
Up to 2.5 A can be delivered to the connectors listed as pin 1 (although these are only rated at 1 A).
:::

## Відладочний порт

The system's serial console and SWD interface operate on the **DSU7** port.
Просто підключіть кабель FTDI до роз'єму DSU7 (у списку продуктів є кабель CUAV FTDI).

The [PX4 System Console](../debug/system_console.md) and [SWD interface](../debug/swd_debug.md) operate on the **FMU Debug** port (`DSU7`).

The debug port (`DSU7`) uses a [JST BM06B](https://www.digikey.com.au/product-detail/en/jst-sales-america-inc/BM06B-GHS-TBT-LF-SN-N/455-1582-1-ND/807850) connector and has the following pinout:

| Pin                        | Сигнал                            | Вольтаж               |
| -------------------------- | --------------------------------- | --------------------- |
| 1 (red) | 5V+                               | +5V                   |
| 2 (blk) | DEBUG TX (OUT) | +3.3V |
| 3 (blk) | DEBUG RX (IN)  | +3.3V |
| 4 (blk) | FMU_SWDIO    | +3.3V |
| 5 (blk) | FMU_SWCLK    | +3.3V |
| 6 (blk) | GND                               | GND                   |

CUAV provides a dedicated debugging cable, which can be connected to the `DSU7` port.
This splits out an FTDI cable for connecting the [PX4 System Console](../debug/system_console.md) to a computer USB port, and SWD pins used for SWD/JTAG debugging.
The provided debug cable does not connect to the SWD port `Vref` pin (1).

![CUAV Debug cable](../../assets/flight_controller/cuav_v5_plus/cuav_v5_debug_cable.jpg)

:::warning
The SWD Vref pin (1) uses 5V as Vref but the CPU is run at 3.3V!

Деякі JTAG-адаптери (SEGGER J-Link) використовують напругу Vref для встановлення напруги на лініях SWD.
For direct connection to _Segger Jlink_ we recommended you use the 3.3 Volts from pin 4 of the connector marked `DSM`/`SBUS`/`RSSI` to provide `Vtref` to the JTAG (i.e. providing 3.3V and _NOT_ 5V).
:::

## Підтримувані платформи / Конструкції

Будь-який мультикоптер / літак / наземна платформа / човен, який може керуватися звичайними RC сервоприводами або сервоприводами Futaba S-Bus.
The complete set of supported configurations can be seen in the [Airframes Reference](../airframes/airframe_reference.md).

## Подальша інформація

- [Quick start](https://doc.cuav.net/flight-controller/x7/en/quick-start/quick-start-nora.html)
- [CUAV docs](http://doc.cuav.net)
- [nora schematic](https://github.com/cuav/hardware/tree/master/X7_Autopilot)
