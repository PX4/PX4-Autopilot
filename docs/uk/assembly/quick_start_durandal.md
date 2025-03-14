# Швидке підключення Durandal Wiring

<Badge type="tip" text="PX4 v1.11" />

:::warning
PX4 не розробляє цей (або будь-який інший) автопілот.
Contact the [manufacturer](https://holybro.com/) for hardware support or compliance issues.
:::

This quick start guide shows how to power the Holybro [Durandal](../flight_controller/durandal.md)<sup>&reg;</sup> flight controller and connect its most important peripherals.

![Durandal](../../assets/flight_controller/durandal/durandal_hero.jpg)

## Розпаковка

Durandal is sold bundled with a number of different combinations of accessories, including power modules: _PM02 V3_ and _PM07_, and the _Pixhawk 4 GPS/Compass_ ( u-blox NEO-M8N).

The content of the box with the _PM02 V3_ power module is shown below (the box also includes a pinout guide and power module instructions).

![Durandal Box](../../assets/flight_controller/durandal/durandal_unboxing_schematics.jpg)

## Огляд схеми підключення

На зображенні нижче показано, як під'єднати найважливіші датчики та периферійні пристрої (за винятком виходів мотора та сервоприводів).
Ми розглянемо кожну з них докладно в наступних розділах.

![Durandal Wiring Overview](../../assets/flight_controller/durandal/durandal_wiring_overview.jpg)

:::tip
More information about available ports can be found here: [Durandal > Pinouts](../flight_controller/durandal.md#pinouts).
:::

## Монтаж та орієнтація контролера

_Durandal_ should be mounted on the frame positioned as close to your vehicle’s center of gravity as possible, oriented top-side up with the arrow pointing towards the front of the vehicle.

![Mounting/Orientation](../../assets/flight_controller/durandal/orientation.jpg)

Якщо контролер не може бути змонтований у рекомендованому/стандартному положенні (наприклад, через обмеження місця), вам потрібно буде налаштувати програмне забезпечення автопілота з орієнтацією, яку ви фактично використовували: [Орієнтація контролера польоту](../config/flight_controller_orientation.md).

:::tip
The board has internal vibration-isolation.
Не використовуйте віброізоляційну пінку для монтажу контролера (подвійна стрічка на клейовій основі зазвичай достатня).
:::

## GPS + компас + зумер + захисний вимикач + світлодіод

Durandal is designed to work well with the _Pixhawk 4 GPS module_, which has an integrated compass, safety switch, buzzer and LED.
It connects directly to the [GPS port](../flight_controller/durandal.md#gps) using the 10 pin cable.

GPS/Компас слід монтувати на раму якомога подалі від інших електронних пристроїв, з напрямком вперед транспортного засобу (відокремлення компаса від інших електронних пристроїв зменшить втручання).

![Connect compass/GPS to Durandal](../../assets/flight_controller/durandal/connection_gps_compass.jpg)

:::info
Вбудований безпечний вимикач в GPS-модулі увімкнений _за замовчуванням_ (коли включений, PX4 не дозволить вам готувати до польоту).
Щоб вимкнути безпеку, натисніть і утримуйте безпечний вимикач протягом 1 секунди.
Ви можете натиснути безпечний вимикач знову, щоб увімкнути безпеку та відключити транспортний засіб (це може бути корисно, якщо, з якихось причин, ви не можете вимкнути транспортний засіб за допомогою вашого пульта дистанційного керування або наземної станції).
:::

## Power

Ви можете використовувати модуль живлення або розподільник живлення для живлення двигунів/сервоприводів та виміру споживаної потужності.
Рекомендовані модулі живлення показані нижче.

<a id="pm02_v3"></a>

### PM02D Power Module

The [Power Module (PM02 v3)](../power_module/holybro_pm02.md) can be bundled with _Durandal_.
Він надає регульоване живлення контролеру польоту та надсилає напругу/силу струму акумулятора контролеру польоту.

Connect the output of the _Power Module_ as shown.

![Durandal PM02v3 Power connections](../../assets/flight_controller/durandal/connection_power.jpg)

- PM voltage/current port: connect to [POWER1](../flight_controller/durandal.md#power) port (or `POWER2`) using the 6-wire GH cable supplied.
- Вхід PM (роз'єм XT60): підключіть до ліпо-акумулятора (2~12S).
- Вихід живлення PM (роз'єм XT60): підведіть до будь-якого контролера регулятора обертів мотора.

:::tip
As this power module does not include power distribution wiring, you would normally just connect all the ESCs in parallel to the power module output (the ESC must be appropriate for the supplied voltage level).
:::

:::tip
The 8 pin power (+) rail of **MAIN/AUX** is not powered by the power module supply to the flight controller.
Якщо вона повинна бути окремо живленою для керування сервоприводами для рульових поверхонь, елеронами тощо, лінію живлення потрібно підключити до ESC зі вбудованим BEC або окремого BEC напругою 5V або акумулятора LiPo 2S.
Переконайтеся, що напруга сервопривода, яку ви збираєтеся використовувати, відповідає.
:::

Модуль живлення має наступні характеристики/обмеження:

- Максимальна вхідна напруга: 60V
- Максимальне вимірювання струму: 120A Voltage
- Вимірювання струму, налаштоване для SV АЦП Перемикання виходів регулятора 5,2 В і 3А макс
- Вага: 20г
- Пакет включає:
  - Плата PM02
  - 6-контактний кабель MLX (1)
  - 6pin GH cable (1)

<a id="pm07"></a>

### Модуль живлення Pixhawk 4 (PM07)

The [Pixhawk 4 Power Module (PM07)](https://holybro.com/collections/power-modules-pdbs/products/pixhawk-4-power-module-pm07) can be bundled/used with _Durandal_.
Це виступає як модуль живлення та розподільник живлення, який забезпечує регульоване живлення контролеру польоту та регуляторам швидкості, а також надсилає напругу/сили до контролера польоту.

This is wired up in the same way as described in the [Pixhawk 4 Quick Start > Power](../assembly/quick_start_pixhawk4.md#power) documentation.

Має наступні характеристики/обмеження:

- Поточний PCB: загальна потужність виходів 120A (MAX)
- UBEC вихідний струм 5В: 3А
- Напруга входу UBEC: 7~51v (2~12s LiPo)
- Dimensions: 68_50_8 mm
- Отвір для монтажу: 45\*45мм
- Вага: 36g
- Пакет включає:
  - Плата PM07 (1)
  - 80мм коннекторія XT60 (1)

:::info
See also [PM07 Quick Start Guide](https://docs.holybro.com/power-module-and-pdb/power-module/pm07-quick-start-guide) (Holybro).
:::

### Конфігурація батареї

The battery/power setup must be configured in [Battery Estimation Tuning](../config/battery.md).
For either Power Module you will need to configure the _Number of Cells_.

You will not need to update the _voltage divider_ unless you are using some other power module (e.g. the one from the Pixracer).

## Радіоуправління

Для того щоб керувати транспортним засобом _вручну_, потрібна система радіоуправління (RC) (PX4 не потребує системи радіоуправління для автономних режимів польоту).

Вам потрібно [вибрати сумісний передавач/приймач](../getting_started/rc_transmitter_receiver.md) і _зв'язати_ їх таким чином, щоб вони взаємодіяли (ознайомтеся з інструкціями, що додаються до вашого конкретного передавача/приймача).

The instructions below show how to connect the different types of receivers to _Durandal_:

- Spektrum/DSM receivers connect to the [DSM RC](../flight_controller/durandal.md#dsm-rc-port) input.

  ![Durandal - DSM](../../assets/flight_controller/durandal/dsm.jpg)

- PPM and S.Bus receivers connect to the [SBUS_IN/PPM_IN](../flight_controller/durandal.md#rc-in) input port (marked as RC IN, next to the MAIN/AUX inputs).

  ![Durandal - Back Pinouts (Schematic)](../../assets/flight_controller/durandal/durandal_pinouts_back.jpg)

- PPM and PWM receivers that have an _individual wire for each channel_ must connect to the **PPM RC** port _via a PPM encoder_ [like this one](http://www.getfpv.com/radios/radio-accessories/holybro-ppm-encoder-module.html) (PPM-Sum receivers use a single signal wire for all channels).

Для отримання додаткової інформації про вибір радіосистеми, сумісність приймача та зв'язок вашої передавача/приймача, див. статтю: [Пульт керування передавачів & приймачів](../getting_started/rc_transmitter_receiver.md).

## Телеметричні радіостанції (Опціонально)

Телеметричні радіостанції можуть використовуватися для зв'язку та управління транспортним засобом у польоті з наземної станції (наприклад, ви можете направляти БПЛА до певної позиції або завантажувати нове завдання).

The vehicle-based radio should be connected to the [TELEM1](../flight_controller/durandal.md#telem1_2_3) port as shown below using one of the 6-pos connectors (if connected to this port, no further configuration is required).
Інша радіостанція підключається до вашого комп'ютера або мобільного пристрою наземної станції (зазвичай за допомогою USB).

![Durandal/Telemetry Radio](../../assets/flight_controller/durandal/holybro_telemetry_radio.jpg)

## SD-карта (Опціонально)

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert an SD card into the _Durandal_ where indicated below.

![Durandal SD Card](../../assets/flight_controller/durandal/durandal_sd_slot.jpg)

:::tip
Для отримання додаткової інформації див. [Основні концепції > SD-карти (знімна пам'ять)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

## Двигуни

Motors/servos control signals are connected to the **I/O PWM OUT** (**MAIN OUT**) and **FMU PWM OUT** (**AUX**) ports in the order specified for your vehicle in the [Airframe Reference](../airframes/airframe_reference.md).

![Durandal - Back Pinouts (Schematic)](../../assets/flight_controller/durandal/durandal_pinouts_back.jpg)

The motors must be separately [powered](#power).

:::info
If your frame is not listed in the airframe reference then use a "generic" airframe of the correct type.
:::

:::tip
_Durandal_ has 5 AUX ports, so cannot be used with airframes that map AUX6, AUX7, AUX8 to motors or other critical flight controls.
:::

## Інші периферійні пристрої

Підключення та конфігурація додаткових/менш поширених компонентів описано в темах для окремих [периферійних пристроїв](../peripherals/index.md).

## Схема розташування виводів

[Durandal > Pinouts](../flight_controller/durandal.md#pinouts)

<a id="configuration"></a>

## Конфігурація PX4

First you will need to install [PX4 "Master" Firmware](../config/firmware.md#custom) onto the controller using _QGroundControl_.

:::info
Durandal support will be in the _stable_ PX4 release that follows PX4 v1.10.
:::

Further general configuration information is covered in: [Autopilot Configuration](../config/index.md).

Конкретні конфігурації QuadPlane тут: [QuadPlane VTOL налаштування](../config_vtol/vtol_quad_configuration.md)

## Подальша інформація

- [Durandal Overview](../flight_controller/durandal.md)
- [Durandal Technical Data Sheet](https://cdn.shopify.com/s/files/1/0604/5905/7341/files/Durandal_technical_data_sheet_90f8875d-8035-4632-a936-a0d178062077.pdf) (Holybro)
- [Durandal Pinouts](https://holybro.com/collections/autopilot-flight-controllers/products/Durandal-Pinouts) (Holybro)
- [Durandal_MB_H743sch.pdf](https://github.com/PX4/PX4-user_guide/raw/main/assets/flight_controller/durandal/Durandal_MB_H743sch.pdf) (Durandal Schematics)
- [STM32H743IIK_pinout.pdf](https://github.com/PX4/PX4-user_guide/raw/main/assets/flight_controller/durandal/STM32H743IIK_pinout.pdf) (Durandal Pinmap)
