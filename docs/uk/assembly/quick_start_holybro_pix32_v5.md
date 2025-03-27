# Швидкий старт Pix32 v5 Wiring

:::warning
PX4 не розробляє цей (або будь-який інший) автопілот.
Contact the [manufacturer](https://holybro.com/) for hardware support or compliance issues.
:::

This quick start guide shows how to power the [Holybro Pix32v5](../flight_controller/holybro_pix32_v5.md)<sup>&reg;</sup> flight controller and connect its most important peripherals.

![Pix32 v5 With Base](../../assets/flight_controller/holybro_pix32_v5/IMG_3165.jpg)

## Розпаковка

Pix32 v5 is sold bundled with a number of different combinations of accessories, including the _pix32 v5 Base board_, power module _PM02 V3_, and the [Holybro M8N GPS](https://holybro.com/collections/gps/products/m8n-gps) (UBLOX NEO-M8N).

The content of the box with the _PM02 V3_ power module and _Pixhawk 4 GPS/Compass_ is shown below.
У коробці також є керівництво з роз'ємів та інструкції щодо модуля живлення, а також базова плата (не показана на схемі нижче).

![Pix32 v5 Box](../../assets/flight_controller/holybro_pix32_v5/pix32_v5_unboxing_schematics.png)

## Огляд схеми підключення

На зображенні нижче показано, як під'єднати найважливіші датчики та периферійні пристрої (за винятком виходів мотора та сервоприводів).
Ми розглянемо кожну з них докладно в наступних розділах.

![Pix32 v5 Wiring Overview](../../assets/flight_controller/holybro_pix32_v5/pix32_v5_wiring_overview.jpg)

:::tip
More information about available ports can be found [here](https://cdn.shopify.com/s/files/1/0604/5905/7341/files/Holybro_Pix32-V5-Base-Mini-Pinouts.pdf).
:::

## Монтаж та орієнтація контролера

_Pix32 v5_ should be mounted on the frame positioned as close to your vehicle’s center of gravity as possible, oriented top-side up with the arrow pointing towards the front of the vehicle.

![Pix32 v5 With Orientation](../../assets/flight_controller/holybro_pix32_v5/pix32_v5_orientation.png)

:::info
Якщо контролер не може бути змонтований у рекомендованому/стандартному положенні (наприклад, через обмеження місця), вам потрібно буде налаштувати програмне забезпечення автопілота з орієнтацією, яку ви фактично використовували: [Орієнтація контролера польоту](../config/flight_controller_orientation.md).
:::

:::tip
The board has internal vibration-isolation.
Не використовуйте віброізоляційну пінку для монтажу контролера (подвійна стрічка на клейовій основі зазвичай достатня).
:::

## GPS + компас + зумер + захисний вимикач + світлодіод

Pix32 v5 is designed to work well with the [Holybro M8N GPS](https://holybro.com/collections/gps/products/m8n-gps), which has an integrated compass, safety switch, buzzer and LED.
It connects directly to the **GPS port** using the 10 pin cable.

![Pix32 v5 with GPS](../../assets/flight_controller/holybro_pix32_v5/pix32_v5_connection_gps_compass.jpg)

GPS/Компас слід монтувати на раму якомога подалі від інших електронних пристроїв, з напрямком вперед транспортного засобу (відокремлення компаса від інших електронних пристроїв зменшить втручання).

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

The [Power Module (PM02 v3)](../power_module/holybro_pm02.md) can be bundled with _pix32 v5_.
Він надає регульоване живлення контролеру польоту та надсилає напругу/силу струму акумулятора контролеру польоту.

Connect the output of the _Power Module_ as shown.

![Pix32 v5 With Power Module](../../assets/flight_controller/holybro_pix32_v5/pix32_v5_connection_power.jpg)

- PM voltage/current port: connect to POWER1 port (or `POWER2`) using the 6-wire GH cable supplied.
- Вхід PM (роз'єм XT60): підключіть до ліпо-акумулятора (2~12S).
- Вихід живлення PM (роз'єм XT60): підведіть до будь-якого контролера регулятора обертів мотора.

:::info
As this power module does not include power distribution wiring, you would normally just connect all the ESCs in parallel to the power module output (the ESC must be appropriate for the supplied voltage level).
:::

:::info
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

### Конфігурація батареї

The battery/power setup must be configured in [Battery Estimation Tuning](../config/battery.md).
For either Power Module you will need to configure the _Number of Cells_.

You will not need to update the _voltage divider_ unless you are using some other power module (e.g. the one from the Pixracer).

## Радіоуправління

Для того щоб керувати транспортним засобом _вручну_, потрібна система радіоуправління (RC) (PX4 не потребує системи радіоуправління для автономних режимів польоту).

Вам потрібно [вибрати сумісний передавач/приймач](../getting_started/rc_transmitter_receiver.md) і _зв'язати_ їх таким чином, щоб вони взаємодіяли (ознайомтеся з інструкціями, що додаються до вашого конкретного передавача/приймача).

The instructions below show how to connect the different types of receivers to _Pix32 v5_ with Baseboard:

- Spektrum/DSM receivers connect to the **DSM RC** input shown below.

  ![Pix32v5 rc receivers](../../assets/flight_controller/holybro_pix32_v5/pix32_v5_receivers_connection.jpg)

- PPM and S.Bus receivers connect to the **SBUS_IN/PPM_IN** input port (marked as RC IN):

  ![Pinouts](../../assets/flight_controller/holybro_pix32_v5/pix32_v5_pinouts_back_label.png)

- PPM and PWM receivers that have an _individual wire for each channel_ must connect to the **PPM RC** port _via a PPM encoder_ [like this one](http://www.getfpv.com/radios/radio-accessories/holybro-ppm-encoder-module.html) (PPM-Sum receivers use a single signal wire for all channels).

Для отримання додаткової інформації про вибір радіосистеми, сумісність приймача та зв'язок вашої передавача/приймача, див. статтю: [Пульт керування передавачів & приймачів](../getting_started/rc_transmitter_receiver.md).

## Телеметричні радіостанції (Опціонально)

Телеметричні радіостанції можуть використовуватися для зв'язку та управління транспортним засобом у польоті з наземної станції (наприклад, ви можете направляти БПЛА до певної позиції або завантажувати нове завдання).

The vehicle-based radio should be connected to the **TELEM1** port as shown below (if connected to this port, no further configuration is required).
Інша радіостанція підключається до вашого комп'ютера або мобільного пристрою наземної станції (зазвичай за допомогою USB).

![Pix32 v5 With Telemetry Radios](../../assets/flight_controller/holybro_pix32_v5/pix32_v5_telemetry_radio.jpg)

## SD-карта (Опціонально)

SD cards are most commonly used to [log and analyse flight details](../getting_started/flight_reporting.md).
A micro SD card should come preinstalled on the pix32 v5, if you have your own micro SD card, insert the card into _pix32 v5_ as shown below.

![Pix32 v5 With SD Card](../../assets/flight_controller/holybro_pix32_v5/pix32_v5_sd_card.jpg)

:::tip
The SanDisk Extreme U3 32GB is [highly recommended](../dev_log/logging.md#sd-cards).
:::

## Двигуни

Motors/servos control signals are connected to the **I/O PWM OUT** (**MAIN**) and **FMU PWM OUT** (**AUX**) ports in the order specified for your vehicle in the [Airframe Reference](../airframes/airframe_reference.md).

![Pix32 v5 - Back Pinouts (Schematic)](../../assets/flight_controller/holybro_pix32_v5/pix32_v5_pinouts_back_label.png)

The motors must be separately [powered](#power).

:::info
If your frame is not listed in the airframe reference then use a "generic" airframe of the correct type.
:::

## Інші периферійні пристрої

Підключення та конфігурація додаткових/менш поширених компонентів описано в темах для окремих [периферійних пристроїв](../peripherals/index.md).

## Схема розташування виводів

[Pix32 v5 Pinouts](https://cdn.shopify.com/s/files/1/0604/5905/7341/files/Holybro_Pix32-V5-Base-Mini-Pinouts.pdf) (Holybro)

## Налаштування

Загальну інформацію про конфігурацію описано в: [Конфігурація автопілота](../config/index.md).

Конкретні конфігурації QuadPlane тут: [QuadPlane VTOL налаштування](../config_vtol/vtol_quad_configuration.md)

<!-- Nice to have detailed wiring infographic and instructions for different vehicle types. -->

## Подальша інформація

- [Pix32 v5 Overview](../flight_controller/holybro_pix32_v5.md) (Overview page)
- [Pix32 v5 Technical Data Sheet](https://cdn.shopify.com/s/files/1/0604/5905/7341/files/Holybro_PIX32-V5_technical_data_sheet_v1.1.pdf)
- [Pix32 v5 Pinouts](https://cdn.shopify.com/s/files/1/0604/5905/7341/files/Holybro_Pix32-V5-Base-Mini-Pinouts.pdf)
- [Pix32 v5 Base Schematic Diagram](https://cdn.shopify.com/s/files/1/0604/5905/7341/files/Holybro_PIX32-V5-BASE-Schematic_diagram.pdf)
- [Pix32 v5 Base Components Layout](https://holybro.com/manual/Holybro_PIX32-V5-BASE-ComponentsLayout.pdf)
- [FMUv5 reference design pinout](https://docs.google.com/spreadsheets/d/1-n0__BYDedQrc_2NHqBenG1DNepAgnHpSGglke-QQwY/edit#gid=912976165).
