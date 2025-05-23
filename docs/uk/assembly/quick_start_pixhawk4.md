# Швидкий старт підключення Pixhawk 4

:::warning
PX4 не розробляє цей (або будь-який інший) автопілот.
Contact the [manufacturer](https://holybro.com/) for hardware support or compliance issues.
:::

This quick start guide shows how to power the [Pixhawk 4](../flight_controller/pixhawk4.md)<sup>&reg;</sup> flight controller and connect its most important peripherals.

<img src="../../assets/flight_controller/pixhawk4/pixhawk4_logo_view.jpg" width="420px" title="Pixhawk4 Image" />

## Огляд схеми підключення

На зображенні нижче показано, як під'єднати найважливіші датчики та периферійні пристрої (за винятком виходів мотора та сервоприводів). Ми розглянемо кожну з них докладно в наступних розділах.

![Pixhawk 4 Wiring Overview](../../assets/flight_controller/pixhawk4/pixhawk4_wiring_overview.png)

:::tip
More information about available ports can be found here: [Pixhawk 4 > Connections](../flight_controller/pixhawk4.md#connectors).
:::

## Монтаж та орієнтація контролера

_Pixhawk 4_ should be mounted on the frame using vibration-damping foam pads (included in the kit). Вона повинна розташовуватися якомога ближче до центру тяжіння вашого апарату верхньою стороною вгору зі стрілкою, що вказує в напрямку передньої частини апарату.

<img src="../../assets/flight_controller/pixhawk4/pixhawk4_mounting_and_foam.png" align="center"/>

:::info
If the controller cannot be mounted in the recommended/default orientation (e.g. due to space constraints) you will
need to configure the autopilot software with the orientation that you actually used: [Flight Controller Orientation](../config/flight_controller_orientation.md).
:::

## GPS + компас + зумер + захисний вимикач + світлодіод

Attach the provided GPS with integrated compass, safety switch, buzzer and LED to the **GPS MODULE** port.

The GPS/Compass should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker towards the front of the vehicle (separating the compass from other electronics will reduce interference).

![Connect compass/GPS to Pixhawk 4](../../assets/flight_controller/pixhawk4/pixhawk4_compass_gps.jpg)

:::info
Вбудований безпечний вимикач в GPS-модулі увімкнений _за замовчуванням_ (коли включений, PX4 не дозволить вам готувати до польоту).
Щоб вимкнути безпеку, натисніть і утримуйте безпечний вимикач протягом 1 секунди.
Ви можете натиснути безпечний вимикач знову, щоб увімкнути безпеку та відключити транспортний засіб (це може бути корисно, якщо, з якихось причин, ви не можете вимкнути транспортний засіб за допомогою вашого пульта дистанційного керування або наземної станції).
:::

## Power

Connect the output of the _Power Management Board_ (PM board) that comes with the kit to one of the **POWER** bricks of _Pixhawk 4_ using a 6-wire cable.
The PM input **2~12S** will be connected to your LiPo battery.
Підключення плати управління живленням, включаючи живлення та сигнальні з'єднання з ESC та сервоприводами, пояснені в таблиці нижче.
Note that the PM board does not supply power to the servos via + and - pins of **FMU PWM-OUT**.

The image below shows the power management board provided with _Pixhawk 4_.

![Pixhawk 4 - Power Management Board](../../assets/hardware/power_module/holybro_pm07/pixhawk4_power_management_board.png)

:::info
If using a plane or rover, the 8 pin power (+) rail of **FMU PWM-OUT** will need to be separately powered in order to drive servos for rudders, elevons etc.
Щоб це зробити, живильну рейку потрібно підключити до ESC з BEC або автономного BEC на 5V або 2S LiPo акумулятора.
Будьте обережні з напругою сервопривода, який ви збираєтеся використовувати тут.
:::

| PIN&Connector | Функція                                                                                                     |
| --------------------------------- | ----------------------------------------------------------------------------------------------------------- |
| I/O PWM-IN                        | See note below for connection to _Pixhawk 4_                                                                |
| M1                                | I/O PWM OUT 1: підключіть дрот сигналу до ESC мотору 1 тут                                  |
| M2                                | I/O PWM OUT 2: підключіть сигнальний провід до ESC двигуна 2 тут                            |
| M3                                | I/O PWM OUT 3: підключіть дрот сигналу до ESC мотору 3 тут                                  |
| M4                                | I/O PWM OUT 4: підключіть дрот сигналу до ESC мотору 4 тут                                  |
| M5                                | I/O PWM OUT 5: підключіть дрот сигналу до ESC мотору 5 тут                                  |
| M6                                | I/O PWM OUT 6: підключіть дрот сигналу до ESC мотору 6 тут                                  |
| M7                                | I/O PWM OUT 7: підключіть дрот сигналу до ESC мотору 7 тут                                  |
| M8                                | I/O PWM OUT 8: підключіть дрот сигналу до ESC мотору 8 тут                                  |
| FMU PWM-IN                        | See note below for connection to _Pixhawk 4_                                                                |
| FMU PWM-OUT                       | If FMU PWM-IN is connected to _Pixhawk 4_, connect signal wires to ESC or signal, +, - wires to servos here |
| CAP&ADC-OUT   | connect to CAP & ADC IN port of _Pixhawk 4_                                             |
| CAP&ADC-IN    | CAP&ADC input: Pinouts are printed on the back side of the board        |
| B+                                | підключіться до ESC B+, щоб живити ESC                                                                      |
| GND                               | підключіться до землі ESC                                                                                   |
| PWR1                              | 5v output 3A, connect to _Pixhawk 4_ POWER 1                                                                |
| PWR2                              | 5v output 3A, connect to _Pixhawk 4_ POWER 2                                                                |
| 2~12S             | Живлення, підключіть до акумулятора LiPo ~12s                                               |

:::info
Depending on your airframe type, refer to [Airframe Reference](../airframes/airframe_reference.md) to connect **I/O PWM OUT** and **FMU PWM OUT** ports of _Pixhawk 4_ to PM board.
**MAIN** outputs in PX4 firmware map to **I/O PWM OUT** port of _Pixhawk 4_ whereas **AUX outputs** map to **FMU PWM OUT** of _Pixhawk 4_.
For example, **MAIN1** maps to IO_CH1 pin of **I/O PWM OUT** and **AUX1** maps to FMU_CH1 pin of **FMU PWM OUT**. **FMU PWM-IN** of PM board is internally connected to **FMU PWM-OUT**. **I/O PWM-IN** of PM board is internally connected to **M1-8**.
:::

The following table summarizes how to connect _Pixhawk 4_'s PWM OUT ports to PM board's PWM-IN ports, depending on the Airframe Reference.

| Довідник зі структури літальних апаратів | Connection between _Pixhawk 4_ --> PM board |
| ---------------------------------------- | ------------------------------------------- |
| **MAIN**: motor          | I/O PWM OUT --> I/O PWM IN                  |
| **MAIN**: servo          | I/O PWM OUT --> FMU PWM IN                  |
| **AUX**: motor           | FMU PWM OUT --> I/O PWM IN                  |
| **AUX**: servo           | FMU PWM OUT --> FMU PWM IN                  |

<!--In the future, when Pixhawk 4 kit is available, add wiring images/videos for different airframes.-->

The pinout of _Pixhawk 4_’s power ports is shown below.
Сигнал CURRENT повинен переносити аналогове напругу від 0-3.3V для 0-120A за замовчуванням.
Сигнал VOLTAGE повинен переносити аналогове напругу від 0-3.3V для 0-60V за замовчуванням.
Лінії VCC повинні пропонувати принаймні 3A безперервного струму і за замовчуванням повинні мати напругу 5,1 В.
Нижчий напруга 5V все ще прийнятний, але не рекомендується.

| Pin                         | Сигнал  | Вольтаж               |
| --------------------------- | ------- | --------------------- |
| 1(red)   | VCC     | +5V                   |
| 2(black) | VCC     | +5V                   |
| 3(black) | CURRENT | +3.3V |
| 4(black) | VOLTAGE | +3.3V |
| 5(black) | GND     | GND                   |
| 6(black) | GND     | GND                   |

:::info
Using the Power Module that comes with the kit you will need to configure the _Number of Cells_ in the [Power Settings](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/power.html) but you won't need to calibrate the _voltage divider_.
You will have to update the _voltage divider_ if you are using any other power module (e.g. the one from the Pixracer).
:::

## Радіоуправління

Для того щоб керувати транспортним засобом _вручну_, потрібна система радіоуправління (RC) (PX4 не потребує системи радіоуправління для автономних режимів польоту).

Вам потрібно [вибрати сумісний передавач/приймач](../getting_started/rc_transmitter_receiver.md) і _зв'язати_ їх таким чином, щоб вони взаємодіяли (ознайомтеся з інструкціями, що додаються до вашого конкретного передавача/приймача).

The instructions below show how to connect the different types of receivers to _Pixhawk 4_:

- Spektrum/DSM or S.BUS receivers connect to the **DSM/SBUS RC** input.

  ![Pixhawk 4 - Radio port for Spektrum receivers](../../assets/flight_controller/pixhawk4/pixhawk4_receiver_sbus.png)

- PPM receivers connect to the **PPM RC** input port.

  ![Pixhawk 4 - Radio port for PPM receivers](../../assets/flight_controller/pixhawk4/pixhawk_4_receiver_ppm.png)

- PPM and PWM receivers that have an _individual wire for each channel_ must connect to the **PPM RC** port _via a PPM encoder_ [like this one](http://www.getfpv.com/radios/radio-accessories/holybro-ppm-encoder-module.html) (PPM-Sum receivers use a single signal wire for all channels).

Для отримання додаткової інформації про вибір радіосистеми, сумісність приймача та зв'язок вашої передавача/приймача, див. статтю: [Пульт керування передавачів & приймачів](../getting_started/rc_transmitter_receiver.md).

## Телеметричні радіостанції (Опціонально)

Телеметричні радіостанції можуть використовуватися для зв'язку та управління транспортним засобом у польоті з наземної станції (наприклад, ви можете направляти БПЛА до певної позиції або завантажувати нове завдання).

The vehicle-based radio should be connected to the **TELEM1** port as shown below (if connected to this port, no further configuration is required). Інша радіостанція підключається до вашого комп'ютера або мобільного пристрою наземної станції (зазвичай за допомогою USB).

![Pixhawk 4/Telemetry Radio](../../assets/flight_controller/pixhawk4/pixhawk4_telemetry_radio.jpg)

<a id="sd_card"></a>

## SD-карта (Опціонально)

SD cards are highly recommended as they are needed to [log and analyse flight details](../getting_started/flight_reporting.md), to run missions, and to use UAVCAN-bus hardware.
Insert the card (included in Pixhawk 4 kit) into _Pixhawk 4_ as shown below.

![Pixhawk 4/SD Card](../../assets/flight_controller/pixhawk4/pixhawk4_sd_card.png)

:::tip
Для отримання додаткової інформації див. [Основні концепції > SD-карти (знімна пам'ять)](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory).
:::

## Двигуни

Motors/servos are connected to the **I/O PWM OUT** (**MAIN**) and **FMU PWM OUT** (**AUX**) ports in the order specified for your vehicle in the [Airframe Reference](../airframes/airframe_reference.md).

:::info
Цей довідник містить зіставлення портів виводу до моторів/сервоприводів для всіх підтримуваних повітряних та наземних шасі (якщо ваше шасі не вказане в довіднику, то використовуйте "загальний" планер відповідного типу).
:::

:::warning
Відображення не є однорідним для всіх конструкцій (наприклад, ви не можете покладатися на те, що ручка газу буде на тому ж вихідному порту для всіх повітряних конструкцій). Переконайтеся, що ви використовуєте правильне зіставлення для вашого апарату.
:::

## Інші периферійні пристрої

Підключення та конфігурація додаткових/менш поширених компонентів описано в темах для окремих [периферійних пристроїв](../peripherals/index.md).

## Схема розташування виводів

[Pixhawk 4 Pinouts](https://holybro.com/manual/Pixhawk4-Pinouts.pdf) (Holybro)

## Налаштування

Загальну інформацію про конфігурацію описано в: [Конфігурація автопілота](../config/index.md).

Конкретні конфігурації QuadPlane тут: [QuadPlane VTOL налаштування](../config_vtol/vtol_quad_configuration.md)

<!-- Nice to have detailed wiring infographic and instructions for different vehicle types. -->

## Подальша інформація

- [Pixhawk 4](../flight_controller/pixhawk4.md) (Overview page)
- [Pixhawk 4 Technical Data Sheet](https://github.com/PX4/PX4-user_guide/raw/main/assets/flight_controller/pixhawk4/pixhawk4_technical_data_sheet.pdf)
- [Pixhawk 4 Pinouts](https://holybro.com/manual/Pixhawk4-Pinouts.pdf) (Holybro)
- [Pixhawk 4 Quick Start Guide (Holybro)](https://holybro.com/manual/Pixhawk4-quickstartguide.pdf)
