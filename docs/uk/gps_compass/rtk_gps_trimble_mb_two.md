# Trimble MB-Two

The [Trimble MB-Two RTK GPS receiver](https://www.trimble.com/Precision-GNSS/MB-Two-Board.aspx) is a high-end, dual-frequency [RTK GPS module](../gps_compass/rtk_gps.md) that can be configured as either base or rover.

Крім того, надаючи точну інформацію про положення, MB-Two може оцінити кут напрямку (він має підтримку двох антен). Це корисно для ситуацій, де компас не може надати надійну інформацію про напрямок, наприклад, при польоті близько до металевих конструкцій.

![MB-Two Hero image](../../assets/hardware/gps/rtk_trimble_two_gnss_hero.jpg)

## Необхідні параметри прошивки

При купівлі пристрою необхідно вибрати наступні параметри вбудованого програмного забезпечення:

- \[X\] \[2\] \[N\] \[G\] \[W\] \[Y\] \[J\] для оновлень позиції 20 Гц та підтримки RTK, горизонтальна точність позиції 1 см та вертикальна 2 см
- \[L\] LBAND
- \[D\] DUO - Напрямок з двома антенами
- \[B\] BEIDOU + \[O\] GALILEO, за потреби

## Антени та кабель

Для Trimble MB-Two потрібні дві двохчастотні (L1/L2) антени.
A good example is the [Maxtenna M1227HCT-A2-SMA](http://www.maxtena.com/products/helicore/m1227hct-a2-sma/)
(which can be bought, for instance, from [Farnell](https://uk.farnell.com/maxtena/m1227hct-a2-sma/antenna-1-217-1-25-1-565-1-61ghz/dp/2484959)).

Тип роз'єму антени на пристрої - MMCX.
Підходящі кабелі для вищезазначених антен (коннектор SMA) можна знайти тут:

- [30 cm version](https://www.digikey.com/products/en?mpart=415-0073-012&v=24)
- [45 cm version](https://www.digikey.com/products/en?mpart=415-0073-018&v=24)

## Підключення та з'єднання

Trimble MB-Two підключений до UART на польотному контролері (порт GPS) для передачі даних.

Для живлення модуля вам знадобиться окреме джерело живлення 3,3 В (максимальне споживання 360 мА).

:::info
The module cannot be powered from a Pixhawk.
:::

Контакти на 28-контактному роз'ємі пронумеровані, як показано нижче:

![MB-Two Pinout](../../assets/hardware/gps/rtk_trimble_two_gnss_pinouts.jpg)

| Pin | Назва                    | Опис                                                 |
| --- | ------------------------ | ---------------------------------------------------- |
| 6   | Vcc 3.3V | Джерело живлення                                     |
| 14  | GND                      | Connect to power the supply and GND of the Autopilot |
| 15  | TXD1                     | Connect to RX of the Autopilot                       |
| 16  | RXD1                     | Connect to TX of the Autopilot                       |

## Налаштування

First set the GPS protocol to Trimble ([GPS_x_PROTOCOL=3](../advanced_config/parameter_reference.md#GPS_1_PROTOCOL)).

Для оцінювання заголовку потрібно, щоб дві антени були на одному рівні і були відстані щонайменше 30 см одна від одної.
The direction that they are facing does not matter as it can be configured with the [GPS_YAW_OFFSET](../advanced_config/parameter_reference.md#GPS_YAW_OFFSET) parameter.

:::info
The `GPS_YAW_OFFSET` is the angle made by the _baseline_ (the line between the two GPS antennas) relative to the vehicle x-axis (front/back axis, as shown [here](../config/flight_controller_orientation.md#calculating-orientation)).
:::

[Configure the serial port](../peripherals/serial_configuration.md) on which the Trimple will run using [GPS_1_CONFIG](../advanced_config/parameter_reference.md#GPS_1_CONFIG), and set the baud rate to 115200 using [SER_GPS1_BAUD](../advanced_config/parameter_reference.md#SER_GPS1_BAUD).

To activate heading fusion for the attitude estimation, set the [EKF2_GPS_CTRL](../advanced_config/parameter_reference.md#EKF2_GPS_CTRL) parameter to enable _Dual antenna heading_.

:::info
See also: [GPS > Configuration > GPS as Yaw/Heading Source](../gps_compass/index.md#configuring-gps-as-yaw-heading-source)
:::
