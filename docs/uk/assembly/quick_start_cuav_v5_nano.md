# Швидке підключення CUAV V5 nano

:::warning
PX4 не розробляє цей (або будь-який інший) автопілот.
Contact the [manufacturer](https://store.cuav.net/) for hardware support or compliance issues.
:::

This quick start guide shows how to power the [CUAV V5 nano](../flight_controller/cuav_v5_nano.md) flight controller and connect its most important peripherals.

![Nano Hero Image](../../assets/flight_controller/cuav_v5_nano/v5_nano_01.png)

## Огляд схеми підключення

На зображенні нижче показано, як під'єднати найважливіші датчики та периферійні пристрої (за винятком виходів мотора та сервоприводів).
Ми розглянемо кожну з них докладно в наступних розділах.

![quickstart](../../assets/flight_controller/cuav_v5_nano/connection/v5_nano_quickstart_02.png)

| Основний інтерфейс                 | Функція                                                                                                                                                                                                                                               |
| :--------------------------------- | :---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Power                              | Під'єднати модуль живлення; Надає живлення та виміри напруги та струму ANALOG.                                                                                                                                                        |
| PM2                                | [Do not use with PX4](../flight_controller/cuav_v5_nano.md#compatibility_pm2)                                                                                                                                                                         |
| TF CARD                            | Карта SD для зберігання logs (постачається з картою)                                                                                                                                                                               |
| M1~M8              | PWM виходи. Може бути використаний для управління двигунами або сервоприводами.                                                                                                                                       |
| A1~A3              | Capture pins (not _currently_ supported on PX4).                                                                                                                                                                   |
| nARMED                             | Вказує на стан зброєння FMU. Це активне низьке (низьке під час увімкнення).                                                                                                                        |
| DSU7                               | Використовується для дебагінгу FMU, читання інформації щодо дебагінгу.                                                                                                                                                                |
| I2C2/I2C3/I2C4                     | Підключає пристрій I2C, такий як зовнішній компас.                                                                                                                                                                                    |
| CAN1/CAN2                          | Підключає пристрої UAVCAN, такі як CAN GPS.                                                                                                                                                                                           |
| TYPE-C\(USB\) | Під'єднатися до комп'ютера для зв'язку між контролером польоту та комп'ютером, наприклад, як завантаження прошивки                                                                                                                                    |
| GPS&SAFETY     | Приєднайтесь до Neo GPS, який містить GPS, перемикач безпеки, інтерфейс зумовлювача.                                                                                                                                                  |
| TELEM1/TELEM2                      | Під'єднатися до системи телеметрії.                                                                                                                                                                                                   |
| DSM/SBUS/RSSI                      | Включає інтерфейси введення сигналів DSM, SBUS, RSSI, інтерфейс DSM може бути підключений до приймача DSM-супутника, інтерфейс SBUS - до приймача дистанційного керування SBUS, RSSI - для модуля зворотного повернення сили сигналу. |

:::info
For more interface information, please read [V5 nano Manual](http://manual.cuav.net/V5-nano.pdf).
:::

![quickstart](../../assets/flight_controller/cuav_v5_nano/connection/v5_nano_quickstart_03.png)

:::info
If the controller cannot be mounted in the recommended/default orientation (e.g. due to space constraints) you will need to configure the autopilot software with the orientation that you actually used: [Flight Controller Orientation](../gps_compass/rtk_gps.md).
:::

## GPS + Компас + Безпечний вимикач + Світлодіоди

The recommended GPS module is the _Neo v2 GPS_, which contains GPS, compass, safety switch, buzzer, LED status light.

:::info
Other GPS modules may not work (see [this compatibility issue](../flight_controller/cuav_v5_nano.md#compatibility_gps)).
:::

The GPS/Compass module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker towards the front of the vehicle (Neo GPS arrow is in the same direction as the flight control arrow).
Під'єднайтеся до інтерфейсу GPS контролера польоту за допомогою кабелю.

:::info
If you use CAN GPS, please use the cable to connect to the flight control CAN interface.
:::

![quickstart](../../assets/flight_controller/cuav_v5_nano/connection/v5_nano_quickstart_04.png)

## Запобіжний перемикач

The dedicated safety switch that comes with the V5+ is only required if you are not using the recommended _Neo v2 GPS_ (which has an inbuilt safety switch).

If you are flying without the GPS you must attach the switch directly to the `GPS1` port in order to be able to arm the vehicle and fly (If you use the old 6-pin GPS, please read the definition of the bottom interface to change the line).

## Зумер

If you do not use the recommended _Neo v2 GPS_ the buzzer may not work.

## Радіоуправління

Для ручного керування вашим апаратом потрібна система дистанційного керування радіо (RC) (PX4 не вимагає наявності системи радіо для автономних режимів польоту).
Вам буде потрібно вибрати сумісний передавач/приймач та потім зв'язати їх, щоб вони взаємодіяли (прочитайте інструкції, що додаються до вашого конкретного передавача/приймача).

На малюнку нижче показано, як ви можете отримати доступ до вашого віддаленого приймача (знайдіть кабель S.Bus у комплекті)

![quickstart](../../assets/flight_controller/cuav_v5_nano/connection/v5_nano_quickstart_05.png)

## Супутникові приймачі Spektrum

V5 nano має присвячений кабель DSM.
If using a Spektrum satellite receiver, this should be connected to the flight controller `DSM/SBUS/RSSI` interface.

## Power

The _v5 nano_ kit includes the _HV_PM_ module, which supports 2~14S LiPo batteries.
Connect the 6pin connector of the _HW_PM_ module to the flight control `Power` interface.

:::warning
The supplied power module is unfused.
Power **must** be turned off while connecting peripherals.
:::

![quickstart](../../assets/flight_controller/cuav_v5_nano/connection/v5_nano_quickstart_06.png)

:::info
The power module is not a power source for peripherals connected to the PWM outputs.
Якщо ви підключаєте сервоприводи / приводи, вам потрібно окремо живити їх за допомогою BEC.
:::

## Система телеметрії (Опціонально)

Система телеметрії дозволяє вам спілкуватися, контролювати та управляти транспортним засобом у польоті з наземної станції (наприклад, ви можете направляти БПЛА до певної позиції або завантажувати нове завдання).

Канал зв'язку здійснюється через телеметричні радіостанції.
The vehicle-based radio should be connected to the **TELEM1** or **TELEM2** port (if connected to these ports, no further configuration is required).
Інша радіостанція підключається до вашого комп'ютера або мобільного пристрою наземної станції (зазвичай через USB).

![quickstart](../../assets/flight_controller/cuav_v5_nano/connection/v5_nano_quickstart_07.png)

<a id="sd_card"></a>

## SD-карта (Опціонально)

An [SD card](../getting_started/px4_basic_concepts.md#sd-cards-removable-memory) is inserted in the factory (you do not need to do anything).

## Двигуни

Motors/servos are connected to the MAIN ports in the order specified for your vehicle in the [Airframes Reference](../airframes/airframe_reference.md).

![quickstart](../../assets/flight_controller/cuav_v5_nano/connection/v5_nano_quickstart_06.png)

## Схема розташування виводів

![V5 nano pinouts](../../assets/flight_controller/cuav_v5_nano/v5_nano_pinouts.png)

## Подальша інформація

- [Airframe buildlog using CUAV v5 nano on a DJI FlameWheel450](../frames_multicopter/dji_f450_cuav_5nano.md)
- [CUAV V5 nano](../flight_controller/cuav_v5_nano.md)
- [V5 nano manual](http://manual.cuav.net/V5-nano.pdf) (CUAV)
- [FMUv5 reference design pinout](https://docs.google.com/spreadsheets/d/1-n0__BYDedQrc_2NHqBenG1DNepAgnHpSGglke-QQwY/edit#gid=912976165) (CUAV)
- [CUAV Github](https://github.com/cuav) (CUAV)
