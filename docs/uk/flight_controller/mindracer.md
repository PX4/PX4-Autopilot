# Апаратне забезпечення MindRacer

:::warning
PX4 не розробляє цей (або будь-який інший) автопілот.
Contact the [manufacturer](http://mindpx.net) for hardware support or compliance issues.
:::

The AirMind<sup>&reg;</sup> [MindRacer](http://mindpx.net) series is a fully stackable flight _platform_ for miniature UAVs.
The platform currently has two RTF vehicles: [MindRacer 210](../complete_vehicles_mc/mindracer210.md) and [NanoMind 110](../complete_vehicles_mc/nanomind110.md).

![MindRacer](../../assets/hardware/hardware-mindracer.png)

:::info
This flight controller is [manufacturer supported](../flight_controller/autopilot_manufacturer_supported.md).
:::

## Короткий опис

MindRacer - це повністю стекована платформа для польотів для мініатюрних БПЛА.
Based on [MindPX](../flight_controller/mindpx.md), _MindRacer_ further scales down in formfactor while focused on providing modularity.
MindRacer is a _platform_ rather than a flight controller.

MindRacer імплементує концепт SEP (паяльно-елімінаційний-порт) та WEP (протокол-елімінації-проводів).
Перед SEP та WEP паяння та проводка завжди є основним джерелом проблем та зниження ефективності під час виготовлення та налаштування БПЛА.

:::info
The main hardware documentation is [here](http://mindpx.net/assets/accessories/mindracer_spec_v1.2.pdf).
:::

- Ультра-міні розмір, вага ~6г
- Високопродуктивний процесор з плаваючою точкою STM32F427 на частоті 168 МГц, надзвичайно швидка відповідь "педалі" газу
- Підтримка OneShot ESC
- Підтримка радіоприймачів PPM/SBUS/DSM, підтримка телеметрії D.Port/S.Port/Wifi
- На борту реєстратора даних польоту
- Підтримка ізоляції IMU
- Cтандартний DroneCode<sup>&reg;</sup> сумісний коннектор

|                   Елемент                  |                                                    Опис                                                   |
| :----------------------------------------: | :-------------------------------------------------------------------------------------------------------: |
|         Політний контролер/Процесор        |                                                  F427VIT6                                                 |
|                    Вага                    |                                            ~6г                                            |
|                   Розмір                   |                                                  35х35мм                                                  |
|                 PWM Виходи                 |                                                 максимум 6                                                |
|                     IMU                    |                                                   10DOF                                                   |
|                Ізоляція IMU                |                                              ТАК/Опціонально                                              |
|                Приймач радіо               |                                S.BUS/PPM/DSM/DSM2/DSMX/SUMD                               |
|                 Телеметрія                 | FrSky<sup>&reg;</sup> D.Port, S.Port, Wifi, 3DR radio |
| На борту TF-карти для запису даних польоту |                                                    ТАК                                                    |
|            Підтримка OneShot ESC           |                                                    ТАК                                                    |
|              Розширення слотів             |                                       2x7(pin)x2                                       |
|      На борту годинник реального часу      |                                                    ТАК                                                    |
|                  З’єднання                 |                        JST GH(відповідність стандарту DroneCode)                       |

## Швидкий Старт

### Карта виводів

![Mindracer pinout](../../assets/hardware/hardware-mindracer-pinout.png)

### Як зібрати

:::tip
Most users will not need to build this firmware!
It is pre-built and automatically installed by _QGroundControl_ when appropriate hardware is connected.
:::

To [build PX4](../dev_setup/building_px4.md) for this target:

```
make airmind_mindpx-v2_default
```

### Підключення ПК компаньйона

MindRacer має приєднану до себе плату Adapt IO.

![Attached Adapt IO board](../../assets/hardware/hardware-mindracer-conn.png)

MindRacer має вбудований конвертер UART-to-USB.
Для підключення комп'ютера-компаньйона встановіть MindRacer на плату інтерфейсу та підключіть супутній комп'ютер до USB-порту на платі інтерфейсу.

Максимальна швидкість BAUD така ж, як у родини px4, яка становить до 921600.

### Посібник користувача

:::info
The user guide is [here](http://mindpx.net/assets/accessories/mindracer_user_guide_v1.2.pdf)
:::

## Де купити

MindRacer is available at [AirMind Store](http://drupal.xitronet.com/?q=catalog).
Ви також можете знайти MindRacer на Amazon<sup>&reg;</sup> або на eBay<sup>&reg;</sup>.

## Підтримка

Будь ласка, зайдіть на http://www.mindpx.org для отримання додаткової інформації.
Or you can send email to [support@mindpx.net](mailto::support@mindpx.net) for any inquiries or help.
