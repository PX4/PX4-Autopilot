# Holybro Pixhawk 4 Mini (Знято з виробництва)

:::warning
PX4 не розробляє цей (або будь-який інший) автопілот.
Contact the [manufacturer](https://holybro.com/) for hardware support or compliance issues.
:::

The _Pixhawk<sup>&reg;</sup> 4 Mini_ autopilot is designed for engineers and hobbyists who are looking to tap into the power of _Pixhawk 4_ but are working with smaller drones.
_Pixhawk 4 Mini_ takes the FMU processor and memory resources from the _Pixhawk 4_ while eliminating interfaces that are normally unused.
This allows the _Pixhawk 4 Mini_ to be small enough to fit in a 250mm racer drone.

_Pixhawk 4 Mini_ was designed and developed in collaboration with Holybro<sup>&reg;</sup> and Auterion<sup>&reg;</sup>.
It is based on the [Pixhawk](https://pixhawk.org/) **FMUv5** design standard and is optimized to run PX4 flight control software.

![Pixhawk4 mini](../../assets/flight_controller/pixhawk4mini/pixhawk4mini_iso_1.png)

:::tip
This autopilot is [supported](../flight_controller/autopilot_pixhawk_standard.md) by the PX4 maintenance and test teams.
:::

## Короткий опис

- Головний FMU процесор: STM32F765
  - 32 Bit Arm® Cortex®-M7, 216MHz, 2MB memory, 512KB RAM
- Бортові сенсори:
  - Акселератор/гіроскоп: ICM-20689
  - Accel/Gyro: BMI055 або ICM20602
  - Магнітометр: IST8310
  - Барометр: MS5611
- GPS: u-blox Neo-M8N GPS/ГЛОНАСС приймач; інтегрований магнетометр IST8310
- Інтерфейси:
  - 8 PWM виводів
  - 4 виділених PWM/Capture входи на FMU
  - Виділений R/C вхід для CPPM
  - Виділений R/C вхід для Spektrum / DSM та S.Bus з аналоговим / PWM RSSI входом
  - 3 загальних послідовних портів
  - 2 I2C порти
  - 3 SPI шини
  - 1 CAN шина для CAN ESC
  - Аналогові входи для напруги / струму з батареї
  - 2 додаткових аналогових входи
- Система живлення:
  - Вхід Power Brick: 4.75~5.5V
  - Вхід USB Power: 4.75~5.25V
  - Вхід Servo Rail: 0~24V
  - Максимальний струм у значенні: 120 A
- Вага та розміри:
  - Вага: 37.2g
  - Розміри: 38x55x15.5mm
- Інші характеристики:
  - Температура роботи: -40 ~ 85°c

Additional information can be found in the [_Pixhawk 4 Mini_ Technical Data Sheet](https://github.com/PX4/PX4-user_guide/raw/main/assets/flight_controller/pixhawk4mini/pixhawk4mini_technical_data_sheet.pdf).

## Де купити

Order from [Holybro](https://holybro.com/collections/autopilot-flight-controllers/products/pixhawk4-mini).

## Інтерфейси

![Pixhawk 4 Mini interfaces](../../assets/flight_controller/pixhawk4mini/pixhawk4mini_interfaces.png)

:::warning
The **RC IN** and **PPM** ports are for RC receivers only. Вони працюють на електроживленні! НІКОЛИ не підключайте до нього жодних сервоприводів, джерел живлення або батарей (або до будь-якого підключеного приймача).
:::

## Схема розташування виводів

Download _Pixhawk 4 Mini_ pinouts from [here](https://github.com/PX4/PX4-user_guide/raw/main/assets/flight_controller/pixhawk4mini/pixhawk4mini_pinouts.pdf).

## Розміри

![Pixhawk 4 Mini Dimensions](../../assets/flight_controller/pixhawk4mini/pixhawk4mini_dimensions.png)

## Номінальна напруга

_Pixhawk 4 Mini_ can have power supply redundancy — if two power sources are supplied. The power rails are: **POWER** and **USB**.

:::info
The output power rail of **MAIN OUT** does not power the flight controller board (and is not powered by it).
You must [supply power](../assembly/quick_start_pixhawk4_mini.md#power) to one of **POWER** or **USB** or the board will be unpowered.
:::

**Normal Operation Maximum Ratings**

За таких умов всі джерела живлення будуть використовуватися в цьому порядку для живлення системи:

1. **POWER** (4.75V to 5.5V)
2. **USB** input (4.75V to 5.25V)

**Absolute Maximum Ratings**

За таких умов система залишиться неушкодженою.

1. **POWER** input (0V to 6V undamaged)
2. **USB** input (0V to 6V undamaged)
3. Servo input: VDD_SERVO pin of **MAIN OUT** (0V to 24V undamaged)

## Зборка/інсталяція

The [_Pixhawk 4 Mini_ Wiring Quick Start](../assembly/quick_start_pixhawk4_mini.md) provides instructions on how to assemble required/important peripherals including GPS, Power Management Board, etc.

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

The [PX4 System Console](../debug/system_console.md) and [SWD interface](../debug/swd_debug.md) run on the **FMU Debug** port.
In order to access these ports, the user must remove the _Pixhawk 4 Mini_ casing.

![Pixhawk 4 Mini FMU Debug](../../assets/flight_controller/pixhawk4mini/pixhawk4mini_fmu_debug.png)

The port has a standard serial pinout and can be connected to a standard FTDI cable (3.3V, but it's 5V tolerant) or a [Dronecode probe](https://kb.zubax.com/display/MAINKB/Dronecode+Probe+documentation). The pinout uses the standard [Pixhawk debug connector](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf) pinout. Please refer to the [wiring](../debug/system_console.md) page for details of how to wire up this port.

## Налаштування послідовного порту

|  UART  |  Пристрій  | Опис параметра QGC |               Мітка порту на FC              |
| :----: | :--------: | :----------------: | :------------------------------------------: |
|  UART1 | /dev/ttyS0 |        GPS1        |                  GPS Module                  |
| USART2 | /dev/ttyS1 |       TELEM1       |                    TELEM1                    |
| USART3 | /dev/ttyS2 |       TELEM2       |                      N/A                     |
|  UART4 | /dev/ttyS3 |    TELEM/SERIAL4   |                  UART/l2C B                  |
| USART6 | /dev/ttyS4 |         N/A        |                     RC IN                    |
|  UART7 | /dev/ttyS5 |         N/A        |                     Debug                    |
|  UART8 | /dev/ttyS6 |         N/A        | Не підключено (без PX4IO) |

## Периферійні пристрої

- [Digital Airspeed Sensor](https://holybro.com/products/digital-air-speed-sensor)
- [Telemetry Radio Modules](../telemetry/index.md)
- [Rangefinders/Distance sensors](../sensor/rangefinders.md)

## Підтримувані платформи

Motors and servos are connected to the **MAIN OUT** ports in the order specified for your vehicle in the [Airframe Reference](../airframes/airframe_reference.md).
Цей довідник містить зіставлення портів виводу до моторів/сервоприводів для всіх підтримуваних повітряних та наземних шасі (якщо ваше шасі не вказане в довіднику, то використовуйте "загальний" планер відповідного типу).

:::warning
_Pixhawk 4 Mini_ does not have AUX ports.
Плата не може використовуватися з шасі, яким необхідно більше ніж 8 портів або які використовують AUX порти для моторів або керування.
Вона може бути використана для планерів, які використовують AUX для другорядних периферійних пристроїв (наприклад, "feed-through of RC AUX1 channel").
:::

## Подальша інформація

- [_Pixhawk 4 Mini_ Technical Data Sheet](https://github.com/PX4/PX4-user_guide/raw/main/assets/flight_controller/pixhawk4mini/pixhawk4mini_technical_data_sheet.pdf)
- [FMUv5 reference design pinout](https://docs.google.com/spreadsheets/d/1-n0__BYDedQrc_2NHqBenG1DNepAgnHpSGglke-QQwY/edit#gid=912976165).
