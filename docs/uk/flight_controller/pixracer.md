# mRo Pixracer

:::warning
PX4 не розробляє цей (або будь-який інший) автопілот.
Contact the [manufacturer](https://store.mrobotics.io/) for hardware support or compliance issues.
:::

Сімейство плат Pixhawk<sup>&reg;</sup> XRacer оптимізоване для малих гоночних квадрокоптерів та літаків.
In contrast to [Pixfalcon](../flight_controller/pixfalcon.md) and [Pixhawk](../flight_controller/pixhawk.md) it has in-built WiFi, new sensors, convenient full servo headers, CAN and supports 2M flash.

<img src="../../assets/flight_controller/pixracer/pixracer_hero_grey.jpg" width="300px" title="pixracer + 8266 grey" />

:::tip
This autopilot is [supported](../flight_controller/autopilot_pixhawk_standard.md) by the PX4 maintenance and test teams.
:::

## Основні характеристики

- Main System-on-Chip: [STM32F427VIT6 rev.3](http://www.st.com/web/en/catalog/mmc/FM141/SC1169/SS1577/LN1789)
  - CPU: 180 МГц ARM Cortex<sup>&reg;</sup> M4 з одноточним FPU
  - RAM: 256 KB SRAM (L1)
- Стандартний FPV form factor: 36x36 mm зі стандартним 30.5 mm hole pattern
- Invensense<sup>&reg;</sup> ICM-20608 Accel / Gyro (4 KHz) / MPU9250 Accel / Gyro / Mag (4 KHz)
- Магнітометр HMC5983 з компенсацією температури
- Measurement Specialties MS5611 барометр
- JST GH конектори
- microSD (логування)
- Futaba S.BUS і S.BUS2 / Spektrum DSM2 і DSMX / Graupner SUMD / PPM input / Yuneec ST24
- FrSky<sup>&reg;</sup> порт телеметрії
- OneShot PWM (налаштовується)
- Опціонально: запобіжний перемикач та базер

## Де купити

Pixracer is available from the [mRobotics.io](https://store.mrobotics.io/mRo-PixRacer-R15-Official-p/m10023a.htm).

Аксесуари в комплекті:

- [Digital airspeed sensor](https://hobbyking.com/en_us/hkpilot-32-digital-air-speed-sensor-and-pitot-tube-set.html)
- [Hobbyking<sup>&reg;</sup> OSD + EU Telemetry (433 MHz)](https://hobbyking.com/en_us/micro-hkpilot-telemetry-radio-module-with-on-screen-display-osd-unit-433mhz.html)

## Набір

Pixracer розроблений для використання окремого джерела живлення авіоніки. Це необхідно для того, щоб уникнути стрибків струму від моторів чи ESC, що повертається до політного контролера і турбує його чутливі сенсори.

- Модуль живлення (з вимірюванням напруги та струму)
- Розгалужувач I2C (підтримка AUAV, Hobbyking та 3DR<sup>&reg;</sup> периферійних пристроїв)
- Набір кабелів для всіх поширених периферійних пристроїв

## Wifi (без необхідності USB)

Одна з головних особливостей плати - це її можливість використовувати Wifi для прошивки, налаштування системи та телеметрії в польоті.
Це звільняє його від потреби будь-якої десктопної системи.

- [ESP8266 Wifi](../telemetry/esp8266_wifi_module.md)
- [Custom ESP8266 MAVLink firmware](https://github.com/dogmaphobic/mavesp8266)

:::info
Firmware upgrade is not yet enabled over WiFi (it is supported by the default bootloader but not yet enabled).
Підтримуються налаштування та телеметрія.
:::

## Збірка

See the [Pixracer Wiring Quickstart](../assembly/quick_start_pixracer.md)

## Схеми з'єднань

![Grau setup pixracer top](../../assets/flight_controller/pixracer/grau_setup_pixracer_top.jpg)

:::info
If using `TELEM2` for an external telemetry module you will need to configure it as a MAVLink serial port.
For more information see: [Pixracer Wiring Quickstart > External Telemetry](../assembly/quick_start_pixracer.md#external-telemetry)
:::

![Grau setup pixracer bottom](../../assets/flight_controller/pixracer/grau_setup_pixracer_bottom.jpg)

![setup pixracer GPS](../../assets/flight_controller/pixracer/grau_setup_pixracer_gps.jpg)

![Grau b Pixracer FrSkyS.Port Connection](../../assets/flight_controller/pixracer/grau_b_pixracer_frskys.port_connection.jpg)

![Grau ACSP4 2 roh](../../assets/flight_controller/pixracer/grau_acsp4_2_roh.jpg)

![Grau ACSP5 roh](../../assets/flight_controller/pixracer/grau_acsp5_roh.jpg)

## З’єднання

All connectors follow the [Pixhawk connector standard](https://pixhawk.org/pixhawk-connector-standard/).
Якщо не вказано інше, всі конектори - це JST GH.

## Схема розташування виводів

![Pixracer top pinouts](../../assets/flight_controller/pixracer/pixracer_r09_top_pinouts.jpg)

![Pixracer bottom pinouts](../../assets/flight_controller/pixracer/pixracer_r09_bot_pinouts.jpg)

![Pixracer esp](../../assets/flight_controller/pixracer/pixracer_r09_esp_01.jpg)

#### TELEM1, TELEM2+OSD порти

| Pin                        | Сигнал                       | Вольтаж               |
| -------------------------- | ---------------------------- | --------------------- |
| 1 (red) | VCC                          | +5V                   |
| 2 (blk) | TX (OUT)  | +3.3V |
| 3 (blk) | RX (IN)   | +3.3V |
| 4 (blk) | CTS (IN)  | +3.3V |
| 5 (blk) | RTS (OUT) | +3.3V |
| 6 (blk) | GND                          | GND                   |

#### Порт GPS

| Pin                        | Сигнал                      | Вольтаж               |
| -------------------------- | --------------------------- | --------------------- |
| 1 (red) | VCC                         | +5V                   |
| 2 (blk) | TX (OUT) | +3.3V |
| 3 (blk) | RX (IN)  | +3.3V |
| 4 (blk) | I2C1 SCL                    | +3.3V |
| 5 (blk) | I2C1 SDA                    | +3.3V |
| 6 (blk) | GND                         | GND                   |

#### Телеметрія FrSky / SERIAL4

| Pin                        | Сигнал                      | Вольтаж               |
| -------------------------- | --------------------------- | --------------------- |
| 1 (red) | VCC                         | +5V                   |
| 2 (blk) | TX (OUT) | +3.3V |
| 3 (blk) | RX (IN)  | +3.3V |
| 4 (blk) | GND                         | GND                   |

#### RC вхід (приймає PPM / S.BUS / Spektrum / SUMD / ST24)

| Pin                        | Сигнал  | Вольтаж               |
| -------------------------- | ------- | --------------------- |
| 1 (red) | VCC     | +5V                   |
| 2 (blk) | RC IN   | +3.3V |
| 3 (blk) | RSSI IN | +3.3V |
| 4 (blk) | VDD 3V3 | +3.3V |
| 5 (blk) | GND     | GND                   |

#### CAN

| Pin                        | Сигнал                     | Вольтаж |
| -------------------------- | -------------------------- | ------- |
| 1 (red) | VCC                        | +5V     |
| 2 (blk) | CAN_H | +12V    |
| 3 (blk) | CAN_L | +12V    |
| 4 (blk) | GND                        | GND     |

#### POWER

| Pin                        | Сигнал  | Вольтаж               |
| -------------------------- | ------- | --------------------- |
| 1 (red) | VCC     | +5V                   |
| 2 (blk) | VCC     | +5V                   |
| 3 (blk) | CURRENT | +3.3V |
| 4 (blk) | VOLTAGE | +3.3V |
| 5 (blk) | GND     | GND                   |
| 6 (blk) | GND     | GND                   |

#### SWITCH

| Pin                        | Сигнал                                                   | Вольтаж               |
| -------------------------- | -------------------------------------------------------- | --------------------- |
| 1 (red) | SAFETY                                                   | GND                   |
| 2 (blk) | !IO_LED_SAFETY | GND                   |
| 3 (blk) | VCC                                                      | +3.3V |
| 4 (blk) | BUZZER-                                                  | -                     |
| 5 (blk) | BUZZER+                                                  | -                     |

#### Відладочний порт

The pinouts and connector comply with the [Pixhawk Debug Mini](../debug/swd_debug.md#pixhawk-debug-mini) interface defined in the [Pixhawk Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf) (JST SM06B connector).

| Pin                        | Сигнал                              | Вольтаж               |
| -------------------------- | ----------------------------------- | --------------------- |
| 1 (red) | VCC TARGET SHIFT                    | +3.3V |
| 2 (blk) | CONSOLE TX (OUT) | +3.3V |
| 3 (blk) | CONSOLE RX (IN)  | +3.3V |
| 4 (blk) | SWDIO                               | +3.3V |
| 5 (blk) | SWCLK                               | +3.3V |
| 6 (blk) | GND                                 | GND                   |

Інформацію про використання цього порту див:

- [SWD Debug Port](../debug/swd_debug.md)
- [PX4 System Console](../debug/system_console.md) (Note, the FMU console maps to UART7).

## Налаштування послідовного порту

| UART   | Пристрій   | Порт                                          |
| ------ | ---------- | --------------------------------------------- |
| UART1  | /dev/ttyS0 | WiFi (ESP8266)             |
| USART2 | /dev/ttyS1 | TELEM1 (керування потоком) |
| USART3 | /dev/ttyS2 | TELEM2 (керування потоком) |
| UART4  |            |                                               |
| UART7  | CONSOLE    |                                               |
| UART8  | SERIAL4    |                                               |

<!-- Note: Got ports using https://github.com/PX4/PX4-user_guide/pull/672#issuecomment-598198434 -->

## Креслення

The reference is provided as: [Altium Design Files](https://github.com/AUAV-OpenSource/FMUv4-PixRacer)

The following PDF files are provided for _convenience only_:

- [pixracer-rc12-12-06-2015-1330.pdf](https://github.com/PX4/PX4-user_guide/raw/main/assets/flight_controller/pixracer/pixracer-rc12-12-06-2015-1330.pdf)
- [pixracer-r14.pdf](https://github.com/PX4/PX4-user_guide/raw/main/assets/flight_controller/pixracer/pixracer-r14.pdf) - R14 or RC14 is printed next to the SDCard socket

## Збірка прошивки

:::tip
Most users will not need to build this firmware!
It is pre-built and automatically installed by _QGroundControl_ when appropriate hardware is connected.
:::

To [build PX4](../dev_setup/building_px4.md) for this target:

```
make px4_fmu-v4_default
```

## Налаштування

[Compass calibration](../config/compass.md) should be done with USB disconnected.
Це завжди рекомендується, але необхідно на Pixracer, оскільки підключення USB виробляє особливо великі рівні магнітного перешкодження.

Конфігурація в інших випадках така ж, як для інших плат.

## Автори

Цей дизайн був створений Nick Arsov і Phillip Kocmoud і архітектурно запроєктований Lorenz Meier, David Sidrane і Leonard Hall.
