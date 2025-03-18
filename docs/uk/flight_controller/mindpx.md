# Апаратне забезпечення MindPX

:::warning
PX4 не розробляє цей (або будь-який інший) автопілот.
Contact the [manufacturer](http://mindpx.net) for hardware support or compliance issues.
:::

The AirMind<sup>&reg;</sup> [MindPX](http://mindpx.net) series is a new generation autopilot system branched from Pixhawk<sup>&reg;</sup>.

![MindPX Controller](../../assets/hardware/hardware-mindpx.png)

:::info
These flight controllers are [manufacturer supported](../flight_controller/autopilot_manufacturer_supported.md).
:::

## Короткий опис

:::info
The main hardware documentation is [here](http://mindpx.net/assets/accessories/Specification9.18_3_pdf.pdf).
:::

MindPX - це нова система автопілотів, що створена з Pixhawk<sup>&reg;</sup>, переглянута в схематиці та структурі, і вони були ще більше розширені новими можливостями, щоб безпілотний пристрій був "розумнішим" та простшим у користуванні.

MindPX збільшує загальну кількість каналів виведення PWM до 16 (8 основних виводів + 8 допоміжних виводів).
Це означає, що MindPX може підтримувати більш складні конфігурації VTOL і кращий контроль.
Це особливо важливо для тих контролерів польоту на основі FMU-V4, оскільки MindPX реалізує основний та додатковий вивід в одному FMU.

![](../../assets/hardware/hardware-mindpx-specs.png)

- Головний системний чіп: STM32F427

  - Процесор: 32bits, 168 MHz ARM Cortex<sup>&reg;</sup> M4 з FPU
  - RAM: 256 KB SRAM
  - 2MB Flash
  - ST Micro LSM303D 14 бітний акселерометр/магнітометр
  - MEAS MS5611 барометр
  - Інтегровані датчики 6-осевого сенсора InvenSense<sup>&reg;</sup> MPU6500

- Виділені функції:
  - Корпус із обробленого CNC алюмінієвого сплаву, легкий і міцний
  - Вбудована ізольована резервна IMU
  - Загалом 16 каналів виведення PWM (8 основних + 8 додаткових)
  - 1 додатковий порт I2C для підключення потоку.
  - 1 додатковий USB-порт для підключення компаньйон-комп'ютера (вбудований конвертер UART-to-USB)
  - Відкритий порт для налагодження

## Швидкий Старт

### Встановлення

![MindPX Mounting](../../assets/hardware/hardware-mindpx-mounting.png)

### Підключення

![MindPX Wiring 1](../../assets/hardware/hardware-mindpx-wiring1.png)

![MindPX Wiring 2](../../assets/hardware/hardware-mindpx-wiring2.png)

### Pin

![MindPX Pinout](../../assets/hardware/hardware-mindpx-pin.png)

| Номер. |                            Опис                            | Номер. |                    Опис                   |
| :--------------------: | :--------------------------------------------------------: | :--------------------: | :---------------------------------------: |
|            1           |                            Power                           |            9           |     I2C2 (MindFLow)    |
|            2           | Відлагодження (оновлення завантажувача) |           10           | USB2 (Serial 2 to USB) |
|            3           |        USB1 (оновлення прошивки)        |           11           |                  UART4,5                  |
|            4           |                            Reset                           |           12           |   UART1 (Телеметрія)   |
|            5           |               UART3 (GPS)               |           13           |                    CAN                    |
|            6           |          I2C1(Зовнішній компас)         |           14           |                    ADC                    |
|            7           |                      Слот для TF-карти                     |           15           |             Триколорне світло             |
|            8           |     NRF/SPI(Дистанційне Управління)     |           16           |                   Looper                  |

### Приймач радіо

MindPX підтримує широкий спектр радіоприймачів (починаючи з версії V2.6), включаючи: PPM/SBUS/DSM/DSM2/DSMX.
MindPX також підтримує бі-дирекційну телеметрію FrSky<sup>&reg;</sup> D та S.Port.

For detailed Pin diagram, please refer to the [User Guide](http://mindpx.net/assets/accessories/UserGuide9.18_2_pdf.pdf).

### Збірка прошивки

:::tip
Most users will not need to build this firmware!
It is pre-built and automatically installed by _QGroundControl_ when appropriate hardware is connected.
:::

To [build PX4](../dev_setup/building_px4.md) for this target:

```
make airmind_mindpx-v2_default
```

### Підключення ПК компаньйона

MindPX має USB-TO-UART Bridge IC на платі.
Кабель micro-USB до USB type A використовується для підключення.
Підключіть micro-USB кінець до порту 'OBC' MindPX та USB type A кінець до комп'ютера-компаньйона.

Максимальна швидкість BAUD така ж, як у родини px4, яка становить до 921600.

## Посібник користувача

:::info
The user guide is [here](http://mindpx.net/assets/accessories/UserGuide9.18_2_pdf.pdf).
:::

## Де купити

MindRacer is available at [AirMind Store](http://drupal.xitronet.com/?q=catalog) on internet.
Ви також можете знайти MindRacer на Amazon<sup>&reg;</sup> або на eBay<sup>&reg;</sup>.

## Налаштування послідовного порту

| UART   | Пристрій   | Порт          |
| ------ | ---------- | ------------- |
| USART1 | /dev/ttyS0 | RC            |
| USART2 | /dev/ttyS1 | TELEM1        |
| USART3 | /dev/ttyS2 | TELEM2        |
| UART4  | /dev/ttyS3 | GPS1          |
| USART6 | /dev/ttyS4 | ?             |
| UART7  | /dev/ttyS5 | Debug Console |
| UART8  | /dev/ttyS6 | ?             |

<!-- Note: Got ports using https://github.com/PX4/PX4-user_guide/pull/672#issuecomment-598198434 -->

## Підтримка

Будь ласка, зайдіть на http://www.mindpx.org для отримання додаткової інформації.
Або ви можете надіслати email на [support@mindpx.net](mailto:support@mindpx.net) для будь-яких запитів або допомоги.
