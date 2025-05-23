# ThePeach FCC-R1

:::warning
PX4 не розробляє цей (або будь-який інший) автопілот.
Contact the [manufacturer](https://thepeach.kr/) for hardware support or compliance issues.
:::

**ThePeach FCC-R1** is an advanced autopilot designed and made in **ThePeach**.

It is based on the **Pixhawk-project FMUv3** open hardware design and runs **PX4** on **Nuttx OS**.

![ThePeach_R1](../../assets/flight_controller/thepeach_r1/main.png)

## Характеристики

- Основний процесор: STM32F427VIT6

  - 32-бітний ARM Cortex-M4, 168 МГц 256 КБ ОЗП 2 МБ флеш-пам'яті

- IO процесор: STM32F100C8T6

  - ARM Cortex-M3, 32 бітний ARM Cortex-M3, 24 МГц, 8КБ SRAM

- Сенсори на платі

  - Акселератор/гіроскоп: ICM-20602
  - Акселератор/гіроскоп/Магнітометр: MPU-9250
  - Барометр: MS5611

- Інтерфейси

  - 8+6 PWM виходів (8 з IO, 6 з FMU)
  - Spektrum DSM / DSM2 / DSM-X Satellite сумісний вхід
  - Futaba S.BUS сумісний вхід та вихід
  - Вхід сигналу PPM sum
  - Аналоговий / PWM вхід RSSI
  - Вихід сервоприводу S.Bus
  - Запобіжний вимикач / LED
  - 4x UART: TELEM1, TELEM2(Raspberry Pi CM3+), GPS, SERIAL4
  - 1x I2C порт
  - 1x CAN шина
  - Аналогові входи для напруги / струму з 1 батареї

- Інтерфейси для Raspberry Pi CM3+

  - VBUS
  - DDR2 Connector: Raspberry Pi CM3+
  - 1x UART
  - 2x USB
  - 1x Raspberry Pi камера

- Деталі механічної частини
  - Розміри: 49.2 x 101 x 18.2мм
  - Вага: 100g

## З’єднання

![pinmap_top](../../assets/flight_controller/thepeach_r1/pinmap.png)

## Налаштування послідовного порту

| UART   | Пристрій   | Порт                                          |
| ------ | ---------- | --------------------------------------------- |
| USART1 | /dev/ttyS0 | Відладка процесора вводу-виводу               |
| USART2 | /dev/ttyS1 | TELEM1 (керування потоком) |
| USART3 | /dev/ttyS2 | TELEM2 (Raspberry pi cm3+) |
| UART4  | /dev/ttyS3 | GPS1                                          |
| USART6 | /dev/ttyS4 | PX4IO                                         |
| UART7  | /dev/ttys5 | Debug console                                 |
| UART8  | /dev/ttyS6 | TELEM4                                        |

## Номінальна напруга

**ThePeach FCC-R1** can be double-redundant on the power supply if two power sources are supplied. The two power rails are: **POWER** and **USB**.

Примітка:

1. The output power rails **FMU PWM OUT** and **I/O PWM OUT** do not power the flight controller board (and are not powered by it). You must supply power to one of **POWER** or **USB** or the board will be unpowered.
2. The USB do not power the **Raspberry Pi CM3+**. You must supply power to **POWER** or the Raspberry Pi CM3+ will be unpowered.

**Normal Operation Maximum Ratings**

За таких умов всі джерела живлення будуть використовуватися в цьому порядку для живлення системи:

1. POWER вхід (5В до 5.5В)
2. USB вхід (4.75В до 5.25В)

**Absolute Maximum Ratings**

За таких умов всі джерела живлення спричиняють постійні пошкодження контролеру польоту.

1. POWER вхід (більше 5.5В)

2. USB вхід (більше 5.5В)

## Збірка прошивки

Щоб зібрати PX4 для цього контролера:

```jsx
make thepeach_r1_default
```

## Де купити

Order from [ThePeach](http://thepeach.shop/)
