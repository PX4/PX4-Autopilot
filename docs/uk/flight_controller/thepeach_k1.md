# ThePeach FCC-K1

:::warning
PX4 не розробляє цей (або будь-який інший) автопілот.
Contact the [manufacturer](https://thepeach.kr/) for hardware support or compliance issues.
:::

**ThePeach FCC-K1** is an advanced autopilot designed and manufactured in **ThePeach**.

It is based on the **Pixhawk-project FMUv3** open hardware design and runs **PX4** on **Nuttx OS**.

![ThePeach FCC-K1](../../assets/flight_controller/thepeach_k1/main.png)

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

  - 8+5 PWM виходів (8 з IO, 5 з FMU)
  - Spektrum DSM / DSM2 / DSM-X Satellite сумісний вхід
  - Futaba S.BUS сумісний вхід та вихід
  - Вхід сигналу PPM sum
  - Аналоговий / PWM вхід RSSI
  - Вихід сервоприводу S.Bus
  - Запобіжний вимикач / LED
  - 4x Порти UART: TELEM1, TELEM2, GPS, SERIAL4
  - 2x I2C порти
  - 1x CAN шина
  - 1x ADC
  - Аналогові входи для напруги / струму з 1 батареї

- Деталі механічної частини
  - Розміри: 40.2 x 61.1 x 24.8 mm
  - Вага: 65г

## З’єднання

![pinmap_top](../../assets/flight_controller/thepeach_k1/pinmap_top.png)

![pinmap_bottom](../../assets/flight_controller/thepeach_k1/pinmap_bottom.png)

## Налаштування послідовного порту

| UART   | Пристрій   | Порт                                          |
| ------ | ---------- | --------------------------------------------- |
| USART1 | /dev/ttyS0 | Відладка процесора вводу-виводу               |
| USART2 | /dev/ttyS1 | TELEM1 (керування потоком) |
| USART3 | /dev/ttyS2 | TELEM2 (керування потоком) |
| UART4  | /dev/ttyS3 | GPS1                                          |
| USART6 | /dev/ttyS4 | PX4IO                                         |
| UART7  | /dev/ttyS5 | Debug Console                                 |
| UART8  | /dev/ttyS6 | TELEM4                                        |

## Номінальна напруга

**ThePeach FCC-K1** can be double-redundant on the power supply if two power sources are supplied.
The two power rails are: **POWER** and **USB**.

:::info
The output power rails **FMU PWM OUT** and **I/O PWM OUT** do not power the flight controller board (and are not powered by it).
You must supply power to one of **POWER** or **USB** or the board will be unpowered.
:::

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
make thepeach_k1_default
```

## Де купити

Order from [ThePeach](http://thepeach.shop/)
