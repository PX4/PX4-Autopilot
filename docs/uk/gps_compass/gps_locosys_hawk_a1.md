# LOCOSYS Hawk A1 GPS/GNSS

The [LOCOSYS HAWK A1 GPS/GNSS receiver](https://www.locosystech.com/en/product/hawk-a1-LU23031-V2.html) is a dual frequency multi-constellation GNSS/GPS receiver compatible with PX4.

Основні функції включають:

- Одночасний прийом сигналів смуг L1 і L5
- Підтримка GPS, ГЛОНАСС, BEIDOU, GALILEO, QZSS
- Підтримка SBAS (WAAS, EGNOS, MSAS, GAGAN)
- Підтримка GNSS з 135 каналами
- Швидкий TTFF на низькому рівні сигналу
- Безкоштовне прогнозування гібридних ефемерид для швидшого холодного старту
- За замовчуванням 5 Гц, частота оновлення до 10 Гц (SBAS лише з частотою оновлення 5 Гц)
- Вбудований суперконденсатор для резервування системних даних для швидкого отримання супутникових даних
- Три світлодіодного індикатора для живлення, PPS та передачі даних

![Hawk A1](../../assets/hardware/gps/locosys_hawk_a1/locosys_hawk_a1_gps.png)

## Де купити

- [LOCOSYS](https://www.locosystech.com/en/product/hawk-a1-LU23031-V2.html) (Taiwan)

## Налаштування

Ви можете використовувати Hawk A1 як основну або додаткову GPS-систему.
Параметри PX4 повинні бути встановлені, як показано нижче, для кожного випадку.

### Головний GNSS

Використовуйте Hawk A1 як основний GPS-пристрій:

| Параметр                                                                                                             | Значення                                                              | Опис                                                                                                       |
| -------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------- |
| [GPS_1_CONFIG](../advanced_config/parameter_reference.md#GPS_1_CONFIG)     | 102 (Telem 2 або інший доступний послідовний порт) | Налаштування основного порту GPS                                                                           |
| [GPS_1_PROTOCOL](../advanced_config/parameter_reference.md#GPS_1_PROTOCOL) | 1 (u-blox)                                         | Налаштування протоколу GPS                                                                                 |
| [SER_TEL2_BAUD](../advanced_config/parameter_reference.md#SER_TEL2_BAUD)   | 230400                                                                | Configure the serial port baudrate (here the GPS is connected to `TELEM2` for instance) |

### Другорядний GNSS

Використовувати Hawk A1 як допоміжний GPS-пристрій (на додаток до основного GPS):

| Параметр                                                                                                             | Значення                                                              | Опис                                                                                                   |
| -------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------ |
| [GPS_2_CONFIG](../advanced_config/parameter_reference.md#GPS_2_CONFIG)     | 102 (Telem 2 або інший доступний послідовний порт) | Налаштування основного порту GPS                                                                       |
| [GPS_2_PROTOCOL](../advanced_config/parameter_reference.md#GPS_2_PROTOCOL) | 1 (u-blox)                                         | Налаштування протоколу GPS                                                                             |
| [SER_TEL2_BAUD](../advanced_config/parameter_reference.md#SER_TEL2_BAUD)   | 230400                                                                | Налаштування швидкості послідовного порту (тут GPS підключено до TELEM2, наприклад) |

## Підключення та з'єднання

Locosys GPS поставляється з 6-контактним роз'ємом JST-GH стандарту Pixhawk, який можна вставити безпосередньо в UART-порт GPS1 (або в UART-порти GPS2 від Pixhawk FMUv5).

![GPS cable](../../assets/hardware/gps/locosys_hawk_a1/locosys_gps_cable.png)

### Схема розташування виводів

LOCOSYS схема виводів GPS наведена нижче.
Це може бути використано для модифікації роз'єму для інших плат автопілота.

| pin | Locosys GPS                 | pin | Pixhawk GPS 2               |
| --- | --------------------------- | --- | --------------------------- |
| 1   | VCC_5V | 1   | VCC                         |
| 2   | GPS_RX | 2   | GPS_TX |
| 3   | GPS_TX | 3   | GPS_RX |
| 4   | NC                          | 4   | SDA                         |
| 5   | NC                          | 5   | SCL                         |
| 6   | GND                         | 6   | GND                         |

## Індикатори статусу LEDs

| Колір   | Назва              | Опис                                  |
| ------- | ------------------ | ------------------------------------- |
| Зелений | Індикатор TX       | Передача даних GNSS                   |
| Red     | Індикатор живлення | Power                                 |
| Синій   | PPS                | Активна служба точного позиціонування |

![Hawk A1 LEDs](../../assets/hardware/gps/locosys_hawk_a1/locosys_hawk_a1_leds.png)

## Характеристики

- **Receiver Type:** 135-channel LOCOSYS MC-1612-V2b engine, GPS/QZSS L1 C/A, L5C, GLONASS L1OF, BeiDou B1I, B2a Galileo:E1, E5a SBAS L1 C/A: WAAS, EGNOS, MSAS, GAGAN
- **Navigation Update Rate:** Max: 5Hz default Max: 10 Hz
- **Positioning Accuracy:** 3D Fix
- **Time to first fix:**
  - **Cold start:** 28s
  - **Aided start:** EASY
- **Sensitivity:**
  - **Tracking & Navigation:** -165 dBm
- **Assisted GNSS:** EASY DGPS
- **Oscillator:** 26Mhz TCXO
- **RTC crystal:** 32.768KHz
- **Available Antennas:** L1+L5 multi frequency antenna
- **Signal Integrity:** L1+L5 GPS GLONASS GALILEO BEIDOU QZSS SBAS
- **Protocols & Interfaces:**
  - **UART/I2C:** JST_GH Main interface, Switch internally.

## Подальша інформація

- [LOCOSYS GPS User Manual](https://www.locosystech.com/Templates/att/LU23031-V2%20datasheet_v0.2.pdf?lng=en)
