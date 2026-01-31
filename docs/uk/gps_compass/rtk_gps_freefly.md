# Freefly Системи RTK GPS

The [Freefly Systems RTK GPS Module](https://store.freeflysystems.com/products/rtk-gps-ground-station) is a multiband [RTK GPS module](../gps_compass/rtk_gps.md) from Freefly Systems that provides highly reliable navigation.
Модулі можуть діяти як роувери (коли встановлені на повітряну судну), так і базові станції (коли підключені до комп'ютера).

Основні функції включають:

- Отримувач з багаторозрядним (L1/L2) (u-blox ZED-F9P)
- Одночасний прийом всіх 4 GNSS (GPS, Galileo, GLONASS, BeiDou)
- Вбудований магнітометр (IST8310), баро (BMP388), RGB LED, безпечний вимикач та світлодіод безпеки

:::info
This module can be used with PX4 v1.9 or above (support for the u-blox ZED-F9P was added in PX4 v1.9).
:::

![FreeFly GPS Module](../../assets/hardware/gps/freefly_gps_module.jpg)

## Де купити

- [Freefly Store](https://store.freeflysystems.com/products/rtk-gps-ground-station)

## Вміст набору

Набір RTK GPS включає в себе:

- 2x GPS модулі з антенами
- Кабель USB C на USB A довжиною 3 метри
- Магнітний швидкозамок для базової станції модуля (1/4-20 різьблення для монтажу на штатив)
- Гвинти для кріплення на Freefly AltaX

## Налаштування

RTK setup and use on PX4 via _QGroundControl_ is largely plug and play (see [RTK GPS](../gps_compass/rtk_gps.md) for more information).

For the aircraft, you should set the parameter [SER_GPS1_BAUD](../advanced_config/parameter_reference.md#SER_GPS1_BAUD) to 115200 8N1 to ensure that PX4 configures the correct baudrate.

## Підключення та з'єднання

RTK GPS Freefly поставляється з роз'ємом JST-GH на 8 контактів, який можна підключити до автопілота PixHawk.
Для використання як базова станція, модуль має роз'єм USB-C

### Схема розташування виводів

Схема виводів Freefly GPS наведена нижче.
For some autopilots, like the [Hex Cube](../flight_controller/pixhawk-2.md) and [PixRacer](../flight_controller/pixracer.md), all that is needed is a 1-1 8-pin JST-GH cable.

| Pin | Freefly GPS                     |
| --- | ------------------------------- |
| 1   | VCC_5V     |
| 2   | GPS_RX     |
| 3   | GPS_TX     |
| 4   | I2C_SCL    |
| 5   | I2C_SDA    |
| 6   | BUTTON                          |
| 7   | BUTTON_LED |
| 8   | GND                             |

## Специфікація

- Приймач GPS u-blox ZED-F9P
  - Резервне живлення Ultracap для швидкого (гарячого запуску)
  - Штормова пластина над приймачем для покращення стійкості до ЕМП
- Магнітометр IST8310
- Блок безпеки та безпечний світлодіод LED
- Світлодіоди RGB для показу статусу
  - Драйвер I2C NCP5623CMUTBG
- BMP388 Baro на шині I2C
- Зовнішня активна антена (Maxtena M7HCT)
  - SMA конектор
- STM32 МКП для майбутньої комунікації на основі CAN
  - Оновлення ПЗ через USB-роз'єм
- Підключення:
  - USB-C
  - 2-way USB перемикач до МКП та F9P
  - SMA для активної антени (макс. 20 мА)
  - 4-контактний JST-GH CAN шина (сумісність зі стандартами dronecode)
  - 8-pin JST-GH UART/I2C
    -\*\* Power:
  - Input from either (diode OR'd):
  - USB (5V)
  - CAN (4.7 до 25.2В)
  - (4.7 до 25.2V)
  - Power consumption <1W

## Докладніше

More information can be found on [Freefly's Wiki](https://freefly.gitbook.io/freefly-public/products/rtk-gps)
